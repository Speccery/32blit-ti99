#include "ti99.hpp"
#include <cstring>  // memcpy
#include "tms9900.hpp"
#include "grom.hpp"

// #define VERIFY comes now from cmake

#ifdef VERIFY
#include "cpu9900.h"
class tursi_cpu_t : public CPU9900 {
public:
    virtual Word GetSafeWord(int x, int bank);
};
tursi_cpu_t tursi_cpu;
#endif

extern "C" unsigned char vdptest_data[];

using namespace blit;

// Our 256*192 framebuffer of 4 bits per pixels TMS9918 pixels.
#define TI_WIDTH 256
#define TI_HEIGHT 192
uint8_t ti_rendered[TI_WIDTH/2*TI_HEIGHT];
uint8_t full_screen_color = 0;
bool fill_full_screen = true;
bool debug_show_pixel = false;

uint32_t time_scanlines = 0;
FILE *debug_log = nullptr;

template<class X> struct averager_t {
    X values[8];
    int index;
    averager_t() {
        for(int i=0; i<8; i++)
            values[i] = 0;
        index = 0;
    }
    void new_value(X v) {
        values[index] = v;
        index = (index + 1) & 7;
    }
    X average() {
        X sum = 0 ;
        for(int i=0; i<8; i++)
            sum += values[i];
        return sum / 8;
    }

};

// #define DEBUG_PRINT

// TMS9918 colors from my tms9918.v verilog code.
// Map the 16-colors into 8-bit RGB.
uint8_t palette_lookup[16] = {
    // R[7:5], G[4:2], B[1:0]
    0, // transparent
    0, // black
    0b00011100,
    0b00111101,
    0b01001011,
    0b10010011,
    0b11101001,
    0b00011111,     // cyan
    0b11101001,     // medium red
    0b11110010,     // light red
    0b11111000,     // dark yellow
    0b11111010,     // light yellow
    0b00011000,     // dark green
    0b11110111,     // magenta
    0b10010010,     // gray
    0b11111111      // white
};

struct tms9918_t {
    uint8_t regs[8];
    uint8_t framebuf[16*1024];
    unsigned name_table_addr;
    uint8_t     hold_reg;
    uint16_t    vram_addr;
    bool        write_state;
    unsigned    vdp_writes;     //!< debug counter
    unsigned    vram_writes;    //!< debug counter
    unsigned    reg_writes;     //!< debug counter
    uint8_t     status;

    uint16_t    palette_rgb565[16];
    uint8_t     palette_rgb888[16*4];

    tms9918_t() {
        memset(regs,0,sizeof(regs));
        for(int i=0; i<16; i++) {
            palette_rgb565[i] = unpack_rgb565(i);
            unpack_rgb888(palette_rgb888 + i*4, i);
        }
    }
    void init() {
        /*
        regs[0] = 0x0;
        regs[1] = 0xE0;
        regs[2] = 0xF0;
        regs[3] = 0x0E;
        regs[4] = 0xF9;
        regs[5] = 0x86;
        regs[6] = 0xF8;
        regs[7] = 0xF7;
        */
        memcpy(framebuf, vdptest_data, 16*1024);
        memcpy(regs, vdptest_data+16*1024, 8 );
        name_table_addr = regs[2] << 10;
        hold_reg = 0;
        write_state = false;
        vram_addr = 0;
        vdp_writes = 0;
        vram_writes = 0;
        status = 0;
        reg_writes = 0;
    }     

    uint16_t unpack_rgb565(uint8_t c) {
        uint16_t k = palette_lookup[c & 0xF];
        // SRC:   RRR    GGG    BB
        // DST: BBBBB GGGGGG RRRRR (32blit on picosystem)
        uint16_t r = (k & 0xE0) >> 2;   // 5 bits of red
        uint16_t g = (k & 0x1C) << 1;   // 6 bits of green
        uint16_t b = (k & 3) << 3;      // 5 bits of blue
        return r | (g << 5) | (b << 11);
    }
    
    void unpack_rgb888(uint8_t *d, uint8_t c) {
        uint32_t k = palette_lookup[c & 0xF];
        d[0] = (k & 0xE0);        // R
        d[1] = ((k & 0x1C) << 3); // G
        d[2] = ((k & 3) << 6);    // B        
    }

    void scanline(int y) {
        // Render one scanline from framebuf to ti_rendered.
        name_table_addr = (regs[2] & 0xF) << 10;
        unsigned color_table_addr; 
        const int cell_width = regs[1] & 0x10 ? 6 : 8;
        const int cells = regs[1] & 0x10 ? 40 : 32;
        name_table_addr += (y >> 3) * cells;
        unsigned pattern_addr;

        for(int x=0; x<cells; x++) {
            uint8_t name = framebuf[name_table_addr];
            if(regs[1] & 8)
                pattern_addr = ((regs[4] & 7) << 11) | (name << 3) | ((y  >> 2) & 7); // Multicolor mode
            else if((regs[0] & 2) == 0) {
                pattern_addr = ((regs[4] & 7) << 11) | (name << 3) | (y & 7); // GM1 
                color_table_addr = (regs[3] << 6) | (name >> 3);
            } else {
                // GM2
                pattern_addr = (((name_table_addr >> 8) & (regs[4] & 3)) << 11) | (name << 3) | (y & 7);
                color_table_addr = ((regs[3] & 0x80) << 6) // MSB
                    + (((regs[3] & 0x7F) << 6) & (((name_table_addr & 0x300) | (name & 0xF8)) << 3))
                    + (((name & 7) << 3) | (y & 7)); // 6 LSBs
            }
            uint8_t pattern = framebuf[pattern_addr];
            uint8_t color = framebuf[color_table_addr];

            unsigned color0, color1;
            if(regs[1] & 8) {
                // Multicolor mode
                color1 = pattern >> 4;
                color0 = pattern & 0xF;
            } else {
                color1 = regs[1] & 0x10 ? regs[7] >> 4  : color >> 4;
                color0 = (color & 0xF) == 0 || (regs[1] & 0x10) ? regs[7] & 0xF : color & 0xF;
            }

#ifdef DEBUG_PRINT
            if(x == 0 && (y & 7) == 0) {
                printf("y=%d name=%d name_table_addr=0x%X pattern_addr=0x%X color_table_addr=0x%X color0=%d color1=%d color=0x%02X\n", 
                y>>3, name, name_table_addr, pattern_addr, color_table_addr, color0, color1, color);
            }
#endif

            // Pump pixels.
            uint8_t *p = ti_rendered + (y << 7) + (x*cell_width >> 1);            
#ifdef DEBUG_PRINT            
            if((unsigned long)(p - ti_rendered) > sizeof(ti_rendered)) {
                printf("overflow %ld y=%d\n", p-ti_rendered, y);
            }
#endif            
            for(int n=0; n<cell_width; n += 2) {
                uint8_t px = (pattern & 0x80 ? color1 : color0) << 4;
                px |= pattern & 0x40 ? color1 : color0;
                *p++ = px;
                pattern <<= 2;
            }
            name_table_addr++;
        }
        if(y == 191) {
            // Generate interrupt at the end of the last scanline.
            status |= 0x80;
        }
    }
    uint32_t render(int yoffset) {
        // Now render from ti_rendered to our framebuffer.
        uint32_t start = now_us();
        // My selfmade renderer. Center the image on the screen.
        int w = screen.bounds.w;
        int x_src_offset = 0;
        int x_dest_offset = 0;
        if(w < 256) {
            x_src_offset = (256-w)/2;
            x_src_offset >>= 1; // Divide further by 2 as each byte is 2 pixels.
        } else if(w > 256) {
            x_dest_offset = (w-256)/2;
        }
        if(screen.format == PixelFormat::RGB565) {
            for(int y=0; y<192; y++) {
                uint8_t *p = ti_rendered + (y << 7) + x_src_offset;
                uint16_t *d = (uint16_t *)(screen.data + (yoffset+y)*screen.row_stride+x_dest_offset*2);
                for(int x=0; x<128-x_src_offset*2; x++) {
                    // Two pixels per loop here.
                    uint16_t k = palette_lookup[*p >> 4];
                    // SRC:   RRR    GGG    BB
                    // DST: BBBBB GGGGGG RRRRR (32blit on picosystem)
                    uint16_t r = (k & 0xE0) >> 2;   // 5 bits of red
                    uint16_t g = (k & 0x1C) << 1;   // 6 bits of green
                    uint16_t b = (k & 3) << 3;      // 5 bits of blue
                    d[0] = r | (g << 5) | (b << 11);
                    k = palette_lookup[*p & 0xF];
                    r = (k & 0xE0) >> 2;   // 5 bits of red
                    g = (k & 0x1C) << 1;   // 6 bits of green
                    b = (k & 3) << 3;      // 5 bits of blue
                    d[1] = r | (g << 5) | (b << 11);
                    d += 2;
                    p++;
                }
            }
        } else if(screen.format == PixelFormat::RGB) {
            // 24 bit RGB
            for(int y=0; y<192; y++) {
                uint8_t *p = ti_rendered + (y << 7) + x_src_offset;
                uint8_t *d = screen.data + (yoffset+y)*screen.row_stride+3*x_dest_offset;
                for(int x=0; x<128; x++) {
                    // Two pixels per loop here.
                    uint32_t px = *p >> 4;
                    d[0] = palette_rgb888[px << 2];                    
                    d[1] = palette_rgb888[(px << 2) + 1];
                    d[2] = palette_rgb888[(px << 2) + 2];
                    px = *p & 0xF;
                    d[3] = palette_rgb888[px << 2];                    
                    d[4] = palette_rgb888[(px << 2) + 1];
                    d[5] = palette_rgb888[(px << 2) + 2];
                    d += 6;
                    p++;
                }
            }

        }    
        uint32_t end = now_us();
        return us_diff(start, end);
    }
    /**
     * @brief CPU write to VDP. 
     * 
     * @param regs_vram registers/data. A1 on the TI-99/4A
     * @param data 
     */
    void write(bool regs_vram, uint8_t data) {
        vdp_writes++;
        if(regs_vram) {
            if(!write_state) {
                hold_reg = data;
                write_state = true; 
                // printf("TMS9918 hold reg 0x%02X\n", hold_reg);
            } else {
                switch(data >> 6) {
                    case 0: // read from VRAM setup
                        vram_addr = ((data & 0x3F) << 8) | hold_reg;
                        // printf("Ready for VRAM read from 0x%04X\n", vram_addr);
                        break;
                    case 1: // write to VRAM setup
                        vram_addr = ((data & 0x3F) << 8) | hold_reg;
                        // printf("Ready for VRAM write to 0x%04X\n", vram_addr);
                        break;
                    case 2: // write to VDP register
                        regs[data & 7] = hold_reg;
                        // printf("VDP register %d write 0x%02X\n", data & 7, hold_reg);
                        reg_writes++;
                        break;
                    default:
                        break;
                }
                write_state = false;
            }
        } else {
            write_state = false;    // accesses to VRAM port reset write state
            framebuf[vram_addr] = data;
            vram_addr = 0x3FFF & (vram_addr+1);
            vram_writes++;
        }
    }
    /**
     * @brief CPU reads from VDP.
     * 
     * @param regs_vram      false: read from VRAM. true: read from status register.
     * @return uint8_t  data from VRAM or status register.
     */
    uint8_t read(bool regs_vram) {
        uint8_t result;
        write_state = false;    // any read resets write state
        if(regs_vram) {
            result = status;
            // if(interrupt_pending()) {
            //     printf("VDP interrupt cleared.\n");
            // }
            status &= 0x1F; // clear interrupt request, fifth sprite etc flags.
        } else {
            // read from VRAM.
            result = framebuf[vram_addr];
            // printf("VRAM read [0x%04X]=0x%02X\n", vram_addr, result);
            vram_addr = 0x3FFF & (vram_addr+1);
        }
        return result;
    }
    bool interrupt_pending() {
        return !!(status & 0x80);
    }
};

tms9918_t tms9918;

extern "C" {
    extern const unsigned char rom994a_data[];
    extern const unsigned char grom994a_data[];
    extern const unsigned char rominvaders_data[];
    extern const unsigned char grominvaders_data[];
}

class tigrom_t : public grom_t {
    int addr_complete;
    unsigned selected_groms;
    bool show_cpu;
public:
    tigrom_t() {
        addr_complete = 0;
        selected_groms = 0;
        show_cpu = false;
    }
    virtual void write(unsigned addr, uint8_t d) {
        grom_t::write(addr, d);
        addr_complete++;
        if(addr_complete == 2) {
            if(!(selected_groms & (1 << sel))) {
                printf("GROM sel=%d offset=0x%04X selected for the first time.\n", sel, offset);
                selected_groms |= 1 << sel;
                show_cpu = true;
            }
            if(sel >= 4) {
                printf("GROM sel=%d offset=0x%04X\n", sel, offset);
                show_cpu = true;
            }
        } 
    }
    bool should_cpu_stat_be_shown() {
        bool t = show_cpu;
        show_cpu = false;
        return t;
    }
protected:
    uint8_t read_mem(uint16_t addr) {
        addr_complete = 0;
        if(addr < 0x6000)
            return grom994a_data[addr];
        if(addr < 0x8000)
            return grominvaders_data[addr - 0x6000];
        printf("Reading outside valid GROM area 0x%04X.\n", addr);
        return 0;
    }
};

tigrom_t grom;

uint8_t scratchpad[256];

class cpu_t : public tms9900_t {
public:
    unsigned dsr_mem_counter;       //!< count DSR region accesses
    unsigned cart_counter;
    uint8_t keyboard[8];
    uint8_t keyscan;
    unsigned tms9901_cru;
    
    // Verification system: Tursi's CPU is run first for one cycle.
    // It reads actual "hardware" and stores read data to read verify buffer.
    // It writes to write verify buffer.
    // After that my CPU (tms9900_t) is run. It is not allowed to actually read, but rather
    // we check that the reads are found in read verify buffer. If found, we use that data.
    // If not found, we have a bug and must stop.
    // When my CPU writes, the writes are matched with write verify buffer and then 
    // actually written to "hardware".
    bool     verify;                //!< when true verify mode is on.
    uint16_t verify_wr_addrs[3], verify_wr_datas[3];    //!< Here we store write addresses and datas.
    uint8_t  verify_wr_count;       //!< number of write cycles performed by the instruction
    int      verify_wr_mask;        //!< bitmask corresponding to verify_wr_addrs

    bool     verify_reads;          //!< When true reads target verify read buffer.
    uint16_t verify_rd_addrs[8], verify_rd_datas[8];
    uint8_t  verify_rd_count;
    int      verify_rd_mask;


    cpu_t() {
        dsr_mem_counter = 0;
        cart_counter = 0;
        memset(keyboard, 0xFF, sizeof(keyboard));
        keyscan = 0;
        verify = false;
        tms9901_cru = ~0;
    }
public:
    uint16_t verify_read(uint16_t addr) {
        return read(addr);
    }
    void verify_write(uint16_t addr, uint16_t data) {
        if(verify) {
            if(verify_wr_count >= sizeof(verify_wr_addrs)/sizeof(verify_wr_addrs[0])) {
                printf("verify_write - bug, too many writes per instruction.\n");
                stuck = true;
            } else {
                verify_wr_addrs[verify_wr_count] = addr;
                verify_wr_datas[verify_wr_count] = data;
                verify_wr_count++;
            }
        } else 
            write(addr,data);   // Just write
    }
    void verify_write_cru_bit(uint16_t addr, uint8_t data) {
        write_cru_bit(addr, data);
    }
    uint8_t verify_read_cru_bit(uint16_t addr) {
        return read_cru_bit(addr);
    }
    void verify_begin() {
        verify = true;
        verify_reads = false;
        verify_wr_count = 0;
        verify_wr_mask = 0;
        verify_rd_count = 0;
        verify_rd_mask = 0;
    }
    void verify_switch_on_reads() {
        verify_reads = true;
    }
    void verify_show_rd_buffer() {
        printf("Instruction 0x%04X read verify error, count %d mask %d.\n", 
            prev_pc,
            verify_rd_count, verify_rd_mask);
        for(int i=0; i<verify_rd_count; i++) {
            printf("read buffer[%d]: [%04X]=%04X\n", i, verify_rd_addrs[i], verify_rd_datas[i]);
        }
    }
    void verify_show_wr_buffer() {
        printf("Instruction 0x%04X write verify error, count %d mask %d.\n", 
            prev_pc,
            verify_wr_count, verify_wr_mask);
        for(int i=0; i<verify_wr_count; i++) {
            printf("write buffer[%d]: [%04X]=%04X\n", i, verify_wr_addrs[i], verify_wr_datas[i]);
        }
    }
    void verify_end() {
        // Check that all writes have been accounted for.
        //                    0  1  2  3   4   5   6    7   8
        const int masks[] = { 0, 1, 3, 7, 15, 31, 63, 127, 255 };
        if(verify_wr_mask != masks[verify_wr_count]) {
            stuck = true;
            verify_show_wr_buffer();
        }
        // Check that all reads have been accounted for.
        if(verify_rd_mask != masks[verify_rd_count]) {
            stuck = true;
            verify_show_rd_buffer();
        }
        verify = false;
        verify_reads = false;
    }
    void verify_add_read_to_buffer(uint16_t addr, uint16_t data) {
        if(verify_rd_count >= sizeof(verify_rd_addrs)/sizeof(verify_rd_addrs[0])) {
            printf("verify_read - bug, too many reads per instruction.\n");
            stuck = true;
        } else {
            // Check that this address in not already in the buffer before adding it.
            for(int i=0; i<verify_rd_count; i++) {
                if(addr == verify_rd_addrs[i] && data == verify_rd_datas[i]) 
                    return; // Don't add it again.
            }
            verify_rd_addrs[verify_rd_count] = addr;
            verify_rd_datas[verify_rd_count] = data;
            verify_rd_count++;
        }
    }

protected:
    void check_write_in_verify_buffer(uint16_t addr, uint16_t data) {
        // Make sure that our write verify buffer has this write included.
        for(int i=0; i<verify_wr_count; i++) {
            if(addr == verify_wr_addrs[i] && data == verify_wr_datas[i] && !(verify_wr_mask & (1 << i))) {
                verify_wr_mask |= 1 << i;  // this slot verified properly.
                return;
            }
        }
        // Verify not found in buffer.
        stuck = true;
        printf("Instruction write [%04X]=%04X not found in verify buffer.\n", addr, data);
        for(int i=0; i<verify_wr_count; i++) {
            printf("write buffer[%d]: [%04X]=%04X\n", i, verify_wr_addrs[i], verify_wr_datas[i]);
        }
    }
    uint16_t check_read_in_verify_buffer(uint16_t addr) {
        for(int i=0; i<verify_rd_count; i++) {
            if(addr == verify_rd_addrs[i] && !(verify_rd_mask & (1 << i))) {
                verify_rd_mask |= 1 << i;  // this slot verified properly.
                return verify_rd_datas[i];
            }
        }
        // Make another pass to check if the desired read is in the buffer but already used. This can happen
        // since a CPU can read the same location more than once, and for writes we don't insert
        // the same stuff twice.
        for(int i=0; i<verify_rd_count; i++) {
            if(addr == verify_rd_addrs[i]) {
                return verify_rd_datas[i];
            }
        }
        // Verify not found in buffer.
        stuck = true;
        printf("Instruction at 0x%04X\n", prev_pc);
        printf("Instruction read [%04X] not found in verify buffer. count %d mask %d\n", addr, verify_rd_count, verify_rd_mask);
        for(int i=0; i<verify_rd_count; i++) {
            printf("read buffer[%d]: [%04X]=%04X\n", i, verify_rd_addrs[i], verify_rd_datas[i]);
        }
        return 0xDEAD;
    }

    void show_cpu() {
        printf("%6ld PC:%04X ST:%04X WP:%04X GROM:%04X %02X (%04X) ", 
            inst_count,
            prev_pc, st, wp, grom.get_read_addr(),
            grom.get_read_addr() < 0x6000 ? grom994a_data[grom.get_read_addr()-1] : 0xEE,
            (scratchpad[0x72] << 8) | scratchpad[0x73]
            );
        // show GROM stack entries.
        uint8_t p = scratchpad[0x73];
        for(uint8_t u=0x7E; u<p; u+=2) {
            uint16_t entry = (scratchpad[u] << 8) | scratchpad[u+1];
            printf("%04X ", entry);
        }
    }

     virtual uint16_t read(uint16_t addr) {
         addr &= ~1; // make the address even
         if(verify_reads) {
             return check_read_in_verify_buffer(addr);
         }
         uint16_t read_value = 0xdead;
         if(addr < 0x2000) {
             read_value =  (rom994a_data[addr] << 8) | rom994a_data[addr+1];
         } else if(addr >= 0x4000 && addr< 0x5FFF) {
             // No DSR memory for now.
             dsr_mem_counter++;
             read_value =  0;
         } else if(addr >= 0x6000 && addr < 0x8000) {
             // Cartridge access 
             read_value = (rominvaders_data[addr - 0x6000] << 8) 
                        |  rominvaders_data[addr - 0x6000+1];
             cart_counter++;
         } else if(addr >= 0x8300 && addr <0x8400) {
             read_value =  (scratchpad[addr - 0x8300] << 8) | scratchpad[addr - 0x8300 + 1];
         } else if(addr >= 0x8400 && addr< 0x8800) {
             // sound chip access
         } else if(addr >= 0x8800 && addr< 0x8C00) {
             // VDP read.
             uint16_t r = tms9918.read(!!(addr & 2));
             read_value =  r << 8; 
         } else if(addr >= 0x8C00 && addr< 0x9000) {
             // VDP write port
         } else if(addr >= 0x9000 && addr < 0x9800) {
             // speech synthesizer
             read_value =  0xAC00;
         } else if(addr >= 0x9800 && addr < 0xA000) {
             // GROM reads. 9800..9BFF is the actual read area.
             // 9C00..9FFF is write port, but read due to read-modify-write architecture.
             if(addr >= 9800 && addr < 0x9C00)
                read_value =  grom.read(addr) << 8;
            else
                read_value =  0xAB00;  // dummy
         } /* else if(addr >= 0xBD04 && addr< 0xBD08) {
             show_cpu();
             printf("mystery read from 0x%04X\n", addr);
             read_value =  0xDEAD;
         } */ else {
             stuck = true;
             printf("reading outside of memory: 0x%4X\n", addr);
             if(debug_log) 
                fprintf(debug_log, "reading outside of memory: 0x%4X\n", addr);
         }
         if(verify) {
             // Store this read to verify read buffer.
             verify_add_read_to_buffer(addr, read_value);
         }
         return read_value;
    }
    virtual void write(uint16_t addr, uint16_t data) {
        addr &= ~1; // make the addres even

        if(verify)
            check_write_in_verify_buffer(addr, data);

         if(addr >= 0 && addr < 0x2000) {
             printf("Write to ROM [%04X]=%04X\n", addr, data);
         } else if(addr >= 0x8300 && addr < 0x8400) {
             scratchpad[addr - 0x8300] = data >> 8;
             scratchpad[addr - 0x8300 + 1] = data;  // 8 low bits
         } else if(addr >= 0x8400 && addr < 0x8800) {
             // sound chip access
         } else if(addr >= 0x8C00 && addr < 0x9000) {
             // VDP write
             tms9918.write(!!(addr & 2), data >> 8);
             // static int count = 15;
             // if(--count == 0)
             //    stuck = true;
         } else if (addr >= 0x9c00 && addr < 0xA000) {
             grom.write(addr, data >> 8);
             if(grom.should_cpu_stat_be_shown())
                show_cpu();
         } else if(addr >= 0x9400 && addr < 0x9800) {
             // Speech write, do nothing.
         } else {
             stuck = true;
             printf("writing outside of memory: 0x%04X\n", addr);
         }
         if(debug_log) {
             fprintf(debug_log, "[0x%04X]=0x%04X, GROM addr 0x%04X *0x%02X VRAM writes %d VDP writes %d VRAM addr 0x%04X reg writes %d\n", 
                addr, data, grom.get_read_addr(), 
                grom.get_read_addr() < 0x6000 ? grom994a_data[grom.get_read_addr()-1] : 0xEE,
                    tms9918.vram_writes, tms9918.vdp_writes, tms9918.vram_addr, tms9918.reg_writes);
         }
         /*
         printf("Writing to 0x%04X value 0x%04X, GROM addr 0x%04X VRAM writes %d VDP writes %d VRAM addr 0x%04X reg writes %d\n", 
            addr, data, grom.get_read_addr(), 
            tms9918.vram_writes, tms9918.vdp_writes, tms9918.vram_addr, tms9918.reg_writes);
            */
    }
    virtual void write_cru_bit(uint16_t addr, uint8_t data) {
        // printf("CRU write 0x%04X data %d\n", addr, data);
        if(addr >= 0x24 && addr < 0x2A) {
            // keyboard scanline setting
            addr = (addr - 0x24) >> 1;
            uint8_t bit = 1 << addr;
            keyscan &= ~bit;
            keyscan |= data & 1 ? bit : 0;
        } 
        if(addr < 0x40) {
            unsigned bit = 1 << (addr >> 1);
            tms9901_cru &= ~bit;
            tms9901_cru |= data ? bit : 0;
        }
    }
    virtual uint8_t read_cru_bit(uint16_t addr) {
        // printf("CRU read 0x%04X\n", addr);
        if(addr >= 6 && addr < 0x16) {
            // Keyboard matrix
            uint8_t keyline = keyboard[keyscan];
            addr = (addr - 6) >> 1;
            return (keyline >> addr) & 1;
        }
        // VDP Interrupt bit
        if(addr == 4) {
            // printf("Reading CRU 4\n");
            return tms9918.interrupt_pending() ? 0 : 1;
        }

        return 1;
    }
};

cpu_t cpu;

void show_pixel_RGB(uint8_t *p) {
#ifdef DEBUG_PRINT      
    printf("RGB=(%d,%d,%d)\n", p[0], p[1], p[2]);
#endif
}


///////////////////////////////////////////////////////////////////////////
//
// init()
//
// setup your game here
//
void init() {
    set_screen_mode(ScreenMode::hires);
    tms9918.init();
    printf("About to enter cpu.reset()\n");
    cpu.reset();
    printf("Completed cpu.reset()\n");

#ifdef VERIFY
    printf("About to enter tursi_cpu.reset()\n");   
    tursi_cpu.reset();
    printf("Completed tursi_cpu.reset()\n");
#endif
}

///////////////////////////////////////////////////////////////////////////
//
// render(time)
//
// This function is called to perform rendering of the game. time is the 
// amount if milliseconds elapsed since the start of your game
//
void render(uint32_t time) {
    uint32_t draw_start = now_us();

    // clear the screen -- screen is a reference to the frame buffer and can be used to draw all things with the 32blit
    // screen.clear();

    // draw some text at the top of the screen
    screen.alpha = 255;
    screen.mask = nullptr;
    screen.pen = Pen(255, 255, 255);
    screen.rectangle(Rect(0, 0, 320, 14));
    screen.pen = Pen(0, 0, 0);
    char s[10];
    sprintf(s, "%04X ", cpu.pc);
    screen.text("TI-99/4A " + std::string(s) + " " + std::to_string(cpu.inst_count) + " " + (cpu.stuck ? "STUCK" : ""),
         minimal_font, Point(5, 4));

    if(fill_full_screen) {
        // Fill the whole screen with one of TI's colors
        screen.pen = Pen( palette_lookup[full_screen_color] & 0xE0,
                        (palette_lookup[full_screen_color] & 0x1C) << 3,
                        (palette_lookup[full_screen_color] & 0x3) << 6);
        screen.rectangle(Rect(0,15,TI_WIDTH, TI_HEIGHT+15));
        screen.pen = screen.pen.r ? Pen(0,0,0) : Pen(255,255,255);
        screen.text("color " + std::to_string(full_screen_color), minimal_font, Point(5, 20));
        const char *fo = "?";
        switch(screen.format) {
            case PixelFormat::RGB:   fo = "RGB";     break;
            case PixelFormat::RGBA:  fo = "RGBA";     break;
            case PixelFormat::P:     fo = "P"; break;
            case PixelFormat::M:     fo = "M"; break;
            case PixelFormat::RGB565: fo = "RGB565"; break;
            default:    fo = "unknown";
        }
        screen.text("pixel format " + std::string(fo) + " " + std::to_string(screen.pixel_stride) +
            " row: "+ std::to_string(screen.row_stride),
            minimal_font, Point(5,30));    
    } else {
        uint32_t t = tms9918.render(15);
        if(debug_show_pixel) {
            debug_show_pixel = false;
            show_pixel_RGB(screen.data + screen.row_stride*40);
            show_pixel_RGB(screen.data + screen.row_stride*40+3);
        }
        screen.pen = Pen(0,0,0);
        screen.rectangle(Rect(0,220,320,20));
        screen.pen = Pen(255,255,255);
        uint32_t draw_end = now_us();

        static averager_t<uint32_t> t9918;
        static averager_t<uint32_t> trender;
        static averager_t<uint32_t> tscanlines;
        t9918.new_value(t);
        trender.new_value(us_diff(draw_start, draw_end));
        tscanlines.new_value(time_scanlines);

        screen.text("9918: " + std::to_string(t9918.average()), minimal_font, Point(5, 220));
        screen.text("render: " + std::to_string(trender.average()), minimal_font, Point(160, 220));
        screen.text("scanlns:" + std::to_string(tscanlines.average()), minimal_font, Point(80,220));

    }

}

#ifdef VERIFY
void run_verify_step(bool verbose=true) {
    // First run Tursi's CPU, then mine and compare output.
    uint16_t tpc = tursi_cpu.GetPC();
    uint16_t mpc = cpu.pc;
    if(verbose) {
        char s[80];
        cpu.dasm_instruction(s, tursi_cpu.GetPC());
        printf("%d %04X %s\n", tursi_cpu.GetCycleCount(), tursi_cpu.GetPC(), s);
    }
    cpu.verify_begin();
    if(verbose)
        printf("Tursi's CPU running. ");
    tursi_cpu.ExecuteOpcode();
    cpu.verify_switch_on_reads();
    if(verbose)
        printf("My CPU running.\n");
    cpu.step();
    cpu.verify_end();
    // printf("Verify ended.\n");
    if(tursi_cpu.GetST() != cpu.st || tursi_cpu.GetPC() != cpu.pc || tursi_cpu.GetWP() != cpu.wp) {
        cpu.stuck = true;
        char s[80];
        cpu.dasm_instruction(s, tpc);
        printf("%d %04X %s\n", tursi_cpu.GetCycleCount(), tpc, s);
        printf("ST Tursi %04X my %04X\n", tursi_cpu.GetST(), cpu.st);
        printf("PC Tursi %04X me %04X\n", tursi_cpu.GetPC(), cpu.pc);
        printf("WP Tursi %04X me %04X\n", tursi_cpu.GetWP(), cpu.wp);
        printf("old PC Tursi %04X me %04X\n", tpc, mpc);
        cpu.verify_show_rd_buffer();
        cpu.verify_show_wr_buffer();
    }
}
#endif

///////////////////////////////////////////////////////////////////////////
//
// update(time)
//
// This is called to update your game state. time is the 
// amount if milliseconds elapsed since the start of your game
//
void update(uint32_t time) {
    static bool disasm = false;
    static int vdp_ints = 0;

    if(buttons.pressed & Button::A) {
        // Advance to next color.
        if(++full_screen_color >= 16)
            full_screen_color = 0;
        fill_full_screen = true;
    }
    if(buttons.pressed & Button::B) {
        fill_full_screen = false;
#ifdef DEBUG_PRINT          
        printf("full_screen_color %d\n", (int)full_screen_color);
        show_pixel_RGB(screen.data + screen.row_stride*40);
        show_pixel_RGB(screen.data + screen.row_stride*40+3);
#endif
        for(int y=0; y<192; y++)
            tms9918.scanline(y);
/*            
        // Overwrite with the color as above.
        int z = (full_screen_color << 4) 
                | full_screen_color;
        printf("z=0x%X, c=%d\n", z, full_screen_color);
        for(int i=0; i<192*128; i++)
            ti_rendered[i] = z;
        debug_show_pixel = true;
*/        
    }
    if(buttons.pressed & Button::X) {
        // cpu.reset();
        if(cpu.stuck) {
            printf("CPU Stuck\n");
            return;
        }
#ifdef VERIFY
        run_verify_step();
#else
        int loops = 5000;
        while(!cpu.stuck && loops > 0) {
            char s[80];
            int t = cpu.dasm_instruction(s, cpu.pc);
            printf("%ld %04X %s\n", cpu.inst_count, cpu.pc, s);
            cpu.step();
            loops--;
        }
#endif        
    }

    if(!cpu.stuck && 1) {
        for(int i=0; i<2000 && !cpu.stuck; i++) {
#ifdef VERIFY
            uint16_t a = tursi_cpu.GetPC();
            run_verify_step(false);
            if(cpu.stuck) {
                char s[80];
                cpu.dasm_instruction(s, a);
                printf("STUCK %d %04X %s\n", tursi_cpu.GetCycleCount(), tursi_cpu.GetPC(), s);
            }

            // Check interrupt status every 16 instructions
            if(!(i & 0xF) && tms9918.interrupt_pending() && (cpu.tms9901_cru & 4) ) {
                // printf("Interrupt stuff begin %ld.\n", cpu.inst_count);
                uint16_t cst = cpu.st;
                cpu.verify_begin();
                // BUGBUG CRU masking not done yet.
                if((tursi_cpu.GetST() & 0xF) > 1)
                    tursi_cpu.TriggerInterrupt(4);
                cpu.verify_switch_on_reads();
                bool b = cpu.interrupt(1);
                cpu.verify_end();
                // printf("Interrupt stuff end. CPU did vector: %s \n", b ? "true" : "false");
                if(b) {
                    printf("CPU vectored to interrupt, 0x%04X.\n", cst);
                    vdp_ints++;
                }
            }
#else
            if(disasm) {
                char s[80];
                cpu.dasm_instruction(s, cpu.prev_pc);
                printf("%ld %04X %s\n", cpu.inst_count, cpu.pc, s);
                if(debug_log) {
                    fprintf(debug_log, "%ld %04X %s\n", cpu.inst_count, cpu.pc, s);
                    fflush(debug_log);
                }
            }

            cpu.step();
            if(!(i & 0xF) && tms9918.interrupt_pending() && (cpu.tms9901_cru & 4) ) {
                // VDP interrupt
                uint16_t cst = cpu.st;
                bool b = cpu.interrupt(1);
                if(b) {
                    // printf("CPU vectored to interrupt, 0x%04X.\n", cst);
                    vdp_ints++;
                }
            }
            if(cpu.stuck) {
                // oh no, we became stuck!
                char s[80];
                cpu.dasm_instruction(s, cpu.prev_pc);
                printf("%ld %04X %s\n", cpu.inst_count, cpu.pc, s);
                printf("GROM_addr 0x%04X 0x%02X VRAM writes %d VDP writes %d VRAM addr 0x%04X reg writes %d\n", 
                    grom.get_read_addr(), 
                    grom.get_read_addr() < 0x6000 ? grom994a_data[grom.get_read_addr()-1] : 0xEE,
                    tms9918.vram_writes, tms9918.vdp_writes, tms9918.vram_addr, tms9918.reg_writes);
                if(debug_log) {
                    fprintf(debug_log, "CPU STUCK\n");
                    fprintf(debug_log, "%ld %04X %s\n", cpu.inst_count, cpu.pc, s);
                    fprintf(debug_log, "GROM_addr 0x%04X 0x%02X VRAM writes %d VDP writes %d VRAM addr 0x%04X reg writes %d\n", 
                        grom.get_read_addr(), 
                        grom.get_read_addr() < 0x6000 ? grom994a_data[grom.get_read_addr()-1] : 0xEE,
                        tms9918.vram_writes, tms9918.vdp_writes, tms9918.vram_addr, tms9918.reg_writes);
                    fflush(debug_log);
                }
            }
#endif
        }
        if(!cpu.stuck) {
            fill_full_screen = false;
            uint32_t start = now_us();
            for(int y=0; y<192; y++)
                tms9918.scanline(y);    
            time_scanlines = us_diff(start, now_us());
        }
    }

    if(buttons & Button::DPAD_LEFT) {
        cpu.keyboard[5] &= ~(1 << 4);   // '1' down
/*        
        if(!disasm) {
            disasm = true;
            cpu.inst_count = 0;
            debug_log = fopen("debug.txt", "wt");
        }
*/        
    } else
        cpu.keyboard[5] |= (1 << 4);   // '1' up

    if(buttons & Button::DPAD_RIGHT) {
        cpu.keyboard[1] &= ~(1 << 4);   // '2' down
    } else {
        cpu.keyboard[1] |= (1 << 4);   // '2' up
    }
    if(buttons.pressed & Button::DPAD_UP) {
        printf("TMS9901 CRU 0x%08X VDP pending %d VDP ints %d\n",
            cpu.tms9901_cru, tms9918.interrupt_pending(), vdp_ints
        );
    }
}

#ifdef VERIFY

Word tursi_cpu_t::GetSafeWord(int x, int bank) {
    return cpu.verify_read(x);
}

void wrword(Word x, Word y)
{ 
	x&=0xfffe;		// drop LSB
	// now write the new data
	// wcpubyte(x,(Byte)(y>>8));
	// wcpubyte(x+1,(Byte)(y&0xff));

    cpu.verify_write(x, y);
}

Word romword(Word x, bool rmw)
{ 
	x&=0xfffe;		// drop LSB
//	// TODO: this reads the LSB first. Is this the correct order??
// 	return((rcpubyte(x,rmw)<<8)+rcpubyte(x+1,rmw));
    return cpu.verify_read(x);
}

void wcru(Word addr,int data) {
    cpu.verify_write_cru_bit(addr << 1, data) ;
}

int rcru(Word addr) {
  return cpu.verify_read_cru_bit( addr << 1);
}

#endif
