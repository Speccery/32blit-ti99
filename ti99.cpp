#include "ti99.hpp"
#include <cstring>  // memcpy
#include "tms9900.hpp"
#include "tms9918.hpp"
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


using namespace blit;

// TI Font.
uint8_t ti_font_data[96][8];

static  uint8_t ti_font_width[96] = {
  8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 
  8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
  8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
  8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
  8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
  8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8
};

const Font ti_font(&ti_font_data[0][0], ti_font_width, 8, 8);

#define TI_CPU_CLK (3000000)
uint32_t cpu_clk = TI_CPU_CLK;

// Our 256*192 framebuffer of 4 bits per pixels TMS9918 pixels.
#define TI_WIDTH 256
#define TI_HEIGHT 192

uint8_t ti_rendered[TI_WIDTH/2*TI_HEIGHT];
uint8_t full_screen_color = 0;
bool fill_full_screen = true;
bool debug_show_pixel = false;

uint32_t time_scanlines = 0;
uint32_t time_cpu = 0;      // time taken by CPU cycles between TMS9918 scanlines through the screen
FILE *debug_log = nullptr;
uint32_t bench_result = 0;

template<class X> struct averager_t {
    X values[16];
    int index;
    averager_t() {
        for(int i=0; i<16; i++)
            values[i] = 0;
        index = 0;
    }
    void new_value(X v) {
        values[index] = v;
        index = (index + 1) & 15;
    }
    X average() {
        X sum = 0 ;
        for(int i=0; i<16; i++)
            sum += values[i];
        return sum / 16;
    }

};

// #define DEBUG_PRINT





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

        // Init read function table.
        for(int i=0; i<64; i++)
            read_funcs[i] = &cpu_t::read_unknown;
        for(int i=0; i<8; i++) {
            // Init several 8k regions
            read_funcs[i] = &cpu_t::read_rom;
            read_funcs[16+i] = &cpu_t::read_dsr;
            read_funcs[24+i] = &cpu_t::read_cartridge;
        }
        read_funcs[ 0x8300 >> 10] = &cpu_t::read_scrachpad;
        read_funcs[ 0x8400 >> 10] = &cpu_t::read_soundchip;
        read_funcs[ 0x8800 >> 10] = &cpu_t::read_vdp;
        read_funcs[ 0x8C00 >> 10] = &cpu_t::read_vdp_write_port;
        read_funcs[ 0x9000 >> 10] = &cpu_t::read_speech;
        read_funcs[ 0x9400 >> 10] = &cpu_t::read_speech;
        read_funcs[ 0x9800 >> 10] = &cpu_t::read_grom;
        read_funcs[ 0x9C00 >> 10] = &cpu_t::read_grom_write_port;
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
            get_instructions(),
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

    uint16_t (cpu_t::*read_funcs[64])(uint16_t addr);
    uint16_t read_rom(uint16_t addr) {
        return (rom994a_data[addr] << 8) | rom994a_data[addr+1];
    }
    uint16_t read_dsr(uint16_t addr) {
        // No DSR memory for now. 0x4000
        dsr_mem_counter++;
        add_ext_cycles(4);
        return 0;
    }
    uint16_t read_cartridge(uint16_t addr) {
        // 0x6000..0x7FFF
        cart_counter++;
        add_ext_cycles(4);
        return (rominvaders_data[addr - 0x6000] << 8) 
                |  rominvaders_data[addr - 0x6000+1];
    }
    uint16_t read_scrachpad(uint16_t addr) {
        return (scratchpad[addr & 0xFE] << 8) | scratchpad[(addr & 0xFE) + 1];
    }
    uint16_t read_soundchip(uint16_t addr) {
        // 8400
        add_ext_cycles(4);
        return 0;
    }
    uint16_t read_vdp(uint16_t addr) {
        // VDP read. 0x8800
        uint16_t r = tms9918.read(!!(addr & 2));
        add_ext_cycles(4);
        return r << 8; 
    }
    uint16_t read_vdp_write_port(uint16_t addr) {
        // VDP write port read 0x8C00
        add_ext_cycles(4);
        return 0;
    }
    uint16_t read_speech(uint16_t addr) {
        // speech synthesizer 0x9000
        add_ext_cycles(4);
        return 0xAC00;
    }
    uint16_t read_grom(uint16_t addr) {
        // 0x9800
        add_cycles(4);
        return grom.read(addr) << 8;
    }
    uint16_t read_grom_write_port(uint16_t addr) {
        // VDP write port read 0x9C00
        add_ext_cycles(4);
        return 0;
    }
    uint16_t read_unknown(uint16_t addr) {
        stuck = true;
        printf("reading outside of memory: 0x%4X\n", addr);
        if(debug_log) 
        fprintf(debug_log, "reading outside of memory: 0x%4X\n", addr);
        return 0xDEAD;
    }

    virtual uint16_t read(uint16_t addr) {
        addr &= ~1;
        return (this->*read_funcs[addr >> 10])(addr);
    }

     virtual uint16_t read_all_cases(uint16_t addr) {
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
             add_ext_cycles(4);
         } else if(addr >= 0x6000 && addr < 0x8000) {
             // Cartridge access 
             read_value = (rominvaders_data[addr - 0x6000] << 8) 
                        |  rominvaders_data[addr - 0x6000+1];
             cart_counter++;
             add_ext_cycles(4);
         } else if(addr >= 0x8300 && addr <0x8400) {
             read_value =  (scratchpad[addr - 0x8300] << 8) | scratchpad[addr - 0x8300 + 1];
         } else if(addr >= 0x8400 && addr< 0x8800) {
             // sound chip access
             add_ext_cycles(4);
         } else if(addr >= 0x8800 && addr< 0x8C00) {
             // VDP read.
             uint16_t r = tms9918.read(!!(addr & 2));
             read_value =  r << 8; 
             add_ext_cycles(4);
         } else if(addr >= 0x8C00 && addr< 0x9000) {
             // VDP write port
             add_ext_cycles(4);
         } else if(addr >= 0x9000 && addr < 0x9800) {
             // speech synthesizer
             read_value =  0xAC00;
             add_ext_cycles(4);
         } else if(addr >= 0x9800 && addr < 0xA000) {
             // GROM reads. 9800..9BFF is the actual read area.
             // 9C00..9FFF is write port, but read due to read-modify-write architecture.
             if(addr >= 9800 && addr < 0x9C00)
                read_value =  grom.read(addr) << 8;
            else
                read_value =  0xAB00;  // dummy
            add_ext_cycles(4);
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

#ifdef VERIFY
        if(verify)
            check_write_in_verify_buffer(addr, data);
#endif
         if(addr >= 0x8300 && addr < 0x8400) {
             scratchpad[addr - 0x8300] = data >> 8;
             scratchpad[addr - 0x8300 + 1] = data;  // 8 low bits
         } else if(addr >= 0x8C00 && addr < 0x9000) {
            // VDP write
            tms9918.write(!!(addr & 2), data >> 8);
            // static int count = 15;
            // if(--count == 0)
            //    stuck = true;
            add_ext_cycles(4);
         } else if(addr >= 0 && addr < 0x2000) {
             printf("Write to ROM [%04X]=%04X\n", addr, data);
         } else if(addr >= 0x8400 && addr < 0x8800) {
             // sound chip access
             add_ext_cycles(4);
         } else if (addr >= 0x9c00 && addr < 0xA000) {
            grom.write(addr, data >> 8);
            if(grom.should_cpu_stat_be_shown())
               show_cpu();
            add_ext_cycles(4);
         } else if(addr >= 0x9400 && addr < 0x9800) {
             // Speech write, do nothing.
             add_ext_cycles(4);
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

uint32_t last_render_second=0; // timestamp of last full second
uint32_t last_render_cycles=0; // cycles at last time stamp
int      kHz = 3000;    // last computed kilohertz
float    fps = 10.0f;   // last computed fps
float    vdp_draws = 0;
uint32_t last_render_frames=0;
uint32_t render_frames=0;
uint32_t last_update_time = 0;
uint32_t last_update_tms9918_run_cycles = 0;    // Last CPU cycles we did run the VDP
uint32_t drawn_frames = 0;
uint32_t last_drawn_frames = 0;

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

    last_render_second = 0;
    last_render_cycles = 0;
    last_render_frames = 0;
    render_frames = 0;
    last_update_time = 0;
    last_update_tms9918_run_cycles = 0;
    drawn_frames = 0;
    last_drawn_frames = 0;

    // Copy TI font data over from GROM.
    for(int ch=0; ch<96; ch++) {
        uint8_t *p = ti_font_data[ch];
        // The 32blit seems to store data vertically, i.e. exclamation mark is 0x2E and that's it.
        // It also seems to be upside down, i.e. LSB is the topmost bit.
        // Thus we need to here rotate the content.
        for(int j=0; j<8; j++) {
            uint8_t bits = 0;
            // TI font data is only 7 pixel high.
            for(int y=0; y<7; y++) {
                bits |= grom994a_data[0x6B4+ch*7+(6-y)] & (1 << (7-j)) ? 0x80 >> y : 0; 
            }
            p[j] = bits;
        }
    }
    // Make the TI font somewhat proportional.
    ti_font_width[0] = 3;   // Space is 3 pixels
    ti_font_width[':'-32] = 4;  // Colon is 4 pixels
    for(int i='0'; i<= '9'; i++)        // Make numbers 6 pixels wide
        ti_font_width[i-32] = 6;
    for(int i='A'; i<= 'Z'; i++) {   // Make A to Z 6 pixels wide
        ti_font_width[i-32] = 6;
        ti_font_width[i-32+'a'-'A'] = 6;
    }
    

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
    render_frames++;

    // clear the screen -- screen is a reference to the frame buffer and can be used to draw all things with the 32blit
    // screen.clear();

    // draw some text at the top of the screen
    screen.alpha = 255;
    screen.mask = nullptr;
    screen.pen = Pen(255, 255, 255);
    screen.rectangle(Rect(0, 0, 320, 14));
    screen.pen = Pen(0, 0, 0);
    char s[10];
    sprintf(s, "%04X ", cpu.get_pc());
    screen.text("TI-99/4A " + std::string(s) + " " + (cpu.is_stuck() ? "STUCK" : ""), ti_font, Point(5, 4));
    screen.text(std::to_string((int)(fps+0.5f)), ti_font, Point(200,4));
    screen.text(std::to_string((int)(vdp_draws+0.5f)), ti_font, Point(220, 4));

    if(fill_full_screen) {
        // Fill the whole screen with one of TI's colors
        screen.pen = Pen(tms9918_t::palette_lookup[full_screen_color] & 0xE0,
                        (tms9918_t::palette_lookup[full_screen_color] & 0x1C) << 3,
                        (tms9918_t::palette_lookup[full_screen_color] & 0x3) << 6);
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
        uint32_t t = tms9918.render(ti_rendered, 15);
        if(debug_show_pixel) {
            debug_show_pixel = false;
            show_pixel_RGB(screen.data + screen.row_stride*40);
            show_pixel_RGB(screen.data + screen.row_stride*40+3);
        }

        screen.pen = Pen(0,0,0);
        // on first execution of this code path clear the screen
        static bool clear_needed = true;
        if(clear_needed) {
            clear_needed = false;
            screen.clear();
        }

        screen.rectangle(Rect(0,220,320,20));
        screen.pen = Pen(255,255,255);
        uint32_t draw_end = now_us();

        static averager_t<uint32_t> t9918;
        static averager_t<uint32_t> trender;
        static averager_t<uint32_t> tscanlines;
        static averager_t<uint32_t> tcpu;
        t9918.new_value(t);
        trender.new_value(us_diff(draw_start, draw_end));
        tscanlines.new_value(time_scanlines);
        tcpu.new_value(time_cpu);

        screen.text("CPU: " + std::to_string(tcpu.average()), ti_font,       Point(1, 220));
        screen.text("Lines: " + std::to_string(tscanlines.average()), ti_font, Point(60,220));
        screen.text("Rend: " + std::to_string(trender.average()), ti_font,   Point(128, 220));
        if(time - last_render_second >= 1000) {
            kHz = (cpu.get_cycles() - last_render_cycles) / ((time - last_render_second));
            fps = 1000.0f*(render_frames - last_render_frames) / (time - last_render_second);
            vdp_draws = 1000.0f*(drawn_frames - last_drawn_frames) / (time - last_render_second);
            last_render_frames = render_frames;
            last_drawn_frames = drawn_frames;
            last_render_second = time;
            last_render_cycles = cpu.get_cycles();
        }
        screen.text("kHz: " + std::to_string(kHz), ti_font, Point(188, 220));

    }

    if(bench_result) {
        screen.text("Benchmark: " + std::to_string(bench_result), ti_font, Point(20, 100));
        screen.text("MHz: " + std::to_string(bench_result/100000), ti_font, Point(20, 108));
    }

}

#ifdef VERIFY
void run_verify_step(bool verbose=true) {
    // First run Tursi's CPU, then mine and compare output.
    uint16_t tpc = tursi_cpu.GetPC();
    uint16_t mpc = cpu.get_pc();
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
    if(tursi_cpu.GetST() != cpu.st || tursi_cpu.GetPC() != cpu.get_pc() || tursi_cpu.GetWP() != cpu.wp) {
        cpu.is_stuck() = true;
        char s[80];
        cpu.dasm_instruction(s, tpc);
        printf("%d %04X %s\n", tursi_cpu.GetCycleCount(), tpc, s);
        printf("ST Tursi %04X my %04X\n", tursi_cpu.GetST(), cpu.st);
        printf("PC Tursi %04X me %04X\n", tursi_cpu.GetPC(), cpu.get_pc());
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

    /* if(buttons.pressed & Button::A) {
        // Advance to next color.
        if(++full_screen_color >= 16)
            full_screen_color = 0;
        fill_full_screen = true;
    } */

    // Joystick
    cpu.keyboard[6] |= 0x1F;                 // Assume all joystick buttons not pressed
    if(buttons & Button::A)
        cpu.keyboard[6] &= ~1;               // fire pressed
    if(buttons & Button::DPAD_LEFT)
        cpu.keyboard[6] &= ~2; 
    if(buttons & Button::DPAD_RIGHT)
        cpu.keyboard[6] &= ~4; 
    if(buttons & Button::DPAD_DOWN)
        cpu.keyboard[6] &= ~8; 
    if(buttons & Button::DPAD_UP)
        cpu.keyboard[6] &= ~16; 
    
    // QUIT
    cpu.keyboard[0] |= 0x11;    // FCTN and = not pressed
    if((buttons & Button::DPAD_UP) && (buttons & Button::Y))    // DPAD UP and Y pressed?
        cpu.keyboard[0] &= ~0x11;   // yes FCTN and = both pressed
    // BACK
    cpu.keyboard[1] |= (1 << 3);    // 9 not pressed (FCTN set above)
    if((buttons & Button::DPAD_DOWN) && (buttons & Button::Y)) {
        cpu.keyboard[1] &= ~(1 << 3);   // 9 pressed
        cpu.keyboard[0] &= ~0x10;       // FCTN pressed
    }

    // CPU clock adjustment
    if((buttons & Button::B) && (buttons.pressed & Button::DPAD_UP))
        cpu_clk += 500000;  // add 0.5MHz
    if((buttons & Button::B) && (buttons.pressed & Button::DPAD_DOWN) && cpu_clk > 500000)
        cpu_clk -= 500000;  // sub 0.5MHz

    // Run Benchmark
    if((buttons & Button::B) && (buttons.pressed & Button::DPAD_LEFT)) {
        // Run benchmark: take 0.1 seconds, and see how many clocks we can do.
        cpu.reset();
        uint32_t bench_start = now_us();
        uint32_t bench_end = bench_start + 100000;
        int i;
        while(now_us() < bench_end && !cpu.is_stuck()) {
            for(i=0; i<50 && !cpu.is_stuck(); i++) {
                cpu.step();
            }
            if (tms9918.interrupt_pending() && (cpu.tms9901_cru & 4) ) {
                // VDP interrupt
                // uint16_t cst = cpu.st;
                bool b = cpu.interrupt(1);
                if(b) {
                    // printf("CPU vectored to interrupt, 0x%04X.\n", cst);
                    vdp_ints++;
                }
            }        
        }
        bench_result = cpu.get_cycles();
        // reset again at the end of bench
        tms9918.init();
        cpu.reset();    
        last_update_tms9918_run_cycles = cpu.get_cycles();
        // last_update_time = time  + 1000;
    }

    if(buttons.pressed & Button::X) {
        // cpu.reset();
        if(cpu.is_stuck()) {
            printf("CPU Stuck\n");
            return;
        }
#ifdef VERIFY
        run_verify_step();
#else
        int loops = 5000;
        while(!cpu.is_stuck() && loops > 0) {
            char s[80];
            /* int t = */ cpu.dasm_instruction(s, cpu.get_pc());
            printf("%ld %04X %s\n", cpu.get_instructions(), cpu.get_pc(), s);
            cpu.step();
            loops--;
        }
#endif        
    }

    if(!cpu.is_stuck() && 1) {
        uint32_t cycles_to_run = (time - last_update_time)*(cpu_clk/1000);   // 3000 = 3.0MHz
        last_update_time = time;
        unsigned long start_cycles = cpu.get_cycles();
        int i=0;
        uint32_t start_cpu = now_us();
        while(cpu.get_cycles() - start_cycles < cycles_to_run && !cpu.is_stuck()) { 
            i++;
#ifdef VERIFY
            uint16_t a = tursi_cpu.GetPC();
            run_verify_step(false);
            if(cpu.is_stuck()) {
                char s[80];
                cpu.dasm_instruction(s, a);
                printf("STUCK %d %04X %s\n", tursi_cpu.GetCycleCount(), tursi_cpu.GetPC(), s);
            }

            // Check interrupt status every 16 instructions
            if(!(i & 0xF) && tms9918.interrupt_pending() && (cpu.tms9901_cru & 4) ) {
                // printf("Interrupt stuff begin %ld.\n", cpu.get_instructions());
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
                cpu.dasm_instruction(s, cpu.get_previous_pc());
                printf("%ld %04X %s\n", cpu.get_instructions(), cpu.get_pc(), s);
                if(debug_log) {
                    fprintf(debug_log, "%ld %04X %s\n", cpu.get_instructions(), cpu.get_pc(), s);
                    fflush(debug_log);
                }
            }

            cpu.step();
            if(!(i & 0xF) && !cpu.is_stuck()) { // Run this every 16 instructions
                // We run at 50 fps
                if(cpu.get_cycles() >= last_update_tms9918_run_cycles + cpu_clk/50) {
                    last_update_tms9918_run_cycles = cpu.get_cycles();
                    fill_full_screen = false;
                    uint32_t start = now_us();
                    for(int y=0; y<192; y++)
                        tms9918.scanline(ti_rendered, y);    
                    time_scanlines = us_diff(start, now_us());
                    // Add the time_scanlines to start_cpu to remove the effect of scanline(..) calls from CPU time.
                    start_cpu += time_scanlines;
                    drawn_frames++;
                }                
                if (tms9918.interrupt_pending() && (cpu.tms9901_cru & 4) ) {
                    // VDP interrupt
                    // uint16_t cst = cpu.st;
                    bool b = cpu.interrupt(1);
                    if(b) {
                        // printf("CPU vectored to interrupt, 0x%04X.\n", cst);
                        vdp_ints++;
                    }
                }
            }
            if(cpu.is_stuck()) {
                // oh no, we became stuck!
                char s[80];
                cpu.dasm_instruction(s, cpu.get_previous_pc());
                printf("%ld %04X %s\n", cpu.get_instructions(), cpu.get_pc(), s);
                printf("GROM_addr 0x%04X 0x%02X VRAM writes %d VDP writes %d VRAM addr 0x%04X reg writes %d\n", 
                    grom.get_read_addr(), 
                    grom.get_read_addr() < 0x6000 ? grom994a_data[grom.get_read_addr()-1] : 0xEE,
                    tms9918.vram_writes, tms9918.vdp_writes, tms9918.vram_addr, tms9918.reg_writes);
                if(debug_log) {
                    fprintf(debug_log, "CPU STUCK\n");
                    fprintf(debug_log, "%ld %04X %s\n", cpu.get_instructions(), cpu.get_pc(), s);
                    fprintf(debug_log, "GROM_addr 0x%04X 0x%02X VRAM writes %d VDP writes %d VRAM addr 0x%04X reg writes %d\n", 
                        grom.get_read_addr(), 
                        grom.get_read_addr() < 0x6000 ? grom994a_data[grom.get_read_addr()-1] : 0xEE,
                        tms9918.vram_writes, tms9918.vdp_writes, tms9918.vram_addr, tms9918.reg_writes);
                    fflush(debug_log);
                }
            }
#endif
        }
        time_cpu = now_us() - start_cpu;
    }

    if(buttons & Button::DPAD_LEFT) {
        cpu.keyboard[5] &= ~(1 << 4);   // '1' down
/*        
        if(!disasm) {
            disasm = true;
            cpu.get_instructions() = 0;
            debug_log = fopen("debug.txt", "wt");
        }
*/        
    } else {
        cpu.keyboard[5] |= (1 << 4);    // '1' up
    }

    if(buttons & Button::DPAD_RIGHT) {
        cpu.keyboard[1] &= ~(1 << 4);   // '2' down
    } else {
        cpu.keyboard[1] |= (1 << 4);   // '2' up
    }
    if(buttons.pressed & Button::DPAD_UP) {
        printf("TMS9901 CRU 0x%08X VDP pending %d VDP ints %d\n",
            cpu.tms9901_cru, tms9918.interrupt_pending(), vdp_ints
        );
        if(cpu.is_stuck()) {
                char s[80];
                cpu.dasm_instruction(s, cpu.get_previous_pc());
                printf("%ld %04X %s\n", cpu.get_instructions(), cpu.get_pc(), s);
                printf("GROM_addr 0x%04X 0x%02X VRAM writes %d VDP writes %d VRAM addr 0x%04X reg writes %d\n", 
                    grom.get_read_addr(), 
                    grom.get_read_addr() < 0x6000 ? grom994a_data[grom.get_read_addr()-1] : 0xEE,
                    tms9918.vram_writes, tms9918.vdp_writes, tms9918.vram_addr, tms9918.reg_writes);
        }
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
