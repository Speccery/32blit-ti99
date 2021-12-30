#include "ti99.hpp"
#include <cstring>  // memcpy
#include "tms9900.hpp"
#include "grom.hpp"

extern "C" unsigned char vdptest_data[];

using namespace blit;

// Our 256*192 framebuffer of 4 bits per pixels TMS9918 pixels.
#define TI_WIDTH 256
#define TI_HEIGHT 192
uint8_t ti_rendered[TI_WIDTH/2*TI_HEIGHT];
uint8_t full_screen_color = 0;
bool fill_full_screen = true;
bool debug_show_pixel = false;

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
    tms9918_t() {
        memset(regs,0,sizeof(regs));
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
    }
    void render(int yoffset) {
        // Now render from ti_rendered to our framebuffer.
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
                    uint32_t k = palette_lookup[*p >> 4];
                    d[0] = (k & 0xE0);  // R
                    d[1] = ((k & 0x1C) << 3); // G
                    d[2] = ((k & 3) << 6);    // B
                    k = palette_lookup[*p & 0xF];
                    d[3] = (k & 0xE0);  
                    d[4] = ((k & 0x1C) << 3);
                    d[5] = ((k & 3) << 6);
                    d += 6;
                    p++;
                }
            }

        }    
    }
};

tms9918_t tms9918;

extern "C" {
    extern const unsigned char rom994a_data[];
    extern const unsigned char grom994a_data[];
}

class tigrom_t : public grom_t {
protected:
    uint8_t read_mem(uint16_t addr) {
        return grom994a_data[addr];
    }
};

tigrom_t grom;

uint8_t scratchpad[256];

class cpu_t : public tms9900_t {
protected:
     virtual uint16_t read(uint16_t addr) {
         addr &= ~1; // make the address even
         if(addr < 0x2000) {
             return (rom994a_data[addr] << 8) | rom994a_data[addr+1];
         } else if(addr >= 0x8300 && addr <0x8400) {
             return (scratchpad[addr - 0x8300] << 8) | scratchpad[addr - 0x8300 + 1];
         } else if(addr >= 0x8400 && addr<0x8800) {
             // sound chip access
         } else if(addr >= 0x9000 && addr < 0x9800) {
             // speech synthesizer
             return 0xAC00;
         } else if(addr >= 0x9800 && addr < 0xA000) {
             // GROM reads. 9800..9BFF is the actual read area.
             // 9C00..9FFF is write port, but read due to read-modify-write architecture.
             if(addr >= 9800 && addr < 0x9C00)
                return grom.read(addr) << 8;
            else
                return 0xAB00;  // dummy
         } else {
             stuck = true;
             printf("reading outside of memory: 0x%4X\n", addr);
         }
         return 0xdead;
    }
    virtual void write(uint16_t addr, uint16_t data) {
        addr &= ~1; // make the addres even
         if(addr >= 0x8300 && addr < 0x8400) {
             scratchpad[addr - 0x8300] = data >> 8;
             scratchpad[addr - 0x8300 + 1] = data;  // 8 low bits
         } else if(addr >= 0x8400 && addr < 0x8800) {
             // sound chip access
         } else if (addr >= 0x9c00 && addr < 0xA000) {
             grom.write(addr, data >> 8);
         } else if(addr >= 0x9400 && addr < 0x9800) {
             // Speech write, do nothing.
         } else {
             stuck = true;
             printf("writing outside of memory: 0x%4X\n", addr);
         }
         printf("Writing to 0x%04X value 0x%04X, GROM addr 0x%04X\n", addr, data, grom.get_read_addr());
    }
    virtual void write_cru_bit(uint16_t addr, uint8_t data) {
        printf("CRU write 0x%04X data %d\n", addr, data);
    }
    virtual uint8_t read_cru_bit(uint16_t addr) {
        printf("CRU read 0x%04X\n", addr);
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
}

///////////////////////////////////////////////////////////////////////////
//
// render(time)
//
// This function is called to perform rendering of the game. time is the 
// amount if milliseconds elapsed since the start of your game
//
void render(uint32_t time) {

    // clear the screen -- screen is a reference to the frame buffer and can be used to draw all things with the 32blit
    screen.clear();

    // draw some text at the top of the screen
    screen.alpha = 255;
    screen.mask = nullptr;
    screen.pen = Pen(255, 255, 255);
    screen.rectangle(Rect(0, 0, 320, 14));
    screen.pen = Pen(0, 0, 0);
    screen.text("TI-99/4A " + std::to_string(cpu.pc) + " " + std::to_string(cpu.inst_count) + " " + (cpu.stuck ? "STUCK" : ""),
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
        tms9918.render(15);
        if(debug_show_pixel) {
            debug_show_pixel = false;
            show_pixel_RGB(screen.data + screen.row_stride*40);
            show_pixel_RGB(screen.data + screen.row_stride*40+3);
        }

    }

}

///////////////////////////////////////////////////////////////////////////
//
// update(time)
//
// This is called to update your game state. time is the 
// amount if milliseconds elapsed since the start of your game
//
void update(uint32_t time) {

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
        cpu.reset();
        printf("CPU reset\n");
        while(!cpu.stuck) {
            char s[80];
            int t = cpu.dasm_instruction(s, cpu.pc);
            printf("t=%d %04X %s\n", t, cpu.pc, s);
            cpu.step();
        }
    }
}