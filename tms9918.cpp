// tms9918.cpp

#include "ti99.hpp"

#include "tms9918.hpp"

// TMS9918 colors from my tms9918.v verilog code.
// Map the 16-colors into 8-bit RGB.
const uint8_t tms9918_t::palette_lookup[16] = {
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

uint32_t tms9918_t::render(uint8_t *fp, int yoffset) {
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
    if(screen.format == screen_t::RGB565) {
        for(int y=0; y<192; y++) {
            uint8_t *p = fp + (y << 7) + x_src_offset;
            uint16_t *d = (uint16_t *)(screen.data + (yoffset+y)*screen.row_stride+x_dest_offset*2);
            for(int x=0; x<128-x_src_offset*2; x++) {
                // Two pixels per loop here.
                d[0] = palette_rgb565[*p >> 4];
                d[1] = palette_rgb565[*p & 0xF];
                d += 2;
                p++;
            }
        }
    } else if(screen.format == screen_t::RGB) {
        // 24 bit RGB
        for(int y=0; y<192; y++) {
            uint8_t *p = fp + (y << 7) + x_src_offset;
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


void tms9918_t::scanline(uint8_t *fp, int y) {
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
            // Force pattern for multicolor mode.
            pattern = 0xF0;
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
        uint8_t *p = fp + (y << 7) + (x*cell_width >> 1);            
#ifdef DEBUG_PRINT            
        if((unsigned long)(p - ti_rendered) > sizeof(ti_rendered)) {
            printf("overflow %ld y=%d\n", p-ti_rendered, y);
        }
#endif          
        if(cell_width == 8) {
            p[0] = ((pattern & 0x80 ? color1 : color0) << 4) | (pattern & 0x40 ? color1 : color0);
            p[1] = ((pattern & 0x20 ? color1 : color0) << 4) | (pattern & 0x10 ? color1 : color0);
            p[2] = ((pattern & 0x08 ? color1 : color0) << 4) | (pattern & 0x04 ? color1 : color0);
            p[3] = ((pattern & 0x02 ? color1 : color0) << 4) | (pattern & 0x01 ? color1 : color0);
            p += 4;
        } else {
            // cell width is 6, text mode.
            p[0] = ((pattern & 0x80 ? color1 : color0) << 4) | (pattern & 0x40 ? color1 : color0);
            p[1] = ((pattern & 0x20 ? color1 : color0) << 4) | (pattern & 0x10 ? color1 : color0);
            p[2] = ((pattern & 0x08 ? color1 : color0) << 4) | (pattern & 0x04 ? color1 : color0);
            p += 3;
        }
        name_table_addr++;
    }
    sprites(y, fp + (y << 7));
    if(y == 191) {
        // Generate interrupt at the end of the last scanline.
        status |= 0x80;
    }
}