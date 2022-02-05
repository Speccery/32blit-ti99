// tms9918.hpp
// implementation of the tms9918

struct tms9918_t {
    uint8_t regs[8];
    uint8_t *framebuf;
    unsigned name_table_addr;
    uint8_t     hold_reg;
    uint16_t    vram_addr;
    bool        write_state;
    unsigned    vdp_writes;     //!< debug counter
    unsigned    vram_writes;    //!< debug counter
    unsigned    reg_writes;     //!< debug counter
    uint8_t     status;

    // 32-blit compatible variables.
    struct screen_t {
        enum pixelformat_t { RGB565, RGB } format;  // RGB = RGB888
        uint8_t *data;
        struct bounds_t {
            int w;
        } bounds;
        unsigned row_stride;
    } screen;

    static const uint8_t palette_lookup[16];
    uint16_t    palette_rgb565[16];
    uint8_t     palette_rgb888[16*4];

    tms9918_t() {
        memset(regs,0,sizeof(regs));
        for(int i=0; i<16; i++) {
            palette_rgb565[i] = unpack_rgb565(i);
            unpack_rgb888(palette_rgb888 + i*4, i);
        }
        screen.format = screen_t::RGB565;
        screen.bounds.w = 320;
        screen.row_stride = 640;
        screen.data = nullptr;
        framebuf = nullptr;
    }

    void set_framebuffer(uint8_t *p) {
        framebuf = p;
    }
    uint8_t *get_framebuffer() const {
        return framebuf;
    }

    void init() {
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

    void sprites(int y, uint8_t *destline) {
        // Check number of active sprites
        // printf("sprites %d entry\n", y);
        uint16_t vram_base = (regs[5] & 0x7F) << 7; // start address of sprite attributes
        int i = 0;
        bool sig_5th_pending = false;

        uint8_t     sprites_to_draw[32];
        uint16_t    yys[32];
        unsigned active_sprites = 0;

        for(i=0; i<32; i++) {
            unsigned yy = framebuf[vram_base + (i << 2)];
            if(yy == 0xD0)
                break;
            // sprite active on this line.
            yy = (yy & 0xE0) == 0xE0 ? yy & 0x1F : 0x100 | yy;
            unsigned sprite_line_check = (y | 0x100) - yy - 1;
            if( ((regs[1] & 2) && ((sprite_line_check & ~0xF) == 0)) ||       // 16x16 sprite check
                (!(regs[1] & 2) && ((sprite_line_check & ~0x1F) == 0))) {     // 8x8 sprite check
                    sprites_to_draw[active_sprites] = i;
                    yys[active_sprites] = sprite_line_check;
                    ++ active_sprites;
            }
            if(active_sprites == 5) {
                sig_5th_pending = true;
                break;  // only draw max 5 sprites (already 1 too many)
            }
        }
        // printf("active_sprites %d\n", active_sprites);
        // Draw sprites in the array sprites_to_draw.
        for(int i=active_sprites-1; i>=0; i--) {
            unsigned n = sprites_to_draw[i];
            //  if(n == 0)
            //      printf("Drawing sprite %d vram_base=0x%04X y=%d\n", n, vram_base+(n << 2), framebuf[vram_base + (n << 2) + 0]);
            unsigned ux = framebuf[vram_base + (n << 2) + 1];
            uint8_t name = framebuf[vram_base + (n << 2) + 2];
            uint8_t color = framebuf[vram_base + (n << 2) + 3];
            uint16_t vrama = (regs[6] & 7) << 11;
            if(regs[1] & 2) 
                vrama |= ((name & 0xFC) << 3) | (yys[i] & 0xF);    // 16x16
            else
                vrama |= (name << 3) | (yys[i] & 0x7);    // 8x8
            uint16_t pixels = framebuf[vrama] << 8;
            pixels |= framebuf[vrama | 0x10];               // Read 8 more pixels for 16x16 sprites
            unsigned count = regs[1] & 2 ? 16 : 8;
            unsigned early_clocks = 0;
            if(color & 0x80) {
                // early bit set.
                if(ux >= 32) {
                    ux -= 32;       // Already past 32, just draw
                } else {
                    early_clocks = 32 - (ux & 0x1F);
                    ux = 0;         // sprite bleeds in from the left.
                }
            } else {
                // Normal. Early bit not set.
                early_clocks = 0;
            }
            // if(color & 0x80)
            //    printf("Ready to draw ux=%d early_clocks=%d pixels=0x%04X\n",ux, early_clocks, pixels);
            // Now ux is the X coordinate where the sprite goes to.
            // Draw the sprite.
            color &= 0xF;
            uint8_t *d = destline + (ux >> 1);
            while(count > 0) {
                if(early_clocks) {
                    early_clocks--;
                } else {
                    // Push out pixels.
                    if(pixels & 0x8000) {
                        // Only draw the pixel if the sprite has a color, i.e.
                        // is not transparent. We still need to work on coincidence.
                        if(color) {
                            if(ux & 1)
                                *d = (*d & 0xF0) | color;
                            else    
                                *d = (*d & 0xF) | (color << 4);
                        }
                    }
                    if(ux & 1)
                        d++;
                    ux++;
                }
                pixels <<= 1;
                count--;
            }
        }
        // printf("sprites %d exit\n", y);
    }

    void scanline(uint8_t *fp, int y);
    uint32_t render(uint8_t *fp, int yoffset);

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
