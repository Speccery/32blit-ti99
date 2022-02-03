// ti99.cpp

#include "ti99.hpp"
#include <cstring>  // memcpy
#include "sys.hpp"


using namespace blit;

extern "C" {
    extern const unsigned char rom994a_data[];
    extern const unsigned char grom994a_data[];
    extern const unsigned char rominvaders_data[];
    extern const unsigned char grominvaders_data[];
}

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

// Our 256*192 framebuffer of 4 bits per pixels TMS9918 pixels.
#define TI_WIDTH 256
#define TI_HEIGHT 192

uint8_t ti_rendered[TI_WIDTH/2*TI_HEIGHT];
uint8_t full_screen_color = 0;
bool fill_full_screen = true;
bool debug_show_pixel = false;

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
uint32_t last_drawn_frames = 0;

///////////////////////////////////////////////////////////////////////////
//
// init()
//
// setup your game here
//
void init() {
    set_screen_mode(ScreenMode::hires);
    printf("About to enter cpu.reset()\n");
    cpu.reset();
    printf("Completed cpu.reset()\n");

    last_render_second = 0;
    last_render_cycles = 0;
    last_render_frames = 0;
    render_frames = 0;
    last_update_time = 0;
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
        uint32_t t = cpu.tms9918.render(ti_rendered, 15);
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

        tscanlines.new_value(cpu.get_scanlines_run_time());
        tcpu.new_value(time_cpu);

        screen.text("CPU: " + std::to_string(tcpu.average()), ti_font,       Point(1, 220));
        screen.text("Lines: " + std::to_string(tscanlines.average()), ti_font, Point(60,220));
        screen.text("Rend: " + std::to_string(trender.average()), ti_font,   Point(128, 220));
        if(time - last_render_second >= 1000) {
            kHz = (cpu.get_cycles() - last_render_cycles) / ((time - last_render_second));
            fps = 1000.0f*(render_frames - last_render_frames) / (time - last_render_second);
            vdp_draws = 1000.0f*(cpu.get_drawn_frames() - last_drawn_frames) / (time - last_render_second);
            last_render_frames = render_frames;
            last_drawn_frames = cpu.get_drawn_frames();
            last_render_second = time;
            last_render_cycles = cpu.get_cycles();
        }
        screen.text("kHz: " + std::to_string(kHz), ti_font, Point(188, 220));

        int func_index = 2*((time / 1000) % 16);
        char s[80];
        sprintf(s, "  e%02d: %8X e%02d: %8X", func_index+0, cpu.get_debug_read_funcs_entry(func_index+0),
                                        func_index+1, cpu.get_debug_read_funcs_entry(func_index+1));
        screen.text("debug: " + std::to_string(cpu.get_debug_read_funcs_size()) + 
            " offset:" + std::to_string(cpu.get_debug_read_funcs_offset()) + s,
            ti_font, Point(0, 239-8));

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
        cpu.set_cpu_clk(cpu.get_cpu_clk() + 500000);  // add 0.5MHz
    if((buttons & Button::B) && (buttons.pressed & Button::DPAD_DOWN) && cpu.get_cpu_clk() > 500000)
        cpu.set_cpu_clk(cpu.get_cpu_clk() - 500000);  // sub 0.5MHz

    // Run Benchmark
    if((buttons & Button::B) && (buttons.pressed & Button::DPAD_LEFT)) {
        // Run benchmark: take 0.1 seconds, and see how many clocks we can do.
        cpu.reset();
        uint32_t bench_start = now_us();
        uint32_t bench_end = bench_start + 100000;
        while(now_us() < bench_end && !cpu.is_stuck()) {
            cpu.run_cpu(300, nullptr, false);    // Run the CPU for 300 cycles, i.e. 100 microseconds of CPU time
        }
        bench_result = cpu.get_cycles();
        // reset again at the end of bench
        cpu.reset();    
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
        int delta = time - last_update_time;
        if(delta >= 11) {   // 11 or more since last call (this is called every 10ms)
            uint32_t cycles_to_run = delta*(cpu.get_cpu_clk()/1000);   // 3000 = 3.0MHz
            last_update_time = time;
            uint32_t start_cpu = now_us();
            fill_full_screen = false;
            uint32_t scantime = cpu.run_cpu(cycles_to_run, ti_rendered, disasm);
            time_cpu = now_us() - start_cpu - scantime;
            // printf("st %d cpu %d ", scantime, delta);
        }
    }

    if(buttons & Button::DPAD_LEFT) {
        cpu.keyboard[5] &= ~(1 << 4);   // '1' down
/*        
        if(!disasm) {
            disasm = true;
            cpu.get_instructions() = 0;
            debug_log = fopen("debug.txt", "wt");
            cpu.set_debug_log(debug_log);
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
            cpu.tms9901_cru, cpu.tms9918.interrupt_pending(), cpu.get_vdp_interrupts()
        );
        if(cpu.is_stuck()) {
                char s[80];
                cpu.dasm_instruction(s, cpu.get_previous_pc());
                printf("%ld %04X %s\n", cpu.get_instructions(), cpu.get_pc(), s);
                printf("GROM_addr 0x%04X 0x%02X VRAM writes %d VDP writes %d VRAM addr 0x%04X reg writes %d\n", 
                    cpu.grom.get_read_addr(), 
                    cpu.grom.get_read_addr() < 0x6000 ? grom994a_data[cpu.grom.get_read_addr()-1] : 0xEE,
                    cpu.tms9918.vram_writes, cpu.tms9918.vdp_writes, cpu.tms9918.vram_addr, cpu.tms9918.reg_writes);
        }
    }
}

