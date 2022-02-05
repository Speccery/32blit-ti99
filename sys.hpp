// sys.hpp
// TI-99/4A system module. The idea is that this one is independent of the target platform.
// EP 2022-02-02

#include "tms9900.hpp"
#include "grom.hpp"
#include "tms9918.hpp"


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
    virtual void write(unsigned addr, uint8_t d);
    bool should_cpu_stat_be_shown() {
        bool t = show_cpu;
        show_cpu = false;
        return t;
    }
protected:
    uint8_t read_mem(uint16_t addr);
};

class cpu_t : public tms9900_t {
public:
    uint8_t keyboard[8];
    uint8_t keyscan;
    unsigned tms9901_cru;

    tms9918_t tms9918;
    tigrom_t grom;
  protected:
    static uint32_t vdp_interrupts;
    static uint32_t cpu_clk;                           //!< CPU clock frequency in Hertz
    static uint32_t last_update_tms9918_run_cycles;    //!< Last CPU cycles we did run the VDP
    static uint32_t scanlines_run_time;                //!< us taken by last scanlines run
    static uint32_t drawn_frames;                      //!< Number of frames handled by scanlines routine
    static uint16_t scratchpad[128];    
    static unsigned dsr_mem_counter;       //!< count DSR region accesses
    static unsigned cart_counter;

public:    


#ifdef VERIFY
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
public:  
    uint16_t verify_read(uint16_t addr);
    void verify_write(uint16_t addr, uint16_t data);
    void verify_write_cru_bit(uint16_t addr, uint8_t data);
    uint8_t verify_read_cru_bit(uint16_t addr);
    void verify_begin();
    void verify_switch_on_reads();
    void verify_show_rd_buffer();
    void verify_show_wr_buffer();
    void verify_end();
    void verify_add_read_to_buffer(uint16_t addr, uint16_t data);
#endif

    cpu_t();
    virtual void reset();
    static void set_debug_log(FILE *f) {
      debug_log = f;
    }
    void set_cpu_clk(uint32_t s) {
      cpu_clk = s;
    }
    uint32_t get_cpu_clk() const {
      return cpu_clk;
    }
    uint32_t get_vdp_interrupts() const {
      return vdp_interrupts;
    }
    uint32_t get_last_tms9918_run() const {
      return last_update_tms9918_run_cycles;
    }
    uint32_t run_cpu(uint32_t cycles_to_run, uint8_t *render_buffer, bool disasm);  // returns number of microseconds spent on scanline conversions
    uint32_t now_us() const {
      return blit::now_us();
    }
    uint32_t us_diff(uint32_t start, uint32_t end) const {
      return blit::us_diff(start, end);
    }
    uint32_t get_scanlines_run_time() const {
      return scanlines_run_time;
    }
    uint32_t get_drawn_frames() const {
      return drawn_frames;  
    }

    unsigned get_debug_read_funcs_size() const {
      return sizeof(read_funcs);
    }
    unsigned get_debug_read_funcs_offset() const {
      return  (uint8_t *)read_funcs - (uint8_t *)this;
    }
    unsigned get_debug_read_funcs_entry(int e) const {
      unsigned *p = (unsigned *)read_funcs;
      return p[e];
    }

public:

protected:
  static FILE *debug_log;
  static cpu_t *sys;
#ifdef VERIFY 
    void check_write_in_verify_buffer(uint16_t addr, uint16_t data);
    uint16_t check_read_in_verify_buffer(uint16_t addr);
#endif    
  void show_cpu();
  static tms9900_t::read_type read_rom(uint16_t addr);
  static tms9900_t::read_type read_dsr(uint16_t addr);
  static tms9900_t::read_type read_cartridge(uint16_t addr);
  static tms9900_t::read_type read_scratchpad(uint16_t addr);
  static tms9900_t::read_type read_soundchip(uint16_t addr);
  static tms9900_t::read_type read_vdp(uint16_t addr);
  static tms9900_t::read_type read_vdp_write_port(uint16_t addr);
  static tms9900_t::read_type read_speech(uint16_t addr);
  static tms9900_t::read_type read_grom(uint16_t addr);
  static tms9900_t::read_type read_grom_write_port(uint16_t addr);
  static tms9900_t::read_type read_unknown(uint16_t addr);
  virtual tms9900_t::read_type read_all_cases(uint16_t addr);
  virtual void write_cru_bit(uint16_t addr, uint8_t data);
  virtual uint8_t read_cru_bit(uint16_t addr);
  virtual void write(uint16_t addr, uint16_t data);
};
