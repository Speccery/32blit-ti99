// sys.cpp
// TI-99/4A system module. The idea is that this one is independent of the target platform.
// EP 2022-02-02

#include "ti99.hpp"
#include "sys.hpp"

// #define VERIFY comes now from cmake

#ifdef VERIFY
#include "cpu9900.h"
class tursi_cpu_t : public CPU9900 {
public:
    virtual Word GetSafeWord(int x, int bank);
};
tursi_cpu_t tursi_cpu;
#endif

extern "C" {
    extern const unsigned char rom994a_data[];
    extern const unsigned char grom994a_data[];
    extern const unsigned char rominvaders_data[];
    extern const unsigned char grominvaders_data[];
}

void tigrom_t::write(unsigned addr, uint8_t d) {
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



uint8_t tigrom_t::read_mem(uint16_t addr) {
    addr_complete = 0;
    if(addr < 0x6000)
        return grom994a_data[addr];
    if(addr < 0x8000)
        return grominvaders_data[addr - 0x6000];
    printf("Reading outside valid GROM area 0x%04X.\n", addr);
    return 0;
}

cpu_t::cpu_t() {
    dsr_mem_counter = 0;
    cart_counter = 0;
    memset(keyboard, 0xFF, sizeof(keyboard));
    keyscan = 0;
#ifdef VERIFY    
    verify = false;
#endif
    tms9901_cru = ~0;
    debug_log = nullptr;

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

void cpu_t::reset() {
  tms9918.init();
  tms9900_t::reset();
}

#ifdef VERIFY
uint16_t cpu_t::verify_read(uint16_t addr) {
    return read(addr);
}
void cpu_t::verify_write(uint16_t addr, uint16_t data) {
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
void cpu_t::verify_write_cru_bit(uint16_t addr, uint8_t data) {
    write_cru_bit(addr, data);
}
uint8_t cpu_t::verify_read_cru_bit(uint16_t addr) {
    return read_cru_bit(addr);
}
void cpu_t::verify_begin() {
    verify = true;
    verify_reads = false;
    verify_wr_count = 0;
    verify_wr_mask = 0;
    verify_rd_count = 0;
    verify_rd_mask = 0;
}
void cpu_t::verify_switch_on_reads() {
    verify_reads = true;
}
void cpu_t::verify_show_rd_buffer() {
    printf("Instruction 0x%04X read verify error, count %d mask %d.\n", 
        prev_pc,
        verify_rd_count, verify_rd_mask);
    for(int i=0; i<verify_rd_count; i++) {
        printf("read buffer[%d]: [%04X]=%04X\n", i, verify_rd_addrs[i], verify_rd_datas[i]);
    }
}
void cpu_t::verify_show_wr_buffer() {
    printf("Instruction 0x%04X write verify error, count %d mask %d.\n", 
        prev_pc,
        verify_wr_count, verify_wr_mask);
    for(int i=0; i<verify_wr_count; i++) {
        printf("write buffer[%d]: [%04X]=%04X\n", i, verify_wr_addrs[i], verify_wr_datas[i]);
    }
}
void cpu_t::verify_end() {
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

void cpu_t::verify_add_read_to_buffer(uint16_t addr, uint16_t data) {
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

void cpu_t::check_write_in_verify_buffer(uint16_t addr, uint16_t data) {
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

uint16_t cpu_t::check_read_in_verify_buffer(uint16_t addr) {
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
#endif

void cpu_t::show_cpu() {
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

uint16_t cpu_t::read_rom(uint16_t addr) {
    return (rom994a_data[addr] << 8) | rom994a_data[addr+1];
}

uint16_t cpu_t::read_dsr(uint16_t addr) {
    // No DSR memory for now. 0x4000
    dsr_mem_counter++;
    add_ext_cycles(4);
    return 0;
}
uint16_t cpu_t::read_cartridge(uint16_t addr) {
    // 0x6000..0x7FFF
    cart_counter++;
    add_ext_cycles(4);
    return (rominvaders_data[addr - 0x6000] << 8) 
            |  rominvaders_data[addr - 0x6000+1];
}
uint16_t cpu_t::read_scrachpad(uint16_t addr) {
    return (scratchpad[addr & 0xFE] << 8) | scratchpad[(addr & 0xFE) + 1];
}
uint16_t cpu_t::read_soundchip(uint16_t addr) {
    // 8400
    add_ext_cycles(4);
    return 0;
}
uint16_t cpu_t::read_vdp(uint16_t addr) {
    // VDP read. 0x8800
    uint16_t r = tms9918.read(!!(addr & 2));
    add_ext_cycles(4);
    return r << 8; 
}
uint16_t cpu_t::read_vdp_write_port(uint16_t addr) {
    // VDP write port read 0x8C00
    add_ext_cycles(4);
    return 0;
}
uint16_t cpu_t::read_speech(uint16_t addr) {
    // speech synthesizer 0x9000
    add_ext_cycles(4);
    return 0xAC00;
}
uint16_t cpu_t::read_grom(uint16_t addr) {
    // 0x9800
    add_cycles(4);
    return grom.read(addr) << 8;
}
uint16_t cpu_t::read_grom_write_port(uint16_t addr) {
    // VDP write port read 0x9C00
    add_ext_cycles(4);
    return 0;
}
uint16_t cpu_t::read_unknown(uint16_t addr) {
    stuck = true;
    printf("reading outside of memory: 0x%4X\n", addr);
    if(debug_log) 
      fprintf(debug_log, "reading outside of memory: 0x%4X\n", addr);
    return 0xDEAD;
}

uint16_t cpu_t::read_all_cases(uint16_t addr) {
    addr &= ~1; // make the address even
#ifdef VERIFY    
    if(verify_reads) {
        return check_read_in_verify_buffer(addr);
    }
#endif
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
#ifdef VERIFY    
    if(verify) {
        // Store this read to verify read buffer.
        verify_add_read_to_buffer(addr, read_value);
    }
#endif
    return read_value;
}

void cpu_t::write(uint16_t addr, uint16_t data) {
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

void cpu_t::write_cru_bit(uint16_t addr, uint8_t data) {
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

uint8_t cpu_t::read_cru_bit(uint16_t addr) {
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
