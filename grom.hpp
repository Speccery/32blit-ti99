// grom.hpp
// GROM chip implementation
// Erik Piehl (C) 2021
#include <sys/types.h>
// #include <stdio.h>    // DEBUG printf

struct grom_t {
protected:  
  uint16_t offset;  // 13 bits
  uint16_t sel;     // 3 bits
  uint16_t base;    // 4 bits
  uint16_t read_addr;
public:
  grom_t() {
    offset = 0;
    base = 0;
    sel = 0;
    read_addr = 0;
  }
  virtual ~grom_t() {}
  uint8_t read(uint16_t addr) {
    uint8_t r;
    if(addr & 2) {
      // read address 
      r = read_addr >> 8;
      read_addr <<= 8;
    } else { 
      // read data
      uint16_t a = (sel << 13) | offset;
      r = read_mem( a );
      offset = 0x1FFF & (offset + 1);
      update_read_addr();
    }
    return r;
  }
  void write(unsigned addr, uint8_t d) {
    if(addr & 2) {
      // write to address counter
      uint16_t old_offset = offset;
      offset = 0x1FFF & ((offset << 8) | d);
      sel = (old_offset >> 5) & 7;
      base = 0xF & (addr >> 2);
      update_read_addr();
      // printf("GROM sel=%d offset=0x%04X\n", sel, offset);
    }
  }
  uint16_t get_read_addr() const {
    return read_addr;
  }
protected:
  void update_read_addr() {
    uint16_t next_offset = 0x1FFF & (offset + 1);
    read_addr = (sel << 13) | next_offset;
  }
  virtual uint8_t read_mem(uint16_t addr) = 0;

};
