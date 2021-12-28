// tms9900.hpp
// 
// Erik Piehl (C) 2021-01-28
// 
#include <sys/types.h>

enum ins9900 { dual_op_mult_mult, dual_op_mult_wr, xop, single_op,
    cru_multi_bit, cru_single_bit, jumps, shifts, immediates,
    internal_immediates, internal_store, rtwp };

struct instrucion_t {
    unsigned bin;
    unsigned mask;
    enum ins9900 type;
    char str[8];
};

class tms9900_t {
  public:
    uint16_t wp;  //!< workspace pointer
    uint16_t st;  //!< status register
    uint16_t pc;  //!< program counter
    uint16_t ir;  //!< Current instruction.
    unsigned long cycles;
    unsigned long inst_count;
    bool stuck;
    tms9900_t() {
      cycles = 0;
      inst_count = 0;
      stuck = false;
    }
    virtual ~tms9900_t() {}
    void reset();
    bool step();  //!< returns true if succesful, false if became stuck.
    int dasm_instruction(char *dst, uint16_t addr);
  protected:
    
    void do_exec0();
    void do_exec1();
    void do_exec2();
    void do_exec3();
    void do_exec4();
    void do_exec5();
    void do_exec6();
    void do_exec7();
    void do_exec8();
    void do_exec9();
    void do_execA();
    void do_execB();
    void do_execC();
    void do_execD();
    void do_execE();
    void do_execF();
    void (tms9900_t::*func[16])();
    uint16_t next() {  //!< read next word from instruction stream
      uint16_t d = read(pc);
      pc += 2;
      return d;
    }
    void flags_012_others(uint16_t t);

    virtual uint16_t read(uint16_t addr) = 0;
    virtual void write(uint16_t addr, uint16_t data) = 0;

    void do_blwp(uint16_t addr);
    inline void write_reg(unsigned r, uint16_t val) {
      write(wp+(r << 1), val);
    }
    inline uint16_t read_reg(unsigned r) {
      return read(wp+(r << 1));
    }
  char *dasm_addr_mode(char *s, unsigned mode, int *len);
  int dasm_one(char *buf, int state_in, int opcode);
};
