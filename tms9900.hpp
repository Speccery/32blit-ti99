// tms9900.hpp
// 
// Erik Piehl (C) 2021-01-28
// 
#include <sys/types.h>

const uint16_t ST15 = 1 << 15;
const uint16_t ST14 = 1 << 14;
const uint16_t ST13 = 1 << 13;
const uint16_t ST12 = 1 << 12;
const uint16_t ST11 = 1 << 11;
const uint16_t ST10 = 1 << 10;


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
  private:
    static unsigned long cycles;
    static unsigned long inst_count;
    static unsigned long wait_cycles;
  protected:
    typedef uint16_t read_type;
    uint16_t wp;  //!< workspace pointer
    uint16_t st;  //!< status register
    uint16_t pc;  //!< program counter
    uint16_t ir;  //!< Current instruction.
    uint16_t prev_pc; //!< pc at the start of this instruction cycle.
    static bool stuck;
  public:
    tms9900_t() {
      cycles = 0;
      inst_count = 0;
      wait_cycles = 0;
      stuck = false;
      for(int i=0; i<64; i++)
        read_funcs[i] = nullptr;
    }
    virtual ~tms9900_t() {}
    virtual void reset();
    bool step();      //!< step one instruction. returns true if succesful, false if became stuck.
    bool execute();   //!< execute an instruction in IR after fetch.
    int dasm_instruction(char *dst, uint16_t addr);
    bool interrupt(uint8_t level);
    unsigned long get_cycles() const;
    unsigned long get_instructions() const;
    static void add_ext_cycles(unsigned u) {   //!< Public function to add cycles. External interface enables computation of wait states.
      cycles += u;
      wait_cycles += u;
    }
    uint16_t get_pc() const { return pc; }
    uint16_t get_previous_pc() const { return prev_pc; }
    bool is_stuck() const   { return stuck; }
  protected:
    static void add_cycles(int n) {
      cycles += n;
    }
    void do_exec0();
    void do_jump(bool condition, uint16_t offset);
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
    inline void flags_012_others(uint16_t t) {
      // st[15] <= alu_logical_gt;
      // st[14] <= alu_arithmetic_gt;
      // st[13] <= alu_flag_zero;
      st &= ~(ST15 | ST14 | ST13);
      st |= t ? ST15 : 0;
      st |= !(t & 0x8000) && t ? ST14 : 0;
      st |= t == 0 ? ST13 : 0;
    }
    inline read_type read_operand_word(uint16_t op) {
      uint16_t sa = source_address_word(op);
      return read(sa);
    }
    void do_parity(uint16_t src);
    void write_byte(uint16_t dst_addr, uint16_t dst);
    uint16_t read_byte(uint16_t addr);
    void set_carry_add(unsigned dst) {
      // ST12 is carry
      st &= ~ST12;
      st |= (dst & 0x10000) ? ST12 : 0;
    }
    void set_carry_sub(unsigned dst) {
      // for SUB carry is inverted
      st &= ~ST12;
      st |= !(dst & 0x10000) ? ST12 : 0;
    }
    void set_overflow_add(unsigned d, unsigned src, unsigned src2) {
      st &= ~ST11;
      st |= (src & 0x8000) == (src2 & 0x8000) && (d & 0x8000) != (src & 0x8000) ? ST11 : 0;
    }
    void set_overflow_sub(unsigned d, unsigned src, unsigned src2) {
      const uint16_t k8 = 0x8000;
      st &= ~ST11;
      st |= (src & k8) != (src2 & k8) && (d & k8) != (src & k8) ? ST11 : 0;
    }
    void set_flags_compare(const uint16_t dst, const uint16_t src) {
      unsigned c = dst-src;
      const uint16_t k8 = 0x8000;
      st &= ~(ST15 | ST14 | ST13);
      st |= ((src & k8) && !(dst & k8)) || ((src & k8) == (dst & k8) && (c & k8)) ? ST15 : 0;
      st |= (!(src & k8) && (dst & k8)) || ((src & k8) == (dst & k8) && (c & k8)) ? ST14 : 0;
      st |= c == 0 ? ST13 : 0;
    }

    // Read uses a lookup table.
    // virtual uint16_t  read(uint16_t addr) = 0;
    static read_type (*read_funcs[64])(uint16_t addr);
    read_type read(unsigned addr);
    virtual void      write(uint16_t addr, uint16_t data) = 0;
    virtual uint8_t   read_cru_bit(uint16_t addr) = 0;
    virtual void      write_cru_bit(uint16_t addr, uint8_t data) = 0;

    void do_blwp(uint16_t addr);
    inline void write_reg(unsigned r, uint16_t val) {
      write(wp+(r << 1), val);
    }
    inline uint16_t read_reg(unsigned r) {
      return read(wp+(r << 1));
    }
    char *dasm_addr_mode(char *s, unsigned mode, int *len);
    int dasm_one(char *buf, int state_in, int opcode);
    uint16_t read_operand(uint16_t op, bool word_operation);
    uint16_t source_address(uint16_t op, bool word_operation);
    uint16_t source_address_word(uint16_t op);  
};
