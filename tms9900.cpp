// tms9900.cpp
// Implementation of the TMS9900 CPU.
// Erik Piehl (C) 2021

#include <stdio.h>  // sprintf
#include <string.h> // strcat
#include "tms9900.hpp"

#define INTLEN 32


const instrucion_t instructions[] = {
    // Dual operand instructions.
    // Dual operand with multiple addressing modes for source and destination
    { 0xA000, 0xF000, dual_op_mult_mult, "A" },
    { 0xB000, 0xF000, dual_op_mult_mult, "AB" },
    { 0x8000, 0xF000, dual_op_mult_mult, "C" },
    { 0x9000, 0xF000, dual_op_mult_mult, "CB" },
    { 0x6000, 0xF000, dual_op_mult_mult, "S" },
    { 0x7000, 0xF000, dual_op_mult_mult, "SB" },
    { 0xE000, 0xF000, dual_op_mult_mult, "SOC" },
    { 0xF000, 0xF000, dual_op_mult_mult, "SOCB" },
    { 0x4000, 0xF000, dual_op_mult_mult, "SZC" },
    { 0x5000, 0xF000, dual_op_mult_mult, "SZCB" },
    { 0xC000, 0xF000, dual_op_mult_mult, "MOV" },
    { 0xD000, 0xF000, dual_op_mult_mult, "MOVB" },
    // Dual operand with multiple addressing modes for source and workspace reg for dest
    { 0x2000, 0xFC00, dual_op_mult_wr, "COC" },
    { 0x2400, 0xFC00, dual_op_mult_wr, "CZC" },
    { 0x2800, 0xFC00, dual_op_mult_wr, "XOR" },
    { 0x3800, 0xFC00, dual_op_mult_wr, "MPY" },
    { 0x3C00, 0xFC00, dual_op_mult_wr, "DIV" },
    // XOP
    { 0x2C00, 0xFC00, xop, "XOP" },
    // Single operand instructions
    { 0x0440, 0xFFC0, single_op, "B" },
    { 0x0680, 0xFFC0, single_op, "BL" },
    { 0x040C, 0xFFC0, single_op, "BLWP" },
    { 0x04C0, 0xFFC0, single_op, "CLR" },
    { 0x0700, 0xFFC0, single_op, "SETO" },
    { 0x0540, 0xFFC0, single_op, "INV" },
    { 0x0500, 0xFFC0, single_op, "NEG" },
    { 0x0740, 0xFFC0, single_op, "ABS" },
    { 0x06C0, 0xFFC0, single_op, "SWPB" },
    { 0x0580, 0xFFC0, single_op, "INC" },
    { 0x05C0, 0xFFC0, single_op, "INCT" },
    { 0x0600, 0xFFC0, single_op, "DEC" },
    { 0x0640, 0xFFC0, single_op, "DECT" },
    { 0x0480, 0xFFC0, single_op, "X" },
    // CRU multibit
    { 0x3000, 0xFC00, cru_multi_bit, "LDCR" },
    { 0x3400, 0xFC00, cru_multi_bit, "STCR" },
    // CRU single bit
    { 0x1D00, 0xFF00, cru_single_bit, "SBO"},
    { 0x1E00, 0xFF00, cru_single_bit, "SBZ"},
    { 0x1F00, 0xFF00, cru_single_bit, "TB"},
    // jump instructions
    { 0x1300, 0xFF00, jumps, "JEQ" }, 
    { 0x1500, 0xFF00, jumps, "JGT" }, 
    { 0x1B00, 0xFF00, jumps, "JH" }, 
    { 0x1400, 0xFF00, jumps, "JHE" }, 
    { 0x1A00, 0xFF00, jumps, "JL" }, 
    { 0x1200, 0xFF00, jumps, "JLE" }, 
    { 0x1100, 0xFF00, jumps, "JLT" }, 
    { 0x1000, 0xFF00, jumps, "JMP" }, 
    { 0x1700, 0xFF00, jumps, "JNC" }, 
    { 0x1600, 0xFF00, jumps, "JNE" }, 
    { 0x1900, 0xFF00, jumps, "JNO" }, 
    { 0x1800, 0xFF00, jumps, "JOC" }, 
    { 0x1C00, 0xFF00, jumps, "JOP" }, 
    // shifts
    { 0x0A00, 0xFF00, shifts, "SLA" }, 
    { 0x0800, 0xFF00, shifts, "SRA" }, 
    { 0x0B00, 0xFF00, shifts, "SRC" }, 
    { 0x0900, 0xFF00, shifts, "SRL" }, 
    // immediate instructions, don't care N
    { 0x0220, 0xFFE0, immediates, "AI" },
    { 0x0240, 0xFFE0, immediates, "ANDI" },
    { 0x0280, 0xFFE0, immediates, "CI" },
    { 0x0200, 0xFFE0, immediates, "LI" },
    { 0x0260, 0xFFE0, immediates, "ORI" },
    // internal register load immediate
    { 0x02E0, 0xFFE0, internal_immediates, "LWPI"},
    { 0x0300, 0xFFE0, internal_immediates, "LIMI"},
    // internal register store
    { 0x02C0, 0xFFE0, internal_store, "STST"},
    { 0x02A0, 0xFFE0, internal_store, "STWP"},
    // RTWP and external instructions
    { 0x0380, 0xFFE0, rtwp, "RTWP" },
    { 0x0340, 0xFFE0, rtwp, "IDLE" },
    { 0x0360, 0xFFE0, rtwp, "RSET" },
    { 0x03C0, 0xFFE0, rtwp, "CKOF" },
    { 0x03A0, 0xFFE0, rtwp, "CKON" },
    { 0x03E0, 0xFFE0, rtwp, "LREX" },
    // end

    { 0,0, dual_op_mult_mult, "" }
};


void tms9900_t::reset() {
  func[0] = &tms9900_t::do_exec0;
  func[1] = &tms9900_t::do_exec1;
  func[2] = &tms9900_t::do_exec2;
  func[3] = &tms9900_t::do_exec3;
  func[4] = &tms9900_t::do_exec4;
  func[5] = &tms9900_t::do_exec5;
  func[6]  = &tms9900_t::do_exec6;
  func[7]  = &tms9900_t::do_exec7;
  func[8]  = &tms9900_t::do_exec8;
  func[9]  = &tms9900_t::do_exec9;
  func[10] = &tms9900_t::do_execA;
  func[11] = &tms9900_t::do_execB;
  func[12] = &tms9900_t::do_execC;
  func[13] = &tms9900_t::do_execD;
  func[14] = &tms9900_t::do_execE;
  func[15] = &tms9900_t::do_execF;

  st = 0;
  stuck = false;
  do_blwp(0);
}

unsigned long tms9900_t::get_instructions() const {
  return inst_count;
}

unsigned long tms9900_t::get_cycles() const {
  return cycles;
}

bool tms9900_t::step() {
  if(stuck)
    return true;
  prev_pc = pc;
  ir = next();
  return execute();
}

bool tms9900_t::execute() {
  (this->*func[ir >> 12])();  // call the group of instructions.
  if(!stuck)
    inst_count++; 
  return !stuck;
}

void tms9900_t::do_blwp(uint16_t addr) {
  uint16_t old_wp = wp;
  wp = read(addr);
  write_reg(13, old_wp);
  write_reg(14, pc);
  write_reg(15, st);
  pc = read(addr+2);
}
/**
 * @brief dasm_addr_mode disassemles the addressing mode
 * 
 * @param s     Receives the disassembled string
 * @param mode  the mode bits to decode (6 bit field)
 * @param len   amount of additional words needed
 * @return char* pointer to dest (same as s)
 */
char *tms9900_t::dasm_addr_mode(char *s, unsigned mode, int *len) {
    *len = 0;
    switch((mode >> 4) &3) {
        case 0: sprintf(s, "R%d", mode & 0xF); break;
        case 1: sprintf(s, "*R%d", mode & 0xF); break;
        case 2: 
            if (mode & 0xF)
                sprintf(s, "@_R%d", mode & 0xF); 
            else
                sprintf(s, "@"); 
            *len = 1;
            break;
        case 3: sprintf(s, "*R%d+", mode & 0xF); break;
    }
    return s;
}

// state_in = 0: opcode
//          = 1: a parameter word
//          = 2: a paramater word followed by parameter word
// In other words, state_out is simply state_in -1 
int tms9900_t::dasm_one(char *buf, int state_in, int opcode) {
    int state_out = 0;
    if(state_in) {
        sprintf(buf, ">%04X", opcode);
        return state_in-1;
    }

    // stupid linear search of the instructions....
    int j = -1;
    for(int i=0; instructions[i].bin; i++) {
        if((opcode & instructions[i].mask) == instructions[i].bin) {
            j = i;
            break;
        }
    }
    if(j == -1) {
        sprintf(buf, ">%04X      <<<<<", opcode);
        return 0;
    }
    // Display addressing modes
    int p1=0, p2=0;
    char s1[20], s2[20];
    int count;
    int offset;
    switch(instructions[j].type) {
        case dual_op_mult_mult:
            sprintf(buf, "%-4s %s,%s", instructions[j].str,
                dasm_addr_mode(s1, opcode & 0x3F, &p1), 
                dasm_addr_mode(s2, (opcode >>6) & 0x3F, &p2));
            state_out = p1+p2;
            break;
        case dual_op_mult_wr: 
            sprintf(buf, "%-4s %s,R%d", instructions[j].str,
                dasm_addr_mode(s1, opcode & 0x3F, &p1), 
                (opcode >> 6) & 0xF);
            state_out = p1;
            break;
        case xop:
            sprintf(buf, "%-4s %s,%d ", instructions[j].str,
                dasm_addr_mode(s1, opcode & 0x3F, &p1), (opcode >> 6) & 0xF
                );
            state_out = p1;
            break;
        case single_op:
            sprintf(buf, "%-4s %s", instructions[j].str,
                dasm_addr_mode(s1, opcode & 0x3F, &p1)
                );
            state_out = p1;
            break;
        case cru_multi_bit:
            count = (opcode >> 6) & 0xF;
            sprintf(buf, "%-4s %s,%d", instructions[j].str,
                dasm_addr_mode(s1, opcode & 0x3F, &p1), 
                count ? count : 16);
            state_out = p1;
            break;
        case cru_single_bit:
            offset = ((int)opcode << (INTLEN-8)) >> (INTLEN-8);
            sprintf(buf, "%-4s %-4Xh %d", instructions[j].str, offset & 0xFFFF, offset);        
            break;
        case jumps:
            offset = ((int)opcode << (INTLEN-8)) >> (INTLEN-9);
            sprintf(buf, "%-4s %-4Xh %d", instructions[j].str, offset & 0xFFFF, offset);
            break;
        case shifts: 
            if(opcode & 0x00F0) {
                // count in the instruction
                sprintf(buf, "%-4s R%d,%d", instructions[j].str, 
                    opcode & 0xF, (opcode >> 4) & 0xF);
            } else {
                sprintf(buf, "%-4s R0,R%d", instructions[j].str, (opcode >> 4) & 0xF);
            }
            break;
        case immediates:
            sprintf(buf, "%-4s R%d,#", instructions[j].str, opcode & 0xF); 
            state_out = 1;
            break;
        case internal_immediates:
            sprintf(buf, "%-4s #", instructions[j].str); 
            state_out = 1;  // immediate follows
            break;
        case internal_store:
            sprintf(buf, "%-4s R%d", instructions[j].str, opcode & 0xF); 
            break;
        case rtwp:
            sprintf(buf, "%-4s ", instructions[j].str); 
            break;
        default:
            sprintf(buf, "????");
            break;
    }
    return state_out;
}

int tms9900_t::dasm_instruction(char *dst, uint16_t addr) {
  char tmp[80];
  dst[0] = 0;
  int state = 0;
  int steps = 0;
  do { 
    // printf("calling dasm_one step %d state %d addr %04X\n", steps, state, addr);
    state = dasm_one(steps ? tmp : dst, state, read(addr));
    if(steps)
      strcat(dst, tmp);
    addr += 2;
    steps++;
  } while(state > 0 && steps < 5);
  return steps;
}

void tms9900_t::do_exec0() {
    // { 0x0A00, 0xFF00, shifts, "SLA" }, 
    // { 0x0800, 0xFF00, shifts, "SRA" }, 
    // { 0x0B00, 0xFF00, shifts, "SRC" }, 
    // { 0x0900, 0xFF00, shifts, "SRL" }, 
  if((ir & 0x0C00) == 0x0800) {   // Shift instruction?
    unsigned shift_count = (ir >> 4) & 0xF;
    add_cycles(12);
    if(shift_count == 0) {
      shift_count = read_reg(0) & 0xF;
      if(shift_count == 0)
        shift_count = 16;
      add_cycles(8);
    }
    add_cycles(2*shift_count);
    const uint16_t src = read_reg(ir & 0xF);
    unsigned dst;
    switch((ir >> 8) & 3) {
      case 0: // SRA
        if(src & 0x8000)
          dst = 0xFFFF0000 | src;
        else
          dst = src;
        dst >>= shift_count;
        // Set carry, the last bit shifted out. Carry bit 16 of dst.
        dst &= ~0x10000;
        dst |= (src << (17 - shift_count)) & 0x10000;
        break;
      case 1: // SRL
        dst = src >> shift_count;
        // Set carry, the last bit shifted out. Carry bit 16 of dst.
        dst |= (src << (17 - shift_count)) & 0x10000;
        // Examples
        // src: 0x8000, shift_count=16: shift=1 left, result 0x10000 ok
        // src: 0x0001, shift_count=1: shift=16 left, result 0x10000 ok
        break;
      case 2: { // SLA
        uint16_t original_sign = src & 0x8000;
        // ST11 is set (overflow) for sign changing during shifting.
        // potentially a BUG with icy99 core, needs checking. CHECKME
        st &= ~ST11;  // No sign changing seen
        dst = src;
        for(unsigned u=0; u<shift_count; u++) {
          dst <<= 1;
          if(original_sign != (dst & 0x8000)) {
            st |= ST11; // Sign change detected.
          }
        }
        break;
      }
      case 3: // SRC
        dst = (src << 16) | src;
        dst >>= shift_count;
        // Set carry, the last bit shifted out. Carry bit 16 of dst.
        dst &= ~0x10000;
        dst |= (src << (17 - shift_count)) & 0x10000;
        break;
    }
    write_reg(ir & 0xF, dst);
    // Set ST[15]..ST[12]
    flags_012_others(dst);  // ST15,ST14,ST13
    set_carry_add(dst);
    return;
  }

  //  { 0x040C, 0xFFC0, single_op, "BLWP" },
  //  { 0x0740, 0xFFC0, single_op, "ABS" },
  //  { 0x0480, 0xFFC0, single_op, "X" },
  if((ir & 0x0C00) == 0x0400) {
    uint16_t sa = source_address(ir & 0x3F, true);

    switch((ir >> 6) & 0xF) {
      case  0:                                      // BLWP
        add_cycles(26);
        do_blwp(sa);
        return;
      case  1:                                      // B
        add_cycles(8);
        pc = sa; 
        return;                     
      case  2: {                                    // X
        add_cycles(8);
        ir = read(sa);  
        // printf("About to execute instruction X with 0x%04X\n", ir);
        execute();
        add_cycles(-4); // BUGBUG - need to also subtract one memory access time
        return;
      }
      case  3:                                      // CLR (with dummy read)
        add_cycles(10);
        read(sa); write(sa, 0); return;      
      case  4: {                                    // NEG
        add_cycles(12);
        const uint16_t r = read( sa );
        unsigned result = -r;
        write(sa, result);
        flags_012_others(result);
        set_carry_sub(result);
        set_overflow_sub(result, 0, r);  // CHECKME
        return;
      }
      case  5: {                                    // INV
        add_cycles(10);
        uint16_t r = read( sa );
        r = ~r;
        flags_012_others( r );
        write(sa, r);
        return;
      }
      case  6:                                      // INC st0-st4  
      case  7: {                                    // INCT st0-st4
        add_cycles(10);
        const uint16_t r = read( sa );
        const uint16_t amount = (ir & 0x40) ? 2 : 1;
        unsigned result = r + amount;
        write(sa, result);
        flags_012_others(result);
        set_carry_add(result);
        set_overflow_add(result, r, amount);  // CHECKME
        return;
      }
      case  8:                                      // DEC st0-4
      case  9: {                                    // DECT st0-4  
        add_cycles(10);                          
        const uint16_t r = read( sa );
        const uint16_t amount = (ir & 0x40) ? 2 : 1;
        unsigned result = r - amount;
        write(sa, result);
        flags_012_others(result);
        set_carry_sub(result);
        set_overflow_sub(result, r, amount);  // CHECKME
        return;
      }
      case 10:                                      // BL
        add_cycles(12);
        write_reg(11, pc); pc = sa; return;  
      case 11: {                                    // SWPB
        add_cycles(10);
        const uint16_t src = read( sa );
        write(sa, (src << 8) | (src >> 8));
        return;
      }
      case 12:                                      // SETO with dummy read.
        add_cycles(10);
        read(sa); write(sa, 0xFFFF); return; 
      case 13: {                                    // ABS
        add_cycles(10);                          
        const uint16_t r = read( sa );
        unsigned result = r & 0x8000 ? 0-r : r;
        write(sa, result);
        flags_012_others(result);
        set_carry_sub(result);
        set_overflow_sub(result, r, 0);  // CHECKME
        return;
      }
      default:
        stuck = true;
        return;
    }
  }

  if((ir & 0x0F00) == 0x0200) {
    switch((ir >> 5) & 7) {
      case 0: {  // LI instruction
        add_cycles(12);
        uint16_t imm = next();
        write_reg(ir & 0xF, imm);
        flags_012_others(imm);
        return;
      }
      case 1: { // AI instruction
        add_cycles(14);
        uint16_t imm = next();
        uint16_t sa = wp + ((ir & 0xF) << 1);
        const uint16_t src = read(sa);
        // printf("AI 0x%04X 0x%04X\n", src, imm);
        unsigned d = src + imm;
        write_reg(ir & 0xF, d);
        flags_012_others(d);
        set_carry_add(d);
        set_overflow_add(d, src, imm);
        return;
      }
      case 2: { // ANDI
        add_cycles(14);
        uint16_t imm = next();
        uint16_t sa = wp + ((ir & 0xF) << 1);
        uint16_t d = read(sa);
        // printf("ANDI 0x%04X 0x%04X\n", d, imm);
        d = d & imm;
        write_reg(ir & 0xF, d);
        flags_012_others(d);
        return;
      }
      case 3: { // ORI
        add_cycles(14);
        uint16_t imm = next();
        uint16_t sa = wp + ((ir & 0xF) << 1);
        uint16_t d = read(sa);
        d = d | imm;
        write_reg(ir & 0xF, d);
        flags_012_others(d);
        return;
      }
      case 4: { // CI
        add_cycles(14);
        uint16_t imm = next();
        uint16_t sa = wp + ((ir & 0xF) << 1);
        const uint16_t src = read(sa);
        set_flags_compare(imm, src);
        // printf("CI ST: 0x%04X, imm 0x%04X, src=0x%04X\n", st, imm, src);
        return;
      }
    }
  }

  switch(ir & 0xFFE0) {
    case 0x0300: {  // LIMI
      add_cycles(16);
      uint16_t imm = next();
      st = (st & 0xFFF0) | (imm & 0xF);
      return;
    }
    case 0x02C0: {      // STST
      add_cycles(8);
      write_reg(ir & 0xF, st);
      return;
    }
    case 0x02A0: {      // STWP
      add_cycles(8);
      write_reg(ir & 0xF, wp);
      return;
    }
    case 0x02E0: {      // LWPI
      add_cycles(10);
      uint16_t imm = next();
      wp = imm;
      return;
    }
    case 0x0380: {      // RTWP
      add_cycles(14);
      pc = read(wp + (14 << 1));
      st = read(wp + (15 << 1));
      wp = read(wp + (13 << 1));
      return;
    }
  }

  if((ir & 0xFF00) == 0x0300) {
    switch((ir >> 5) & 7) {
      case 3: // RSET
        st &= ~0xF;
      case 2: // IDLE
        // BUGBUG should stop and wait for interrupt here.
      case 6: // CKOF
      case 5: // CKON
      case 7: // LREX
        add_cycles(12);
        break;
      default:
        break;
    }
  }
  
  stuck = true;
}

void tms9900_t::do_jump(bool condition, uint16_t offset) {
  if(condition) {
    pc += offset;
    add_cycles(10);
  } else {
    add_cycles(8);
  }
}

void tms9900_t::do_exec1() {
  uint16_t offset = (ir & 0xFF) << 1;
  offset += ir & 0x80 ? 0xFE00 : 0;   // sign extend
  switch((ir >> 8) & 0xF) {
    case 0: do_jump(true                         , offset); return; // JMP
    case 1: do_jump(!(st & (ST14 | ST13))        , offset); return; // JLT
    case 2: do_jump(!(st & ST15) || (st & ST13)  , offset); return; // JLE
    case 3: do_jump(st & ST13                    , offset); return; // JEQ
    case 4: do_jump((st & (ST15 | ST13))         , offset); return; // JHE
    case 5: do_jump(st & ST14                    , offset); return; // JGT
    case 6: do_jump(!(st & ST13)                 , offset); return; // JNE
    case 7: do_jump(!(st & ST12)                 , offset); return; // JNC
    case 8: do_jump(st & ST12                    , offset); return; // JOC (on carry)
    case 9: do_jump(!(st & ST11)                 , offset); return; // JNO (no overflow)
    case 10: do_jump(!(st & (ST15 | ST13))       ,offset); return; // JL
    case 11: do_jump((st & ST15) && !(st & ST13) ,offset); return; // JH
    case 12: do_jump(st & ST10                   ,offset); return; // JOP (odd parity)
    case 13: {                                                      // SBO
      add_cycles(12);
      write_cru_bit( read_reg(12) + offset, 1 );
      return;
    }
    case 14: {                                                      // SBZ
      add_cycles(12);                                                       
      write_cru_bit( read_reg(12) + offset, 0 );
      return;
    }
    case 15: {                                                      // TB
      add_cycles(12);
      unsigned b = read_cru_bit( read_reg(12) + offset);
      st &= ~ST13;
      st |= b ? ST13 : 0;
      return;
    }

    default:
      break;
  }

  stuck = true;
}
void tms9900_t::do_exec2() {
  uint16_t s, d, r;
  unsigned reg;
  switch((ir >> 10) & 3) {
    case 0:     // COC st2 ST13
      add_cycles(14);
      s = read_operand(ir & 0x3F, true);
      d = read_reg((ir >> 6) & 0xF);
      r = (s ^ d) & s;
      st &= ~ST13;
      st |= !r ? ST13 : 0;
      return;
    case 1:     // CZC st2 ST13
      add_cycles(14);
      s = read_operand(ir & 0x3F, true);
      d = read_reg((ir >> 6) & 0xF);
      r = (s ^ ~d) & s;
      st &= ~ST13;
      st |= !r ? ST13 : 0;
      return;
    case 2:     // XOR st0-2 ST15,ST14,ST13
      add_cycles(14);
      s = read_operand(ir & 0x3F, true);
      reg = (ir >> 6) & 0xF;
      d = read_reg(reg);
      r = s ^ d;
      write_reg(reg, r);
      flags_012_others(r);
      return;
    case 3:     // XOP 
      add_cycles(36);
      d = 0x0040 + (((ir >> 6) & 0xF) << 2);
      s = source_address(ir & 0x3F, true);
      do_blwp(d);
      write_reg(11, s);
      return;

  }
  stuck = true;
}

void tms9900_t::do_exec3() {
  if((ir & 0xFC00) == 0x3000) { // LDCR
    unsigned count = (ir >> 6) & 0xF;
    if(count == 0)  
      count = 16;
    bool byteop = count <= 8;
    add_cycles(20+2*count);
    uint16_t addr = read_reg(12);
    // uint16_t sa = source_address(
    // uint16_t src = read( sa );
    uint16_t src = read_operand(ir & 0x3F, !byteop);
    flags_012_others(src);
    if(byteop) {
      // printf("LDCR (SA=0x%04X)=0x%04X\n", sa, src);
      // printf("LDCR (SA)=0x%04X\n", src);
      do_parity(src);
      src >>= 8;
    }
    while(count > 0) {
      write_cru_bit(addr, src & 1);
      addr += 2;
      src >>= 1;
      count--;
    }
    return;
  } else if((ir & 0xFC00) == 0x3400) { // STCR read from CRU to register
    unsigned count = (ir >> 6) & 0xF;
    if(count == 0)  
      count = 16;
    bool byteop = count <= 8;

    if(byteop)
      add_cycles(42);
    else
      add_cycles(58);
    if((count & 7) == 0)
      add_cycles(2);

    uint16_t addr = read_reg(12);
    uint16_t result=0;
    for(unsigned u=0; u<count; u++) {
      uint16_t bit = read_cru_bit(addr) & 1;
      addr += 2;
      result |= bit << u;
    }
    if(byteop) {
      result <<= 8;
      do_parity(result);
    }
    flags_012_others(result);
    uint16_t da = source_address(ir & 0x3F, !byteop);
    if(byteop)
      write_byte(da, result);
    else
      write(da, result);
    return;
  } else if((ir & 0xFC00) == 0x3800) { // MPY
    add_cycles(52);
    const uint16_t src = read_operand(ir & 0x3F, true);
    const uint16_t dst_addr = wp + (((ir >> 6) & 0xF) << 1);
    const uint16_t dst = read(dst_addr);
    unsigned result = src*dst;
    write(dst_addr, result >> 16);
    write(dst_addr+2, result & 0xFFFF);
    return;
  } else if((ir & 0xFC00) == 0x3C00) { // DIV
    const uint16_t src = read_operand(ir & 0x3F, true);
    const uint16_t dst_addr = wp + (((ir >> 6) & 0xF) << 1);
    const uint16_t dst0 = read(dst_addr);
    const uint16_t dst1 = read(dst_addr+2);
    st &= ~ST11;
    if(src <= dst0) {
      st |= ST11;
      add_cycles(16);
      return;
    }
    add_cycles((124-92)/2); // BUGBUG the timing depends on the partial quotiotent, we go for average
    unsigned dst = (dst0 << 16) | dst1;
    unsigned quotient = dst / src;
    unsigned remainder = dst % src;
    write(dst_addr, quotient); 
    write(dst_addr + 2, remainder);  // remainder
    return;
  }
  stuck = true;
}
void tms9900_t::do_exec4() {      // SZC
  add_cycles(14);
  uint16_t src = read_operand(ir, true);
  uint16_t dst_addr = source_address(ir >> 6, true);
  uint16_t dst = read(dst_addr);
  dst &= ~src;
  flags_012_others(dst);
  write(dst_addr, dst);
}

void tms9900_t::do_exec5() {       // SZCB
  add_cycles(14);
  uint16_t src = read_operand(ir, false);
  uint16_t dst_addr = source_address(ir >> 6, false);
  uint16_t dst = read_byte(dst_addr);
  dst &= ~src;
  flags_012_others(dst);
  do_parity(dst);
  write_byte(dst_addr, dst);
}

void tms9900_t::do_exec6() {      // S substract
  add_cycles(14);
  const uint16_t src = read_operand(ir, true);
  const uint16_t dst_addr = source_address(ir >> 6, true);
  const uint16_t dst = read(dst_addr);
  const unsigned result = dst - src;
  write(dst_addr, result);
  flags_012_others(result);
  set_overflow_sub(result, dst, src);
  set_carry_sub(result);
}
void tms9900_t::do_exec7() {    // SB
  add_cycles(14);
  const uint16_t src = read_operand(ir, false);
  const uint16_t dst_addr = source_address(ir >> 6, false);
  const uint16_t dst = read_byte(dst_addr);
  const unsigned result = dst - src;
  write_byte(dst_addr, result);
  flags_012_others(result);
  do_parity(result);
  set_overflow_sub(result, dst, src);
  set_carry_sub(result);
}

void tms9900_t::do_exec8() {    // C
  add_cycles(14);
  const uint16_t src = read_operand(ir, true);
  const uint16_t dst_addr = source_address(ir >> 6, true);
  const uint16_t dst = read(dst_addr);
  set_flags_compare(dst, src);
}

void tms9900_t::do_exec9() {    // CB
  add_cycles(14);
  const uint16_t src = read_operand(ir, false);
  const uint16_t dst_addr = source_address(ir >> 6, false);
  const uint16_t dst = read_byte(dst_addr);
  // printf("CB src=0x%04X dst=0x%04X result=0x%X\n", src, dst, result);
  set_flags_compare(dst, src);
  do_parity(src);
}

void tms9900_t::do_execA() {    // A add
  add_cycles(14);
  const uint16_t src = read_operand(ir, true);
  const uint16_t dst_addr = source_address(ir >> 6, true);
  const uint16_t dst = read(dst_addr);
  const unsigned result = dst + src;
  write(dst_addr, result);
  flags_012_others(result);
  set_overflow_add(result, dst, src);
  set_carry_add(result);
}
void tms9900_t::do_execB() {    // AB add bytes
  add_cycles(14);
  const uint16_t src = read_operand(ir, false);
  const uint16_t dst_addr = source_address(ir >> 6, false);
  const uint16_t dst = read_byte(dst_addr);
  const unsigned result = dst + src;
  // printf("AB src=0x%04X dst=0x%04X result=0x%X dst_addr=0x%04X\n", src, dst, result, dst_addr);
  write_byte(dst_addr, result);
  flags_012_others(result);
  do_parity(result);
  set_overflow_add(result, dst, src);
  set_carry_add(result);
}
void tms9900_t::do_execC() {    // MOV
  add_cycles(14);
  // Dual operand cycle.
  // Source operand, bits 5..0
  uint16_t src = read_operand(ir, true);
  uint16_t dst_addr = source_address(ir >> 6, true);
  flags_012_others(src);
  read(dst_addr); // Dummy read but needed for compatibility
  write(dst_addr, src);
}

void tms9900_t::do_execD() {    // MOVB
add_cycles(14);
  // Dual operand cycle.
  // Source operand, bits 5..0
  uint16_t src = read_operand(ir, false);
  uint16_t dst_addr = source_address(ir >> 6, false);
  flags_012_others(src);
  do_parity(src);
  write_byte(dst_addr, src);
}

void tms9900_t::do_execE() {      // SOC
  add_cycles(14);
  uint16_t src = read_operand(ir, true);
  uint16_t dst_addr = source_address(ir >> 6, true);
  uint16_t dst = read(dst_addr);
  dst |= src;
  flags_012_others(dst);
  write(dst_addr, dst);
}

void tms9900_t::do_execF() {      // SOCB
  add_cycles(14);
  uint16_t src = read_operand(ir, false);
  uint16_t dst_addr = source_address(ir >> 6, false);
  uint16_t dst = read_byte(dst_addr);
  dst |= src;
  flags_012_others(dst);
  do_parity(dst);
  write_byte(dst_addr, dst);
}

void tms9900_t::write_byte(uint16_t dst_addr, uint16_t dst) {
  // Write the destination byte. Requires read modify write cycle.
  // High byte of dst is our byte to write.
  uint16_t d = read(dst_addr);
  // printf("write_byte: dst 0x%04X, d 0x%04X from 0x%04X\n", dst, d, dst_addr);
  if(dst_addr & 1) {
    // write low byte
    d = (d & 0xFF00) | (dst >> 8);
  } else {
    // write high byte
    d = (d & 0x00FF) | (dst & 0xFF00);
  }
  // printf("write_byte: writing d 0x%04X to 0x%04X\n", d, dst_addr);
  write(dst_addr, d);
}

uint16_t tms9900_t::read_byte(uint16_t addr) {
  uint16_t d = read(addr);
  // The value returned is always given in the high byte.
  if(addr & 1) {
    // return low byte as high byte
    return d << 8;
  } else {
    return d & 0xFF00;
  }
}

void tms9900_t::do_parity(uint16_t src) {
  // Byte operations set parity from high byte of src.
  st &= ~ST10;
  uint16_t paritys = (src >> 7) ^ (src >> 6) ^ (src >> 5) ^(src >> 4) ^ (src >> 3) ^ (src >> 2) ^ (src >> 1) ^ src;
  st |= 0x100 & paritys ? ST10 : 0;
}

void tms9900_t::flags_012_others(uint16_t t) {
  // st[15] <= alu_logical_gt;
  // st[14] <= alu_arithmetic_gt;
  // st[13] <= alu_flag_zero;
  st &= ~(ST15 | ST14 | ST13);
  st |= t ? ST15 : 0;
  st |= !(t & 0x8000) && t ? ST14 : 0;
  st |= t == 0 ? ST13 : 0;
}

uint16_t tms9900_t::read_operand(uint16_t op, bool word_operation) {
  uint16_t sa = source_address(op, word_operation);
  // Do byte selection.
  if(word_operation)
    return read(sa);

  // byte operations. Return in top 8 bits.
  // remember that TMS9900 is big endian.
  if(sa & 1) {
    // low byte
    return read(sa & ~1) << 8;
  } else {
    return read(sa & ~1) & 0xFF00; // high byte
  }
}

uint16_t tms9900_t::source_address(uint16_t op, bool word_operation) {
  op &= 0x3F;
  switch(op & 0x30) {
    case 0x00: return wp+((op & 0xF) << 1); // workspace register
    case 0x10: 
      add_cycles(4);
      return read_reg(op & 0xF);   // workspace register indirect
    case 0x20: {
      // symbolic or indexed mode.
      add_cycles(8);
      uint16_t t = next();
      if((op & 0xF) == 0) {
        // symbolic addressing mode.
        return t;
      } else {
        // Indexed addressing mode.
        return t + read_reg(op & 0xF);
      }
    }
    case 0x30: {
      // workspace register indirect auto increment
      add_cycles(word_operation ? 8 : 6);
      uint16_t t = read_reg(op & 0xF);
      write_reg(op & 0xF, t + (word_operation ? 2 : 1));
      return t;
    }
  }  
  return 0; // never executed
}

bool tms9900_t::interrupt(uint8_t level) {
  uint8_t current_level = st & 0xF;
  if(level < current_level) {
    add_cycles(22);
    // Now force the interrupt.
    do_blwp(level << 2);
    // change current interrupt priority
    st &= ~0xF;
    st |= level;
    return true;
  }
  return false;
}
