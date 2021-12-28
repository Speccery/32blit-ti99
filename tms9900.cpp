// tms9900.cpp
// Implementation of the TMS9900 CPU.

#include <stdio.h>  // sprintf
#include <string.h> // strcat
#include "tms9900.hpp"

#define INTLEN 32

const uint16_t ST15 = 1 << 15;
const uint16_t ST14 = 1 << 14;
const uint16_t ST13 = 1 << 13;
const uint16_t ST12 = 1 << 12;
const uint16_t ST11 = 1 << 11;
const uint16_t ST10 = 1 << 10;

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
    { 0x0270, 0xFFE0, internal_immediates, "LWPI"},
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
  do_blwp(0);
}

bool tms9900_t::step() {
  if(stuck)
    return true;
  ir = next();
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
  if((ir & 0xFFE0) == 0x0200) {
    // LI instruction
    uint16_t imm = next();
    write_reg(ir & 0xF, imm);
    flags_012_others(imm);
    return;
  }
  
  stuck = true;
}

void tms9900_t::do_exec1() {
    // { 0x1300, 0xFF00, jumps, "JEQ" }, 
    // { 0x1500, 0xFF00, jumps, "JGT" }, 
    // { 0x1B00, 0xFF00, jumps, "JH" }, 
    // { 0x1400, 0xFF00, jumps, "JHE" }, 
    // { 0x1A00, 0xFF00, jumps, "JL" }, 
    // { 0x1200, 0xFF00, jumps, "JLE" }, 
    // { 0x1100, 0xFF00, jumps, "JLT" }, 
    // { 0x1000, 0xFF00, jumps, "JMP" }, 
    // { 0x1700, 0xFF00, jumps, "JNC" }, 
    // { 0x1600, 0xFF00, jumps, "JNE" }, 
    // { 0x1900, 0xFF00, jumps, "JNO" }, 
    // { 0x1800, 0xFF00, jumps, "JOC" }, 
    // { 0x1C00, 0xFF00, jumps, "JOP" }, 
  uint16_t offset = (ir & 0xFF) << 1;
  offset += ir & 0x80 ? 0xFE00 : 0;   // sign extend
  switch((ir >> 8) & 0xF) {
    case 0: pc += offset;                                           return; // JMP
    case 1: if(!(st & (ST14 | ST13)))                 pc += offset; return; // JLT
    case 2: if(!(st & ST15) && (st & ST13))           pc += offset; return; // JLE
    case 3: if(st & ST13)                             pc += offset; return; // JEQ
    case 4: if((st & (ST15 | ST13)) == (ST15 | ST13)) pc += offset; return; // JHE
    case 5: if(st & ST14)                             pc += offset; return; // JGT
    case 6: if(!(st & ST13))                          pc += offset; return; // JNE
    case 7: if(!(st & ST12))                          pc += offset; return; // JNC
    case 8: if(st & ST13)                             pc += offset; return; // JOC (on carry)
    case 9: if(!(st & ST11))                          pc += offset; return; // JNO (no overflow)
    case 10: if(!(st & (ST15 | ST13)))                pc += offset; return; // JL
    case 11: if((st & ST15) && !(st & ST13))          pc += offset; return; // JH
    case 12: if(st & ST10)                            pc += offset; return; // JOP (odd parity)
      case 0xD: case 0xE: case 0xF:
      // not jumps, something different.
    default:
      break;
  }

  stuck = true;
}
void tms9900_t::do_exec2() {
  stuck = true;
}
void tms9900_t::do_exec3() {
  stuck = true;
}
void tms9900_t::do_exec4() {
  stuck = true;
}
void tms9900_t::do_exec5() {
  stuck = true;
}
void tms9900_t::do_exec6() {
  stuck = true;
}
void tms9900_t::do_exec7() {
  stuck = true;
}

void tms9900_t::do_exec8() {
  stuck = true;
}

void tms9900_t::do_exec9() {
  stuck = true;
}
void tms9900_t::do_execA() {
  stuck = true;
}
void tms9900_t::do_execB() {
  stuck = true;
}
void tms9900_t::do_execC() {
  stuck = true;
}
void tms9900_t::do_execD() {
  stuck = true;
}
void tms9900_t::do_execE() {
  stuck = true;
}

void tms9900_t::do_execF() {
  stuck = true;
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
