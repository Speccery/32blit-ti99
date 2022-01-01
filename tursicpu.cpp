
// tursicpu.cpp
// EP add some boilerplate to be able to handle Tursi's CPU when verifying

#include <stdio.h>
#include "cpu9900.h"


// Word GetSafeCpuWord(int x, int bank);
int skip_interrupt = 0;							// flag for some instructions
volatile int xbBank = 0;							// Cartridge bank switch
bool BreakOnIllegal = false;

/*
void wrword(Word x, Word y)
{ 
	x&=0xfffe;		// drop LSB
	// now write the new data
	wcpubyte(x,(Byte)(y>>8));
	wcpubyte(x+1,(Byte)(y&0xff));
}

Word romword(Word x, bool rmw)
{ 
	x&=0xfffe;		// drop LSB
	// TODO: this reads the LSB first. Is this the correct order??
	return((rcpubyte(x,rmw)<<8)+rcpubyte(x+1,rmw));
}
*/



void warn(const char *n) {
  printf("TURSI WARN: %s\n", n);
}
void TriggerBreakPoint() {}
void debug_write(const char *s, ...) {
  printf("TURSI DEBUG WRITE: %s\n", s);
}
Byte VDPREG[59];								// VDP read-only registers
