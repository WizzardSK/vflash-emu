#pragma once
#include <stdint.h>

/* Simple ARM/Thumb disassembler for V.Flash debugging */

int  arm_disasm(uint32_t addr, uint32_t insn, char *buf, int buflen);
int  thumb_disasm(uint32_t addr, uint16_t insn, char *buf, int buflen);
void arm9_dump_regs(const uint32_t r[16], uint32_t cpsr);
