#include "disasm.h"
#include <stdio.h>
#include <string.h>

static const char *cond_str[] = {
    "EQ","NE","CS","CC","MI","PL","VS","VC","HI","LS","GE","LT","GT","LE","","NV"
};
static const char *reg_str[] = {
    "R0","R1","R2","R3","R4","R5","R6","R7",
    "R8","R9","R10","R11","R12","SP","LR","PC"
};
static const char *shift_str[] = { "LSL","LSR","ASR","ROR" };
static const char *dp_str[] = {
    "AND","EOR","SUB","RSB","ADD","ADC","SBC","RSC",
    "TST","TEQ","CMP","CMN","ORR","MOV","BIC","MVN"
};

static void fmt_op2(uint32_t insn, char *buf) {
    if (insn & (1<<25)) {
        uint32_t imm = insn & 0xFF;
        uint32_t rot = ((insn >> 8) & 0xF) * 2;
        uint32_t val = rot ? ((imm>>rot)|(imm<<(32-rot))) : imm;
        sprintf(buf, "#0x%X", val);
    } else {
        uint32_t rm   = insn & 0xF;
        uint32_t type = (insn >> 5) & 3;
        uint32_t amt;
        if (insn & (1<<4)) {
            uint32_t rs = (insn >> 8) & 0xF;
            sprintf(buf, "%s,%s %s", reg_str[rm], shift_str[type], reg_str[rs]);
        } else {
            amt = (insn >> 7) & 0x1F;
            if (amt == 0 && type == 0)
                sprintf(buf, "%s", reg_str[rm]);
            else
                sprintf(buf, "%s,%s #%u", reg_str[rm], shift_str[type], amt);
        }
    }
}

int arm_disasm(uint32_t addr, uint32_t insn, char *buf, int buflen) {
    char op2[32] = "";
    uint32_t cond = insn >> 28;
    const char *cs = cond_str[cond];
    uint32_t type = (insn >> 25) & 7;

    /* Branch */
    if (type == 5) {
        int32_t off = (int32_t)((insn & 0xFFFFFF) << 8) >> 6;
        uint32_t target = addr + 8 + (uint32_t)off;
        int link = (insn >> 24) & 1;
        snprintf(buf, buflen, "B%s%s 0x%08X", link ? "L" : "", cs, target);
        return 4;
    }

    /* SWI */
    if (type == 7) {
        snprintf(buf, buflen, "SWI%s #0x%06X", cs, insn & 0xFFFFFF);
        return 4;
    }

    /* BX/BLX */
    if ((insn & 0x0FF000F0) == 0x01200010) {
        snprintf(buf, buflen, "BX%s %s", cs, reg_str[insn & 0xF]);
        return 4;
    }
    if ((insn & 0x0FF000F0) == 0x01200030) {
        snprintf(buf, buflen, "BLX%s %s", cs, reg_str[insn & 0xF]);
        return 4;
    }

    /* LDM/STM */
    if (type == 4) {
        int l = (insn >> 20) & 1;
        int w = (insn >> 21) & 1;
        int u = (insn >> 23) & 1;
        int p = (insn >> 24) & 1;
        uint32_t rn = (insn >> 16) & 0xF;
        uint16_t list = insn & 0xFFFF;
        const char *mode = (u && p) ? "IB" : (u && !p) ? "IA" : (!u && p) ? "DB" : "DA";
        char regs[64] = "{";
        int first = 1;
        for (int i = 0; i < 16; i++) {
            if (list & (1<<i)) {
                if (!first) strcat(regs, ",");
                strcat(regs, reg_str[i]);
                first = 0;
            }
        }
        strcat(regs, "}");
        snprintf(buf, buflen, "%s%s%s %s%s,%s",
                 l ? "LDM" : "STM", cs, mode,
                 reg_str[rn], w ? "!" : "", regs);
        return 4;
    }

    /* LDR/STR */
    if (type == 2 || type == 3) {
        int l = (insn >> 20) & 1;
        int b = (insn >> 22) & 1;
        int w = (insn >> 21) & 1;
        int u = (insn >> 23) & 1;
        int p = (insn >> 24) & 1;
        uint32_t rn = (insn >> 16) & 0xF;
        uint32_t rd = (insn >> 12) & 0xF;
        char offset[32];
        if (insn & (1<<25)) { fmt_op2(insn, offset); }
        else { snprintf(offset, sizeof(offset), "#%u", insn & 0xFFF); }
        snprintf(buf, buflen, "%s%s%s %s,[%s%s,%s%s]%s",
                 l ? "LDR" : "STR", b ? "B" : "", cs,
                 reg_str[rd], reg_str[rn],
                 p ? "" : "]",
                 u ? "" : "-", offset,
                 (p && w) ? "!" : (!p ? "[" : ""));
        return 4;
    }

    /* MRS/MSR */
    if ((insn & 0x0C000000) == 0 && (insn & 0x01900000) == 0x01000000) {
        int msr = (insn >> 21) & 1;
        int r   = (insn >> 22) & 1;
        if (msr) {
            fmt_op2(insn, op2);
            snprintf(buf, buflen, "MSR%s %s,%s", cs, r ? "SPSR" : "CPSR", op2);
        } else {
            snprintf(buf, buflen, "MRS%s %s,%s", cs, reg_str[(insn>>12)&0xF], r ? "SPSR" : "CPSR");
        }
        return 4;
    }

    /* Multiply */
    if ((insn & 0x0F0000F0) == 0x00000090) {
        uint32_t op = (insn >> 21) & 7;
        static const char *mul_ops[] = {"MUL","MLA","","","UMULL","UMLAL","SMULL","SMLAL"};
        snprintf(buf, buflen, "%s%s R%u,R%u,R%u",
                 mul_ops[op], cs,
                 (insn>>16)&0xF, insn&0xF, (insn>>8)&0xF);
        return 4;
    }

    /* Data processing */
    if (type <= 1) {
        uint32_t op  = (insn >> 21) & 0xF;
        uint32_t s   = (insn >> 20) & 1;
        uint32_t rn  = (insn >> 16) & 0xF;
        uint32_t rd  = (insn >> 12) & 0xF;
        fmt_op2(insn, op2);
        if (op == 0xD || op == 0xF) { /* MOV/MVN — no Rn */
            snprintf(buf, buflen, "%s%s%s %s,%s",
                     dp_str[op], s?"S":"", cs, reg_str[rd], op2);
        } else if (op >= 8 && op <= 0xB) { /* TST/TEQ/CMP/CMN — no Rd */
            snprintf(buf, buflen, "%s%s %s,%s",
                     dp_str[op], cs, reg_str[rn], op2);
        } else {
            snprintf(buf, buflen, "%s%s%s %s,%s,%s",
                     dp_str[op], s?"S":"", cs, reg_str[rd], reg_str[rn], op2);
        }
        return 4;
    }

    snprintf(buf, buflen, "??? 0x%08X", insn);
    return 4;
}

int thumb_disasm(uint32_t addr, uint16_t insn, char *buf, int buflen) {
    uint32_t op = insn >> 13;
    (void)addr;

    switch (op) {
        case 1: {
            static const char *ops[] = {"MOV","CMP","ADD","SUB"};
            snprintf(buf, buflen, "%s R%u,#%u", ops[(insn>>11)&3], (insn>>8)&7, insn&0xFF);
            break;
        }
        case 5: {
            uint32_t s = (insn>>11)&3;
            if (s == 3) {
                int l = (insn>>11)&1;
                snprintf(buf, buflen, "%s {R0-R7%s}",
                         l ? "POP" : "PUSH", ((insn>>8)&1) ? (l?",PC":",LR") : "");
            } else {
                snprintf(buf, buflen, "T_MISC 0x%04X", insn);
            }
            break;
        }
        case 7: {
            uint32_t s = (insn>>11)&3;
            if (s==0) {
                int32_t off = (int32_t)((insn&0x3FF)<<22)>>21;
                snprintf(buf, buflen, "B 0x%08X", addr+4+(uint32_t)off);
            } else if (s==3) {
                snprintf(buf, buflen, "BL_low");
            } else {
                snprintf(buf, buflen, "BL_high");
            }
            break;
        }
        default:
            snprintf(buf, buflen, "T_%04X", insn);
    }
    return 2;
}

void arm9_dump_regs(const uint32_t r[16], uint32_t cpsr) {
    printf("R0=%08X R1=%08X R2=%08X R3=%08X\n", r[0],r[1],r[2],r[3]);
    printf("R4=%08X R5=%08X R6=%08X R7=%08X\n", r[4],r[5],r[6],r[7]);
    printf("R8=%08X R9=%08X R10=%08X R11=%08X\n", r[8],r[9],r[10],r[11]);
    printf("R12=%08X SP=%08X LR=%08X PC=%08X\n", r[12],r[13],r[14],r[15]);
    printf("CPSR=%08X [%c%c%c%c] %s mode\n", cpsr,
           (cpsr>>31)&1?'N':'-', (cpsr>>30)&1?'Z':'-',
           (cpsr>>29)&1?'C':'-', (cpsr>>28)&1?'V':'-',
           (cpsr&(1<<5))?"Thumb":"ARM");
}
