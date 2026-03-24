#include "arm9.h"
#include "cp15.h"
#include <string.h>
#include <stdio.h>

#define PC   cpu->r[15]
#define SP   cpu->r[13]
#define LR   cpu->r[14]
#define CPSR cpu->cpsr

#define N_FLAG  ((CPSR >> 31) & 1)
#define Z_FLAG  ((CPSR >> 30) & 1)
#define C_FLAG  ((CPSR >> 29) & 1)
#define V_FLAG  ((CPSR >> 28) & 1)
#define T_FLAG  ((CPSR >>  5) & 1)

#define SET_N(v) do { CPSR = (CPSR & ~ARM9_FLAG_N) | ((v) ? ARM9_FLAG_N : 0); } while(0)
#define SET_Z(v) do { CPSR = (CPSR & ~ARM9_FLAG_Z) | ((v) ? ARM9_FLAG_Z : 0); } while(0)
#define SET_C(v) do { CPSR = (CPSR & ~ARM9_FLAG_C) | ((v) ? ARM9_FLAG_C : 0); } while(0)
#define SET_V(v) do { CPSR = (CPSR & ~ARM9_FLAG_V) | ((v) ? ARM9_FLAG_V : 0); } while(0)
#define SET_NZ(r)    do { SET_N((r)>>31); SET_Z((r)==0); } while(0)
#define SET_NZC(r,c) do { SET_NZ(r); SET_C(c); } while(0)

static inline uint32_t r32(ARM9 *c, uint32_t a){ return c->mem_read32(c->mem_ctx, a&~3u); }
static inline uint16_t r16(ARM9 *c, uint32_t a){ return c->mem_read16(c->mem_ctx, a&~1u); }
static inline uint8_t  r8 (ARM9 *c, uint32_t a){ return c->mem_read8 (c->mem_ctx, a);     }
static inline void w32(ARM9 *c, uint32_t a, uint32_t v){ c->mem_write32(c->mem_ctx,a&~3u,v); }
static inline void w16(ARM9 *c, uint32_t a, uint16_t v){ c->mem_write16(c->mem_ctx,a&~1u,v); }
static inline void w8 (ARM9 *c, uint32_t a, uint8_t  v){ c->mem_write8 (c->mem_ctx,a,v);     }

static int cond_ok(ARM9 *cpu, uint32_t cond) {
    switch(cond){
    case 0: return  Z_FLAG; case 1: return !Z_FLAG;
    case 2: return  C_FLAG; case 3: return !C_FLAG;
    case 4: return  N_FLAG; case 5: return !N_FLAG;
    case 6: return  V_FLAG; case 7: return !V_FLAG;
    case 8: return  C_FLAG && !Z_FLAG; case 9: return !C_FLAG || Z_FLAG;
    case 10: return N_FLAG == V_FLAG;  case 11: return N_FLAG != V_FLAG;
    case 12: return !Z_FLAG && (N_FLAG == V_FLAG);
    case 13: return  Z_FLAG || (N_FLAG != V_FLAG);
    case 14: return 1; default: return 0;
    }
}

typedef struct { uint32_t v; int c; } SR;
static SR bshift(uint32_t val, int type, int amt, int cin) {
    SR r = {val, cin};
    if (!amt) return r;
    switch(type) {
    case 0: r.c=(amt<=32)?(int)((val>>(32-amt))&1):0; r.v=(amt<32)?(val<<amt):0; break;
    case 1: r.c=(amt<=32)?(int)((val>>(amt-1))&1):0;  r.v=(amt<32)?(val>>amt):0; break;
    case 2: r.c=(int)(((int32_t)val>>(amt<32?amt-1:31))&1);
            r.v=(uint32_t)((int32_t)val>>(amt<32?amt:31)); break;
    case 3: amt&=31; if(amt){ r.v=(val>>amt)|(val<<(32-amt)); r.c=(int)((val>>(amt-1))&1); } break;
    }
    return r;
}

static SR decode_shift(ARM9 *cpu, uint32_t insn) {
    int rm=insn&0xF, type=(insn>>5)&3, amt;
    uint32_t val=cpu->r[rm]; if(rm==15) val+=4;
    if(insn&(1<<4)) { amt=cpu->r[(insn>>8)&0xF]&0xFF; }
    else { amt=(insn>>7)&0x1F;
        if(!amt && type==3){ SR r; r.c=val&1; r.v=(val>>1)|((uint32_t)C_FLAG<<31); return r; }
    }
    return bshift(val,type,amt,C_FLAG);
}
static uint32_t decode_imm(uint32_t insn){
    uint32_t i=insn&0xFF; int rot=((insn>>8)&0xF)*2;
    return rot?(i>>rot)|(i<<(32-rot)):i;
}

static void save_bank(ARM9 *cpu, int m) {
    switch(m&0x1F){
    case ARM9_MODE_FIQ:
        cpu->r8_fiq=cpu->r[8];cpu->r9_fiq=cpu->r[9];cpu->r10_fiq=cpu->r[10];
        cpu->r11_fiq=cpu->r[11];cpu->r12_fiq=cpu->r[12];
        cpu->r13_fiq=cpu->r[13];cpu->r14_fiq=cpu->r[14];cpu->spsr_fiq=cpu->spsr; break;
    case ARM9_MODE_IRQ: cpu->r13_irq=cpu->r[13];cpu->r14_irq=cpu->r[14];cpu->spsr_irq=cpu->spsr; break;
    case ARM9_MODE_SVC: cpu->r13_svc=cpu->r[13];cpu->r14_svc=cpu->r[14];cpu->spsr_svc=cpu->spsr; break;
    case ARM9_MODE_ABT: cpu->r13_abt=cpu->r[13];cpu->r14_abt=cpu->r[14];cpu->spsr_abt=cpu->spsr; break;
    case ARM9_MODE_UND: cpu->r13_und=cpu->r[13];cpu->r14_und=cpu->r[14];cpu->spsr_und=cpu->spsr; break;
    }
}
static void load_bank(ARM9 *cpu, int m) {
    switch(m&0x1F){
    case ARM9_MODE_FIQ:
        cpu->r[8]=cpu->r8_fiq;cpu->r[9]=cpu->r9_fiq;cpu->r[10]=cpu->r10_fiq;
        cpu->r[11]=cpu->r11_fiq;cpu->r[12]=cpu->r12_fiq;
        cpu->r[13]=cpu->r13_fiq;cpu->r[14]=cpu->r14_fiq;cpu->spsr=cpu->spsr_fiq; break;
    case ARM9_MODE_IRQ: cpu->r[13]=cpu->r13_irq;cpu->r[14]=cpu->r14_irq;cpu->spsr=cpu->spsr_irq; break;
    case ARM9_MODE_SVC: cpu->r[13]=cpu->r13_svc;cpu->r[14]=cpu->r14_svc;cpu->spsr=cpu->spsr_svc; break;
    case ARM9_MODE_ABT: cpu->r[13]=cpu->r13_abt;cpu->r[14]=cpu->r14_abt;cpu->spsr=cpu->spsr_abt; break;
    case ARM9_MODE_UND: cpu->r[13]=cpu->r13_und;cpu->r[14]=cpu->r14_und;cpu->spsr=cpu->spsr_und; break;
    }
}
static void set_mode(ARM9 *cpu, uint32_t ncpsr){
    int om=CPSR&0x1F, nm=ncpsr&0x1F;
    if(om!=nm){ save_bank(cpu,om); CPSR=ncpsr; load_bank(cpu,nm); }
    else CPSR=ncpsr;
}

static uint32_t do_add(ARM9 *cpu,uint32_t a,uint32_t b,int s){
    uint64_t r=(uint64_t)a+b; uint32_t res=(uint32_t)r;
    if(s){SET_N(res>>31);SET_Z(res==0);SET_C(r>>32);SET_V((!((a^b)>>31))&&((a^res)>>31));}
    return res;
}
static uint32_t do_adc(ARM9 *cpu,uint32_t a,uint32_t b,int s){
    uint64_t r=(uint64_t)a+b+C_FLAG; uint32_t res=(uint32_t)r;
    if(s){SET_N(res>>31);SET_Z(res==0);SET_C(r>>32);SET_V((!((a^b)>>31))&&((a^res)>>31));}
    return res;
}
static uint32_t do_sub(ARM9 *cpu,uint32_t a,uint32_t b,int s){
    uint64_t r=(uint64_t)a-b; uint32_t res=(uint32_t)r;
    if(s){SET_N(res>>31);SET_Z(res==0);SET_C(a>=b);SET_V(((a^b)>>31)&&((a^res)>>31));}
    return res;
}
static uint32_t do_sbc(ARM9 *cpu,uint32_t a,uint32_t b,int s){
    uint64_t r=(uint64_t)a-b-(1-C_FLAG); uint32_t res=(uint32_t)r;
    if(s){SET_N(res>>31);SET_Z(res==0);SET_C((uint64_t)a>=(uint64_t)b+(1-C_FLAG));SET_V(((a^b)>>31)&&((a^res)>>31));}
    return res;
}

static void exec_arm(ARM9 *cpu, uint32_t insn) {
    if(!cond_ok(cpu,insn>>28)) return;

    /* BX Rm — Branch and Exchange (ARM→Thumb or Thumb→ARM) */
    if((insn&0x0FFFFFF0)==0x012FFF10){
        uint32_t a=cpu->r[insn&0xF];
        if(a&1){CPSR|=ARM9_FLAG_T; PC=a&~1u;}
        else   {CPSR&=~ARM9_FLAG_T; PC=a&~3u;}
        return;
    }
    /* BLX Rm — Branch with Link and Exchange.
     * LR = addr of next instruction = PC-4 (since PC=inst+8 here). */
    if((insn&0x0FFFFFF0)==0x012FFF30){
        uint32_t a=cpu->r[insn&0xF];
        LR = PC - 4;                          /* return address = inst+4 */
        if(a&1){CPSR|=ARM9_FLAG_T; PC=a&~1u;}
        else   {CPSR&=~ARM9_FLAG_T; PC=a&~3u;}
        return;
    }
    /* B / BL */
    if((insn&0x0E000000)==0x0A000000){
        int32_t off=(int32_t)(insn<<8)>>6;
        if(insn&(1<<24)) LR=PC-4;            /* BL: LR = inst+4 */
        PC=(uint32_t)((int32_t)PC+off);
        return;
    }
    /* SWI — Software Interrupt */
    if((insn&0x0F000000)==0x0F000000){ arm9_swi(cpu); return; }
    /* Undefined instruction — trigger UND exception */
    if((insn&0x0E000010)==0x06000010){ arm9_undef(cpu); return; }
    if((insn&0x0FBF0FFF)==0x010F0000){ cpu->r[(insn>>12)&0xF]=(insn&(1<<22))?cpu->spsr:CPSR; return; }
    if((insn&0x0FB00000)==0x03200000||(insn&0x0FB00FF0)==0x01200000){
        uint32_t val=(insn&(1<<25))?decode_imm(insn):cpu->r[insn&0xF];
        uint32_t mask=0; if(insn&(1<<19)) mask|=0xF0000000u; if(insn&(1<<16)) mask|=0xDFu; /* 0xDF not 0xFF: T bit (bit5) must not be changed by MSR */
        if(insn&(1<<22)) cpu->spsr=(cpu->spsr&~mask)|(val&mask);
        else set_mode(cpu,(CPSR&~mask)|(val&mask));
        return;
    }
    if((insn&0x0FC000F0)==0x00000090){
        int rd=(insn>>16)&0xF,rn=(insn>>12)&0xF,rs=(insn>>8)&0xF,rm=insn&0xF;
        int s=(insn>>20)&1,a=(insn>>21)&1;
        uint32_t res=cpu->r[rm]*cpu->r[rs]; if(a) res+=cpu->r[rn];
        cpu->r[rd]=res; if(s){SET_N(res>>31);SET_Z(res==0);} return;
    }
    if((insn&0x0F8000F0)==0x00800090){
        int rdhi=(insn>>16)&0xF,rdlo=(insn>>12)&0xF,rs=(insn>>8)&0xF,rm=insn&0xF;
        int s=(insn>>20)&1,a=(insn>>21)&1,sgn=!((insn>>22)&1);
        uint64_t res=sgn?((uint64_t)(int64_t)(int32_t)cpu->r[rm]*(int64_t)(int32_t)cpu->r[rs]):((uint64_t)cpu->r[rm]*cpu->r[rs]);
        if(a) res+=((uint64_t)cpu->r[rdhi]<<32)|cpu->r[rdlo];
        cpu->r[rdhi]=(uint32_t)(res>>32); cpu->r[rdlo]=(uint32_t)res;
        if(s){SET_N(res>>63);SET_Z(res==0);} return;
    }
    if((insn&0x0FB00FF0)==0x01000090){
        int rn=(insn>>16)&0xF,rd=(insn>>12)&0xF,rm=insn&0xF,b=(insn>>22)&1;
        uint32_t addr=cpu->r[rn];
        if(b){uint8_t t=r8(cpu,addr);w8(cpu,addr,(uint8_t)cpu->r[rm]);cpu->r[rd]=t;}
        else {uint32_t t=r32(cpu,addr);w32(cpu,addr,cpu->r[rm]);cpu->r[rd]=t;}
        return;
    }
    if((insn&0x0E000090)==0x00000090&&(insn&0x60)){
        int p=(insn>>24)&1,u=(insn>>23)&1,imm=(insn>>22)&1,w=(insn>>21)&1;
        int l=(insn>>20)&1,rn=(insn>>16)&0xF,rd=(insn>>12)&0xF,sh=(insn>>5)&3;
        uint32_t off=imm?(((insn>>4)&0xF0)|(insn&0xF)):cpu->r[insn&0xF];
        uint32_t base=cpu->r[rn], addr=p?(u?base+off:base-off):base;
        if(l){ uint32_t v=0;
            if(sh==1) v=r16(cpu,addr);
            else if(sh==2) v=(uint32_t)(int32_t)(int8_t)r8(cpu,addr);
            else if(sh==3) v=(uint32_t)(int32_t)(int16_t)r16(cpu,addr);
            cpu->r[rd]=v;
        } else if(sh==1) w16(cpu,addr,(uint16_t)cpu->r[rd]);
        if(!p) addr=u?base+off:base-off;
        if(!p||w) cpu->r[rn]=addr;
        return;
    }
    /* CLZ — must be before data processing (both match 0x0C000000==0) */
    if((insn&0x0FFF0FF0)==0x016F0F10){
        int rd=(insn>>12)&0xF, rm=insn&0xF;
        cpu->r[rd] = cpu->r[rm] ? (uint32_t)__builtin_clz(cpu->r[rm]) : 32;
        return;
    }
    /* PLD — preload hint, NOP */
    if((insn&0x0D70F000)==0x0550F000) return;
    /* LDRD */
    if((insn&0x0E1000D0)==0x000000D0 && ((insn>>5)&3)==2){
        int p=(insn>>24)&1,u=(insn>>23)&1,imm=(insn>>22)&1,w=(insn>>21)&1;
        int rn=(insn>>16)&0xF,rd=(insn>>12)&0xF;
        uint32_t off=imm?(((insn>>4)&0xF0)|(insn&0xF)):cpu->r[insn&0xF];
        uint32_t base=cpu->r[rn], addr=p?(u?base+off:base-off):base;
        cpu->r[rd]   = r32(cpu, addr);
        cpu->r[rd+1] = r32(cpu, addr+4);
        if(!p) addr=u?base+off:base-off;
        if(!p||w) cpu->r[rn]=addr;
        return;
    }
    /* STRD */
    if((insn&0x0E1000F0)==0x000000F0){
        int p=(insn>>24)&1,u=(insn>>23)&1,imm=(insn>>22)&1,w=(insn>>21)&1;
        int rn=(insn>>16)&0xF,rd=(insn>>12)&0xF;
        uint32_t off=imm?(((insn>>4)&0xF0)|(insn&0xF)):cpu->r[insn&0xF];
        uint32_t base=cpu->r[rn], addr=p?(u?base+off:base-off):base;
        w32(cpu, addr,   cpu->r[rd]);
        w32(cpu, addr+4, cpu->r[rd+1]);
        if(!p) addr=u?base+off:base-off;
        if(!p||w) cpu->r[rn]=addr;
        return;
    }
    /* DSP multiply (SMLA/SMUL/SMLAW/SMLAL/SMULW) */
    if((insn&0x0F900090)==0x01000080){
        int op=(insn>>21)&3;
        int rd=(insn>>16)&0xF,rn=(insn>>12)&0xF,rs=(insn>>8)&0xF,rm=insn&0xF;
        int x=(insn>>5)&1, y=(insn>>6)&1;
        int16_t a=(int16_t)(x ? cpu->r[rm]>>16 : cpu->r[rm]);
        int16_t b=(int16_t)(y ? cpu->r[rs]>>16 : cpu->r[rs]);
        switch(op){
            case 0: cpu->r[rd]=(uint32_t)((int32_t)a*b + (int32_t)cpu->r[rn]); break;
            case 1: if(!x){ int32_t res=(int32_t)(((int64_t)(int32_t)cpu->r[rm]*b)>>16); cpu->r[rd]=(uint32_t)(res+(int32_t)cpu->r[rn]); }
                    else   { cpu->r[rd]=(uint32_t)(((int64_t)(int32_t)cpu->r[rm]*b)>>16); }
                    break;
            case 2: { int64_t acc=((int64_t)cpu->r[rd]<<32)|cpu->r[rn]; acc+=(int64_t)a*b; cpu->r[rd]=(uint32_t)(acc>>32); cpu->r[rn]=(uint32_t)acc; break; }
            case 3: cpu->r[rd]=(uint32_t)((int32_t)a*b); break;
        }
        return;
    }
    /* QADD/QSUB/QDADD/QDSUB */
    if((insn&0x0F900FF0)==0x01000050){
        int op=(insn>>21)&3;
        int rd=(insn>>12)&0xF,rn=(insn>>16)&0xF,rm=insn&0xF;
        int64_t a=(int32_t)cpu->r[rm], b=(int32_t)cpu->r[rn];
        if(op==1||op==3) b*=2;
        int64_t res=(op==0||op==2)?a+b:a-b;
        if(res>0x7FFFFFFF) res=0x7FFFFFFF;
        if(res<-0x80000000LL) res=-0x80000000LL;
        cpu->r[rd]=(uint32_t)(int32_t)res;
        return;
    }
    if((insn&0x0C000000)==0x00000000){
        int imm=(insn>>25)&1,op=(insn>>21)&0xF,s=(insn>>20)&1;
        int rn=(insn>>16)&0xF,rd=(insn>>12)&0xF;
        uint32_t a=cpu->r[rn]; if(rn==15) a=PC;
        uint32_t b; int shc=C_FLAG;
        if(imm){ b=decode_imm(insn); int rot=((insn>>8)&0xF)*2; if(rot) shc=(b>>31)&1; }
        else { SR sr=decode_shift(cpu,insn); b=sr.v; shc=sr.c; }
        uint32_t res=0; int wr=1;
        switch(op){
        case 0x0: res=a&b; if(s){SET_NZ(res);SET_C(shc);} break;
        case 0x1: res=a^b; if(s){SET_NZ(res);SET_C(shc);} break;
        case 0x2: res=do_sub(cpu,a,b,s); break;
        case 0x3: res=do_sub(cpu,b,a,s); break;
        case 0x4: res=do_add(cpu,a,b,s); break;
        case 0x5: res=do_adc(cpu,a,b,s); break;
        case 0x6: res=do_sbc(cpu,a,b,s); break;
        case 0x7: res=do_sbc(cpu,b,a,s); break;
        case 0x8: { uint32_t r=a&b; SET_NZ(r); SET_C(shc); wr=0; } break;
        case 0x9: { uint32_t r=a^b; SET_NZ(r); SET_C(shc); wr=0; } break;
        case 0xA: do_sub(cpu,a,b,1); wr=0; break;
        case 0xB: do_add(cpu,a,b,1); wr=0; break;
        case 0xC: res=a|b;  if(s){SET_NZ(res);SET_C(shc);} break;
        case 0xD: res=b;    if(s){SET_NZ(res);SET_C(shc);} break;
        case 0xE: res=a&~b; if(s){SET_NZ(res);SET_C(shc);} break;
        case 0xF: res=~b;   if(s){SET_NZ(res);SET_C(shc);} break;
        }
        if(wr){ cpu->r[rd]=res; if(rd==15){ if(s) set_mode(cpu,cpu->spsr); PC&=~3u; } }
        return;
    }
    if((insn&0x0C000000)==0x04000000){
        int i=(insn>>25)&1,p=(insn>>24)&1,u=(insn>>23)&1,b=(insn>>22)&1;
        int w=(insn>>21)&1,l=(insn>>20)&1,rn=(insn>>16)&0xF,rd=(insn>>12)&0xF;
        uint32_t base=cpu->r[rn]; if(rn==15) base=PC;
        uint32_t off=i?decode_shift(cpu,insn).v:(insn&0xFFF);
        uint32_t addr=p?(u?base+off:base-off):base;
        if(l){
            uint32_t v=b?r8(cpu,addr):r32(cpu,addr);
            cpu->r[rd]=v;
            /* LDR PC: loaded value becomes new PC (word-aligned) */
            if(rd==15) PC = v & ~3u;
        } else {
            /* STR PC: stored value = current PC = inst+8 (already set) */
            uint32_t v = (rd==15) ? PC : cpu->r[rd];
            if(b) w8(cpu,addr,(uint8_t)v); else w32(cpu,addr,v);
        }
        if(!p) addr=u?base+off:base-off;
        if((!p||w)&&rn!=rd) cpu->r[rn]=addr;
        return;
    }
    if((insn&0x0E000000)==0x08000000){
        int p=(insn>>24)&1,u=(insn>>23)&1,w=(insn>>21)&1,l=(insn>>20)&1,rn=(insn>>16)&0xF;
        uint16_t rlist=insn&0xFFFF;
        uint32_t base=cpu->r[rn];
        int cnt=__builtin_popcount(rlist);
        /* Start address for iteration (always increments addr by 4 per register):
         * IA (p=0,u=1): addr = base
         * IB (p=1,u=1): addr = base + 4
         * DA (p=0,u=0): addr = base - cnt*4 + 4
         * DB (p=1,u=0): addr = base - cnt*4        ← PUSH / STMDB */
        uint32_t addr = u ? (base + (uint32_t)(p?4:0))
                          : (base - (uint32_t)(cnt*4) + (uint32_t)(p?0:4));
        for(int i=0;i<16;i++){
            if(!(rlist&(1<<i))) continue;
            if(l){
                uint32_t v=r32(cpu,addr);
                cpu->r[i]=v;
                /* LDM with PC in list: loaded value IS the new PC.
                 * If S-bit (bit22) set and PC in list → restore CPSR from SPSR */
                if(i==15) {
                    PC = v & ~3u;
                    if(insn & (1<<22))
                        set_mode(cpu, cpu->spsr);
                }
            } else {
                w32(cpu,addr,cpu->r[i]);
            }
            addr+=4;
        }
        if(w) cpu->r[rn]=u?base+(uint32_t)(cnt*4):base-(uint32_t)(cnt*4);
        return;
    }
    /* MCR/MRC — Coprocessor register transfer (CP15) */
    if((insn&0x0F000010)==0x0E000010){
        int l=(insn>>20)&1, crn=(insn>>16)&0xF, rd=(insn>>12)&0xF;
        int cp=(insn>>8)&0xF, op2=(insn>>5)&7, crm=insn&0xF;
        if(cp==15){
            if(l) cpu->r[rd]=cp15_read(&cpu->cp15,(uint32_t)crn,(uint32_t)crm,(uint32_t)op2);
            else  cp15_write(&cpu->cp15,(uint32_t)crn,(uint32_t)crm,(uint32_t)op2,cpu->r[rd]);
        } else fprintf(stderr,"[ARM9] MCR/MRC unknown CP%d\n",cp);
        return;
    }

    /* Unhandled — trigger Undefined Instruction exception */
    fprintf(stderr,"[ARM9] Undef→UND: 0x%08X PC=0x%08X\n",insn,PC-8);
    arm9_undef(cpu);
}

static void exec_thumb(ARM9 *cpu, uint16_t insn) {
    if((insn>>13)==0){ int op=(insn>>11)&3,sh=(insn>>6)&0x1F,rs=(insn>>3)&7,rd=insn&7; SR sr=bshift(cpu->r[rs],op,sh?sh:32,C_FLAG); cpu->r[rd]=sr.v; SET_NZ(sr.v); SET_C(sr.c); return; }
    if((insn>>11)==3){ int op=(insn>>9)&1,i=(insn>>10)&1,rn=(insn>>6)&7,rs=(insn>>3)&7,rd=insn&7; uint32_t b=i?(uint32_t)rn:cpu->r[rn]; cpu->r[rd]=op?do_sub(cpu,cpu->r[rs],b,1):do_add(cpu,cpu->r[rs],b,1); return; }
    if((insn>>13)==1){ int op=(insn>>11)&3,rd=(insn>>8)&7; uint32_t imm=insn&0xFF; switch(op){ case 0: cpu->r[rd]=imm; SET_NZ(imm); break; case 1: do_sub(cpu,cpu->r[rd],imm,1); break; case 2: cpu->r[rd]=do_add(cpu,cpu->r[rd],imm,1); break; case 3: cpu->r[rd]=do_sub(cpu,cpu->r[rd],imm,1); break; } return; }
    if((insn>>10)==0x10){ int op=(insn>>6)&0xF,rs=(insn>>3)&7,rd=insn&7; uint32_t a=cpu->r[rd],b=cpu->r[rs]; switch(op){ case 0: cpu->r[rd]=a&b;SET_NZ(a&b);break; case 1: cpu->r[rd]=a^b;SET_NZ(a^b);break; case 2:{SR s=bshift(a,0,b&0xFF,C_FLAG);cpu->r[rd]=s.v;SET_NZC(s.v,s.c);}break; case 3:{SR s=bshift(a,1,b&0xFF,C_FLAG);cpu->r[rd]=s.v;SET_NZC(s.v,s.c);}break; case 4:{SR s=bshift(a,2,b&0xFF,C_FLAG);cpu->r[rd]=s.v;SET_NZC(s.v,s.c);}break; case 5: cpu->r[rd]=do_adc(cpu,a,b,1);break; case 6: cpu->r[rd]=do_sbc(cpu,a,b,1);break; case 7:{SR s=bshift(a,3,b&0xFF,C_FLAG);cpu->r[rd]=s.v;SET_NZC(s.v,s.c);}break; case 8:{uint32_t r=a&b;SET_NZ(r);}break; case 9: cpu->r[rd]=do_sub(cpu,0,b,1);break; case 10: do_sub(cpu,a,b,1);break; case 11: do_add(cpu,a,b,1);break; case 12: cpu->r[rd]=a|b;SET_NZ(a|b);break; case 13: cpu->r[rd]=a*b;SET_NZ(a*b);break; case 14: cpu->r[rd]=a&~b;SET_NZ(a&~b);break; case 15: cpu->r[rd]=~b;SET_NZ(~b);break; } return; }
    if((insn>>10)==0x11){ int op=(insn>>8)&3,h1=(insn>>7)&1,h2=(insn>>6)&1; int rs=((insn>>3)&7)+(h2?8:0),rd=(insn&7)+(h1?8:0); switch(op){ case 0: cpu->r[rd]+=cpu->r[rs];break; case 1: do_sub(cpu,cpu->r[rd],cpu->r[rs],1);break; case 2: cpu->r[rd]=cpu->r[rs];break; case 3:{ uint32_t a=cpu->r[rs]; if(a&1){CPSR|=ARM9_FLAG_T;PC=a&~1u;}else{CPSR&=~ARM9_FLAG_T;PC=a&~3u;} }break; } return; }
    /* Thumb LDR PC-relative: LDR Rd,[PC,#imm8*4]
     * addr = (PC & ~2) + imm8*4  — PC is already inst+4 here */
    if((insn>>11)==0x9){ int rd=(insn>>8)&7; cpu->r[rd]=r32(cpu,(PC&~2u)+((insn&0xFF)<<2)); return; }
    /* Thumb SP-relative LDR/STR: encoding 0x9xxx
     * bit11=1(LDR)/0(STR), bits10:8=Rd, bits7:0=imm8 (word-scaled) */
    if((insn>>12)==0x9){
        int l=(insn>>11)&1, rd=(insn>>8)&7;
        uint32_t addr=SP+((uint32_t)(insn&0xFF)<<2);
        if(l) cpu->r[rd]=r32(cpu,addr);
        else  w32(cpu,addr,cpu->r[rd]);
        return;
    }
    /* ADR: ADD Rd,PC,#imm8*4  and  ADD Rd,SP,#imm8*4
     * bit11=0→PC-based, bit11=1→SP-based */
    if((insn>>12)==0xA){
        int sp=(insn>>11)&1, rd=(insn>>8)&7;
        uint32_t base = sp ? SP : (PC&~2u);
        cpu->r[rd] = base + ((uint32_t)(insn&0xFF)<<2);
        return;
    }
    if((insn>>12)==5&&!((insn>>9)&1)){ int l=(insn>>11)&1,b=(insn>>10)&1,ro=(insn>>6)&7,rb=(insn>>3)&7,rd=insn&7; uint32_t addr=cpu->r[rb]+cpu->r[ro]; if(l) cpu->r[rd]=b?r8(cpu,addr):r32(cpu,addr); else { if(b) w8(cpu,addr,(uint8_t)cpu->r[rd]); else w32(cpu,addr,cpu->r[rd]); } return; }
    if((insn>>13)==3){ int b=(insn>>12)&1,l=(insn>>11)&1,off=(insn>>6)&0x1F,rb=(insn>>3)&7,rd=insn&7; uint32_t addr=cpu->r[rb]+(b?(uint32_t)off:(uint32_t)(off<<2)); if(l) cpu->r[rd]=b?r8(cpu,addr):r32(cpu,addr); else { if(b) w8(cpu,addr,(uint8_t)cpu->r[rd]); else w32(cpu,addr,cpu->r[rd]); } return; }
    if((insn>>12)==0x8){ int l=(insn>>11)&1,off=((insn>>6)&0x1F)<<1,rb=(insn>>3)&7,rd=insn&7; uint32_t addr=cpu->r[rb]+(uint32_t)off; if(l) cpu->r[rd]=r16(cpu,addr); else w16(cpu,addr,(uint16_t)cpu->r[rd]); return; }
    if((insn>>8)==0xB0){ int s=(insn>>7)&1; uint32_t off=(uint32_t)((insn&0x7F)<<2); SP=s?SP-off:SP+off; return; }
    if((insn>>12)==0xB){ int l=(insn>>11)&1,r=(insn>>8)&1; uint8_t rlist=(uint8_t)(insn&0xFF); if(!l){ if(r){SP-=4;w32(cpu,SP,LR);} for(int i=7;i>=0;i--) if(rlist&(1<<i)){SP-=4;w32(cpu,SP,cpu->r[i]);} } else { for(int i=0;i<8;i++) if(rlist&(1<<i)){cpu->r[i]=r32(cpu,SP);SP+=4;} if(r){PC=r32(cpu,SP)&~1u;SP+=4;} } return; }
    if((insn>>12)==0xC){ int l=(insn>>11)&1,rb=(insn>>8)&7; uint8_t rlist=(uint8_t)(insn&0xFF); uint32_t addr=cpu->r[rb]; for(int i=0;i<8;i++){ if(!(rlist&(1<<i))) continue; if(l) cpu->r[i]=r32(cpu,addr); else w32(cpu,addr,cpu->r[i]); addr+=4; } cpu->r[rb]=addr; return; }
    if((insn>>12)==0xD){ int cond=(insn>>8)&0xF; if(cond==0xF){ arm9_swi(cpu); return; } if(cond_ok(cpu,(uint32_t)cond)){ PC=(uint32_t)((int32_t)PC+(int8_t)(insn&0xFF)*2); } return; }
    /* Thumb unconditional B: signed 11-bit offset in halfwords */
    if((insn>>11)==0x1C){ int32_t off=(int32_t)(((int16_t)((insn&0x7FF)<<5))>>4); PC=(uint32_t)((int32_t)PC+off); return; }
    if((insn>>12)==0xF){ int h=(insn>>11)&1; if(!h){ int32_t off=(int32_t)(((int16_t)(insn<<5))>>4)<<1; LR=(uint32_t)((int32_t)PC+off); } else { uint32_t addr=LR+((uint32_t)(insn&0x7FF)<<1); LR=(PC-2)|1; PC=addr; } return; }
    /* BKPT — treat as NOP in emulator context */
    if((insn>>8)==0xBE) return;
    /* Unhandled Thumb — trigger Undefined Instruction exception */
    fprintf(stderr,"[ARM9] Thumb undef→UND: 0x%04X PC=0x%08X\n",insn,PC-4);
    arm9_undef(cpu);
}

void arm9_reset(ARM9 *cpu) {
    memset(cpu->r,0,sizeof(cpu->r));
    CPSR=ARM9_MODE_SVC|ARM9_FLAG_I|ARM9_FLAG_F; cpu->spsr=0; PC=0; cpu->cycles=0;
    cpu->null_trap_enabled = 0;
    cp15_reset(&cpu->cp15);
    printf("[ARM9] Reset PC=0x%08X CPSR=0x%08X\n",PC,CPSR);
}

/* Vector base: 0x00000000 or 0xFFFF0000 depending on CP15 HIVEC bit */
static inline uint32_t vec_base(ARM9 *cpu) {
    return cpu->cp15.hivec ? 0xFFFF0000u : 0x00000000u;
}

/* Estimate cycles consumed by one instruction.
 * ARM926EJ-S pipeline: most ALU = 1 cycle, LDR = 2, LDM/STM = 1+N,
 * branch/BL = 3 (pipeline flush), MUL = 2-5.
 * We use a simple lookup on the top 4 bits of the ARM encoding. */
static int insn_cycles_arm(uint32_t insn) {
    uint32_t grp = (insn >> 25) & 7;
    switch (grp) {
        case 0: case 1: {           /* Data processing / misc */
            uint32_t op = (insn >> 21) & 0xF;
            /* MUL/MLA encoded in grp 0 with bits[7:4]=1001 */
            if ((insn & 0x0FC000F0) == 0x00000090) return 3;
            if ((insn & 0x0F8000F0) == 0x00800090) return 4; /* MULL */
            /* LDR/STR in grp 0: LDRH/STRH */
            if ((insn & 0x0E000090) == 0x00000090 && (insn & 0x60)) return 2;
            (void)op;
            return 1;
        }
        case 2: case 3:             /* LDR/STR */
            return 2;
        case 4:                     /* LDM/STM */
            return 1 + __builtin_popcount(insn & 0xFFFF);
        case 5:                     /* B/BL — pipeline flush */
            return 3;
        default:
            return 1;
    }
}

static int insn_cycles_thumb(uint16_t insn) {
    /* BL prefix/suffix, B, BX = 3; LDR/STR = 2; PUSH/POP = 1+N; else 1 */
    if ((insn >> 12) == 0xF)  return 3;  /* BL */
    if ((insn >> 11) == 0x1C) return 3;  /* B unconditional */
    if ((insn >> 12) == 0xD)  return 1;  /* B conditional (predicted not taken) */
    if ((insn >> 13) == 3)    return 2;  /* LDR/STR */
    if ((insn >> 12) == 0x8)  return 2;  /* LDRH/STRH */
    if ((insn >> 9)  == 0x16) return 2;  /* LDR SP-relative */
    if ((insn >> 12) == 0xB)  /* PUSH/POP */
        return 1 + __builtin_popcount(insn & 0x1FF);
    if ((insn >> 12) == 0xC)  /* LDMIA/STMIA */
        return 1 + __builtin_popcount(insn & 0xFF);
    return 1;
}

/* arm9_step — fetch, advance PC to simulate 3-stage pipeline, execute.
 *
 * ARM pipeline convention: during execution, PC = instruction_addr + 8.
 * We simulate this by setting PC = inst_addr + 8 before exec_arm so that
 * all PC-relative ops (LDR, ADR, B, BL) work correctly without any special
 * casing inside exec_arm.
 *
 * After exec_arm:
 *   - If no branch was taken: PC is still inst+8 → restore to inst+4
 *     (sequential execution, next instruction)
 *   - If a branch was taken: exec_arm set PC to target → leave it
 *
 * Thumb pipeline: PC = inst+4 during execution. Same logic, smaller delta.
 */
int arm9_step(ARM9 *cpu) {
    int cyc;
    uint32_t inst_addr;

    if (T_FLAG) {
        inst_addr = PC;
        uint16_t i = r16(cpu, inst_addr);
        PC = inst_addr + 4;             /* Thumb pipeline: PC = inst+4 */
        exec_thumb(cpu, i);
        if (PC == inst_addr + 4)        /* sequential: advance to next */
            PC = inst_addr + 2;
        cyc = insn_cycles_thumb(i);
    } else {
        inst_addr = PC;
        uint32_t i = r32(cpu, inst_addr);
        PC = inst_addr + 8;

        /* Game init trace: log BL calls and returns */
        /* NULL pointer trap: when game code (LR in BOOT.BIN) calls through
         * a NULL pointer into BSS, auto-return to skip the call.
         * Only active for calls FROM BOOT.BIN code (0x10C00000+). */
        /* Trace init BLs and scheduler */
        {
            /* Trace memcpy: focus on CMN+BNE at 0x328-0x330 */
            if (inst_addr >= 0x324 && inst_addr <= 0x334) {
                static int fn_log = 0;
                if (fn_log < 10) {
                    printf("[FN2B4] PC=%08X insn=%08X R2=%08X CPSR=%08X Z=%d\n",
                           inst_addr, i, cpu->r[2], CPSR, (CPSR >> 30) & 1);
                    fn_log++;
                }
            }
            if (inst_addr == 0x340) {
                static int mcpy_log = 0;
                if (mcpy_log < 3) {
                    printf("[MCPY] STR: *0x%08X = 0x%08X (iter %d)\n",
                           cpu->r[3], cpu->r[2], mcpy_log);
                    mcpy_log++;
                }
            }
            /* Trace when 0x10010234 gets written */
            if (inst_addr == 0x340 && cpu->r[3] == 0x10010234) {
                printf("[MCPY] *** Writing scheduler: *0x10010234 = 0x%08X ***\n", cpu->r[2]);
            }
            /* Check ram[0x10234] after each init BL returns */
            if (inst_addr >= 0x118 && inst_addr <= 0x138) {
                uint32_t chk = cpu->mem_read32(cpu->mem_ctx, 0x10010234);
                static uint32_t last_chk = 0;
                if (chk != last_chk) {
                    printf("[CHK] at PC=%08X: ram[0x10234]=%08X\n", inst_addr, chk);
                    last_chk = chk;
                }
            }
            if (inst_addr >= 0x118 && inst_addr <= 0x138 && (i & 0x0F000000) == 0x0B000000) {
                static int bl_log = 0;
                if (bl_log < 10) {
                    int32_t off = i & 0xFFFFFF;
                    if (off & 0x800000) off |= (int32_t)0xFF000000;
                    uint32_t target = inst_addr + 8 + (uint32_t)(off * 4);
                    printf("[INIT-BL] PC=%08X → BL 0x%08X\n", inst_addr, target);
                    bl_log++;
                }
            }
            /* Log scheduler entry and BL calls from init */
            if (inst_addr == 0x10010234) {
                printf("[SCHED] Entry: insn=%08X SP=%08X\n", i, cpu->r[13]);
            }
            /* Trace BLs from scheduler init code */
            if (inst_addr >= 0x10010240 && inst_addr <= 0x10010290 &&
                (i & 0x0F000000) == 0x0B000000) {
                static int sbl = 0;
                if (sbl < 10) {
                    int32_t boff = i & 0xFFFFFF;
                    if (boff & 0x800000) boff |= (int32_t)0xFF000000;
                    uint32_t tgt = inst_addr + 8 + (uint32_t)(boff * 4);
                    printf("[SCHED-BL] 0x%08X → BL 0x%08X R0=%08X R1=%08X\n",
                           inst_addr, tgt, cpu->r[0], cpu->r[1]);
                    sbl++;
                }
            }
            /* Track PLL wait loop (VA 0x18E0, not 0x100018E0) */
            if (inst_addr == 0x18E0 || inst_addr == 0x100018E0) {
                static int pll_log = 0;
                if (pll_log < 3) {
                    printf("[PLL-WAIT] R0=%08X R3=%08X SP=%08X\n",
                           cpu->r[0], cpu->r[3], cpu->r[13]);
                    pll_log++;
                }
            }
            if (inst_addr == 0x18E4 || inst_addr == 0x100018E4) {
                static int pll_ret = 0;
                if (pll_ret < 3)
                    printf("[PLL-RET] R0=%08X (bit7=%d)\n", cpu->r[0], (cpu->r[0]>>7)&1);
                pll_ret++;
            }
            /* Track when game code (0x10C00000+) transitions to low addr */
            {
                static int game_started = 0, crash_log = 0;
                if (inst_addr >= 0x10C00000 && inst_addr < 0x10E00000)
                    game_started = 1;
                if (game_started && !crash_log && inst_addr < 0x10001000 &&
                    inst_addr != 0x18 && inst_addr != 0x1C &&
                    !(inst_addr >= 0xD00 && inst_addr < 0xE00)) { /* ROM FIQ handler */
                    printf("[GAME-CRASH] PC=0x%08X insn=0x%08X LR=0x%08X SP=0x%08X CPSR=0x%08X\n",
                           inst_addr, i, cpu->r[14], cpu->r[13], CPSR);
                    crash_log = 1;
                }
            }
            /* Trace task_start function */
            if (inst_addr >= 0x10085E50 && inst_addr <= 0x10085F50) {
                static int ts_log = 0;
                if (ts_log < 30) {
                    printf("[TSTART] PC=%08X insn=%08X R0=%08X R1=%08X R3=%08X SP=%08X\n",
                           inst_addr, i, cpu->r[0], cpu->r[1], cpu->r[3], cpu->r[13]);
                    ts_log++;
                }
            }
        }

        /* Catch BLX to NULL (address 0) — game code calls NULL func pointers.
         * Also catch any instruction fetch at low SDRAM addresses that
         * shouldn't be executing (0x10000000-0x10001000 = kernel vectors) */
        if ((inst_addr == 0 || (inst_addr >= 0x10000000 && inst_addr < 0x10000100))
            && cpu->null_trap_enabled) {
            static int null_blx = 0;
            if (null_blx < 10)
                printf("[NULL-BLX] PC=0x%08X LR=0x%08X R0=0x%08X SP=0x%08X\n",
                       inst_addr, cpu->r[14], cpu->r[0], cpu->r[13]);
            null_blx++;
            cpu->r[0] = 0;
            if (cpu->r[14] != 0 && cpu->r[14] != inst_addr)
                PC = cpu->r[14] & ~3u;
            else {
                /* Can't return — go to idle, let scheduler take over */
                PC = 0x10FFF000;
                cpu->cpsr = 0x000000D3; /* SVC, IRQ off */
            }
            cpu->cycles += 1;
            return 1;
        }

        /* HLE service intercept: check BEFORE null trap (services are at zero-init addresses) */
        if (cpu->hle_intercept && cpu->hle_intercept(cpu->hle_ctx, inst_addr)) {
            cpu->cycles += 1;
            return 1;
        }

        if (i == 0 && cpu->null_trap_enabled && inst_addr >= 0x10000100 && inst_addr < 0x10C00000) {
            static int ntp = 0;
            if (ntp < 200)
                printf("[NULL-TRAP] 0x%08X (LR=0x%08X R0=0x%08X)\n", inst_addr, cpu->r[14], cpu->r[0]);
            ntp++;
            cpu->r[0] = 0;
            PC = cpu->r[14] & ~3u;
            cpu->cycles += 1;
            return 1;
        }

        PC = inst_addr + 8;
        exec_arm(cpu, i);
        /* Detect branch-forward-by-0: if PC == inst_addr+8 AND the
         * instruction is a B/BL, exec_arm intentionally set PC there.
         * For non-branch instructions, PC==inst_addr+8 means no change. */
        if (PC == inst_addr + 8) {
            if ((i & 0x0E000000) == 0x0A000000 && cond_ok(cpu, i >> 28))
                { /* branch was taken, PC is correct */ }
            else
                PC = inst_addr + 4;
        }
        cyc = insn_cycles_arm(i);
    }
    cpu->cycles += (uint64_t)cyc;
    return cyc;
}

/* Run until at least 'cycles' CPU cycles have elapsed */
void arm9_run(ARM9 *cpu, int cycles) {
    int done = 0;
    while (done < cycles)
        done += arm9_step(cpu);
}

/* IRQ: called between instructions.
 * PC = address of next instruction to execute.
 * LR_irq = address to return to after SUBS PC,LR,#4 = next instruction + 4.
 * (Standard ARM: LR_irq = interrupted_instruction + 8) */
void arm9_irq(ARM9 *cpu) {
    if(CPSR&ARM9_FLAG_I) return;
    save_bank(cpu,CPSR&0x1F);
    cpu->spsr_irq = CPSR;
    cpu->r14_irq  = PC + 4;   /* LR_irq so that SUBS PC,LR,#4 returns to PC */
    CPSR = (CPSR & ~0x3Fu) | ARM9_MODE_IRQ | ARM9_FLAG_I;
    load_bank(cpu, ARM9_MODE_IRQ);
    PC = vec_base(cpu) + 0x18;
}

void arm9_fiq(ARM9 *cpu) {
    static int fiq_log = 0;
    if (fiq_log < 200) {
        printf("[ARM9-FIQ] CPSR=%08X F=%d → %s PC=%08X\n",
               CPSR, (CPSR&ARM9_FLAG_F)?1:0, (CPSR&ARM9_FLAG_F)?"BLOCKED":"DELIVER", PC);
    }
    if(CPSR&ARM9_FLAG_F) { fiq_log++; return; }
    if (fiq_log < 200) {
        printf("[ARM9-FIQ] Delivering: PC=%08X→0x%08X CPSR=%08X→FIQ\n",
               PC, vec_base(cpu)+0x1C, CPSR);
        fiq_log++;
    }
    save_bank(cpu,CPSR&0x1F);
    cpu->spsr_fiq = CPSR;
    cpu->r14_fiq  = PC + 4;
    CPSR = (CPSR & ~0x3Fu) | ARM9_MODE_FIQ | ARM9_FLAG_I | ARM9_FLAG_F;
    load_bank(cpu, ARM9_MODE_FIQ);
    PC = vec_base(cpu) + 0x1C;
}

/* SWI: called from exec_arm when PC = inst+8.
 * LR_svc = address of instruction after SWI = inst+4 = PC-4. */
void arm9_swi(ARM9 *cpu) {
    save_bank(cpu,CPSR&0x1F);
    cpu->spsr_svc = CPSR;
    cpu->r14_svc  = PC - 4;   /* inst+4 = return address after SWI */
    CPSR = (CPSR & ~0x3Fu) | ARM9_MODE_SVC | ARM9_FLAG_I;
    load_bank(cpu, ARM9_MODE_SVC);
    PC = vec_base(cpu) + 0x08;
}

void arm9_undef(ARM9 *cpu) {
    static int undef_count = 0;
    if (undef_count < 20) {
        uint32_t bad_pc = PC - 8;
        uint32_t insn = cpu->mem_read32(cpu->mem_ctx, bad_pc);
        fprintf(stderr, "[UNDEF] #%d PC=0x%08X insn=0x%08X LR=0x%08X\n",
                undef_count, bad_pc, insn, cpu->r[14]);
    }
    undef_count++;

    /* ROM boot recovery: if UNDEF fires with BOOT.BIN pre-loaded,
     * the ROM failed to load BOOT.BIN via ATAPI. Redirect to BOOT.BIN
     * entry at 0x10C00010 (V.Flash BOOT format trampoline).
     * Also trigger ROM→RAM copy callback if set (for µMORE kernel). */
    if (undef_count == 1 && (CPSR & 0x1F) == ARM9_MODE_SVC) {
        /* ROM init hit a NULL function pointer (garbage from SDRAM calibration).
         * Copy ROM kernel code to RAM to fix this, then RESUME ROM init
         * (don't redirect to BOOT.BIN). This allows ROM init's task
         * registration code to run after disc load + BSS clear. */
        if (cpu->undef_callback)
            cpu->undef_callback(cpu->mem_ctx);
        /* NOP the faulting instruction area so it becomes harmless */
        cpu->mem_write32(cpu->mem_ctx, PC - 8, 0xE1A00000); /* NOP at fault addr */
        fprintf(stderr, "[UNDEF] Patched + resuming ROM init at 0x%08X\n", PC - 8);
        PC = PC - 8; /* re-execute the (now NOP'd) instruction */
        return;
    }

    save_bank(cpu,CPSR&0x1F);
    cpu->spsr_und = CPSR;
    cpu->r14_und  = PC - 4;
    CPSR = (CPSR & ~0x3Fu) | ARM9_MODE_UND | ARM9_FLAG_I;
    load_bank(cpu, ARM9_MODE_UND);
    PC = vec_base(cpu) + 0x04;
}

uint32_t arm9_get_pc(ARM9 *cpu) { return PC; }
