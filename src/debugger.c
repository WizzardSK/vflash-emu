/* debugger.c — Interactive debugger for V.Flash emulator
 *
 * Commands:
 *   s [N]           step N instructions (default 1)
 *   c               continue until next breakpoint
 *   n               next — step over BL/SWI (run until PC > current)
 *   b <addr>        set breakpoint at hex address
 *   bc <addr>|all   clear breakpoint(s)
 *   bl              list breakpoints
 *   r               dump registers
 *   m <addr> [N]    dump N bytes of memory (default 64)
 *   mw <addr> [N]   dump N words of memory (default 16)
 *   mw <addr>=<val> write word to memory
 *   d <addr> [N]    disassemble N instructions (default 16)
 *   pc              show PC + disasm of current instruction
 *   w <addr>=<val>  write register or memory
 *   setreg <r>=<v>  set register r0-r15/cpsr
 *   q               quit emulator
 *   h / ?           help
 *
 * Address format: hex without 0x prefix, or 0x prefix, or decimal.
 * All output prefixed with [DBG].
 */

#include "debugger.h"
#include "vflash.h"
#include "disasm.h"
#include "arm9.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <sys/select.h>
#include <sys/time.h>

/* ---- Utilities ---- */

static uint32_t parse_addr(const char *s) {
    if (!s) return 0;
    while (isspace((unsigned char)*s)) s++;
    if (s[0]=='0' && (s[1]=='x'||s[1]=='X')) return (uint32_t)strtoul(s+2, NULL, 16);
    /* All-hex chars → hex; else decimal */
    int all_hex = 1;
    for (const char *p = s; *p; p++)
        if (!isxdigit((unsigned char)*p)) { all_hex = 0; break; }
    return all_hex ? (uint32_t)strtoul(s, NULL, 16)
                   : (uint32_t)strtoul(s, NULL, 10);
}

static const char *mode_name(uint32_t cpsr) {
    switch (cpsr & 0x1F) {
        case 0x10: return "USR";
        case 0x11: return "FIQ";
        case 0x12: return "IRQ";
        case 0x13: return "SVC";
        case 0x17: return "ABT";
        case 0x1B: return "UND";
        case 0x1F: return "SYS";
        default:   return "???";
    }
}

static void dump_regs(VFlash *vf) {
    uint32_t cpsr = vflash_get_cpsr(vf);
    int thumb = vflash_is_thumb(vf);

    fprintf(stderr, "[DBG] ---- Registers (%s %s) ----\n",
            mode_name(cpsr), thumb ? "THUMB" : "ARM");
    for (int i = 0; i < 16; i += 4) {
        fprintf(stderr, "[DBG]  r%-2d=%08X  r%-2d=%08X  r%-2d=%08X  r%-2d=%08X\n",
                i,   vflash_get_reg(vf, i),
                i+1, vflash_get_reg(vf, i+1),
                i+2, vflash_get_reg(vf, i+2),
                i+3, vflash_get_reg(vf, i+3));
    }
    fprintf(stderr, "[DBG]  CPSR=%08X  [%c%c%c%c%c%c%c]  mode=%s\n",
            cpsr,
            (cpsr>>31)&1 ? 'N' : 'n',
            (cpsr>>30)&1 ? 'Z' : 'z',
            (cpsr>>29)&1 ? 'C' : 'c',
            (cpsr>>28)&1 ? 'V' : 'v',
            (cpsr>>7) &1 ? 'I' : 'i',
            (cpsr>>6) &1 ? 'F' : 'f',
            (cpsr>>5) &1 ? 'T' : 't',
            mode_name(cpsr));
}

static void disasm_at(VFlash *vf, uint32_t addr, int n) {
    int thumb = vflash_is_thumb(vf);
    /* Detect thumb from address bit if PC is given */
    if (addr & 1) { thumb = 1; addr &= ~1u; }

    for (int i = 0; i < n; i++) {
        char buf[80];
        uint32_t cur = addr;
        if (thumb) {
            uint16_t insn = (uint16_t)(vflash_read32(vf, addr & ~3u) >> ((addr & 2) * 8));
            thumb_disasm(cur, insn, buf, sizeof(buf));
            fprintf(stderr, "[DBG]  %08X: %04X        %s\n", cur, insn, buf);
            addr += 2;
        } else {
            uint32_t insn = vflash_read32(vf, addr);
            arm_disasm(cur, insn, buf, sizeof(buf));
            fprintf(stderr, "[DBG]  %08X: %08X  %s\n", cur, insn, buf);
            addr += 4;
        }
    }
}

static void dump_mem_bytes(VFlash *vf, uint32_t addr, int n) {
    addr &= ~0xFu;
    for (int i = 0; i < n; i += 16) {
        fprintf(stderr, "[DBG]  %08X:", addr + i);
        for (int j = 0; j < 16 && i+j < n; j++) {
            if (j == 8) fprintf(stderr, " ");
            fprintf(stderr, " %02X", vflash_read8(vf, addr + i + j));
        }
        /* ASCII column */
        fprintf(stderr, "  |");
        for (int j = 0; j < 16 && i+j < n; j++) {
            uint8_t c = vflash_read8(vf, addr + i + j);
            fprintf(stderr, "%c", (c >= 0x20 && c < 0x7F) ? c : '.');
        }
        fprintf(stderr, "|\n");
    }
}

static void dump_mem_words(VFlash *vf, uint32_t addr, int n) {
    addr &= ~3u;
    for (int i = 0; i < n; i += 4) {
        if (i % 4 == 0) fprintf(stderr, "[DBG]  %08X:", addr + i * 4);
        fprintf(stderr, "  %08X", vflash_read32(vf, addr + i * 4));
        if ((i+1) % 4 == 0) fprintf(stderr, "\n");
    }
    if (n % 4) fprintf(stderr, "\n");
}

static void show_pc(VFlash *vf) {
    uint32_t pc = vflash_get_pc(vf);
    int thumb    = vflash_is_thumb(vf);
    char buf[80];
    if (thumb) {
        uint16_t insn = (uint16_t)(vflash_read32(vf, pc & ~3u) >> ((pc & 2) * 8));
        thumb_disasm(pc, insn, buf, sizeof(buf));
        fprintf(stderr, "[DBG]  PC=%08X  %04X  %s  [THUMB]\n", pc, insn, buf);
    } else {
        uint32_t insn = vflash_read32(vf, pc);
        arm_disasm(pc, insn, buf, sizeof(buf));
        fprintf(stderr, "[DBG]  PC=%08X  %08X  %s\n", pc, insn, buf);
    }
}

static void list_bps(VFlash *vf) {
    uint32_t bps[16];
    uint32_t n = vflash_bp_list(vf, bps, 16);
    if (n == 0) { fprintf(stderr, "[DBG] No breakpoints\n"); return; }
    for (uint32_t i = 0; i < n; i++)
        fprintf(stderr, "[DBG]  BP[%u] = 0x%08X\n", i, bps[i]);
}

static void print_help(void) {
    fprintf(stderr,
        "[DBG] Commands:\n"
        "[DBG]  s [N]           step N instructions (default 1)\n"
        "[DBG]  c               continue to next breakpoint\n"
        "[DBG]  n               step over (run until PC advances past call)\n"
        "[DBG]  b <addr>        set breakpoint (hex)\n"
        "[DBG]  bc <addr>|all   clear breakpoint(s)\n"
        "[DBG]  bl              list breakpoints\n"
        "[DBG]  r               dump registers\n"
        "[DBG]  m <addr> [N]    hex dump N bytes (default 64)\n"
        "[DBG]  mw <addr> [N]   dump N words (default 16)\n"
        "[DBG]  d <addr> [N]    disassemble N instrs (default 16)\n"
        "[DBG]  pc              show current instruction\n"
        "[DBG]  setreg <r>=<v>  write register (r0-r15 or cpsr)\n"
        "[DBG]  mwrite <a>=<v>  write word to memory address\n"
        "[DBG]  q               quit\n"
        "[DBG]  h/?             this help\n");
}

/* ---- Debugger state ---- */

typedef struct {
    VFlash  *vf;
    int      running;    /* 1=free-run, 0=paused */
    int      quit;
    int      step_n;     /* >0: step this many more instructions then pause */
    uint32_t run_until;  /* non-zero: run until PC == this, then pause */
} DBGState;

static DBGState g_dbg;

/* Called from main loop on every instruction when debugger is active */
int dbg_check(VFlash *vf) {
    if (g_dbg.quit) return DBG_QUIT;

    /* Step-N mode */
    if (g_dbg.step_n > 0) {
        g_dbg.step_n--;
        if (g_dbg.step_n == 0) {
            g_dbg.running = 0;
            show_pc(vf);
            return DBG_PAUSE;
        }
    }

    /* run-until mode */
    if (g_dbg.run_until && vflash_get_pc(vf) == g_dbg.run_until) {
        g_dbg.run_until = 0;
        g_dbg.running   = 0;
        show_pc(vf);
        return DBG_PAUSE;
    }

    /* Breakpoint */
    if (vflash_bp_hit(vf)) {
        fprintf(stderr, "[DBG] *** Breakpoint hit at 0x%08X ***\n", vflash_get_pc(vf));
        show_pc(vf);
        dump_regs(vf);
        g_dbg.running = 0;
        return DBG_PAUSE;
    }

    return g_dbg.running ? DBG_CONTINUE : DBG_PAUSE;
}

/* ---- Command parser ---- */

static void process_command(const char *line, VFlash *vf) {
    char cmd[32] = {0};
    char arg1[64] = {0};
    char arg2[64] = {0};

    /* Tokenise */
    int n = sscanf(line, "%31s %63s %63s", cmd, arg1, arg2);
    if (n <= 0 || cmd[0] == '#') return;

    /* s [N] — step */
    if (strcmp(cmd, "s") == 0 || strcmp(cmd, "si") == 0) {
        int count = (n >= 2) ? atoi(arg1) : 1;
        if (count < 1) count = 1;
        for (int i = 0; i < count; i++) {
            vflash_step(vf);
            if (vflash_bp_hit(vf)) {
                fprintf(stderr, "[DBG] *** Breakpoint at 0x%08X ***\n", vflash_get_pc(vf));
                break;
            }
        }
        show_pc(vf);
        return;
    }

    /* c — continue */
    if (strcmp(cmd, "c") == 0 || strcmp(cmd, "cont") == 0) {
        g_dbg.running = 1;
        fprintf(stderr, "[DBG] Running...\n");
        return;
    }

    /* n — step over (run until PC > current + instr size) */
    if (strcmp(cmd, "n") == 0) {
        uint32_t pc   = vflash_get_pc(vf);
        uint32_t next = pc + (vflash_is_thumb(vf) ? 2u : 4u);
        /* If current insn is BL/BLX, set temp BP at next, run */
        vflash_bp_set(vf, next);
        g_dbg.running   = 1;
        g_dbg.run_until = 0;
        fprintf(stderr, "[DBG] Step-over: running to 0x%08X\n", next);
        /* We return; main loop will run and hit the BP */
        return;
    }

    /* b <addr> — breakpoint */
    if (strcmp(cmd, "b") == 0 || strcmp(cmd, "break") == 0) {
        if (n < 2) { fprintf(stderr, "[DBG] Usage: b <addr>\n"); return; }
        uint32_t addr = parse_addr(arg1);
        vflash_bp_set(vf, addr);
        fprintf(stderr, "[DBG] Breakpoint set at 0x%08X\n", addr);
        return;
    }

    /* bc <addr>|all — clear breakpoint(s) */
    if (strcmp(cmd, "bc") == 0 || strcmp(cmd, "bdel") == 0) {
        if (n < 2) { fprintf(stderr, "[DBG] Usage: bc <addr>|all\n"); return; }
        if (strcmp(arg1, "all") == 0) {
            vflash_bp_clear_all(vf);
            fprintf(stderr, "[DBG] All breakpoints cleared\n");
        } else {
            uint32_t addr = parse_addr(arg1);
            vflash_bp_clear(vf, addr);
            fprintf(stderr, "[DBG] Breakpoint 0x%08X cleared\n", addr);
        }
        return;
    }

    /* bl — list breakpoints */
    if (strcmp(cmd, "bl") == 0 || strcmp(cmd, "blist") == 0) {
        list_bps(vf);
        return;
    }

    /* r — registers */
    if (strcmp(cmd, "r") == 0 || strcmp(cmd, "reg") == 0 || strcmp(cmd, "regs") == 0) {
        dump_regs(vf);
        return;
    }

    /* pc — current instruction */
    if (strcmp(cmd, "pc") == 0) {
        show_pc(vf);
        return;
    }

    /* m <addr> [N] — memory hex dump */
    if (strcmp(cmd, "m") == 0 || strcmp(cmd, "mem") == 0) {
        if (n < 2) { fprintf(stderr, "[DBG] Usage: m <addr> [N]\n"); return; }
        uint32_t addr = parse_addr(arg1);
        int bytes = (n >= 3) ? atoi(arg2) : 64;
        if (bytes < 1 || bytes > 4096) bytes = 64;
        dump_mem_bytes(vf, addr, bytes);
        return;
    }

    /* mw <addr> [N] — word dump */
    if (strcmp(cmd, "mw") == 0 || strcmp(cmd, "memw") == 0) {
        if (n < 2) { fprintf(stderr, "[DBG] Usage: mw <addr> [N]\n"); return; }
        uint32_t addr = parse_addr(arg1);
        int words = (n >= 3) ? atoi(arg2) : 16;
        if (words < 1 || words > 1024) words = 16;
        dump_mem_words(vf, addr, words);
        return;
    }

    /* mwrite <addr>=<val> — write word to memory */
    if (strcmp(cmd, "mwrite") == 0 || strcmp(cmd, "mw=") == 0) {
        char *eq = strchr(arg1, '=');
        if (!eq) { fprintf(stderr, "[DBG] Usage: mwrite <addr>=<val>\n"); return; }
        *eq = 0;
        uint32_t addr = parse_addr(arg1);
        uint32_t val  = parse_addr(eq + 1);
        vflash_write32(vf, addr, val);
        fprintf(stderr, "[DBG] [0x%08X] = 0x%08X\n", addr, val);
        return;
    }

    /* d <addr> [N] — disassemble */
    if (strcmp(cmd, "d") == 0 || strcmp(cmd, "dis") == 0 || strcmp(cmd, "disasm") == 0) {
        uint32_t addr = (n >= 2) ? parse_addr(arg1) : vflash_get_pc(vf);
        int count = (n >= 3) ? atoi(arg2) : 16;
        if (count < 1 || count > 256) count = 16;
        disasm_at(vf, addr, count);
        return;
    }

    /* setreg r<N>=<val> or setreg cpsr=<val> */
    if (strcmp(cmd, "setreg") == 0 || strcmp(cmd, "wr") == 0) {
        char *eq = strchr(arg1, '=');
        if (!eq) { fprintf(stderr, "[DBG] Usage: setreg r0=1234\n"); return; }
        *eq = 0;
        uint32_t val = parse_addr(eq + 1);
        char *rname  = arg1;
        if (rname[0] == 'r' || rname[0] == 'R') {
            int rnum = atoi(rname + 1);
            if (rnum >= 0 && rnum < 16) {
                vflash_set_reg(vf, rnum, val);
                fprintf(stderr, "[DBG] r%d = 0x%08X\n", rnum, val);
            } else { fprintf(stderr, "[DBG] Invalid register number\n"); }
        } else if (strcasecmp(rname, "pc") == 0) {
            vflash_set_reg(vf, 15, val);
            fprintf(stderr, "[DBG] PC = 0x%08X\n", val);
        } else if (strcasecmp(rname, "sp") == 0) {
            vflash_set_reg(vf, 13, val);
            fprintf(stderr, "[DBG] SP = 0x%08X\n", val);
        } else if (strcasecmp(rname, "lr") == 0) {
            vflash_set_reg(vf, 14, val);
            fprintf(stderr, "[DBG] LR = 0x%08X\n", val);
        } else {
            fprintf(stderr, "[DBG] Unknown register '%s'\n", rname);
        }
        return;
    }

    /* q — quit */
    if (strcmp(cmd, "q") == 0 || strcmp(cmd, "quit") == 0 || strcmp(cmd, "exit") == 0) {
        g_dbg.quit    = 1;
        g_dbg.running = 0;
        fprintf(stderr, "[DBG] Quitting\n");
        return;
    }

    /* h/? — help */
    if (strcmp(cmd, "h") == 0 || strcmp(cmd, "help") == 0 || strcmp(cmd, "?") == 0) {
        print_help();
        return;
    }

    /* stack trace shortcut */
    if (strcmp(cmd, "bt") == 0 || strcmp(cmd, "stack") == 0) {
        uint32_t sp = vflash_get_reg(vf, 13);
        fprintf(stderr, "[DBG] Stack dump (SP=0x%08X):\n", sp);
        dump_mem_words(vf, sp, 16);
        return;
    }

    fprintf(stderr, "[DBG] Unknown command '%s' — type h for help\n", cmd);
}

/* ---- Public interface ---- */

void dbg_init(VFlash *vf, int start_paused) {
    memset(&g_dbg, 0, sizeof(g_dbg));
    g_dbg.vf      = vf;
    g_dbg.running = !start_paused;
    if (start_paused) {
        fprintf(stderr, "[DBG] Debugger active — emulator paused\n");
        fprintf(stderr, "[DBG] Type 'h' for help, 'c' to run, 's' to step\n");
        show_pc(vf);
    } else {
        fprintf(stderr, "[DBG] Debugger active — running (Enter to pause)\n");
    }
}

/* Pause without resetting breakpoints or state */
void dbg_pause(VFlash *vf) {
    if (g_dbg.running) {
        g_dbg.running = 0;
        fprintf(stderr, "[DBG] Paused at 0x%08X\n", vflash_get_pc(vf));
        show_pc(vf);
    }
}

/* Resume without resetting state */
void dbg_resume(void) {
    if (!g_dbg.running) {
        g_dbg.running = 1;
        fprintf(stderr, "[DBG] Resumed\n");
    }
}

/* Read one line from stdin (non-blocking if possible).
 * Returns 1 if a line was read, 0 if no input available. */
static int read_line_nonblock(char *buf, int maxlen) {
    /* Use select() with timeout=0 for non-blocking check */
    fd_set fds;
    struct timeval tv = {0, 0};
    FD_ZERO(&fds);
    FD_SET(0, &fds);  /* stdin = fd 0 */
    if (select(1, &fds, NULL, NULL, &tv) <= 0) return 0;

    if (!fgets(buf, maxlen, stdin)) return 0;
    /* Strip newline */
    int len = strlen(buf);
    while (len > 0 && (buf[len-1] == '\n' || buf[len-1] == '\r'))
        buf[--len] = 0;
    return 1;
}

/* Called once per emulator frame. Returns DBG_QUIT, DBG_PAUSE, or DBG_CONTINUE. */
int dbg_frame(VFlash *vf) {
    if (g_dbg.quit) return DBG_QUIT;

    char line[256];

    if (!g_dbg.running) {
        /* Paused: prompt and wait for command (blocking) */
        fprintf(stderr, "[DBG]> ");
        fflush(stderr);
        if (fgets(line, sizeof(line), stdin)) {
            int len = strlen(line);
            while (len > 0 && (line[len-1]=='\n'||line[len-1]=='\r')) line[--len] = 0;
            process_command(line, vf);
        }
        return g_dbg.quit ? DBG_QUIT : (g_dbg.running ? DBG_CONTINUE : DBG_PAUSE);
    }

    /* Running: poll stdin for commands without blocking */
    if (read_line_nonblock(line, sizeof(line))) {
        /* Any input while running pauses */
        if (strlen(line) == 0) {
            g_dbg.running = 0;
            fprintf(stderr, "[DBG] Paused at 0x%08X\n", vflash_get_pc(vf));
            show_pc(vf);
            return DBG_PAUSE;
        }
        process_command(line, vf);
    }

    return DBG_CONTINUE;
}

int dbg_is_running(void) { return g_dbg.running; }
int dbg_should_quit(void) { return g_dbg.quit; }
