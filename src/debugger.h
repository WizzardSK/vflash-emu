#pragma once
#include "vflash.h"

/* Return codes from dbg_frame() and dbg_check() */
#define DBG_CONTINUE  0   /* keep running */
#define DBG_PAUSE     1   /* emulator should pause (wait for next dbg_frame) */
#define DBG_QUIT      2   /* user typed 'q' — exit emulator */

/* Initialise debugger.
 * start_paused=1: break immediately at boot (useful with --dbg flag)
 * start_paused=0: run freely, break only on BP or user input */
void dbg_init(VFlash *vf, int start_paused);
void dbg_pause(VFlash *vf);   /* pause preserving breakpoints/state */
void dbg_resume(void);         /* resume preserving breakpoints/state */

/* Call once per emulator frame.
 * Returns DBG_CONTINUE, DBG_PAUSE, or DBG_QUIT. */
int  dbg_frame(VFlash *vf);

/* Call after each vflash_step() to check for breakpoints/step-N.
 * Returns DBG_CONTINUE, DBG_PAUSE, or DBG_QUIT. */
int  dbg_check(VFlash *vf);

int  dbg_is_running(void);
int  dbg_should_quit(void);
