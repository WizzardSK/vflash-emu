# V.Flash Emulator

First emulator for the VTech V.Flash (V.Smile Pro) educational console (2006).
No other emulator exists for this system.

## Hardware

| Component | Details |
|-----------|---------|
| CPU | LSI Logic ZEVIO 1020 SoC, ARM926EJ-S @ 150MHz |
| RAM | 16MB SDRAM |
| Media | CD-ROM, ISO 9660, no copy protection |
| Video | Motion JPEG (.mjp), raw bitmap (.ptx), scene modules (.vff) |
| Audio | PCM WAV, IMA ADPCM |
| OS | µMORE v4.0 RTOS (same as TI-Nspire) |
| Boot ROM | 70004.bin (2MB, required for real boot flow) |

## Controls

| Key | Action |
|-----|--------|
| Left / Right | Browse PTX image gallery (88 images) |
| Up / Down | Browse VFF game scenes (20 scenes) |
| Z (Red) | Next MJP video (20 videos) |
| X (Yellow) | Stop video playback |
| F5 | Save screenshot (BMP) |
| F11 | Toggle fullscreen |
| Esc | Quit |

Background voice/SFX audio plays automatically from 561 WAV files on disc.

## Status

**µMORE v4.0 RTOS boots natively** — full warm reboot chain, RTOS prints version banner,
FIQ dispatch with populated vector table. Two warm reboots complete naturally.

Games tested: Cars, SpongeBob, Scooby-Doo, Disney Princess, The Incredibles, Spider-Man.

### Boot Flow

| Step | Status |
|------|--------|
| ROM boot + flash remap | ✅ |
| SDRAM calibration | ✅ Passes naturally |
| 7 init functions (PLL, memcpy, BSS, MMU, cache) | ✅ All native |
| µMORE scheduler + heap (1MB) | ✅ Active at 0x10010234 |
| Init task → warm reboot #1 | ✅ 0x100113DC → 0x900A0008 |
| BOOT.BIN bootstrap → warm reboot #2 | ✅ Init functions + 0x1880 |
| BOOT.BIN init (0x10C0011C) | ✅ Populates service entry + vectors |
| **µMORE service entry (0x109D11E0)** | ✅ Registers services, prints banner |
| **µMORE version banner** | ✅ "µMORE v4.0 SDK ARM9T version" |
| **FIQ vector dispatch** | ✅ ROM→0xD00→SDRAM[0xFF9C]→0x10A15DA4 |
| **FIQ handler (0x10A15DA4)** | 🔧 Runs but doesn't clear timer IRQ |

### Remaining for gameplay

µMORE RTOS boots fully: two warm reboots, BOOT.BIN init populates service entry and
FIQ vector table, scheduler registers services 2/3/4, prints version banner. Timer FIQ
fires and dispatches to µMORE FIQ handler at 0x10A15DA4. Handler runs but doesn't clear
the timer interrupt, causing FIQ re-delivery. Need to debug the FIQ handler's timer
clear path or fix the VIC register interface.

### Asset Browser

- **PTX gallery** — 88 XBGR1555 images, Left/Right navigation + WAV audio
- **MJP video** — 20 Motion JPEG cutscenes with IMA ADPCM, Z to switch, X to stop
- **VFF scenes** — 20 game scene modules (background color display)
- **WAV audio** — 561 voice/SFX files, auto-play during navigation

### CPU — ARM926EJ-S (ARMv5TE)
- Full ARM32 and Thumb instruction sets
- CLZ, LDRD/STRD, DSP multiply (SMLA/SMUL/SMLAL/SMULW/QADD/QSUB)
- SWI/IRQ/FIQ/UNDEF exceptions with correct register banking
- CP15 coprocessor (MMU, cache, TLB invalidate, HIVEC, domain access)
- Fixed: branch-forward-by-0 (B/BL with offset 0 to PC+8)

### MMU
- ARM926EJ-S two-level page table translation
- L1 section (1MB) and coarse page table (4KB/64KB pages)
- TLB cache (4096-entry direct-mapped, ~30% speedup)
- TLB invalidate via CP15 MCR c8
- Full 4096-section identity map (built natively by ROM init at 0x704)

### Boot Flow

```
ROM[0x00] → flash copy → flash remap (0x118)
  → PLL setup (0x1C8) → cache init (0x78C)
  → 64KB memcpy (0x288) → large memcpy (0x2B4)
  → BSS clear (0x5D0) → page table + MMU (0x704)
  → cache invalidate (0x7D0)
  → scheduler (0x10010234) → heap (1MB) → task_start ($TCB)
  → init task (0x100113DC) → warm reboot #1
  → BOOT.BIN bootstrap (0x10C00010) → warm reboot #2
  → BOOT.BIN init (0x10C0011C) → populates 0x109D11E0 + vectors
  → µMORE service entry → "µMORE v4.0 SDK ARM9T version"
  → scheduler halt → FIQ dispatch (0x10A15DA4)
```

### Physical Memory Map (ZEVIO 1020)
| Address | Size | Region |
|---------|------|--------|
| `0x00000000` | 2MB | Boot ROM (NOR flash, read-only) |
| `0x0FFE0000` | 128KB | Internal SRAM / TCM |
| `0x10000000` | 16MB | Main SDRAM |
| `0x8FFF0000` | 4KB | SDRAM / DMA controller |
| `0x90010000` | 4KB | SP804 Timer |
| `0x90020000` | 4KB | PL011 UART |
| `0x90060000` | 4KB | Watchdog |
| `0x900A0000` | 4KB | System control (reset, boot flags) |
| `0x900B0000` | 4KB | PMU (clock/PLL) |
| `0xAA000000` | | ATAPI CD-ROM controller |
| `0xB8000000` | 2MB | NOR flash mirror + write buffer |
| `0xC0000000` | | PL111 LCD controller |

### Key Bug Fixes

| Bug | Impact | Fix |
|-----|--------|-----|
| Branch-forward-by-0 | Memcpy broken → scheduler had garbage | Check B/BL encoding after PC compare |
| TLB invalidate ignored | Stale page table after MMU enable | Handle CP15 MCR c8 writes |
| VIC IntSelect = all FIQ | IRQ delivery never happened | Deliver FIQ when fiq_sel routes there |
| NULL trap wrote to RAM | Data reads got instruction bytes | Return R0=0 via LR without memory write |
| PA 0 / SDRAM aliasing | IRQ vectors corrupted by game code | Separate low_ram buffer |
| UART reg 0x14 returned 0 | Scheduler stuck in idle loop | Return bit5=1 (event present) |
| Forced vsync IRQ raise | IRQ storm after every clear | Let timer fire naturally |
| RTC constant value | Init task stuck in RTC wait loop | Increment RTC on each read |
| Sleep/wait blocking | Init task never completes | Intercept 0x10011D88, return 0 |
| Warm reboot val=2 | Reboot not recognized | Accept val=1 or val=2 |
| sched_state reset to 0 | Scheduler refused to dispatch | Force sched_state=3 on write |
| Timer IRQ during init | BOOT.BIN crashed on IRQ storm | Disable timer during BOOT.BIN init |
| NULL BLX (PC=0) | Game crash on NULL function ptr | Catch and return to caller |
| IRQ vector via low_ram | IRQ never reached handler (PA 0 = ROM) | Patch SDRAM[0xFF98] in ROM vector chain |
| BOOT.BIN overwrites IRQ | SDRAM[0xFF98] restored by init | Re-patch after BOOT.BIN init completes |
| Game init at 0x10C16CB8 | POP {PC} = immediate return | Use real entry at 0x10C16CC0 |
| Phase 100 PC range | Kernel idle at 0x10095Bxx not detected | Widen range to 0x10090000-0x10110000 |
| Event controller bit5 | Scheduler stuck in event wait | Return bit5 when timer IRQ pending |
| HLE blocked real code | Service at 0x109D11E0 intercepted | Skip HLE when memory is non-zero |
| NOR flash write verify | Flash polling loop stuck | Track last written value, return on read |
| ZEVIO timer bit0 enable | Timer didn't tick (not SP804) | Accept bit0 or bit7 as enable |
| One-shot timer re-fire | FIQ storm from cnt=0 loop | Disable timer after one-shot fires |
| µMORE FIQ routing | All IRQs routed to FIQ | Handle fiq_sel=0xFFFFFFFF correctly |
| RTC 0x8C FIQ flag | ROM FIQ handler skipped dispatch | Store writes, return for FIQ flag |

### Subsystems
- ATAPI CD-ROM: INQUIRY, READ(10), READ CD, READ TOC, READ CAPACITY, MODE SENSE
- MJP video: byte-swap, byte-stuffing restore, in-place decode, scaling
- IMA ADPCM + PCM WAV audio, 22050→44100Hz stereo resampling
- PTX: XBGR1555, 512px stride, interleaved scene/sprite
- VFF: "vfD0" container, 3 sections (code + graphics + background)
- ISO 9660 with recursive search, BIN/CUE auto-detect
- Interactive debugger with breakpoints, disassembly, memory dump

## Build

```bash
sudo apt install libsdl2-dev libjpeg-dev
make
```

## Run

```bash
./vflash game.cue                  # BIN/CUE image
./vflash game.iso                  # ISO image
./vflash --debug game.cue          # I/O trace
./vflash --dbg game.cue            # debugger, paused
./vflash --dbg-run game.cue        # debugger, running
./vflash --headless game.cue       # no display
./vflash --scale 3 game.cue        # 3x window
```

Boot ROM `70004.bin` is auto-detected from current directory or `../vflash-roms/`.

## Debugger

```
s [N]      step N instructions    c         continue
n          step over              b <addr>  set breakpoint
bc <a>|all clear BP(s)           bl        list BPs
r          registers              pc        current instruction
d <a> [N]  disassemble            m <a> [N] memory dump
bt         stack dump              setreg r0=val
q          quit
```

## Tools

```bash
./disc_analyze  game.iso           # list disc contents
./disc_compare  g1.iso g2.iso      # compare disc structures
./mjp_extract   game.iso frames/   # extract MJP video frames
./ptx_extract   game.iso images/   # extract PTX images
```
