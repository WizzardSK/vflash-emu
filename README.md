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

**Complete natural boot flow** — ROM → µMORE RTOS → warm reboot → BOOT.BIN init → game code.
BSS service dispatch table populated with real code. Game code executes from BOOT.BIN.

Games tested: Cars, SpongeBob, Scooby-Doo, Disney Princess, The Incredibles, Spider-Man.

### Boot Flow (complete!)

| Step | Status |
|------|--------|
| ROM boot + flash remap | ✅ |
| SDRAM calibration | ✅ Passes naturally |
| 7 init functions (PLL, memcpy, BSS, MMU, cache) | ✅ All native |
| µMORE scheduler | ✅ Active at 0x10010234 |
| Heap init (1MB free-list) | ✅ 0x103596B0 |
| Task registration ($TCB) | ✅ 0x100AC0E0 |
| Init task execution | ✅ RTC wait, peripheral setup |
| **Warm reboot** | ✅ Write 2 to 0x900A0008 |
| **BOOT.BIN init** | ✅ At 0x10C00010, callback → idle |
| **BSS services populated** | ✅ Real LDR instructions! |
| **Game init (0x10C16CB8)** | ✅ Executes with real services |
| **Game code (0x10CAAD84)** | 🔧 Runs but some NULL pointers |

### Remaining for gameplay

Game code runs from BOOT.BIN with real µMORE services but hits 3 NULL function
pointer calls in dynamically allocated service structures. Need to trace which
specific service registrations are missing from BOOT.BIN init.

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
  → init task (0x100113DC) → RTC wait → warm reboot!
  → BOOT.BIN init (0x10C00010) → BSS services populated
  → game init (0x10C16CB8) → game code (0x10CAAD84)
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
