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

**µMORE RTOS boots natively** — full ROM boot with SDRAM calibration, kernel init,
heap allocation, and task registration. Asset browser for images/video/audio.

Games tested: Cars, SpongeBob, Scooby-Doo, Disney Princess, The Incredibles, Spider-Man.

### µMORE RTOS Boot (fully working)

| Step | Status |
|------|--------|
| ROM boot + flash remap | ✅ |
| SDRAM calibration | ✅ Passes (emulated SDRAM always works) |
| PLL / clock setup | ✅ |
| Kernel memcpy (ROM → SDRAM) | ✅ Three copy phases |
| BSS clear | ✅ |
| MMU page table + enable | ✅ Built natively by ROM init |
| Cache invalidate | ✅ |
| Scheduler entry (0x10010234) | ✅ |
| µMORE heap init (1MB) | ✅ Free-list allocator at 0x103596B0 |
| Task registration ($TCB) | ✅ Task at 0x100AC0E0, entry 0x100113DC |
| Timer (SP804 periodic) | ✅ 37500 cycles, IRQ delivery |
| Custom IRQ handler | ✅ Clears timer, returns to kernel |
| Scheduler active loop | ✅ Polls I/O for events |
| **Task dispatch** | 🔧 Scheduler polls but doesn't wake task |

### Remaining for gameplay

The scheduler polls peripheral status registers (0x90020014) waiting for events.
On real hardware, the timer interrupt handler wakes tasks via context switch.
Our custom IRQ handler clears the timer but doesn't call the µMORE task wakeup.
Need: either fix the VIC interrupt dispatch chain or implement task wakeup in the IRQ handler.

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
  → scheduler entry (0x10010234)
    → kernel init (0x1009CFB4) — heap 1MB
    → task_start (0x10085E50) — $TCB registered
    → scheduler idle loop (0x10095Bxx)
    → timer IRQ → custom handler → kernel active
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
