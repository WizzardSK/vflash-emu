# V.Flash Emulator

First emulator for the VTech V.Flash (V.Smile Pro) educational console (2006).
No other emulator exists for this system.

## Hardware

| Component | Details |
|-----------|---------|
| CPU | LSI Logic ZEVIO 1020 SoC, ARM926EJ-S @ 150MHz |
| RAM | 16MB SDRAM |
| Media | CD-ROM, ISO 9660, no copy protection |
| Video | Motion JPEG (.mjp), raw bitmap (.ptx) |
| Audio | PCM WAV (.snd) |
| OS | µMORE v4.0 RTOS (on-disc per game, no boot ROM needed) |

## Status

### CPU — ARM926EJ-S (ARMv5TE)
- Full ARM32 and Thumb instruction sets
- CLZ, LDRD/STRD, DSP multiply (SMLA/SMUL/SMLAL/SMULW/QADD/QSUB)
- SWI/IRQ/FIQ/UNDEF exceptions with correct register banking
- CPI timing model; CP15 coprocessor (HIVEC, cache stubs)

### Memory map (estimated ZEVIO 1020)
| Address | Size | Region |
|---------|------|--------|
| `0x00000000` | 16MB | RAM mirror (alias of 0x10000000) |
| `0x0FFE0000` | 128KB | Internal SRAM / TCM |
| `0x10000000` | 16MB | Main SDRAM |
| `0x80000000` | 512B | IRQ controller + Timer0/1 |
| `0x80001000` | 4KB | CD-ROM DMA |
| `0x80002000` | 4KB | Video DMA (framebuffer + JPEG decode) |
| `0x80003000` | 4KB | Audio DMA |
| `0x80004000` | 4KB | GPIO / buttons (active-low) |
| `0x80005000` | 4KB | UART stub (TX→stderr as `[UART]`) |
| `0xFFFF0000` | 64KB | High vector mirror (ARM HIVEC) |

### MMU
- ARM926EJ-S two-level page table translation
- L1 section (1MB) and coarse page table (4KB/64KB pages)
- Automatically enabled when game code sets CP15 control register

### Other subsystems
- ISO 9660 with full subdirectory traversal and recursive disc-wide search
- BIN/CUE disc image support (auto-detects raw 2352-byte sectors vs 2048-byte ISO)
- Motion JPEG decoding (libjpeg), `.ptx` bitmap auto-detection
- V.Flash BOOT format parser (magic, load address, entry point, REL table)
- HLE boot: scans disc for `BOOT.BIN`/`GAME/BOOT.BIN`/etc., detects ELF / V.Flash BOOT / raw ARM
- SDL2 display + audio output

## Build

```bash
sudo apt install libsdl2-dev libjpeg-dev
make
make testrom.bin   # build ARM32 test ROM (no cross-compiler needed)
```

## Run

```bash
./vflash game.iso                  # ISO image
./vflash game.cue                  # BIN/CUE image (auto-detected)
./vflash game.bin                  # raw BIN (auto-detected if >4MB)
./vflash --debug game.iso          # I/O trace on stderr
./vflash --dbg game.iso            # debugger, paused at boot
./vflash --dbg-run game.iso        # debugger, running
./vflash --headless game.iso       # no display
./vflash --scale 3 game.iso        # 3× window
```

## Debugger (stdin commands, F2 = pause/resume)

```
s [N]      step N instructions    c         continue
n          step over              b <addr>  set breakpoint
bc <a>|all clear BP(s)           bl        list BPs
r          registers              pc        current instruction
d <a> [N]  disassemble            m <a> [N] memory dump
mw <a> [N] word dump              bt        stack dump
setreg r0=val                     mwrite a=v
q          quit
```

## Tools

```bash
./disc_analyze  game.iso
./disc_compare  g1.iso g2.iso ...
./mjp_extract   game.iso frames/
./ptx_extract   game.iso images/
./testrom_gen                      # generates testrom.bin
```

## Test ROM

`testrom.bin` — bare ARM32 binary (716 bytes), no OS needed. Tests:
arithmetic, CLZ, MUL, timer read, input read, video DMA.

Expected output on stderr:
```
[UART] VFlash Test ROM v1.0
[UART] PASS: arithmetic
[UART] PASS: CLZ
[UART] PASS: MUL
[UART] PASS: timer
[UART] PASS: input=0x...
[UART] PASS: video DMA
[UART] ALL TESTS DONE
```

## Controls

| Key | Action |
|-----|--------|
| Arrow keys | D-Pad |
| Z/X/C/V | Red/Yellow/Green/Blue |
| Enter | OK |
| F1 | Toggle I/O trace |
| F2 | Debugger pause/resume |
| F11 | Fullscreen |
| Esc | Quit |

## BOOT.BIN format

V.Flash game discs contain a `BOOT.BIN` with a custom header:

| Offset | Size | Field |
|--------|------|-------|
| 0x00 | 4 | Magic `"BOOT"` |
| 0x04 | 4 | File size |
| 0x08 | 4 | Load address (e.g. `0x10C00000`) |
| 0x0C | 4 | Entry point offset (typically `0x10`) |
| 0x10 | 4 | `LDR PC,[PC,#-4]` trampoline |
| 0x14 | 4 | Real entry point address |
| 0x18 | 4 | `"REL\0"` relocation marker |
| 0x1C | 4 | Relocation table address |
| 0x20 | 4 | Relocation table size |

The entire file is loaded at the specified load address. The entry point trampoline at offset 0x10 jumps to the real init code. REL table processing is not yet implemented.

## Current limitations

- **No video output yet** — CPU boots and runs init code (MMU setup, page table, CP15 config) but game code hasn't reached framebuffer writes
- REL (relocation table) processing not implemented — may be needed for some games
- I/O register map is estimated; may need adjustment with real game discs
- Only one game tested (Spider-Man: Countdown to Doom)

## Notes

- Games ship with µMORE v4.0 RTOS on disc — no boot ROM dump needed
- No copy protection; games run from CD-R on real hardware
- The ZEVIO 1020 SoC memory map is estimated from code analysis — corrections welcome
