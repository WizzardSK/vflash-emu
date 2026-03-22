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
| OS | µMORE v4.0 RTOS |
| Boot ROM | 70004.bin (2MB, optional — enables real boot flow) |

## Controls

| Key | Action |
|-----|--------|
| Left / Right | Browse PTX image gallery (88 images) |
| Up / Down | Browse VFF game scenes (20 scenes) |
| Enter | Load selected VFF scene |
| Z (Red) | Next MJP video (20 videos) |
| X (Yellow) | Stop video playback |
| F11 | Toggle fullscreen |
| Esc | Quit |

Background voice/SFX audio plays automatically from 561 WAV files on disc.

## Status

**Asset browser working** — displays all game content: images, video, scenes, audio.
Boot flow reaches game code, but interactive gameplay requires µMORE RTOS HLE (in progress).

Games tested: Cars, SpongeBob, Scooby-Doo, Disney Princess, The Incredibles, Spider-Man.

### Working features

- **PTX gallery** — 88 XBGR1555 images (splash screens, backgrounds, characters)
- **MJP video** — 20 Motion JPEG cutscenes with IMA ADPCM audio, fullscreen
- **VFF scenes** — 20 game scene modules with embedded graphics
- **WAV audio** — 561 voice lines and sound effects, auto-play
- **Boot ROM flow** — flash remap, SDRAM skip, MMU page table, kernel loop
- **Game code execution** — BOOT.BIN init + game init run stably

### CPU — ARM926EJ-S (ARMv5TE)
- Full ARM32 and Thumb instruction sets
- CLZ, LDRD/STRD, DSP multiply (SMLA/SMUL/SMLAL/SMULW/QADD/QSUB)
- SWI/IRQ/FIQ/UNDEF exceptions with correct register banking
- CPI timing model; CP15 coprocessor (MMU, cache, HIVEC, domain)

### MMU
- ARM926EJ-S two-level page table translation
- L1 section (1MB) and coarse page table (4KB/64KB pages)
- TLB cache (4096-entry direct-mapped, ~30% speedup)
- Full 4096-section identity map (matching real ROM init at 0x704)

### Boot flow (with 70004.bin)
1. ROM boot at PA 0 → flash remap → SDRAM calibration skipped
2. ROM/kernel/modules copied to SDRAM (matching real init functions)
3. MMU page table built, BSS cleared, PA 0 remapped to separate buffer
4. Kernel loop → timer + IRQ enabled → BOOT.BIN init → game init
5. Game code runs at 0x10CAA2xx (BOOT.BIN main loop)

### Physical memory map (ZEVIO 1020)
| Address | Size | Region |
|---------|------|--------|
| `0x00000000` | 8KB | Low RAM (vector table, separate from SDRAM) |
| `0x0FFE0000` | 128KB | Internal SRAM / TCM |
| `0x10000000` | 16MB | Main SDRAM |
| `0x8FFF0000` | 4KB | DMA controller |
| `0x90000000` | | I/O: timers, VIC, UART, GPIO, PMU |
| `0xAA000000` | | ATAPI CD-ROM controller |
| `0xB8000000` | 2MB | NOR flash (boot ROM mirror) |
| `0xC0000000` | | PL111 LCD controller |

### ATAPI CD-ROM Controller
- Full PACKET command protocol at `0xAA000000`
- Commands: INQUIRY, IDENTIFY, READ(10), READ CD, READ TOC, READ CAPACITY, TEST UNIT READY, REQUEST SENSE, MODE SENSE
- ZEVIO IDE wrapper registers for DMA and interrupt control

### Video — MJP (Motion JPEG)
- MIAV container with interleaved audio/video chunks
- 16-bit byte-swap + automatic byte-stuffing restoration
- SOS-only P-frame support (reuses I-frame DHT/DQT tables)
- Nearest-neighbor scaling to 320x240, video loops at end
- libjpeg decode with partial frame tolerance

### Audio
- IMA ADPCM from MIAV `01wb` chunks (22050Hz mono → 44100Hz stereo)
- PCM WAV playback (16-bit, 22050Hz mono, resampled to 44100Hz stereo)
- SDL2 audio output with ring buffer

### PTX — Static Images
- XBGR1555 pixel format, 44-byte header, 512px stride
- Interleaved scene/sprite layout (even rows = scene, odd rows = sprite)
- Scaled to fill 320x240 display

### VFF — Game Scene Modules
- "vfD0" container with 3 sections: code + framebuffer + data
- Sections loaded to specific RAM addresses
- Entry point and callback pointers in header

### Other subsystems
- ISO 9660 with full subdirectory traversal and recursive search
- BIN/CUE disc image support (auto-detects raw 2352-byte vs 2048-byte)
- V.Flash BOOT format parser (magic, load address, entry point, REL table)
- HLE boot with disc-wide file search
- Non-destructive NULL trap for BSS function pointers
- HLE service intercept framework for µMORE RTOS
- SDL2 display + audio output

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

Boot ROM (70004.bin) is auto-detected from current directory or `../vflash-roms/`.

## Debugger (stdin commands, F2 = pause/resume)

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
./disc_analyze  game.iso
./disc_compare  g1.iso g2.iso ...
./mjp_extract   game.iso frames/
./ptx_extract   game.iso images/
```
