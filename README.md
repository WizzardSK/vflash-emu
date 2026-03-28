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

**VFF tile decompression working, scene dispatch table decoded** — µMORE v4.0 RTOS
boots, game task launched, native kernel service dispatch through vtable, CD-ROM
init succeeds natively. VFF scene data loaded from disc, scene init populates 58-entry
dispatch table with render layer callbacks. 65 scene callbacks execute natively,
decompressing 1.2MB of 8bpp tile pixel data (29 tiles, 512×256 to 128×16). Tilemap
decompressed at 0x101B8000. Decompressed tiles rendered to LCD framebuffer. Game loop
runs with all engine subsystems active. 73 unrelocated functions (100KB) recovered
from BOOT.BIN. PTX sprite artwork from disc displayed on screen.

Games tested: Cars, SpongeBob, Scooby-Doo, Disney Princess, The Incredibles, Spider-Man.

### Boot Flow

| Step | Status |
|------|--------|
| ROM boot + flash remap | ✅ |
| SDRAM calibration | ✅ Passes naturally |
| 7 init functions (PLL, memcpy, BSS, MMU, cache) | ✅ All native |
| µMORE scheduler + heap (1MB) | ✅ Active at 0x10010234 |
| Init task → warm reboot #1 | ✅ 0x100113DC → 0x900A0008 |
| ROM warm boot → BOOT.BIN → warm reboot #2 | ✅ |
| BOOT.BIN init → service entry → µMORE init | ✅ Registers services 2/3/4 |
| **µMORE version banner** | ✅ "µMORE v4.0 SDK ARM9T version" |
| **Init task completes** | ✅ Peripheral setup → B . halt |
| **BOOT.BIN bootstrap → warm reboot #3** | ✅ Callback 0x1880 |
| **ROM warm boot → scheduler idle** | ✅ 0x10A219xx loop |
| **Secondary VIC (0xDC000000)** | ✅ Timer IRQ clear |
| **Nucleus TCB discovery** | ✅ Relaxed $QCB scan (no $QBT required) |
| **Game task launch** | ✅ Correct entry via TCB (not service entry) |
| **Kernel service vtable** | ✅ Initialized from ROM, native dispatch works |
| **CD-ROM init** | ✅ Native through kernel vtable, disc query succeeds |
| **VFF scene loading** | ✅ ARM code + graphics + tilemap loaded to RAM |
| **VFF scene init** | ✅ 58-entry dispatch table (graphics/audio/render layers) |
| **VFF scene builder** | ✅ 65 callbacks decompress 1.2MB tile data (29 tiles) |
| **VFF tile decompression** | ✅ 8bpp tiles decompressed, rendered to framebuffer |
| **VFF tile display** | 🔧 Grayscale rendering (palette lookup not yet found) |
| **Game loop** | ✅ tick→sync→render, all subsystems active |
| **NULL entity protection** | ✅ Dummy vtable + read trap for safe execution |
| **Native render function** | ✅ Copied from BOOT.BIN (10D615FC→10B265E8) |
| **HLE render_init** | ✅ 3-step state machine bypassed, all flags set |
| **VBlank toggle** | ✅ MMIO 0x900A0018 bit25 triggers entity draw path |
| **Display list vector** | ✅ 100 entities, vtable[2] draw called 5000×/session |
| **73 unrelocated functions** | ✅ 100KB code block copied from BOOT.BIN |
| **Render processing** | 🔧 200K budget, entity draw calls work but stubs return 0 |
| **PTX display** | ✅ Game artwork (XBGR1555 sprites) on screen |

### Remaining for gameplay

**VFF tile decompression working**: Scene callbacks (65 total) decompress sec[1]
compressed tile data into 1.2MB of 8bpp indexed pixels at 0x10240000 (29 tiles).
Tilemap at 0x101B8000 (64-byte stride) maps tile indices to screen positions.
Dispatch table (58 entries in 0x30-byte groups) decoded: graphics layers with
compressed sizes, audio channels (22050Hz stereo), and render layers with tile
data pointers + X/Y position offsets.

**Color palette needed**: Tiles use 8bpp palette indices but the 256-color lookup
table has not been located. Engine writes palette to PL111 LCD controller during
render_processing which we don't fully execute. Currently rendered as grayscale.

**Tilemap composition**: Decompressed tilemap + tile pixel data need to be
combined to render full scene backgrounds. Render layer X/Y offsets available
in dispatch table at 0x104F8D64+ (values like 64, 192, 32 = pixel positions).

**Native alloc**: 10A775E0 gets stuck in uninitialized game heap. HLE'd with
separate bump area at 0x390000. 10A77648 (variant) HLE'd at 0x320000.

**Key HLE components**:
- Game alloc (10A775E0, 10A77648): HLE bump allocators at 0x390000/0x320000
- NULL entity read trap: returns ready(1) for [0x00-0x1F] (breaks ALL polling loops)
- Dummy vtable at 0x10310800: 64 ARM stubs (MOV R0,#0; BX LR)
- Engine wait skip (10A20B30): breaks BEQ loop waiting for entity state
- 73 unrelocated functions: 100KB block copy (10D48B20→10B0DB0C)
- HLE render_init (10A881F0): sets 5 ready flags, skips RTOS callbacks
- VBlank toggle: MMIO 0x900A0018 bit25 flips on each read
- Display list vector: 100 entity pointers at 0x10B90000
- Render budget: 200K instructions per frame, force return if stuck
- Skip task_notify (10A6FC60): prevents RTOS tree traversal stuck
- State machine fix: force [param_1+0x104] = 0x84 (done) at 109D2754
- Game mode fix: intercept [10B902C0] reads, return 3 (not 4=error)
- ATAPI sense fix: no error (0x00) instead of unit attention (0x06)
- Kernel vtable: 7 function pointers from ROM at [1000C76C+0x2C..+0x44]

**VFF format decoded**: header (0x60 bytes) with entry point (+0x10), init callback
(+0x20), section count (+0x2C). sec[0]=ARM scene script (up to 1.8MB), sec[1]=tile
descriptors (29×44B: width/height/8bpp) + compressed tile pixel data, sec[2]=compressed
tilemap. Dispatch table at header+0x10: 0x30-byte groups of {graphics_layer, audio,
render_layer} with IDs, compressed sizes, callbacks, tile data pointers, and XY offsets.

**Ghidra RE completed**: full game task entry, render pipeline, VFF scene format,
BOOT.BIN CD driver (10C21680), kernel service dispatch, game engine object system.

**Nucleus RTOS** confirmed (NU_ API). TCB: `$QCB`/`$QBT`, entry +0x2C, stack +0x30/+0x34.
**Firebird interrupt controller** at 0xDC000000. Tested 6 games.

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
| Scheduler check 2-ptr | FUN_10a08ed8 checks two pointers | Intercept [0x10B606C0] return 1 |
| Kernel sleep 98K loop | Game init stuck in polling sleep | Skip 0x10007304 and 0x100086F0 |
| Invalid ptr deref | 10A355A4/109D2754 loop on garbage | Skip when R0 outside RAM range |
| Auto-launch too late | Frame 200 unreachable (delay loop) | Changed to frame 75 |
| IRQ handler stuck | 0x900D0018 returns wrong value | Return 0 during game phase |
| Render fn not relocated | BL 10B265E8 hit data, not code | Copy 100KB from BOOT.BIN (73 functions) |
| Render_init stuck | RTOS task queue tree loop (10A8CDE4) | HLE: set 5 ready flags, return |
| Render_proc infinite | Entity draw stubs never finish | 200K instruction budget per frame |
| No VBlank signal | Render never enters draw path | Toggle bit25 of MMIO 0x900A0018 |
| task_notify stuck | RTOS priority tree traversal | Skip 10A6FC60 during game phase |
| GPU write intercept wide | All ctx writes forced to 1 | Narrow to single byte [10BE3CA0] |
| Native alloc stuck | Game heap not initialized | HLE bump alloc at 0x390000 |
| VFF entry hardcoded | Only one scene worked | Dynamic entry scan (LDR+PUSH pattern) |
| VFF section offset | Data shifted by 0x3A0 | Header is 0x60 bytes, loader uses 0x400 padded |
| Scene callbacks stub | Tiles not decompressed | Call all 65 dispatch table callbacks |

### Subsystems
- ATAPI CD-ROM: INQUIRY, READ(10), READ CD, READ TOC, READ CAPACITY, MODE SENSE
- MJP video: byte-swap, byte-stuffing restore, in-place decode, scaling
- IMA ADPCM + PCM WAV audio, 22050→44100Hz stereo resampling
- PTX: XBGR1555, 512px stride, interleaved scene/sprite
- VFF: "vfD0" container, 3 sections (ARM script + tile descriptors/pixels + tilemap)
- VFF tile decompression: scene callbacks decompress 8bpp tiles (29 tiles, 1.2MB)
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
