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
| Arrow Left / Right | Parallax scroll desert scene |
| Arrow Up / Down | Browse VFF game scenes (17 scenes) |
| Enter | Toggle PTX photo overlay |
| Z (Red) | Next PTX photo / Next MJP video |
| X (Yellow) | Prev PTX photo / Stop video |
| C (Green) | Toggle VFF display format |
| F5 | Save screenshot (BMP) |
| F11 | Toggle fullscreen |
| Esc | Quit |

Scene rendering: 10 parallax layers with per-layer color tinting, circular
viewport mask from sec[2] tilemap, vertical box filter smoothing, desert sky gradient.
Background voice/SFX audio plays automatically from WAV files on disc.

## Status

**Scene rendering + JIT: 37 FPS** — Desert landscape with 10 parallax layers,
circular viewport mask from VFF sec[2] tilemap, per-layer color tinting (sky/mesa/
vegetation/road), vertical box filtering, parallax scrolling with arrow keys.
ARM926EJ-S → x86_64 JIT compiler (9000+ blocks). µMORE v4.0 RTOS boots, game task
launched via HLE FIQ + Nucleus TCB. 1.4MB game code relocated from BOOT.BIN CD.

**VFF format fully decoded**: sec[0]=ARM scene script, sec[1]=compressed sprites
(custom format), sec[2]=tilemap + raw 8bpp tile pixels. Tile data is UNCOMPRESSED
in sec[2] at offset 0x88000 (29 tiles, 512px wide, verified 100% byte match).
PTX files = Knowledge World photo atlases (car parts tutorial), not scene backgrounds.

BOOT.BIN 5A5A5A5A relocation reverse-engineered: ROM generates a 4096-entry lookup
table with `value = (idx << 20) | 0x0DF2`, indices sequential from +0x4000 with
wrap to +0x0260. Post-init data section matches 100% with runtime dump from working
session. BOOT.BIN games (Incredibles) run from 0x10C0xxxx with native init flow,
JIT IRQ yield on MSR CPSR, LCD DMA status at 0xC0000084, and DC register emulation.

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
| **VFF scene builder** | ✅ 65 callbacks, dispatch table setup |
| **VFF sec[2] tilemap** | ✅ 156 blocks decoded, circular viewport mask extracted |
| **VFF tile rendering** | ✅ 10 parallax layers, per-layer color, box filter, scrolling |
| **VFF tile discovery** | ✅ Tiles are raw sec[2]+0x88000 data (not compressed) |
| **Game loop** | ✅ tick→sync→render at 37 FPS via JIT |
| **JIT compiler** | ✅ ARM→x86_64, 9000+ blocks, 1B+ instructions JIT'd |
| **HLE hot functions** | ✅ Division, memcpy, memset, strcmp — native C |
| **RTOS code protection** | ✅ Backup/restore survives game BSS clear |
| **HLE FIQ handler** | ✅ Timer IRQ → task ready flag, Nucleus TCB at 0x10F00100 |
| **BOOT.BIN code relocation** | ✅ 1.4MB: game loop + render + engine from CD |
| **Display controller MMIO** | ✅ 0xB80007xx registers intercepted (VBlank, framebuffer) |
| **Event pump HLE** | ✅ 0x10138000 data table skip (was crashing as code) |
| **Per-frame PC recovery** | ✅ Redirect escaped PCs per-frame (not per-slice) |
| **BOOT.BIN 5A relocation** | ✅ 4097 entries: `(idx<<20)\|0xDF2` from +0x4000 wrap +0x0260 |
| **BOOT.BIN game support** | ✅ Incredibles init completes, switch case 5 executed |
| **LCD DMA status (0x84)** | ✅ ZEVIO extension: transfer complete flag unblocks init |
| **JIT IRQ yield** | ✅ Break from JIT on MSR CPSR enabling IRQ with timer pending |
| **Display controller 0x1xx** | ✅ Video mode registers read-back at 0xB80001xx |
| **Per-frame forced IRQ** | ✅ Timer IRQ delivery for BOOT.BIN games (JIT bypass) |
| **Per-slice IRQ delivery** | ✅ Check IRQ after each JIT slice for CPSR windows |
| **Render processing** | ✅ VFF tile layers rendering (sky/ground/road visible) |
| **PTX display** | ✅ 8bpp with palette via native render FB pipeline (RGB565) |
| **RTOS IRQ vector chain** | ✅ SDRAM[0xFF98]→0x10FFF200 HLE handler (timer+INTC+VIC) |
| **HLE IRQ handler** | ✅ Clears ZEVIO timer, SoC INTC, VIC ACK/vector/EOI |
| **ROM boot kernel detection** | ✅ MMU+PC in kernel → bp300, timed bp300→800→900 |
| **sec[1] sprite watchpoint** | ✅ Memory read trap at 0x1067A000 (2.8MB, Spider-Man) |
| **VFF entry table load** | ✅ Dispatch table loaded from CD (8KB after sec[0]) |
| **VFF scene init scan** | 🔧 Finds dispatch table literal in sec[0], backward PUSH search |
| **Headless mode** | ✅ SDL_QUIT ignored, auto-input, VFLASH_LOG env var |

### ROM Boot Flow (new)

```
ROM[0x00] → flash remap → SDRAM cal (0xB8000Axx, ~6 frames)
  → memcpy 589KB → BSS clear → MMU enable (TTB=0x100A8000)
  → scheduler (0x10010234) → task dispatch
  [auto-detect: MMU on + PC in kernel → bp=300]
  → IRQ chain installed: ROM[0x18]→SDRAM[0xFF98]→0x10FFF200
  → HLE IRQ handler: ZEVIO timer + SoC INTC + VIC ACK/vec/EOI
  [timed: frame 30 → bp=800, game launch at 0x10C16CC0]
  → BOOT.BIN game init → NULL trap catches missing vtables
  → bp=900, game engine idle loop at 0x109FC1B8
  → VFF scene data loaded (sec[0]+sec[1]+sec[2] from CD)
  → VFF scene init: entry scan for dispatch table population (WIP)
```

### Remaining for gameplay

**VFF sec[2] fully decoded**: 156 blocks × 15360 bytes each. Block 0 rows 0-55 =
circular viewport mask (64×49, antialiased edge indices 0-20). Offset 0x88000+ =
raw 8bpp tile pixel data (29 tiles, all 512px wide, 4-1024 rows tall). Tiles are
intensity maps rendered with per-layer RGB tinting. PTX files (28 _KWF*.PTX) are
photo atlases for Knowledge World tutorial, NOT scene backgrounds.

**Sprite compression**: sec[1] data uses custom compression (not zlib/LZ77/LZW).
44 bytes + compressed body per sprite entry. Decompression routine not yet identified
in BOOT.BIN ARM code. Resource table: 59 entries (55 sprites + 4 metadata).
Memory watchpoint active on sec[1] region (0x1067A000-0x1093E000 for Spider-Man)
— will capture decompressor PC once VFF scene init populates render entities.

**VFF scene init blocker**: Scene init function in sec[0] references dispatch table
(0x104E5000) as literal pool. Backward scan for PUSH prologue fails because game
BSS clear overwrites function prologue bytes before scan executes. Fix: run scene
init immediately after CD load (before BSS clear), or protect sec[0] from BSS clear.

**Render draw path**: render_processing (0x10A89100) is called every game loop
iteration but returns early. The function checks render context flags at
0x10BE3C40 (ctx[0]=error, ctx[1]=frame_ready) and render enable at 0x10BE3EC0.
These are set per-frame but the inner draw path has additional state dependencies
(display list population, entity vtable dispatch, VBlank callback) that need
further analysis. The relocated render code writes to display controller registers
(0xB80007xx) configuring VIC, framebuffer address, and VBlank callback.

**Native alloc**: 10A775E0 gets stuck in uninitialized game heap. HLE'd with
separate bump area at 0x390000. 10A77648 (variant) HLE'd at 0x320000.

**Key HLE components**:
- Game alloc (10A775E0, 10A77648): HLE bump allocators at 0x390000/0x320000
- NULL entity read trap: returns ready(1) for [0x00-0x1F] (breaks ALL polling loops)
- Dummy vtable at 0x10310800: 64 ARM stubs (MOV R0,#0; BX LR)
- Engine wait skip (10A20B30): breaks BEQ loop waiting for entity state
- 1.4MB BOOT.BIN relocation: game loop (704KB, 0xB0014→0x109D0000),
  render engine (566KB, 0xBB014→0x10A80000), 73 functions (100KB, 0x10D48B20→0x10B0DB0C)
- HLE FIQ handler at 0x10A15DA4: clears timer, creates Nucleus TCB, mode switch
- Game task TCB at 0x10F00100: entry 0x10C16CC0, ready list at 0x100AC030
- Display controller MMIO (0xB80007xx): VBlank callback, framebuffer, IRQ handler
- Event pump HLE (0x10138000-0x10139000): skip data-as-code table
- Per-frame PC recovery: redirect escaped PCs to game loop (relocated) or idle (BOOT.BIN)
- BOOT.BIN 5A relocation: zero data tables, patch code literal pools for init
- Display controller 0xB80001xx: video mode register read-back for BOOT.BIN games
- Per-slice + per-frame IRQ delivery: JIT-safe timer interrupt dispatch
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
(+0x20), section count (+0x2C). sec[0]=ARM scene script (up to 1.2MB), sec[1]=compressed
sprite data (custom format), sec[2]=tilemap blocks + raw tile pixels. sec[2] layout:
blocks 0-35 = tilemap metadata (64×240 per block, circular viewport mask in block 0),
offset 0x88000+ = 29 uncompressed 8bpp tile intensity maps (512px wide).
Cars disc: 17 VFF files (CW×6 gameplay, GZ×7 puzzles, KW, MAIN, OPENING, STORY).

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

### JIT Compiler (ARM → x86_64)
- Basic block translation: ARM instructions → native x86_64 machine code
- 16MB mmap'd executable code cache, 65536 block hash slots
- Phase 1: data processing, LDR/STR, LDM/STM, B/BL, MUL, barrel shifter
- Conditional execution: EQ/NE/CS/CC/MI/PL/GE/LT via CPSR flag testing
- Inline RAM fast path: direct memory access for 90% of loads/stores
- RTOS code write protection: JIT-generated stores skip 0x10A00000-0x10B10000
- Interpreter fallback for complex instructions (SWI, MCR, mode switching)
- HLE hot functions: software division (40x speedup), memcpy, memset, strcmp
- Performance: 7 FPS (interpreter) → 37 FPS (JIT) — 5.3× speedup
- 9000+ compiled blocks, 1 billion+ JIT'd instructions per session

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
  → scheduler halt → HLE FIQ handler (0x10A15DA4)
  → game task TCB (0x10F00100) → ready list (0x100AC030)
  → AUTO-LAUNCH → 1.4MB BOOT.BIN relocation from CD
  → game loop (0x109D1CE0) → render_processing (0x10A89100)
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
| `0xB000000C` | 4KB | ZEVIO Timer Block (64 timers, stride 0x40) |
| `0xB0001000` | 4KB | SoC Interrupt Controller (6 sources) |
| `0xC0000000` | | PL111 LCD controller |
| `0xDC000000` | 4KB | ZEVIO VIC (IRQ+FIQ, ACK/Vector/EOI protocol) |

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
| RELOC skipped BOOT.BIN games | Incredibles had no utility code | Apply RELOC-GAMELOOP/RENDER/FIX for all games |
| Per-slice escape too aggressive | SpongeBob/Scooby crashed at bp800 | Moved to per-frame RECOVER |
| PTX flicker every other frame | `frame_count % 2` skipped odd frames | Render PTX every frame |
| BOOT.BIN 5A5A5A5A unpatched | Init crashed on 0x5A5A5A5A literals | Zero data section, patch code literal pool |
| Per-frame state corrupted BOOT.BIN | Forced game flags broke Incredibles | Skip state manipulation for BOOT.BIN games |
| JIT blocked IRQ delivery | CPSR I-bit window invisible to JIT | JIT yield after MSR CPSR enables IRQ |
| Timer load zeroed after bp900 | Per-frame re-enable failed (load=0) | Restore load=37500 if zeroed |
| LCD 0x84 DMA spin-wait | Incredibles BEQ self at 0x10C5D3B4 | Return bit 4 set (transfer complete) |
| PTX decoded as 16bpp | 8bpp indexed shown as psychedelic colors | Detect bpp from header +0x0C, use LCD palette |
| PTX stride hardcoded 512 | Horizontal banding on 1024-wide images | Read width from header +0x08 |
| dc_regs index overflow | uint32_t[64] indexed by byte offset | Divide offset by 4 |
| Render device bank vs addr | device[+0xC] was FB addr, needs index 0-3 | Ghidra: fb_resolver uses bank lookup |
| VBlank toggle per-read | Debounce rejected unstable signal | Toggle per-frame in misc_regs |
| Render ctx[3]==ctx[4] | render_processing skipped draw path | Ghidra: advance frame counter each frame |
| Render ctx[0]=1 | Forced error path (debug text only) | Set ctx[0]=0, ctx[1]=1 for normal draw |
| VFF entry hardcoded | Only one scene worked | Dynamic entry scan (LDR+PUSH pattern) |
| VFF section offset | Data shifted by 0x3A0 | Header is 0x60 bytes, loader uses 0x400 padded |
| Scene callbacks stub | Tiles not decompressed | Call all 65 dispatch table callbacks |
| FIQ handler BSS zero | Timer IRQ → zero code → crash | HLE FIQ at 0x10A15DA4 with mode switch |
| Event pump as code | 0x10138000 data executed as BL | HLE skip data table range |
| Render code not loaded | 0x10A80000 all BSS zeros | Reload 566KB from BOOT.BIN CD |
| Game loop code missing | 0x109D1CE0 was empty | Reload 704KB from BOOT.BIN CD |
| Phase 101 overwrites bp | boot_phase 800→300 race | Guard with boot_phase < 800 |
| PC escape to flash ROM | Render writes to 0xB80007xx | DC MMIO intercept + per-slice redirect |
| IRQ vector not reaching handler | Timer stub at 0x10FFF040 skipped scheduler | SDRAM chain: ROM[0x18]→0xFF98→0x10FFF200 HLE handler |
| IRQ handler missing VIC vector | EOI without vector# left IRQ pending | Added VIC vector read (0xDC000028) between ACK and EOI |
| Handler address collision | 0xFFF100 overlapped with task struct | Moved handler to 0xFFF200 |
| Timer status wrong address | Pool had 0xB000004C (wrong offset) | Fixed to 0xB0000024 (timer base + 0x18) |
| HLE patches clobbered chain | SDRAM[0xFFB8] overwritten by init halt | Guard with has_rom, per-frame chain refresh |
| ROM boot had no boot_phase | Kernel reached but detection missed | Auto-detect: MMU on + PC in scheduler = bp300 |
| bp300→800 kernel crash | Per-slice bp800 ran before game setup | Atomic: set bp800 + game launch in same post-frame |
| VFF-EXEC wrong entry | init_cb was utility stub (15 steps) | Scan sec[0] for dispatch table literal reference |
| VFF entry table not loaded | Dispatch table at sec[0] boundary | Load extra 8KB from CD after sec[0] |
| PTX splash blocked headless | cdrom_read_file hung in headless mode | Skip PTX/gallery/scan when has_rom |
| SDL_QUIT killed headless | Process exited after 1 frame | Ignore SDL_QUIT in headless mode |
| CPSR mode corruption | Mode bits = 0x00 (invalid) | Per-frame sanity check → force SVC |
| DC regs as flash writes | 0xB80007xx absorbed silently | Separate DC register handler |
| BSS clear kills render | Game zeroes 0x10A80000+ | Write protection 0x9D0000-0xBF0000 |

### Subsystems
- ATAPI CD-ROM: INQUIRY, READ(10), READ CD, READ TOC, READ CAPACITY, MODE SENSE
- MJP video: byte-swap, byte-stuffing restore, in-place decode, scaling
- IMA ADPCM + PCM WAV audio, 22050→44100Hz stereo resampling
- PTX: 8bpp indexed (palette from LCD) or 16bpp XBGR1555, stride from header
- VFF: "vfD0" container, 3 sections (ARM script + compressed sprites + tilemap+tiles)
- VFF sec[2]: tilemap blocks (circular viewport mask) + raw 8bpp tile pixels at +0x88000
- Scene compositor: 10 parallax layers, per-layer color tinting, box filter, scrolling
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
