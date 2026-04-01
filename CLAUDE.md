# V.Flash Emulator — Current Task

## State (2026-04-02)
- Scene init FIXED — all 20 layers initialize (commit 78252f8)
- C-side frame driver implemented (commit 8ddb263)
- Per-layer callbacks execute (6.5M steps for Scooby-Doo) but produce sparse output
- **VFLASH_NOJIT=1** required (JIT has SIGILL crash)

## Blocking: Render Output
Scooby-Doo is a 3D game — per-layer callbacks run but need:
- Decompressed textures from sec[1] (custom compression, unknown format)
- 3D scene graph (camera, mesh positions, animations)
- Full render context (RTOS render state machine)

SpongeBob init gets stuck in RTOS loop at 10A07C74 (waiting for semaphore/timer).

## Frame Driver Results (Scooby-Doo)
- Layers 0-3, 8-9, 14: <500 steps (quick return)
- Layers 4, 12-13: ~500K steps (hit budget)
- Layers 6-7, 10-11, 16: ~1M steps (2 callbacks × 500K)
- Framebuffer: 3-6 non-zero pixels (nearly empty)

## Bugs Fixed This Session (78252f8)
1. BSS zeroing protection blocked stack writes during native code execution
2. HLE property setters set R0=0, breaking object pointer chains
3. Property setters wrote to stack-resident objects, corrupting loop counter
4. AA5AA0 returned NULL → crash in virtual dispatch

## Next Steps
1. **sec[1] decompression** — reverse-engineer custom compression (6.5-7.3 bits/byte)
2. **SpongeBob RTOS loop** — HLE 10A07C74 so init completes
3. **Render context setup** — properly populate 0x10BE3C40 for native engine
4. **Try simpler game** — Spider-Man or other 2D title

## Tools
- RAM dump: /tmp/vflash_ram2.bin (16MB, base 0x10000000)
- Disassembler: /home/wizzard/share/GitHub/tmp/disasm_cb <ram.bin> <VA_hex> [count]
