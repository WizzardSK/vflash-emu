# V.Flash Emulator — Current Task

## Stav (2026-04-01)
- Alokátor overflow opravený (commit d177062), sec[0] kód zachovaný
- Garbled framebuffer odstránený, displej je čistý čierny
- Scooby-Doo dosahuje bp=900
- **VFLASH_NOJIT=1** povinné (JIT SIGILL crash)

## Blocking: Čierna obrazovka
Natívny render pipeline beží (50M budget) ale nič nekreslí.
RTOS funkcie (10A9082C, 10ABCB24, 10AA0FC0/C8) sú HLE'd a vracajú 0.
Render objekty nemajú správne tile data pointery, palety, rozmery.

Init funkcia 0x104BC818 (NEPOŠKODENÁ) iteruje 20 vrstiev:
1. Alokuje objekt (0x10A3D9E4 → bump 0x700000)
2. Inicializuje cez RTOS (0x10AADCB8 → bump 0x680000)
3. Volá 0x10A9082C(mode, desc_table[layer*16]) → scene handle
4. Volá 0x10AA0FC0(handle) → width, 0x10AA0FC8(handle) → height
5. Ukladá cez property settery (10AF3F18=width, 10AF3F3C=height)

Descriptor table 0x10648B20/2C = ZEROED (RTOS nezaplnil).
Layer table 0x10643998 = OK (20 entít, callback pointery).

## Prístupy na riešenie
A. Disassemblovať 10A9082C — zistiť čo vracia, HLE s reálnymi hodnotami
B. Nechať viac kódu bežať natívne (init 104BC818 je celý)
C. C-side renderer čítajúci layer table 0x643998 priamo

## Nástroje
- tmp adresár: /home/wizzard/share/GitHub/tmp/
- RAM dump: /tmp/vflash_ram2.bin (16MB, base 0x10000000)
- Disassembler: /home/wizzard/share/GitHub/tmp/disasm_cb <ram.bin> <VA_hex> [count]
- Detailný progress: pozri memory/vflash-progress.md
