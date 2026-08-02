# Reverse engineering — toolkit and re-verification after a game update

Everything the plugin patches belongs to `SOVIET64.exe` **v1.1.1.7**
(64-bit, image base `0x140000000`). This document is how the knowledge was
produced and how to re-produce it when the game updates.

## Toolkit

- **Ghidra** (12.x used) with the analysed project kept — re-importing
  costs minutes. Headless pattern:
  ```bash
  PATH=/path/to/jdk21/bin:$PATH JAVA_HOME=/path/to/jdk21 \
    ghidra/support/analyzeHeadless <projdir> Soviet \
    -process SOVIET64.exe -noanalysis \
    -scriptPath scripts -postScript DecompOne.java out.c <hexaddr>...
  ```
  (Ghidra 12 has no Jython — write post-scripts in Java.)
- **radare2** for quick disassembly and constant reads.
- The plugin log as the runtime oracle: hook first, understand later.

## Finding things: the anchor chain

1. Vehicle ini tokens are plain strings in `.rdata` (`$MOVEMENT_POWER_KW`
   etc.). One xref each → the vehicle script parser at `0x1403EB7D0`, which
   yields the **type struct offsets** (top speed `+0x8624`, power kW
   `+0x8678`, empty weight `+0x867C`, electric/steam/horse flags
   `+0x8680..82`, derived fuel coefficients `+0x8684/+0x8688`).
2. The infrastructure speed-limit lookup `0x1403BCEB0` (known from the
   railspeed plugin) has 22 callers; the rail one is the movement update
   `FUN_1406a7410`.
3. From there everything is local reading: the divisor call, the two fuel
   calls, the brake/slope/curve sites documented in
   [01-design.md](01-design.md).

## Patch sites to re-verify after an update

All RVAs from image base; expected bytes are checked at plugin load and a
mismatch skips the subsystem safely. To check them against a new build
without running anything, read the file at the site's RVA through the
`.text` section mapping (vaddr `0x140001000` ↔ file offset `0x400` here)
and compare:

| Site | Bytes |
|---|---|
| divisor call `0x6A7860` | `e8 db 12 ff ff` |
| fuel call cruise `0x6A773F` | `e8 fc c2 00 00` |
| fuel call station `0x6A7786` | `e8 b5 c2 00 00` |
| brake hook `0x6A8443` (16 B) | `f3 0f 5c f0 f3 0f 11 b6 24 0d 00 00 44 0f 2f ce` |
| slope branch `0x6A8545` | `76 41` |
| slope hook `0x6A8566` (21 B) | `f3 41 0f 59 c2 f3 0f 58 86 24 0d 00 00 f3 0f 11 86 24 0d 00 00` |
| curve hook `0x6A8326` (17 B) | `44 38 ae 41 0d 00 00 74 51 f3 0f 10 b6 24 0d 00 00` |
| grid hook `0x1BDF1C` (16 B) | `48 8b 85 70 09 00 00 f3 0f 59 84 1f 8c 00 00 00` |

If a site moved: find the same code in the new build (the divisor/fuel
calls are inside the rail update; the brake site follows the accel block;
the grid site is in the plant outflow computation), update the RVA *and*
the expected bytes, and — for the mid-function hooks — re-check that the
stolen window still ends on an instruction boundary and contains no
RIP-relative instruction. **Disassemble the replayed stub and compare it
to the site's bytes**: the one time this was skipped, a SIB operand was
replayed wrong and the game crashed on every grid connection.

## Structure offsets used

Vehicle instance: wagons vector `+0x3A0/+0x3A8`, emergency flag `+0x5DF`,
route segments `+0x6B8/+0x6C0`, route index `+0x700`, current segment
`+0x798`, direction `+0x7A0`, position `+0x7A4`, speed `+0xD24` (km/h),
slope `+0xD30`, stop-intent flags `+0xD39/+0xD3C`, type `+0x1708`, wagon
inactive `+0x1794`.

Vehicle type: category `+0x294` (3 vagon, 4 locomotive, 5 rail service),
top speed `+0x8624`, power `+0x8678`, empty weight `+0x867C`, electric
flag `+0x8680`.

Track segment: polyline `+0x08/+0x10` (stride 0x18: x,y,z, lateral dir,
cumulative metres), nodes `+0x20/+0x28` (position at node `+0x04/+0x08/+0x0C`),
length `+0x11C`.

Stations: global chain vector `exe+0x9E6A18`; chain type at
`chain+0x318 → +0x360` (0 cargo, 1 passenger, 0x60 waiting); membership
table `+0xA10` (stride 0x60, entry `+0x20` = segment ptr).

Game functions called: total mass `0x140698E70`, total power
`0x140698BE0`, original divisor `0x140698B40`, original fuel
`0x1406B3A40`.

## Full findings

The raw reports are in [findings/](findings/): vanilla physics
(01), routes and curves (02), stations (03), stops (04), grid feed (05).
