# Stop-Point Findings — SOVIET64.exe 1.1.1.7

Companion to `RAIL_PHYSICS_FINDINGS.md`, `CURVE_FINDINGS.md`, `STATION_FINDINGS.md`.
Goal: exact distance from a train to its scheduled stop point, for smooth braking with
clean handover to vanilla's final approach.

Artifacts: `out/stopsched.c` (FUN_140668e50), `out/stopmarker.c` (FUN_1406ad850,
FUN_1406b8da0), `out/path604.c` (FUN_14051f8f0), `out/track604.c` (FUN_1403bd6f0),
`out/poi_stride.txt`, `out/stopfields.txt`, plus `out/train_update.asm` /
`out/physics_cand1.c` from earlier.

Legend: **[C]** confirmed, **[I]** inferred, **[?]** unknown.

---

## Q1 — POI table semantics

Layout (chain/section object, e.g. train+0x528's chain): **[C]**

- `+0x610 / +0x618` = `std::vector`-like range, entry stride **0x21B8**.
- `entry+0x00` float = **distance along the section's master path** (chain+0x5D0
  polyline; its length at +0x11C is the metric's max).
- `entry+0x04` int32 = poi type.
- `entry+0x08 / +0x10` = sub-entry vector, stride 0x10: `{obj* @+0 (valid iff
  obj+0x44 != -1), float start @+8, float end @+0xC}` — used for platform sub-ranges
  (start/end on the master path) **[I]**.

POI types seen in code: **[C for dispatch sites, I for meaning]**

| type | consumer | meaning |
|---|---|---|
| 6 | dispatcher 0x14066xxxx (FUN_14066d460) → FUN_1406692b0 | station **stop marker** (fires when chain+0x12C != 4); stop position computed from sub-entry ranges (see Q4/Q5) — likely passenger **[I]** |
| 8 | dispatcher → FUN_140669490 | station **stop marker**, proportional-position variant — likely cargo **[I]** |
| 10 (0xA) | rail update signal block (0x1406a79xx) | signal (braking table) |
| 0xC | same, gated by type->0x85E0 skill 0x1B | signal (chain/constr-related) |
| 0x10 | same | signal |
| 1, 0xE | station loader FUN_140142440 (+0x50C codes) | waypoint/stop sub-kinds |

Creation/maintenance: entries are built when track/sections are (re)built and
**rescaled/reindexed on track split** — FUN_1403bd6f0 (0x1403bd6f0) copies the table,
scales sub-entry start/end by the split ratio, and recomputes +0x604 (see Q2).
FUN_14051f8f0 (track build/edit) consumes/maintains the table. **[C]**

Consumption: (a) the rail update's signal block (0x1406a79a0–0x1406a7a85) — types
10/0xC/0x10 only; (b) the per-frame dispatcher (FUN_14066d460, 0x14066exxx) — when the
train registered on the chain (chain+0xBB0 == consist head) crosses a POI of type 6/8,
it calls FUN_1406692b0 / FUN_140669490 → FUN_140668e50. **[C]**

## Q2 — chain+0x604

- It is a **cursor (float, meters) on the same master-path metric as POI dist** — all
  readers use it as "find first POI beyond +0x604". **[C]**
- It is written by **track/section code**, not by the train update: FUN_1403bd6f0
  recomputes it on track split (`+0x604 = scaled position`), FUN_14051f8f0 writes it
  during build/edit. **No writer exists in the vehicle update path** (verified by full
  scalar scan of .text: no `MOVSS [reg+0x604]` in 0x14066xxxx/0x1406xxxxx). **[C]**
- Therefore: **do not use +0x604 as the train's live position.** Its live semantics
  (who moves it while a train occupies) is unresolved [?]. The train's live position is
  `train+0x798` (segment) + `train+0x7A4` (pos); distances ahead must be computed by
  route walking (as the curve feature already does).
- Note the rail update's signal block mixes metrics: it computes
  `frac = (+0x604 − prevPOI)/(nextPOI − prevPOI)`, `brakeDist = masterLen*frac − pos`,
  comparing against `pos/masterLen` — only meaningful because the engaged chain's
  master path and the current segment share geometry. **[I]**

## Q3 — +0x528 / +0x530 / +0x4F0 lifecycle

From FUN_1406bc890 (segment transition) and FUN_1406b7a10 (arrive handler): **[C]**

- `+0x528` = engaged chain (block reservation; drives the signal-braking block). Set at
  segment transition when the consist *doesn't* fit the remaining segment (`fVar29<0`);
  followed by the arrive handler.
- `+0x530` = stop-scaling chain. Set when the consist fits (`fVar29≥0`), or assigned
  inside the rail update during route transitions from the chain's +0xA10 table walk
  (0x1406a91xx/0x1406a94xx). Can be a depot (0xF), constr office (0x1C), waiting
  station (0x60), or station (0/1) — the 20 m stop block explicitly *requires*
  type ∉ {0xF, 0x1C}. **[C]**
- On arrival (FUN_1406b7a10): `+0x4F0 = +0x528; +0x528 = 0; +0x530 = 0; +0x4F8 =
  +0x4F0;` train appended to `chain+0xC70` (vehicles at station).
- **Neither is populated 600 m ahead** — engagement happens when the train is already
  on/near the chain's track. For lookahead distance-to-stop you must walk the route. **[C]**

## Q4 — Vanilla stop point and the 20/10/7 m scaling (FUN_1406a7410)

**The vanilla stop point for a scheduled stop is the END of the route's last leg
(current segment's +0x11C when `+0x700 == routeLen−1`).** No mid-segment target is used
by the braking logic — the train creeps to the segment end. **[C]** (The exact standing
position is then set by arrival/reposition code; POI stop markers (type 6/8) exist on
the chain path and are consumed at crossing time by FUN_1406692b0/140669490 →
FUN_140668e50, which places the train node at a platform-derived distance — this is
position placement, not the braking target.) **[C/I]**

Stop blocks (all in the rail update, after the main accel/brake integration;
`R14 = current segment`, `XMM2 = pos`, `XMM15 = speed copy fVar52`, `XMM14 = 1.0`): **[C]**

| Block | Gate (all require last route leg `+0x700 == routeLen−1`) | Window | Effect |
|---|---|---|---|
| A @0x1406a8666 | `+0x530 != 0` and chain type ∉ {0xF,0x1C} | 20 m (DAT_14090a940) | `frac = (segLen−pos)/20`; `fVar52 = frac*fVar52 + 1.0`; `d28 = 0.5`; throttle est. zeroed |
| B @0x1406a8727 | `(+0x688−+0x680)>>3 > 1` && `+0x698==0` && `+0x528==0` && front route obj && its +0x5D0 | 7 m (DAT_14090a7c8) | same formula with 7 |
| C @0x1406a8932 | `DAT_1409e9360==0` (game-mode flag) && (`+0xd39` ‖ (`+0xd3a` && `+0xd3b`)) | 10 m (DAT_14090a840) | same + **`if fVar52 < 2.0: d24 = fVar52`** (hard clamp) |
| D @~0x1406a8a00 | `DAT_1409e9360!=0` && next-leg/junction checks | 20 m | same; **`if fVar52 < 1.5: {FUN_1406bc890(train,0); d24 = 0}`** — the actual stop + arrival |
| E @~0x1406a8b32 | `+0xd3c != 0` && `+0x700 == routeLen−2` | 10 m | scale, clamp to ≥2.0, `d28 = 0.5` (pre-stop for the *next* stop) |
| F @~0x1406a8a76 | `FUN_1406ac5d0(train)==0` && `pos > segLen−10` | 10 m | signal-hold creep: `fVar52 *= clamp01((segLen−(pos+fVar47))/divisor)`; `+0x19F8` stuck-timer (200 s → warning) |

So the approach profile is: linear ramp toward ~1 km/h starting 20 m out (soft), a hard
d24 clamp inside 10 m, full stop when the scaled speed < 1.5 km/h, then
FUN_1406bc890 → FUN_1406b7a10 (arrival, chain promoted to +0x4F0/+0x4F8, dwell, loading
via FUN_140142440, departure via FUN_1406b8da0). **[C]**

Stop-intent flags: **`+0xd39`** = stop at the upcoming station (set by the route/line
assignment FUN_140680920; cleared after the stop), **`+0xd3a/+0xd3b`** = arrival
sequence state (set to 0x101 by arrive handlers FUN_1406b50b0/1406b6f60/1406b7560,
+0xd3b by FUN_1406b5db0), **`+0xd3c`** = another stop follows after this one
(set in FUN_140680920, cleared in FUN_1406a7410). **[C]**

## Q5 — Platform extent

- POI entry sub-entries (`entry+0x08` vector, stride 0x10): `{obj* slot, float start,
  float end}` — platform sub-ranges on the master path; validated by `slot+0x44 != -1`.
  FUN_1406692b0 uses `min(start/end)` across slots to place the stop proportionally;
  track-split rescales start/end. **[I]** (start/end vs len/frac pairing is inferred
  from the split code — treat as [I].)
- `chain+0xF90 / +0xF98` = vector of platform/slot sub-objects; each has its own
  +0x5D0 path, +0x604 cursor, +0x610 POI table, `+0xE04` int kind. Same section
  layout as chains. **[C structure]**

## Recommended distance-to-stop for the plugin

Vanilla stops at the **end of the last route leg**, so:

```c
// RSI = train (from the existing hook frame)
bool stopping = (*(u8*)(train+0xD39) || (*(u8*)(train+0xD3A) && *(u8*)(train+0xD3B))
                 || *(u8*)(train+0xD3C));                                  // [C]
if (stopping) {
    int routeLen = (*(u64*)(train+0x6A8) - *(u64*)(train+0x6A0)) >> 3;      // [C]
    char** segs = *(char***)(train+0x6B8);                                  // parallel to route [C]
    // current leg index: pointer-match train+0x798 (robust) or derive from +0x700
    float dist = 0; bool pastCurrent = false;
    for (int i = legIndexOf(train); i < legCount(segs); ++i) {              // last leg = stop leg
        float L = *(float*)(segs[i] + 0x11C);
        if (i == curLeg) {
            float pos = *(float*)(train+0x7A4);
            float rem = (*(u8*)(train+0x7A0) ? L - pos : pos);              // [C]
            dist += rem;
        } else dist += L;
    }
    // dist = exact meters to the vanilla stop point (end of last leg)      [C]
    // Plugin curve: brake with own decel model until dist < ~25 m, then
    // RELEASE the XMM9 clamp (or raise it above vanilla's) — blocks A–F take
    // over from 20 m and stop the train. [I: 25 m = 20 m window + margin]
}
```

Caveats:
- If the route is later extended (next line leg assigned at the station), the last-leg
  end is still the stop — re-evaluate each frame; cheap.
- The `+0x700 == routeLen−1` gates mean vanilla's own windows only engage on the last
  leg; if the plugin clamps earlier with a *lower* value than vanilla's ramp, the train
  will simply follow the plugin curve — also fine, but releasing at ~25 m gives the
  smoothest handover. **[I]**
- `DAT_1409e9360` selects between block C (hard clamp @10 m) and block D (stop @<1.5
  km/h @20 m) — game-mode dependent; both leave `d28 = 0.5`. **[C]**
- For pass-through (no stop flags), none of blocks A–F fire — station speed limits
  remain the plugin's job (STATION_FINDINGS.md). **[C]**
- +0x604/POI machinery is *not* needed for the stop distance; keep using the route
  walk. (Its one unresolved piece: live updater of +0x604 [?].)
