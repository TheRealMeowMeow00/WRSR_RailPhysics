# Grid Feed / Catenary Findings — SOVIET64.exe 1.1.1.7

Companion to `RAIL_PHYSICS_FINDINGS.md` (Q3) and `STATION_FINDINGS.md`. Question: why a
10 MW electric trainset (hi_tot = 4.9, load = 1.0) sees a stable ~0.48 delivered fraction
from `FUN_1406b3a40` even on a strong grid, and how to raise sustainable catenary feed.

Artifacts: `out/grid_fns.c` (FUN_1401bbf70, FUN_1401b9640, FUN_1401bb700, FUN_1401bdd40),
`out/gauge_init.c` (FUN_1401bc020, FUN_1401bd4c0), `out/transfer.asm` (byte-level disasm
of FUN_1401bdd40/FUN_1401bd4c0), `out/cap25.asm`, `out/btype_chain2.asm`,
`out/grid_xrefs.txt`, `out/capconsts.txt`.

**Erratum** to STATION_FINDINGS.md: type id 0x12C (300) = `$TYPE_TRAM_GATE` (track-gate
objects), not "plain track block section" (now resolved from the $TYPE chain).

Legend: **[C]** confirmed, **[I]** inferred, **[?]** unknown.

---

## Q1 — What caps delivery to the catenary

### The draw side (fuel function, electric branch, 0x1406b3a40) **[C]**

Sources = substation buildings attached to the **track segment's power-source vector
`segment+0x1A0/+0x1A8`** (8-byte ptrs), reached via `routeNode->+0x28` (first attached
segment; route nodes are `train+0x6A0[0x700]`, fallback `0x700-1`, `0x700-2`). Each source
is a building with the standard gauge pointers (set by `FUN_1401bc020`, 0x1401bc020):

| building field | points to | meaning |
|---|---|---|
| +0x10E0 | class-9 storage slot +0x08 | **stored charge A** (energy units) |
| +0x10E8 | class-9 storage slot +0x0C | **voltage/quality V** (0..1) |
| +0x10F0 | class-9 storage +0xA8 | out-this-tick accumulator |
| +0x10F8 | class-9 storage +0x8C | **capacity C** |
| +0x1100 | inline float | smoothed copy of A (dial) |
| +0x112C | inline float | stat accumulator |

Demand and per-source draw (verified in decompilation and disassembly):

```
D      = hi_tot * load                       // units/s (0.49 * P_MW * load)
demand = PowerTime(0.001) * D                // per frame
N      = source count; share = 1/N
for each source i:
    frac_i = src[i].+0x1100 / C_i            // charge fraction (dial copy!)
    demand *= frac_i                         // ask decays along the list
    ask_i  = share * demand
    draw_i = min(A_i * V_i, ask_i)           // requires V_i >= 0.1
    A_i   -= draw_i;  out_i += draw_i;  stats_i += draw_i
    deliveredFrac_i = min(1, A_i*V_i / ask_i)
return Σ deliveredFrac_i * share * frac_i    // <- the 0..1 power factor
```

The source filter `FUN_1401bbf70` skips power plants (0x11), customhouse (0x14),
electricity exporters (0x1F), and dead sources. **[C]**

### The voltage model (FUN_1401bd4c0, 0x1401bd4c0) **[C]**

For building types **0x11 (power plant), 0x12 (substation), 0x13 (transformator) and
0x23 (rail trafo)**, per tick:

```
if A/C < 0.5:  V = 2 * (A/C)        // brownout below half charge
else:          V = 1.0
(other types:  V = 0 when A/C < 0.02)
```

**Rail trafo ($TYPE_RAIL_TRAFO = type 0x23)** catenary storage: default **capacity
C = 14.0** (per-type default in the building parser at 0x14011a1e2; the rail_trafo.ini
declares no $STORAGE. Type defaults: 0x13 → 19.0, 0x12 → 2.5 — matches the accumulator
probe's substation reading of 2.50). **[C]**

### The refill side (grid transfer FUN_1401bdd40, 0x1401bdd40) **[C]**

The rail trafo is an ordinary grid receiver (ELETRIC_LOW_INPUT wires). Per tick, each
feeding building pushes through each class-9 storage:

```
available = min( dt * storage.capacity, slot.amount )     // 0x1401bdf23: MULSS XMM0,[storage+0x8C]
per receiver:
    share    = shortfall_receiver / shortfall_total * available
    lineCap  = min over wires on path of FUN_1403bceb0(game, wire->+0x120,+0x124,+0x12C, 1, 0) * dt
    moved    = min(share, lineCap, shortfall_receiver, slot.amount) * sourceQuality
```

Wire ratings (from FUN_1403bceb0, flag=1): 35 / 60 / 70 / 80 / 85 / 100 / 110 / 120 /
150 units/s depending on line class — **not** a bottleneck for a 4.9 demand. **[C]**

### So why 0.48

The delivered fraction is not capped by any per-substation wattage constant; it is the
equilibrium `R / demand` where **R = what the grid transfer actually routes into the
trafo per second**. With demand 4.9/s and observed 0.48, R ≈ 2.35 units/s. The candidate
binding elements, in order of likelihood: **[I]**

1. **Source-side outflow ceiling** `dt × capacity` per feeding storage, *shared by all
   receivers of that source*. Vanilla power plants auto-size their electric storage to
   ~production/3 (a 70-unit plant has capacity 23.34 → max 23.34 units/s total grid
   throughput, split by shortfall across the whole town + all trafos). On a loaded grid
   this is the wall the catenary hits. **[C formula, I that this is the binding one here]**
2. **Plant slot amount**: transfers also can't exceed what's actually in the plant's
   storage this tick — same ceiling effectively.
3. Trafo shortfall (14 − A): only binding if the refill cadence is slow relative to the
   draw — it isn't the observed cap (draw is per frame, transfer per sim tick).
4. Wire rating: excluded (≥ 35/s).

At equilibrium the trafo sits at low charge and its voltage sags (`V = 2A/14`), which is
what throttles the train draw to exactly R — the 0.48 is the *result*, not the cause. **[I]**

## Q2 — How the catenary storage refills

Through the ordinary grid: power plants (and transformers/batteries) run their
distributors (FUN_1401bb700 for grid nodes, FUN_1401b9640 for substation catchments,
both called only from the building dispatcher FUN_140139a80), which end in
FUN_1401bdd40. The trafo's class-9 storage is filled like any consumer's, bounded by:
`dt×source.capacity` per source storage, wire rating×dt on the weakest link, the
trafo's own shortfall, and the proportional shortfall split. **There is no separate
catenary wattage rating** — the rail trafo's "breaker" is the same `dt×capacity` rule
when it distributes, and its storage capacity (14.0 default) is set per building type in
the parser (site 0x14011a1e2), overridable per building via `$STORAGE
RESOURCE_TRANSPORT_ELETRIC` (which rail_trafo.ini does not declare). **[C]**

## Q4 — Is the trafo a class-9 node like the accumulator battery?

Yes: identical storage layout (building+0x970 vector, stride 0xE0, class 9), identical
transfer machinery, identical quality=voltage semantics (building+0x10E8 ↔ slot+0x0C),
same brownout rule (0x23 is in the 2×charge-fraction group). The train feed is an
*extra* consumer path that bypasses the wire transfer: the fuel function decrements the
trafo's slot amount directly. **[C]**

## Q3 — Intervention options (ranked)

### 1. Hook the plant outflow ceiling — FUN_1401bdd40 @ 0x1401bdf23 (best leverage)

```
1401bdf1c  48 8b 85 70 09 00 00          MOV  RAX,[RBP+0x970]
1401bdf23  f3 0f 59 84 1f 8c 00 00 00   MULSS XMM0,[RDI+RBX+0x8c]   ; dt * capacity
1401bdf2c  48 8b 0c 07                   MOV  RCX,[RDI+RAX]
1401bdf30  f3 41 0f 59 c7               MULSS XMM0,XMM15            ; * param_7 (1.0)
```
Hook at 0x1401bdf1c (7+9 = 16 bytes, no RIP-relative operands; XMM0 = dt on entry,
RDI/RBX select the storage) and scale XMM0 by K (e.g. ×3) before the MINSS at
0x1401bdf3b. Effect: every grid source can push K× more per tick — directly raises R
for trafos *and* the town. Side effects: the accumulator plugin's batteries
charge/discharge faster too (its capacity-throttle rewrites capacity itself, so the
mechanisms multiply — verify combined behavior); statistics (+0xA8) record actual moved
energy as before, so no stat corruption; save format untouched. **[C site, I effect]**

### 2. Raise the rail trafo catenary capacity (helps when plant output is contended)

Parser site: `0x14011a1e2: f3 44 0f 10 05 cd 06 7f 00  MOVSS XMM8,[0x14090a8b8]`
(14.0f for type 0x23). **Do not patch the constant** — DAT_14090a8b8 is pooled (25+
readers incl. wire ratings and GUI). Instead repoint the rel32 in this 9-byte
RIP-relative instruction to a plugin-owned float (same length, no trampoline needed),
or patch each existing trafo's `storage+0x8C` at runtime. Effect: larger shortfall →
larger share of grid transfers, larger buffer; note it does **not** raise the
equilibrium rate on its own (delivered = R/demand regardless of C) and lowers V for the
same absolute charge — pair with option 1. **[C site, I effect]**

### 3. Demand-side, inside the plugin's own fuel call (cheapest, no game patch)

The plugin passes `load` into FUN_1406b3a40. Options: (a) accept the delivered fraction
but spread demand better (the game already aggregates all substations linked to the
current route node — N sources, ask = demand/N each); (b) bypass the voltage term by
hooking the draw's voltage multiply in the fuel function (≈0x1406b3dxx region:
`fVar25 = fVar2 * fVar25` before the min with ask) so trains take `min(A, ask)` —
up to 2× draw while sagging, but it just drains trafos to empty faster; only useful
together with option 1. **[C mechanics]**

### 4. Gameplay: more substations

R aggregates over every substation linked to the segment the train is on — the vanilla
scaling path. **[C]**

### Not recommended

- Patching pooled float constants (2.5 @ 0x14090a37c, 14.0 @ 0x14090a8b8, 19.0 @
  0x14090a930) — dozens of readers each.
- Zeroing the brownout rule in FUN_1401bd4c0 (`*pfVar3 = fVar16+fVar16`) alone — creates
  no energy; trains stall harder once storages hit zero.

## Concrete numbers (vanilla)

- Rail trafo catenary storage: C = 14.0 units (type 0x23 default), V = min(1, 2A/14).
- Train demand: D = 0.49 × P_MW × load units/s (10 MW trainset @ load 1 → 4.9/s).
- Plant grid throughput: ≤ storage capacity per second (≈ production/3; 70-unit plant →
  23.34/s), shared by shortfall across all receivers.
- Observed 0.48 ⇒ trafo receives ≈ 2.35 units/s from the grid under those conditions. **[I]**
