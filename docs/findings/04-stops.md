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

## Q6 — Customhouse (border) stops

Live telemetry (2026-08 run, train at 318 km/h into a border crossing): the smooth
stop never engaged — the train hit vanilla's 20 m/10 m ramps at full speed. **[C]**

- **The stop-intent bytes `+0xd39`/`+0xd3c` are NOT set for customhouse stops** —
  they come from line scheduling (FUN_140680920), and a border clearance is not a
  line stop. Any engagement logic keyed on those bytes alone is blind to customs. **[C]**
- `+0xd3a`/`+0xd3b` (block C's alternative gate) are **arrival-sequence** bytes set
  by the arrive handlers — they only turn on once the train is already arriving, so
  they cannot drive early braking either. **[C]**
- Vanilla brakes customs stops through **stop block A** (`+0x530` chain; the
  `type ∉ {0xF, 0x1C}` gate passes customhouse 0x14), gated on `+0x700 ==
  routeLen−1`, scaling `(segLen−pos)/20` — i.e. when the stop engages, the stop
  point is the END of the route's last leg (a segment end). **[C]**
- Customhouse chains DO appear in `DAT_1409e6a18` and DO register their track in
  the +0xA10 table like stations: the init sweep found 74 segments on a map with
  border crossings (`customs: 74 track segments in customhouse zones`). **[C]**

### Route-terminal detection — FAILED in game (2026-08 test)

The first plugin attempt engaged the smooth stop when the route's *last leg* lay in
a customhouse chain. Result: the train still entered at full speed; the
`(customs stop)` tag appeared only twice, the second time at `v=328 km/h,
limit=39 km/h` — 39 km/h is exactly the parabola value ~117 m out, so the route
end moved to the customs only when the train was ~117 m away. **[C]** Conclusions:

- **The route does not reach the customs from afar.** (First-run reading: it
  runs through the border and is cut at the last moment; refined by the
  second run below — the route is in fact capped AT the zone boundary and
  only extended into the zone ~115 m out.) **[C]**
- Vanilla's `+0x530`/block A can therefore engage only at the same late moment —
  which is exactly the observed slam. **[C]**
- Any detector keyed on route termination is useless for early braking. **[C]**

### Zone-in-route detection — FAILED in game (2026-08 test, second run)

The second attempt scanned the route for customhouse segments and braked to zero
at the run's far end. Result: the train slammed into the customs at 265 km/h.
Decisive telemetry:

```
customs: zone 0-115 m ahead, route left 115 m, v=265 km/h
customs: route truncated, left 1 m vs zone far 1 m, v=259 km/h
```

- **The route array does not contain the customhouse legs until the train is
  essentially on top of them.** From afar the route is capped at the customs
  zone boundary; extension into the zone happens ~0–115 m out. The cap is
  customs-specific — on open mainline the same route arrays reach kilometres
  (curves are seen and braked for at R≈3000–4000 several km out). **[C]**
- The first line shows the zone entering the scan at near=0/far=115 m at
  265 km/h — the route AND the zone appeared together, one braking distance
  too late (265 km/h needs ~4.2 km). **[C]**
- The second line ("left 1 m vs zone far 1 m") is a **measurement artifact**:
  both distances are measured from the train once it is inside the zone, so
  they are equal by construction and degenerate to ~0 at the stop. It never
  captured a truncation moment; the diagnostic was dropped. **[C]**
- Conclusion: no detector keyed on route CONTENT can fire early — the route
  data itself does not reach the zone until the last second. **[C]**

### Route-end-at-boundary detection (H3) — FAILED in game (2026-08 test, third run)

Every approach logged `boundary match no`; the route end never sat on a customs
boundary node. Telemetry:

```
customs: 74 segments in 37 customhouse zones, max depth 239 m
customs: route ends 6050 m ahead, terminal node 0xD89103C0, boundary match no, v=330
customs: route ends 4672 m ahead, ... boundary match no, v=255
customs: route ends 1485 m ahead, ... boundary match no, v=1
customs: route ends 1081 m ahead, ... boundary match no, v=67
customs: route ends 47 m ahead,   ... boundary match no, v=1
```

- **The route array ends kilometres short of the customs on ORDINARY track
  nodes and is extended by the game in chunks as the train advances.** There
  is no fixed cap point — not the zone boundary, not a signal. **[C]**
- Combined with the second run (customhouse legs enter the route only ~115 m
  out): **no route-based detector can ever brake early.** The route data
  itself does not reach the customs until the last second, by construction. **[C]**
- Component build works: 74 segments → 37 zones, max depth 239 m (user's map).
  User experiment: deleting track beyond the customs changes nothing — the
  customhouse building straddles the map border (~50/50). **[C]**

### Static approach corridors — current mechanism

Since the route can't see the customs, the geometry is precomputed from the
WORLD side instead: at each 30 s sweep, a **multi-source Dijkstra** runs from
every customs-related node we can collect (customs-segment endpoints AND the
chain table's +0x30 nodes; the whole zone sits at corridor distance 0) over
the track graph (node+0x28/+0x30 = attached segments [C],
findings/02 Q1), tagging every reached segment with the graph distance from
the nearest customs track to each endpoint, out to **8 km** (~6.4 km is the
330→50 km/h braking distance at the default budget). No global track registry
exists in the findings — the walk is pure node↔segment adjacency. [C: implemented]

**Seeding pitfall (2026-08 test, fourth run):** the first corridor build seeded
only *degree-1 boundary* nodes and produced `corridors 0 segs`. The user's map
has 74 segments in exactly 37 zones = two segments per customhouse; a
double-track customhouse is two PARALLEL segments sharing both endpoint nodes,
so no node has degree 1 and the seed set was empty. **[C]**

**Chain-local islands (2026-08 test, fifth run):** reseeding from every
distinct customs-segment node built corridors covering only the customs
segments themselves:

```
seed[0] 0xD8B97B00: +28=0xD91674E0 +30=0xD91674E8 -> 1 segs (ok)
corridor build: 74 seeds, 74 nodes visited, 37 segs tagged (complete)
```

Every customs node lists exactly ONE attached segment — the customhouse track
and its nodes are **chain-local islands, not shared node objects with the
mainline** (route legs share endpoint node objects with each other — the
scan's shared-node orientation test relies on it — but customhouse chain
segments do NOT share nodes with the mainline they connect to). **[C]** So
node-adjacency BFS cannot escape a customs island through segment endpoint
nodes. (Game context from the user: large customhouse = 3 rail + 3 road
connections, medium = 1+1, small = road only — per-building rail connection
counts vary; nothing may assume 2 segments.)

**The bridge candidate: chain table entry+0x30 — CONFIRMED.** Sixth run:
corridors built onto the mainline (`corridor entry 7749 m ahead` at
detection), so the +0x30 chain-table nodes ARE mainline-connected. **[C]**
One direction then worked end to end (parabola to ~50, clean zero at the
building). **[C]**

**Reverse-direction failure (2026-08 test, sixth run) — two bugs:**

```
customs: corridor entry 6050 m ahead, v=329 km/h
curve: v=301 limit=261 (customs entry)   ... limit collapses 259->161 ...
... limit rises 161->...->235, train re-accelerates 210->234 ...
customs: corridor on route, not descending toward entry (no brake), v=171 km/h
curve: v=176 limit=36 (customs stop)     <- zone-run catches it, hard stop
```

- **Estimate growth = throttle.** Dijkstra tags are shortest-path distances;
  near the border (loops, parallel track, crowded customhouses) the visible
  route end moves between differently-tagged legs and the raw estimate both
  collapsed (259→161) and GREW (161→235) mid-approach — the growth re-opened
  the throttle. **[C]** Fix: temporal monotone clamp in the cache slot
  (never grow while the same engagement continues; drop at most traveled +
  25 m per scan; no-ops on a steady approach; reset on reroute/release). [C: implemented]
- **Premature release on untagged legs.** Chain-local track (stations, the
  customs islands) is unreachable by the node walk, so such legs are always
  untagged — and the all-must-descend guard dropped the engagement on them
  mid-approach. **[C]** Fix: untagged = unknown, not evidence of leaving;
  only a tagged ASCENT rejects; the first customs leg hands the approach to
  the zone run, so the two detectors can never both be off inside an
  approach. [C: implemented]

**Iteration-7 regression (2026-08 test, seventh run):** the carry-forward +
clamp version regressed the previously-good direction:

```
customs: corridor entry 3736 m ahead, v=257 km/h
curve: v=257 limit=253 (customs entry) ... limit decays far too slowly ...
curve: v=171 limit=39 (customs stop)          <- 171 km/h at the fence
customs: corridor entry 11778 m ahead, v=251 km/h   <- corridor cap is 8 km!
```

- **Root cause (carry math):** the carry (`corrExit -= len`, floored at 0)
  combined with `corrDist = sLeg + corrExit` — once the floor bites, the
  estimate grows with `sLeg` instead of tracking the entry; and engagement
  no longer required the train's own leg to be tagged, so estimates could
  stack the full visible route on top of a far tag (11 778 m > 8 km build
  reach — impossible for a raw tag). The monotone clamp, which forbids
  growth, then **preserved** the inflation: the estimate could only crawl
  down by `traveled + 25` per scan → braking too weak → 171 km/h at the
  fence. **[C]**
- **Fix:** no carry at all. The estimate is RAW TAGS ONLY — `corrRaw =
  dist-to-this-leg's-exit + exitTag` at the last descending tagged leg;
  untagged legs (chain-local islands) merely tolerate the gap. Engagement
  additionally requires the train's CURRENT leg to be a tagged descent
  (entering the corridor IS one braking distance out — the 8 km reach
  covers 330→50 with ~1 km slack), plus a sanity cap
  (`corrMax + zoneDepth + 250 m`). The clamp stays, bounding tag noise
  only. The island handover distance is pure route geometry (distance to
  the first customs leg). **[C: implemented]**
- The reference good approach (iteration 6 log): engage 7749 m at limit
  ~345, v chases the falling limit 177→266, they meet ~266/264 at ~4 km
  out, then ride down together — that profile is the acceptance target for
  both directions. **[C]**

**Iteration-8 failure (2026-08 test, eighth run) — wrong customhouse:**

```
customs: corridor entry 3736 m ahead (raw), v=257 km/h   (matches the 3736 m
                                                          parabola exactly)
curve: v=172 limit=39 (customs stop)   <- estimate still said ~1.6 km to go
                                           at the fence
```

- **The estimate was off by a constant ~1.6 km = the distance to the NEXT
  customhouse along the border.** Tags are multi-source nearest-by-graph;
  with 37 customhouses crowding the fence, a route leg's nearest source can
  be customhouse B while the train stops at A. Distance-to-B also decreases
  while approaching A, so the descent check passed and the parabola targeted
  B's entry, 1.6 km past A. **[C]**
- **Fix: corridor identity.** Tags carry the source component id
  (propagated through the Dijkstra; per-endpoint on segments), kept for
  diagnostics. [C: implemented]
- **Sawtooth contributor found:** the clamp reset fired on every route
  chunk EXTENSION (`routeN` grows as the game extends the route), letting
  raw noise re-enter mid-approach. Reset now requires a shrinking route or
  a same/shorter pointer swap. **[C]** The residual ±1–2 km/h hunting while
  tracking the falling limit is vanilla's bang-bang speed controller
  (brake when v > limit, throttle when v < limit); the plugin's brake
  helper only sets the deceleration RATE, not the timing — not ours to
  pulse, nothing changed there. **[C/I]**

**Iteration-9 failure (2026-08 test, ninth run) — identity lock picks the
wrong customhouse:**

```
customs: corridor entry 7882 m ahead (raw, id 34), v=156 km/h
curve: v=177 limit=352 ... v=267 limit=311 (customs entry)   <- slow decay
curve: v=248 limit=39 (customs stop)   <- estimate still had ~3.4 km at the fence
```

- The lock rule (id at the train's own leg) picks the GRAPH-nearest
  customhouse, but the train stops at the FIRST customhouse ALONG ITS
  ROUTE. In a border chain those differ: Voronoi assignment puts the
  train's leg nearer (by graph) to a customhouse the route never reaches.
  Iteration 8's 1.6 km residual was the same disease on the other side of
  the Voronoi boundary. **[C]**
- **The saving invariant: every border-crossing train stops at the first
  customhouse on its path.** Fix: the FIRST-ZERO rule — the estimate is
  the MINIMUM over descending legs of (distance to the leg's exit + exit
  tag). A leg tagged to the near customhouse measures exactly the true
  distance (s + (D−s) = D); a leg tagged to a farther one always measures
  longer and loses the min; the leg at the zone's edge has tag ≈ 0 and
  pins the min. Single-customhouse case: all candidates equal — identical
  to the iteration-6 reference. **[C: implemented]**
- Direction guard without a lock: mixed-id legs (Voronoi boundary
  straddlers) and untagged legs are tolerated; ascent rejects only on the
  train's OWN leg (then it is physically moving away from every
  customhouse); mid-route ascents are tolerated as boundary artifacts.
  Engagement starts only when the current leg descends; the 500 m
  evidence-free decay covers island gaps mid-engagement. [C: implemented]
- The diag line's `id N` is now the WINNING candidate's id — the predicted
  customhouse; check it against where the train actually stops. [C: implemented]

**Iteration-10 failure (2026-08 test, tenth run) — seed coverage gap:**

```
corridor build: 74 zone + 35 conn seeds, 255 nodes visited, 209 segs tagged
```

35 conn seeds for 37 zones — and both approaches still overshot by a
CONSTANT residual (3.4 km / 4.1 km) that the first-zero rule could not fix:
there was no distance-0 seed for the target customhouse on the mainline at
all, so its mainline was tagged by the NEIGHBOUR's corridor. **[C]**

- **Root cause:** conn-node→component resolution went through the table
  entry's own segment (+0x20), but +0xA10 entries with a NULL segment exist
  (the path walker follows the table through segment-less entries,
  findings/02 Q2), and those conn nodes silently resolved to −1 and were
  skipped. **[C: mechanism; the exact failing entries were not individually
  logged — the audit below shows per-zone coverage on the next load.]**
- **Fix:** the chain IS the customhouse — every conn node in chain C's
  table belongs to C's component by construction. Each conn node now
  borrows the chain's first non-null segment for resolution, so NULL-entry
  segments can't lose the zone. [C: implemented]
- **Audit diagnostic (load-time):** one line per zone —
  `customs: zone <id>: <n> segs, <k> conn seeds` with a
  `<-- NO MAINLINE SEED` marker, plus a skip-reason summary
  (`no-segment chain` / `segment not in set`). Expected after the fix:
  every rail customhouse zone has k ≥ 1; conn seeds ≥ 37. [C: implemented]

**Iteration-11 audit result (2026-08 test, eleventh run):** the repSeg fix
changed nothing — still `74 zone + 35 conn seeds`, and the audit shows only
**9 of 37 zones have any conn seeds at all** (ids 10,19,22,23,26,29,30,33,36).
The chains' +0xA10 tables simply don't yield mainline-connected nodes for
most customhouses: the +0x30 bridge exists for only ~1/4 of them. **[C]**
Both test approaches engaged seeded NEIGHBOUR corridors (id 33, id 19) and
overshot the actual unseeded target by kilometres — first-zero cannot find
a zero that was never seeded. **[C]**

### Probe T — coordinate bridge: SILENT (2026-08 test, twelfth run)

The mainline-node harvest never reached its coverage floor — nothing
printed. **[C]** Superseded by the geo match below (which needs no harvest:
the island nodes' coords stored at sweep time are the bridge data), and the
harvest/matcher were removed to keep the hot scan lean.

### Probe U — route objects: THE ANSWER (same run)

```
customs: route objs: 2 [14! 14! ], route left 24159 m, v=107 km/h
```

- **Customhouse chains appear in the train's route-object vector
  (train+0x680/+0x688, 8-byte entries) at assignment time — 24 km out.**
  **[C]**
- **The route is complete to the border from assignment.** Reinterpreting
  the earlier "route ends N m ahead" telemetry: those distances WERE the
  distance to the border (24 km at departure → 6050/4672/1485/1081/47 m
  approaching), not chunk-extension artifacts. The route extends PAST the
  border only after clearance. **[C]** (The "customhouse legs enter at
  ~115 m" fact still holds — the route's legs through the island appear
  late; the route's END sits at the boundary from afar.)
- H3 failed only because it compared the terminal node against ISLAND node
  pointers: the terminal node is a mainline node, a different object at
  the SAME world position as an island node. **[C]**

### Geo match — current mechanism

- At sweep time the island nodes' world coords (+0x04/+0x08/+0x0C) are
  stored alongside `g_custSeeds`. In the scan, when the route ends inside
  the horizon, the terminal leg's far node is matched BY POSITION against
  every island node (epsilon 2 m [I: pending the gap readout]; nearest gap
  is always reported for calibration). Match ⇒ the route ends at a customs
  boundary ⇒ distance to the entry = remaining route length (exact). [C: implemented]
- Priority: island handover (customs legs in route) > geo match > corridor
  tags. The corridor graph machinery is now the FALLBACK only — kept
  because the 9 conn-seeded zones' approaches are known-good and it covers
  whatever the geo epsilon might miss. The zone-run zero-parabola owns the
  last ~115 m as before. [C: implemented]
- Release: after clearance the route extends past the border, the terminal
  node sits beyond it, and the match releases by itself. The next border's
  customhouse is already in the route objects, and its boundary becomes
  the new route end — the second border brakes the same way. [I — watch in game]
- The direction-guard question evaporates: a customs boundary at the
  route's own END is for this train by construction. **[C]**
- Diagnostics: engagement `customs: route obj customhouse <D> m ahead
  (geo, gap <g> m), v=…`; no-match witness (customhouse in route objs +
  route ends in horizon + no island node within epsilon) `customs: route
  end <L> m, nearest customs node gap <g> m (no match), v=…` — the gap
  values calibrate the epsilon. [C: implemented]

- **Engagement (per scanned route leg):** travel-direction corridor distance
  must strictly descend (`exit tag < entry tag`; untagged = ∞, so fringe legs
  count as descending on entry, and any leg leaving a corridor breaks it).
  Every scanned leg must descend → the train is heading for the border.
  Target: `customs_entry_kmh` (default 50) at corridor distance 0, same
  aBudget parabola, 25 m margin. `sLeg + exitTag` is exact even beyond the
  scan horizon — the tags extend the effective reach to 8 km. **[I: in-game
  verification pending]**
- **Direction guard:** only a tagged ASCENT rejects — trains moving away
  from the border or turning onto tagged track that leads elsewhere. Legs
  with no tags (chain-local station/customs islands) are unknown, not
  evidence of leaving, and are carried over. Residual false positive: a
  route descending toward the entry that turns off onto UNtagged track
  keeps braking until an ascending or horizon-end leg says otherwise — a
  premature slowdown toward `customs_entry_kmh`, never a stop. **[I]**
- **Composition:** inside the zone (route extended, ~115 m out) the zone-run
  zero-parabola is stricter and wins the `min()`; corridor releases at the
  entry because zone legs ascend. After clearance the route leaves the
  corridor → no brake. **[I]**
- Diagnostics (one-shot per train per approach, log_curves):
  `customs: corridor entry <D> m ahead (raw, id <N>)|(zone), v=…` at
  engagement start (raw = tag-based estimate to locked component N,
  zone = island-handover route geometry);
  `customs: corridor on route, not descending toward entry (no brake), v=…`
  as the direction-guard witness. The limit tag is `(customs entry)`.
  Build-time diagnostics (always-on, logged when the seed sets change):
  per-seed `customs: zone|conn seed[i] <ptr>: +28=<p> +30=<p> -> <n> segs
  (<verdict>)` (first 5 of each set), per-chain-entry `customs: chain <p>
  entry[j]: seg=… n0=… n1=… e30=… (<relation>) e30segs=<n>` (first 3 chains ×
  2 entries, once per load) with a raw 0x60-byte entry dump and a 0x40-byte
  e30 dump when its adjacency is thin, then `customs: corridor build: <Z>
  zone + <C> conn seeds, <N> nodes visited, <T> segs tagged (<stop>)`.
  **[C: implemented]**
- Open: whether any route legitimately passes THROUGH customhouse track
  without stopping — such a train brakes to `customs_entry_kmh` at the
  border for no reason (bounded, no full stop). **[?]**

## Q7 — Station stops: same island disease, same cure

2026-08 cyclic test (Customhouse–Station–Station loop):

- `route objs: 6 [14! 1 1 14! 1 1 ]` — **stations appear in the route-object
  vector (train+0x680/+0x688) as type 1** (passenger), alongside the
  customhouses (0x14). **[C]**
- **The stop-intent bytes +0xD39/+0xD3C never fired all run** — zero
  `(stop ahead)` lines; the scheduled smooth stop never engaged. The bytes'
  setter (FUN_140680920) evidently doesn't cover this line setup. **[C]**
- Station track segments are **chain-local islands** like the customhouses:
  the train entered both station zones at 219/164 km/h with the 60 km/h
  zone cap biting only at the boundary (`curve: v=219 limit=60 (station
  zone)`), because station segments are invisible in the route array until
  late. **[C]**
- **The route array extends to the NEXT STOP from departure** (`route left
  1981 m` between stops) — same geometry as the customs case. Vanilla's
  stop blocks all require `+0x700 == routeLen−1`, so **route-terminal-at-
  station IS the stop intent**; no intent bytes needed. **[C]**
- Fix (implemented): station segment endpoint coords are collected with the
  station sweep (`stations: N track segments in station zones, M stop
  nodes`); when the route ends inside the horizon, the terminal node's
  position is matched against them (same 2 m epsilon — the customs gap
  measured 0.00 m) and a match engages the scheduled-stop parabola with
  the remaining route length. The intent-bytes path stays as OR-fallback.
  **[C: implemented]**
- Pass-through safety: a route that continues past a station doesn't
  terminate there, so no match; end-of-line reversal points (route ends at
  bare track) match nothing. Residual: a route chunk boundary coinciding
  with a pass-through station's node would falsely trigger a stop approach
  until the route extends — watch the cyclic test. **[I]**
- Still open: early braking toward pass-through station ZONES (the
  station_limit cap only sees in-route segments). The geo trick can't help
  there directly — a pass-through route doesn't terminate at the station.
  **[?]**

## Q8 — Decision flight recorder (diagnostics)

For the reverse-direction pulsing failure (customs perfect one way,
brake on/off pulsing and a fast entry the other), the plugin now has a
per-train event log behind `log_decisions` — no behavior change. Lines
(greppable prefix `railphysics  decision T…<hex>`):

```
decision T…a91f engage customs entry(geo) dist=6050 limit=253 v=257 gap=0.00 term=0x… routeleft=6050 horizon=6650
decision T…a91f target customs entry(geo) dist 6050->5100 limit=230 v=250
decision T…a91f routeleft=6050 term=0x… gapcust=0.00 gapstat=43.7 horizon=6650 corr=6050
decision T…a91f brake ON v=257 limit=253 (customs entry)
decision T…a91f snap v=240 limit=235 dist=4200 src=customs entry gapcust=0.00 gapstat=43.7
decision T…a91f release customs entry v=58 (routeleft=115 gapcust=0.00 gapstat=180.2)
```

Engage/release/target/routeleft fire only on scans (transitions); brake
ON/OFF and the 1 s snapshot also fire on cache-hit frames, so pulsing
appears as repeated ON/OFF pairs with the active source attached. The
geo-match gaps are now computed for BOTH sets whenever the route ends in
the horizon (engagement semantics unchanged) — the near-miss distance
above the epsilon is the datum the reverse-direction hunt needs. **[C:
implemented]**

### Q8 follow-up — the flight recorder's first catch (2026-08, iteration 15)

Reverse-direction customs approach, train T…53e0: corridor engagement at
4802 m, geo handover mid-approach (gapcust 0.00), then at the fence:

```
routeleft=501 term=0xD9C636A0 gapcust=0.00 corr=2071
routeleft=36  … corr=1757
engage customs stop dist=1719 limit=39 v=177     <- slam
```

Diagnoses, confirmed against the code:

1. **The decrease-rate bound caused the slam.** The clamp was
   `lo = prev − (traveled + 25); corrDist = max(min(raw, prev), lo)`. With
   the corridor estimate at ~2071 m and the geo match delivering the true
   501→36 m, `raw < lo` pinned the estimate to ~2 km all the way in — the
   train rode a 2 km parabola (limit ~172) until the zone-run fired at
   ~117 m. **[C]** Fix: the clamp is now a pure running MINIMUM within an
   engagement — never increase, downward corrections from any source adopt
   immediately. The geo formula itself was correct (pinned to routeLeft);
   the clamp buried it. The "4× disagreement" was the clamped corridor
   value being logged while geo had matched.
2. **The log conflated sources.** `engage customs stop dist=1719` paired
   the winning 117 m zone-run limit (39 km/h) with the corridor path's
   1719 m. The recorder now reports each source's OWN distance (zero-target
   distances exported separately). **[C: implemented]**
3. **Source flapping at the fence** (customs stop → entry → stop within a
   second) re-raised the limit mid-approach whenever the zone-run flickered
   out for a scan. Fix: all zero-target stop distances (zone-run, scheduled
   intent bytes, station geo) share one monotone `zeroClamp`; when the zero
   source flickers out but a stop engagement is still active, the stop
   parabola holds at the last distance decayed by travel. A spent clamp
   (≤25 m) releases so fresh sightings start clean. **[C: implemented]**
4. **Station geo-stop growth** (37→74→107→137 m as the route re-extends
   into the platform island in chunks) — the same never-increase clamp now
   covers it; the target stays at the first match. **[C: implemented]**

Reverse-direction root cause summary: the route reached the border late
(~500 m) in that direction, the corridor tags measured to a seeded
NEIGHBOUR 1.6 km beyond (this customhouse has no mainline seed), and the
clamp's decrease bound prevented the correct geo estimate from displacing
the corridor one. Any one of the three fixes (min-clamp) removes the slam;
the zeroClamp removes the fence flapping. **[C]**
