# Station Detection Findings — SOVIET64.exe 1.1.1.7

Companion to `RAIL_PHYSICS_FINDINGS.md` and `CURVE_FINDINGS.md`. Goal: decide, per route
segment ahead of a train, whether it belongs to a rail station (passenger/cargo), and the
distance to the station boundary along the route.

Artifacts: `out/btype_chain.asm`, `out/btype_reginit.asm` (building $TYPE→id strcmp chain),
`out/chain_fns.c` (FUN_1406b7a10 arrive-handler), `out/poi_fns.c` (FUN_1406692b0 /
FUN_140669490 / FUN_1406a64f0), `out/chain_xrefs.txt`.

Legend: **[C]** confirmed, **[I]** inferred, **[?]** unknown.

---

## Q2 first (it frames everything): building/chain type ids

`buildingObj+0x318` = type descriptor; **`descriptor+0x360` = int32 type id**,
`descriptor+0x368` = int32 subtype. The id is assigned by the building ini parser
`FUN_14010e200` (0x14010e200) via a strcmp chain on `$TYPE_*` keywords; values proven in
disassembly (`out/btype_chain.asm`, `out/btype_reginit.asm`):

| id | $TYPE | id | $TYPE |
|---|---|---|---|
| 0 | **CARGO_STATION** | 0x14 | CUSTOMHOUSE |
| 1 | **PASSANGER_STATION** | 0x17 | PUB |
| 2 | LIVING | 0x18 | SPORT |
| 3 | SHOP | 0x19 | HOSPITAL |
| 4 | SCHOOL | 0x1a | FIRESTATION |
| 5 | STORAGE | 0x1b | UNIVERSITY |
| 6 | FACTORY | 0x1C | **CONSTRUCTION_OFFICE_RAIL** |
| 7 | MINE_* (subtype in +0x368: 0 oil, 1 iron, 2 coal/uranium…, 4 gravel…) | 0x22 | GAS_STATION |
| 8 | FIELD | 0x27 | CITYHALL |
| 9 | FARM | 0x2a | SHIP_DOCK |
| 0xa | KINDERGARTEN | 0x60 | **WAITING_STATION** |
| 0xb | ENGINE | 0x6a | DEMOLITION_OFFICE |
| 0xc | CONSTRUCTION_OFFICE | 0x6c | REPAIR_OFFICE |
| 0xe | ROADDEPO | 0x12C (300) | *(plain track block-section, see below)* |
| **0xF** | **RAILDEPO** | | |

So the chain types seen in the rail update are: **0xF = rail depot**, **0x1C = rail
construction office** — they gate depot/construction stop behavior, *not* station stops.
**Stations are type 0 (cargo), 1 (passenger), 0x60 (waiting station).** 0x14 = customhouse
(also has platform track). **[C]**

Cross-checks: the fuel-station search in the vehicle dispatcher looks for 0x22 (gas
station) ✓; construction-office logic uses 0xC ✓; mine logic uses 7 + subtype ✓ —
so descriptor+0x360 ↔ parser id mapping is confirmed by multiple consumers.

---

## Q1/Q4 — Stations are "chain" objects; the chain table IS the track extent

**Global chain list: `DAT_1409e6a18` / `DAT_1409e6a20`** — a `std::vector<chain*>`
(8-byte pointers). **[C]** A "chain" is a contiguous track section object. The same object
doubles as the building for station/depot sections (the arrive handler treats it as the
station building: storage vector, vehicles-present vector). Station buildings are in this
list — the rail update's stop-scaling gate reads `chain->+0x318->+0x360 == 0x60 || 0xF`,
and the arrival handler promotes a chain into `train+0x4F0` (see Q3). **[C]**

Chain object layout (all [C] unless noted):

| Offset | Content |
|---|---|
| +0x318 | type descriptor ptr (→ +0x360 type id, +0x368 subtype) |
| +0x5D0 | **master path object** — same geometry layout as a track segment: polyline vector at +0x08/+0x10 (stride 0x18, xyz @ +0/+4/+8, cumdist @ +0x14), length float at +0x11C, class int at +0x120 (values 0xA/0x14 seen) |
| +0x604 | float — distance along the master path of the train currently engaging the section (block sections are single-occupancy) **[I]** |
| +0x610 / +0x618 | **POI table**, stride **0x21B8**: entry+0x00 float distance along master path; entry+0x04 int32 poi type; entry+0x08/+0x10 sub-entry vector (stride 0x10: {obj* @0 (valid if obj+0x44 != -1), float a @8, float b @0xC}) |
| +0xA10 / +0xA18 | **section membership table**, stride **0x60**: **entry+0x20 = segment ptr**, **entry+0x30 = node ptr** — the segments/nodes that make up the section, in order |
| +0xC70 / +0xC78 | vector of vehicle instances currently at this chain (pushed on arrival) |
| +0x970 / +0x978 | building storage vector (stride 0xE0; fuel/coal lookup for refuel) |
| +0xF90 / +0xF98 | vector of platform/slot sub-objects (each with own +0x5D0 path, +0x604, +0x610 POI table, +0xE04 int kind) |

POI types (entry+0x04) seen: **6 and 8 → station stop markers** (they trigger
FUN_1406692b0 / FUN_140669490, which compute a stop position on the master path from the
platform sub-entries and schedule the stop via FUN_140668e50) **[I: likely passenger vs
cargo platform]**; 10 (0xA), 0xC, 0x10 → signal kinds; 1, 0xE → other stop/waypoint kinds
(used in the station loader FUN_140142440). **[C for the dispatch, I for semantics]**

**"Type 300" (0x12C)** from the path walker's tail path: node+0x70/+0x78 is a vector of
chain pointers the node belongs to; the walker follows a chain's +0xA10 table to continue
the track *through* it when the table entry has no segment, but only when
`chain->+0x318->+0x360 == 300`. So 300 = the type id of a **plain track block section**
(no building) — chains through which track simply passes. **[I]** (300 is outside the
$TYPE id range, so plain sections use a separate/static descriptor.)

Node-side alternative: a route node that lies inside a station has its station chain in
**node+0x70/+0x78** (8-byte ptrs). So "node belongs to station X" = any chain in that
vector with station type id. **[C structure, I that stations always register their nodes
there — the walker relies on it]**

---

## Q3 — Distance to station; train+0x528 / +0x530 / +0x4F0 / +0x4F8 roles

Roles (from FUN_1406bc890 segment-transition + FUN_1406b7a10 arrive handler): **[C]**

- `train+0x528` = chain the train is currently **engaged with / approaching** (block
  reservation; its +0x604/+0x610 drive the signal-braking code in the rail update).
- `train+0x530` = chain used by the **stop-scaling logic** (depot/waiting-station/rail
  construction office; types 0xF/0x1C/0x60 gates).
- `train+0x4F0` = chain the train has **arrived at** (promoted from +0x528 by
  FUN_1406b7a10; `+0x528/+0x530` cleared; train pushed into chain+0xC70).
- `train+0x4F8` = same station object after arrival (station loader FUN_140142440 uses
  it: storage, platforms).

**Cleanest distance readout: none precomputed for arbitrary lookahead — compute it by
walking the route** (the game itself only tracks distance *within* the engaged chain via
+0x604). Route nodes don't know their station directly; nodes know their chains via
+0x70/+0x78 (see Q1). The +0x528 chain's +0x604 is "distance of this train along the
current block section's master path" — only valid while inside one section, so it's not a
station-distance source. **[C/I]**

Also confirmed here (bonus for the curve walker): **FUN_1406a64f0 proves +0x6A0 and
+0x6B8 are parallel**: it iterates `segs[i] = +0x6B8[i]` with cumulative distance
(`fVar4 += seg->+0x11C`) and computes direction as `seg->+0x28 == routeNodes[i]` — i.e.
**route node[i] = the node at the +0x28 end test for leg i**. **[C]**

---

## Recommended detection algorithm

```c
// ---- refresh pass (every few seconds, or on chain-list change) ----
struct StationInfo { void* chain; int type; };
unordered_map<void*, StationInfo> segStation;   // segment ptr -> station

for (void** p = *(void***)0x1409e6a18; p != *(void***)0x1409e6a20; ++p) {      // [C]
    char* chain = *p;
    char* desc  = *(char**)(chain + 0x318);
    if (!desc) continue;
    int type = *(int*)(desc + 0x360);                                          // [C]
    bool station = (type == 0 || type == 1 || type == 0x60);   // cargo/passenger/waiting
    // optionally also: 0x14 customhouse; 0xF rail depot; 0x1C rail construction office
    if (!station) continue;
    char* t = *(char**)(chain + 0xA10), *te = *(char**)(chain + 0xA18);        // [C]
    for (; t != te; t += 0x60) {
        void* seg = *(void**)(t + 0x20);                                       // [C]
        if (seg) segStation[seg] = {chain, type};
    }
}

// ---- per frame, inside the existing railphysics hook (RSI = train) ----
float distToStationRun(char* train, float* outRunLen /*distance to FAR boundary*/) {
    char* seg = *(char**)(train + 0x798);
    float rem = *(char*)(train + 0x7A0) ? *(float*)(seg + 0x11C) - *(float*)(train + 0x7A4)
                                        : *(float*)(train + 0x7A4);            // [C]
    char** v = *(char***)(train + 0x6B8);
    char** e = *(char***)(train + 0x6C0);
    size_t k = find(v, e, seg);                     // pointer match current leg [C]
    float d = 0; bool in = false; float start = -1, end = -1;
    for (char** q = v + k; q != e && d < LOOKAHEAD; ++q) {   // note: forward direction
        // (verify iteration order once at runtime; if distances come out negative,
        //  iterate the other way — +0x6B8 order vs travel is documented in
        //  CURVE_FINDINGS.md as "current leg by pointer match, forward = index++",
        //  direction per leg: seg->+0x28 == routeNodes[i] test from FUN_1406a64f0) [C/I]
        float len = *(float*)((*q) + 0x11C);
        bool s = segStation.count(*q);
        if (s && !in) { in = true;  start = d; }
        if (!s && in) { end = d; break; }
        d += len;
    }
    if (in && end < 0) end = d;
    float distHeadToNear = start < 0 ? FLT_MAX : start + (first-leg remainder handling: rem);
    ...
}
```

Precision notes:
- First leg: use `rem` (distance to leg end) instead of full length for `*q == current
  segment`. If the train is *already inside* a station run (current segment mapped),
  treat near distance as 0 and compute only the far boundary (for the exit-reacceleration
  rule).
- Query cost: one hash lookup per route segment per frame — a handful per train. Refresh
  cost: trivial (chains × their segments). **[I]**
- The walk naturally handles pass-through trains: they brake to the configured station
  limit before the near boundary and accelerate past the far boundary. Trains that stop
  anyway get vanilla's own braking on top (d28 forced to 0.5 etc.) — no conflict, the
  minimum applies.
- Braking budget: vanilla service decel is `100/(3·divisor)` km/h/s (railphysics
  findings) — compute the approach limit accordingly.

## Caveats / open items

- Whether *every* station register its platform segments in its +0xA10 table is inferred
  from FUN_1406bc890 (route building through chains) and the depot search — spot-check
  once at runtime by comparing a known station's segment set. **[I→verify]**
- Type-300 = plain block section is inferred from the walker; plain sections may or may
  not appear in DAT_1409e6a18 — irrelevant for station detection either way.
- POI types 6 vs 8 (passenger vs cargo stop markers) — semantics not fully pinned. [?]
- Metro/tram stations: metro uses its own station buildings; their $TYPE ids are outside
  the dumped chain range (strings like "metro_endstation" exist). If metro support is
  wanted, extend the table — the chain mechanism is shared. [?]
