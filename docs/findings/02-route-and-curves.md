# Curve / Route-Following Findings — SOVIET64.exe 1.1.1.7

Companion to `RAIL_PHYSICS_FINDINGS.md`. Goal: walk a train's path forward and sample
headings for curve speed limits. Artifacts: `out/pathwalker.c` (FUN_140521910,
FUN_14054be10, FUN_1406c1540, FUN_1406ac5d0), `out/route_fns.c` (FUN_1406bc800,
FUN_1406bc890, FUN_140521d70), `out/train_update.asm` (full byte-level disasm of the
rail update FUN_1406a7410).

Legend: **[C]** confirmed, **[I]** inferred, **[?]** unknown.

---

## Q1 — Path representation

### Track segment object (= world rail track piece; `instance+0x798` points to one) **[C]**

| Offset | Content |
|---|---|
| +0x08 / +0x10 | **polyline point vector** (begin/end), stride **0x18** per point |
| +0x20 | start node ptr (distance origin) |
| +0x28 | end node ptr |
| +0x38 / +0x40 | vector of **vehicle instances currently on this segment** (8-byte ptrs) |
| +0xA0 / +0xA8 | vector of connection records (stride 0x18; +0xC int link idx, +0x14 float distance) — junction linkage used by the lookahead in FUN_1406a7410 |
| +0x11C | float **length (m)** |
| +0x120 / +0x124 / +0x12C | int class ids passed to `FUN_1403bceb0` (infrastructure speed limit) |
| +0x129 | u8 flag (chain/crossing-related, read in FUN_1406bc890) |

**Polyline point (stride 0x18), from FUN_14054be10:** 

| Offset | Content |
|---|---|
| +0x00 | float x |
| +0x04 | float y |
| +0x08 | float z |
| +0x0C | float lateral-dir x (horizontal perpendicular, for gauge offsets) |
| +0x10 | float lateral-dir z |
| +0x14 | float **cumulative distance from segment start (m)** |

First/last interval endpoints come from the endpoint **node objects**: node position =
3 floats at **node+0x04/+0x08/+0x0C** (x,y,z). Node connectivity: **node+0x28/+0x30 =
vector of attached segment ptrs** (8-byte), node+0x70/+0x78 = connection objects,
node+0x88 = ptr to int index.

Interpolation between points is **piecewise linear** (FUN_14054be10 lerps x,y,z and the
lateral vector) — the polyline is the actual driven geometry, no spline evaluation at
runtime.

### Train instance route fields **[C]**

| Offset | Content |
|---|---|
| +0x528 / +0x530 | block-section ("chain") objects the train is engaged with (from global list `DAT_1409e6a18`; chain has +0x318→+0x360 type 0xf/0x1c, and a **+0xA10/+0xA18 table, stride 0x60: entry+0x20 = segment, entry+0x30 = node**) |
| +0x6A0 / +0x6A8 | **route vector: node objects**, 8-byte entries, in travel order |
| +0x6B8 / +0x6C0 | parallel-ish vector of **segment objects** (current segment pushed by the rail update at 0x1406a9xxx / route-rebuild in FUN_1406bc890) |
| +0x6D0 / +0x6D8 | related route vector (cross-checked against +0x6B8 tail in FUN_1406ac5d0) |
| +0x700 | int current route index into +0x6A0 |
| +0x7D0 .. +0xC80 | **occupancy ring: 75 entries × 0x10 bytes = {segment* @+0, u8 dir @+8}** — segments the train currently *straddles*, head first (entry 0 == +0x798), then rearward to the tail; rebuilt by FUN_1406bc890 on each segment transition. **Not** a future-path list. |
| +0x798 | current segment |
| +0x7A0 | u8 direction: 0 = position measured from +0x20-node end; 1 = measured from +0x28-node end (i.e. `distFromStart = dir ? seg->len - pos : pos`) |
| +0x7A4 | float position along segment (m, in travel-direction measure) |

Route entries being *nodes* is confirmed by FUN_1406bc800 (detects route[i+1]==route[i+2],
i.e. immediate revisit = reversal) and by FUN_1406bc890 writing `route[0x700] = nextNode`
where nextNode = `seg->+0x20/+0x28`. **[C]**

---

## Q2 — The existing path walker: `FUN_140521910` (0x140521910) **[C]**

Signature: `FUN_140521910(networkMgr DAT_1409e4818, segment, char dir, float distance, routeCtx, routeIdx)` — recursive. Called from the rail update (0x1406a7bca, 0x1406a7da0)
as `FUN_140521910(DAT_1409e4818, seg, dir, probeDist, 0, 0)` in a loop with probeDist
growing 1.5, 6.5, 11.5, … up to 500 m (DAT_14090af50).

Algorithm (from decompilation, `out/pathwalker.c`):
1. `node = dir ? seg->+0x20 : seg->+0x28` — the node we are heading toward.
   If `routeCtx != 0`: the next segment is taken from the route table instead:
   `*(routeIdx±1)*0x60 + 0x30 + *(routeCtx->+0xA10)` (chain table, stride 0x60).
2. If `0 <= distance <= seg->+0x11C`: call `FUN_140521d70(net, seg, distance)` — the
   per-segment probe (see below); return it if non-zero.
3. If distance overshoots/undershoots: recurse into **every** neighbor segment of `node`
   (`node->+0x28` vector, skipping the current segment), with
   `dir2 = (seg2->+0x28 == node)` and `dist2 = dist - segLen` (mirrored if entering via
   the +0x28 end). Returns first non-zero probe result. With routeCtx==0 it explores
   **all branches** (BFS-ish; fine for "any obstacle", wrong for "the train's path").
4. Tail path: follows node connection objects (node+0x70/+0x78, +0x88 index) through
   station/building chains (type 300) — track continuity through buildings.

`FUN_140521d70(net, seg, dist)` (0x140521d70): iterates `seg->+0x38` (vehicles on
segment), computes each train's occupied span `[pos−consistLen, pos]` (consist length =
Σ type->0x7a78; uses the +0x7D0 ring), returns the vehicle if `dist` is inside. So the
"obstacle lookahead" is really "next train ahead on the track". **[C]**

Other path walkers:
- `FUN_1406ac5d0` (0x1406ac5d0): signal/block occupancy at the upcoming route nodes —
  walks the +0x6A0 route vector forward from +0x700; route[i]->+0xB8 = signal object
  (+0x14 int type, 10 = none; +0x298/+0x2A0 block sections; +0x1C4..+0x1DC state
  floats/timers). This is the code to mirror for **walking the train's actual route**.
- `FUN_1406bc890` (0x1406bc890): segment-transition / route advance; maintains +0x6A0,
  +0x6B8, +0x700, +0x798, +0x7A0, +0x7A4 and the +0x7D0 ring.
- Signal braking table: route/chain object at +0x528 has +0x604 = current distance,
  +0x610/+0x618 = table stride **0x21B8** {+0x00 float distance, +0x04 int poi-type
  (10/0xC/0x10 = signal kinds)}; the rail update computes approach speed
  `(1-f)*23.0 + 2.0` with `f = 1 − distToSignal/5.0`.

### Recommended forward-walk for the curve plugin **[I, all pieces C]**

```c
// state: train inst (RSI in the hooked frame), all offsets confirmed
seg  = inst->0x798;  pos = inst->0x7A4;  dir = inst->0x7A0;
// find current leg in the route segment vector (robust against index drift):
segs = inst->0x6B8;  n = (inst->0x6C0 - inst->0x6B8) >> 3;
k = index_of(segs, seg);              // linear search for pointer equality
// heading samples:
for (float ahead = 0; ahead < 600; ahead += DS) {
    d = (dir ? seg->len(+0x11C) - pos : pos) + ahead_remaining_in_seg;
    while (d > seg->len) {            // advance to next route segment
        d -= seg->len;
        seg = segs[++k];              // may run past vector end -> stop sampling
        dir = (seg->+0x28 == shared_node_with_prev) ? 1 : 0;  // see walker rule
    }
    p = sample_polyline(seg, d);      // see Q3
    headings.push_back(atan2 of p - p_prev);
}
```

If the +0x6B8 vector turns out to be unreliable past the current leg, fall back to the
walker rule with the **route node vector**: next node = `route[min(0x700+1, …)]` from
+0x6A0; at that node pick the attached segment (node+0x28 vector) ≠ current whose other
node matches the following route entry. The game's own lookahead (all-branches
FUN_140521910) is NOT route-aware and will cut across junctions — avoid copying it
blindly. **[I]**

---

## Q3 — Curvature / heading data **[C]**

There is **no stored per-segment curvature, yaw, or start/end direction**. Headings must
be derived from positions:

- Per-segment polyline: positions at `*(seg->+0x08)` vector, stride 0x18, xyz at
  +0x00/+0x04/+0x08, cumulative meters at +0x14. Endpoint nodes extend the polyline by
  one point at each end (position at node+0x04/+0x08/+0x0C); the final interval's end
  cumdist = seg->+0x11C.
- The game's own sampler **`FUN_14054be10(idx, outVec3, seg, dist, dirFlag, lateral)`**
  (0x14054be10) returns the world position at `dist` (handles the dir mirror and lateral
  offset); it is what rail positioning uses (called from FUN_1406ace30). The plugin can
  call it directly, or inline the lerp (cheap).
- Per-point horizontal perpendiculars (+0x0C/+0x10) exist if a lateral-free heading is
  wanted: heading ⟂ (perpX, perpZ).
- Practical curvature: sample positions every DS ≈ 2–5 m for ~600 m ahead, compute
  yaw_i = atan2f(Δx, Δz-ish) per interval (watch the engine's axis convention: the
  lateral offset math treats the +0x04 component as vertical, so heading lives in the
  x/z pair at +0x00/+0x08 **[I]**), then curveLimit from min radius / max |Δyaw| per
  window.

---

## Q4 — Limit-finalization hook site in FUN_1406a7410 **[C]**

XMM9 lifecycle (all writes): `1406a7869 XMM9=XMM14` (consist top speed) → signal block
may set XMM9 (1406a7a18/7a46, restored via XMM12) → `1406a7ac0 MINSS XMM0,XMM9;
1406a7ac5 XMM9=XMM0` (min with infra limit from FUN_1403bceb0) → forced 0 if
[RSI+0x5df] (1406a7ad2) or FUN_1406c1540() (1406a7ae2) → `1406a81b5 XMM9=XMM11`
conditional min with next-train-ahead speed. **After 0x1406a81b5 XMM9 is never written
before the accel compare at 0x1406a8389** — but 0x1406a81b5 is conditional, so the
common-path finalization point is later:

### Site C (recommended): 0x1406a8326 — common path, XMM9 final

All control paths (horn block taken or not, obstacle clamp taken or not) merge here;
the next write to anything relevant is the accel/brake logic itself.

```
1406a8326  44 38 ae 41 0d 00 00       CMP  byte ptr [RSI+0xd41],R13B   ; 7 B
1406a832d  74 51                      JZ   0x1406a8380                  ; 2 B (rel8!)
1406a832f  f3 0f 10 b6 24 0d 00 00    MOVSS XMM6,[RSI+0xd24]           ; 8 B
```
17 stealable bytes (≥14), ends on instruction boundary. **No RIP-relative memory
operands** (CMP/MOVSS are RSI-relative). The only fixup: the `JZ rel8` → re-emit in the
trampoline as a near `JZ rel32` to 0x1406a8380 (trampoline must be within ±2 GB of the
image — allocate accordingly), or `jnz +5; jmp rax`-style absolute emulation.
After the trampoline body, resume at 0x1406a8337.

At the hook point: **RSI = train instance, XMM9 = final effective limit (km/h)**,
XMM15 = physics divisor, XMM14 = consist top speed, XMM11 = obstacle speed,
XMM12 = 0.15, R13 = 0. Apply `XMM9 = min(XMM9, curveLimit)` and optionally also stash
curveLimit for the d28 interaction (Q5).

### Site D (alternative, tighter to the compare): 0x1406a8380

```
1406a8380  f3 44 0f 10 85 38 02 00 00  MOVSS XMM8,[RBP+0x238]           ; 9 B
1406a8389  44 0f 2f 8e 24 0d 00 00     COMISS XMM9,[RSI+0xd24]          ; 8 B
1406a8391  76 53                       JBE  0x1406a83e6                  ; 2 B (rel8)
```
19 bytes, no RIP-relative operands, JBE needs the same relocation treatment; resume at
0x1406a8393. **Caveat:** when the roll-out flag [RSI+0xd41] is set, the block at
0x1406a832f…0x1406a837e ends with `JMP 0x1406a8389` — it **bypasses** 0x1406a8380, so
this site is skipped on roll-out frames (train already force-stopping; mostly harmless).
Site C has no such caveat.

### Verification anchors (bytes)

- `1406a8326: 44 38 ae 41 0d 00 00 74 51 f3 0f 10 b6 24 0d 00 00` (17 B, site C)
- `1406a8380: f3 44 0f 10 85 38 02 00 00 44 0f 2f 8e 24 0d 00 00 76 53` (19 B, site D)
- neighboring anchor: `1406a8389 44 0f 2f 8e 24 0d 00 00` = COMISS XMM9,[RSI+0xd24]
  (the accel/brake decision).

---

## Q5 — d28 (target speed) interactions **[C]**

d28 is written at exactly 7 sites in the rail update:

- `0x1406a860f: MOVSS [RSI+0xd28],XMM0` — the normal update:
  `d28 = (1 − brakeRes) * limit` where `limit` = XMM9 and `brakeRes` =
  obstacle-braking reserve (0…0.95·obstacle-proximity). **This is downstream of our
  clamp** — clamping XMM9 at site C automatically lowers d28 proportionally. Consistent:
  the throttle estimator (`sr = d24/d28`, load factor, fuel) then sees the curve limit
  as "the limit", so the train won't be treated as "at speed" while under a curve cap.
- Six station/stop-approach blocks force `d28 = 0.5 km/h` (MOV dword, imm 0x3F000000)
  at 0x1406a86a2, 0x1406a8763, 0x1406a88ff, 0x1406a89c5, 0x1406a8a5c, 0x1406a8b32 —
  these fire inside the last 20/10/7 m of a stop and **override** whatever d28 our clamp
  produced (they run after 0x1406a860f and also rescale/override d24 directly, e.g.
  `d24 = min(d24, scaledSpeed)` or 0 at the stop point). No conflict: a station stop is
  a stronger constraint than any curve limit.
- The actual distance advanced per frame uses `PowerKmh(fVar52)` where fVar52 is the
  post-station-scaling speed copy — so station logic can move the train slower than
  d24 suggests, but never faster than the limit path allows. Our clamp only lowers
  XMM9; it cannot be overridden upward by anything later in the frame. **[C]**

One caution: the obstacle/next-train brake block (0x1406a81b9–0x1406a8230) runs **before**
site C and consumes the *unclamped* limit; harmless (it only ever lowers d24 toward the
obstacle speed). The service-brake block (0x1406a83ef) is what will execute our curve
braking: decel = `100/(3·divisor)` km/h/s — the curve-limit computation should budget
braking distance with that rate (or the emergency ×6.67 rate if [RSI+0x5df] is set).
