# Rail Physics Findings — Workers & Resources: Soviet Republic (SOVIET64.exe 1.1.1.7)

Image base `0x140000000`. All RVAs below are absolute VAs (subtract 0x140000000 for file RVA).
Analysis artifacts (full decompilations/disassembly) are in `/home/meow/soviet_re/out/`.

Legend: **[C]** confirmed in decompilation/disassembly, **[I]** inferred, **[?]** unknown.

---

## Q1 — Vehicle type struct layout

### Type object

- The script.ini parser (`FUN_1403eb7d0`) builds a **0x9f08-byte** type object on the stack
  (Ghidra split it into locals; base = the `char local_b258[512]` buffer — the decompiler
  undersized it). **[C]**
- After parsing it is finalized by `FUN_1403f0530` and appended **by value** to the global
  vector at **`0x1409e7750`** (`{begin,end,cap}`, element stride **0x9f08**) by
  `FUN_1404485a0`. **[C]**
- Vehicle instances reach their type via **`instance+0x1708`** (pointer into that vector). **[C]**

### Field offsets (relative to type object base) **[C]**

| Offset | Content | ini key / source |
|---|---|---|
| 0x294 | int32 category (`FUN_1406633c0` maps `$TYPE`): 0 NOTSPECIFIED, 1 ROAD, 2 ROAD_SERVICE, 3 RAIL_VAGON, **4 RAIL_LOCOMOTIVE**, 5 RAIL_SERVICE, 6 SHIP, 7 CABIN, 8 AIRPLANE, 9 CONTAINER, 10 HELICOPTER | `$TYPE` |
| 0x7a60 | float, wear/dirt accumulation rate-ish | (parser line ~194) |
| 0x7a70 | float, used in fuel fn (horse-drawn check `>0`) | capacity-related |
| 0x7a78 | float vehicle length (m); summed over consist in the rail update | derived |
| 0x7c70 | float (passenger capacity-ish; used in price/fuel checks) | |
| 0x7c78/0x7c80 | vector, elem 0x110 (cargo capacity table) | |
| 0x85e0/0x85e8 | vector of int32 skill/usage ids (0x1b etc.) | `$SKILL_*` |
| 0x8600 | int32 (7 = passenger-type body, 2/8/0xd seen) | `$VEHICLETYPE_...` body class |
| **0x8620** | float `$MOVEMENT_CONSPUMPTION` — **parsed but never read anywhere in .text** (dead field) **[C]** | `$MOVEMENT_CONSPUMPTION` |
| **0x8624** | float top speed, **km/h** (`$MOVEMENT_SPEED`) | `$MOVEMENT_SPEED` |
| **0x8628** | float `$MOVEMENT_OFFROAD` (used by road update 140699310) | `$MOVEMENT_OFFROAD` |
| **0x862c** | float `$MOVEMENT_HILL_SPEED` — **never read in .text (dead)** **[C]** | `$MOVEMENT_HILL_SPEED` |
| **0x8630** | float `$MOVEMENT_HILL_ACCELERATION` — read only by road update `140699310` (as `0x8630` SUBSS at 14069c1dd) and airplane mover `1406ce060` (1406ce224). **Not used by the rail updater.** **[C]** | `$MOVEMENT_HILL_ACCELERATION` |
| 0x8634 | float gearbox[0] (`$GEARBOX_COMPLEX` elem 1); defaulted in `FUN_1403f0530` | `$GEARBOX_COMPLEX` |
| 0x8638 | float gearbox[1] (`$GEARBOX_BASE`, also `$GEARBOX_COMPLEX` elem 2). Defaults: if ≤0 → 750.0 (category 4) else 5000.0 | `$GEARBOX_BASE` |
| 0x863c | float gearbox[2]; default = 0x8638 × 0.1 | `$GEARBOX_COMPLEX` |
| 0x8640 | float gearbox[3]; default = 0x8638 × 0.35 | `$GEARBOX_COMPLEX` |
| **0x8678** | float engine power, kW (`$MOVEMENT_POWER_KW`) | `$MOVEMENT_POWER_KW` |
| **0x867c** | float empty weight, tonnes (`$MOVEMENT_EMPTY_WEIGHT`) | `$MOVEMENT_EMPTY_WEIGHT` |
| **0x8680** | u8 **electric flag** — set by `$TRAINGROUP_METRO` or `$PARTICLE_MOVEMENT train_eletric` | |
| **0x8681** | u8 **steam flag** — `$TRAINGROUP_LOCOMOTIVE_STEAM` / `_TRACKBUILDER_STEAM` | |
| **0x8682** | u8 **horse-powered flag** — `$HORSE_POWERED` | |
| **0x8684** | float **coeff_lo** (derived; fuel-tank capacity / idle baseline) | see below |
| **0x8688** | float **coeff_hi** (derived; consumption-rate coefficient) | see below |
| 0x868c/0x8690 | float min/max seconds between electric-spark sounds | |
| 0x8694 | sound handle (spark sound) | |

`FUN_1403f0530` (finalizer, 0x1403f0530): swaps 0x8634/0x8638 so 0x8634 ≤ 0x8638;
if 0x8638 ≤ 0 → 750 (cat 4) / 5000 (others); if 0x8634 ≤ 0 → 0x8638×0.25;
if 0x863c ≤ 0 → 0x8638×0.1; if 0x8640 ≤ 0 → 0x8638×0.35. **[C]**

### Derived coefficient qword at 0x8684/0x8688 (parser lines 627–683 of `out/parser_full.c`) **[C]**

`fVar4 = 1000.0` (DAT_14090b094). For **category 4 (rail locomotive)**:

```
P = MOVEMENT_POWER_KW / 1000.0                     // MW
coeff_hi (0x8688) = (P / 150.0) * 0.7              // DAT_14090ac38=150, DAT_140909e6c=0.7
coeff_lo (0x8684) = P * 5.2                        // DAT_14090a6d4=5.2
if steam (0x8681):  coeff_lo *= 3.0; coeff_hi *= 3.0   // DAT_14090a45c=3.0
if electric (0x8680): coeff_lo = 0; coeff_hi = P * 0.49  // 0.7*0.7
```

Category 1 (road): same base formulas; additionally if top speed < 1.0 → `lo=0, hi=P*0.49`;
horse vehicles (`$SINGLE_HORSE_POWER` > 0): `lo=0.047 (DAT_140909c20), hi=7.5e-05 (DAT_140909a9c)`.
Categories 6 (ship): `P /= 1`; 8 (airplane): `P /= 10000`; 10 (helicopter): `P /= 2000`;
then same 150/0.7/5.2 formulas.

Semantics (from runtime use, Q3): **coeff_lo = onboard fuel-tank capacity** (tonnes of fuel)
and **coeff_hi = fuel/energy burn rate per second at load factor 1.0**. For a 4000 kW diesel
loco: tank = 20.8 t, burn = 0.01867 t/s at full load (~67 t/game-hour).

### Vehicle instance fields (base = instance ptr) **[C]**

| Offset | Content |
|---|---|
| 0x398 | parent vehicle (wagons delegate to consist head) |
| 0x3a0/0x3a8 | std::vector of attached wagon instance ptrs |
| 0x3b8/0x3c0 | cargo vector, stride 0x10: {resource*, float weight(t) @+8} |
| 0x488/0x490 | vector of child vehicles (recursed into mass) |
| 0x4a8/0x4b0 | passengers vector (each counts 0.07 t — DAT_140909c58) |
| 0x4f0 / 0x4f8 | current station/loading object |
| 0x528 | route/chain object (signal block profile) |
| 0x5f0 | **onboard fuel store (t)** |
| 0x5fc | u8 "fuel empty" flag (forces power availability 0) |
| 0x5de / 0x5df / 0x5e0 | u8 flags; 0x5df = hard-stop/emergency brake request |
| 0x700 | int current route-segment index |
| 0x798 | current track segment object ptr (segment length float at +0x11c; class ints at +0x120/+0x124/+0x12c) |
| 0x7a0 | u8 direction on segment |
| **0x7a4** | float position along current segment (m) |
| **0xd24** | float **current speed, km/h** |
| **0xd28** | float effective target/limit speed, km/h (set to `(1-brakeFactor)*limit` each frame) |
| **0xd30** | float **current slope** = node pitch / 0.12 (rail; DAT_140909ca0, others /0.15 DAT_140909cb8), clamped ±1 ≈ grade/12 % |
| 0xd39/0xd3a/0xd3b/0xd3c | u8 station-approach / braking state flags |
| 0xd41 | u8 "forced roll-out" flag (applies −0.02×1000 = −20 km/h/s down to 0.5 km/h floor) |
| 0x1708 | vehicle type ptr |
| 0x1770 | gearbox/engine sub-object (0x40 bytes, created for cat 1 & 4 by `FUN_14067df10`) |
| 0x1778 | float engine power/throttle factor 0..1 (smoothed) |
| 0x177c / 0x178c | float throttle estimate current/previous frame |
| 0x1780 / 0x1784 | float smoothed electric power gauges (GUI) |
| 0x1794 | int "inactive/dead" flag (excludes wagon from consist sums) |
| 0x1a04 | int movement mode (0 = normal path follow; 1,2,3 = special; else speed forced 0) |
| 0x1a40 | double accumulated wear (see power derating) |

---

## Q2 — Rail movement simulation

### Update chain **[C]**

Per frame, per vehicle: `FUN_14066d460` (vehicle dispatcher, 0x14066d460)
→ category 3/4/5 (rail) → **`FUN_1406a7410` (rail update, 0x1406a7410, ~21k addrs)**.
Wagons (instance+0x398 ≠ 0) delegate (`FUN_1406a5300`). Movement-mode (0x1a04) dispatch for
the *path follower* lives in `FUN_1406cccd0` — **but that whole branch is category 8
(airplane)**, as is `FUN_1406ce060` (the substepping mover that reads HILL_ACCELERATION).
For rail, position is advanced inside `FUN_1406a7410` itself.

### Speed integration (all in FUN_1406a7410, speed in km/h) **[C]**

Time scaling: `C3D_TIMER::PowerTime(x)` (C3DDLL64.dll 0x1800fd670) = `x * 1000.0 / fps`
= x × frame-ms; integrated per frame this yields `x * 1000` per second. `fps` =
timer field +0 (game-speed-scaled), so physics scales with game speed. **`[C]`**
`C3D_TIMER::Power(x)` = `x * 30.0 / fps`. `PowerKmh(x)` = `x/3.6 * 1000/fps * 0.001 * unitScale`
= distance-in-km per frame for x in km/h (used to advance position / subdivide).

Per frame (RSI = vehicle instance):

```
// --- consist aggregates
slopeAvg = (Σ wagon->0xd30 + self->0xd30) / (wagonCount + 1)      // signed, ±1 ≈ grade/12%
slopeUp  = max(slopeAvg, 0)
topSpeed = self.type->0x8624;  for each wagon w/ type->0x294==4:  topSpeed = min(topSpeed, w.type->0x8624)
           // NOTE: only *locomotive* wagons (cat 4) lower it; plain vagons (cat 3) do not [C]

// --- engine load factor (throttle estimate), only if d28>1 and d24>1
sr   = clamp(d24 / d28, 0, 1)
load = (1 - sr*sr)*0.85 + 0.15 + slopeUp*0.35      // DAT_140909eec=0.85, DAT_140909cb8=0.15, DAT_140909d7c=0.35
powerAvail = FuelConsume(self, load)               // FUN_1406b3a40, normally 1.0
if at station loading (0x4f0) and has cargo list:  FuelConsume(self, 0.65)   // DAT_140909e44
if type->0x8684 > 0 and self->0x5fc != 0:          powerAvail = 0            // out of fuel

// --- mass/power divisor, fVar42 = FUN_140698b40(self):
mass  = TotalMass(self)      // FUN_140698e70: empty weight + cargo weights + 0.07*passengers + recursive wagons/children
power = TotalPower(self,1)   // FUN_140698be0: type->0x8678 (+ motorized cat-4 wagons recursively),
                             // wear-derated if wear setting on (DAT_1409d54cc>0):
                             //   wf = 1 - wear(0x1a40)/maxWear; if wf>0.5: power *= 1.5-wf   (up to -50 %)
if type->0x294 == 1 && type->0x8680:  power *= 3.2   // DAT_14090a4b8 — NOTE: category 1 = ROAD
                                                     // (trolleybus!). Electric RAIL gets NO bonus. [C]
eff     = max(mass * 0.025, power)                   // DAT_140909be4=0.025
divisor = (0.277 / sqrt(eff / (mass * 5.0))) * 100.0 // DAT_140909d2c=0.277, DAT_14090a338=5.0(double), DAT_14090ab88=100

// --- speed limit, fVar43:
limit = topSpeed
<signal/route block: if approaching restrictive signal within its braking table:
      limit = min(limit, (1-f)*23.0 + 2.0) with f = 1 - distToSignal/5.0>   // DAT_14090a974=23, DAT_14090a298=2, DAT_14090a6c0=5
infra = SpeedLimitFUN_1403bceb0(seg->+0x120,+0x124,+0x12c, ...)  // track/bridge/tunnel class limit, km/h
limit = min(limit, infra);  if self->0x5df: limit = 0;  if FUN_1406c1540(self): limit = 0

// --- obstacle lookahead (0..500 m ahead on path, FUN_140521910), target obstacleSpeed fVar47
if obstacleSpeed < limit:                       // brake toward obstacle
    f = clamp((d24 - obstacleSpeed)/10.0, 0, 1) // DAT_14090a840=10
    brakeRes = f * 0.95                          // DAT_140909f40=0.95
    d24 = max(d24 - PowerTime(0.15)*f, obstacleSpeed)
if self->0xd41:  d24 = max(d24 - PowerTime(0.02), 0.5)   // forced roll-out, DAT_140909bdc=0.02

// --- ACCELERATION (1406a8389–1406a83e4)
if d24 <= limit:
    d24 += PowerTime((0.001/divisor)*100.0) * powerAvail        // = +100/divisor km/h per second * pa
    d24 = min(d24, limit)

// --- BRAKING (1406a83ef–1406a845a)
if limit < d24:
    if self->0x5df: divisor *= 0.15             // emergency: 6.67× stronger
    d24 -= PowerTime((0.001/(divisor*3.0))*100.0)   // = −100/(3*divisor) km/h per second (DAT_14090a45c=3)
    d24 = max(d24, limit)

// --- power starvation (electric grid sag), 1406a845e+
if powerAvail < 0.15:
    d24 -= PowerTime((0.001/divisor)*100.0) * (1 - powerAvail/0.15); clamp ≤ topSpeed, ≥ 0

// --- slope resistance, 1406a84e3+   (slopeAvg>0 = uphill only)
s = slopeAvg * (-0.5)                            // DAT_14090b2e8=-0.5
if s < 0:  d24 = max(d24 + PowerTime((0.001/divisor)*100.0) * s, 0)
            // uphill drag = −50/divisor × slopeAvg km/h per second
            // (downhill gives NO acceleration; the s>1 branch is dead code since |slopeAvg|≤1)

d28 = (1 - brakeRes) * limit
<station-approach blocks: within 20/10/7 m of stop point (DAT_14090a940=20, DAT_14090a840=10,
  DAT_14090a7c8=7) scale speed by remaining-distance fraction and force d28 = 0.5>

// --- throttle smoothing + position advance
self->0x1778 tracks clamp01(min(1,load)*powerAvail*(1-sr²))  at ±Power(0.3)/frame rate (skipped if gearbox obj exists)
dist = PowerKmh(d24)           // km this frame
FUN_1406ace30(self, max(dist,1.5-ish))  // 0x7a4 += dist (capped), handles segment transitions
```

Effective magnitudes (diesel, power-dominated): **accel = 3.61·sqrt(power_kW/(5·mass_t)) km/h/s**,
service brake = accel/3, emergency brake = 2.22·accel, uphill drag = 1.80·sqrt(power/(5·mass))·slopeAvg.
Example 4000 kW / 82 t loco: accel ≈ 11.3 km/h/s (3.1 m/s²); brake ≈ 3.8 km/h/s. **[C formulas, I interpretation of units]**

### Slope semantics **[C]**

`instance+0xd30` is written every frame in `FUN_14066d460` from the node orientation
(`pitch/0.12` for cat 3/4, clamped ±1). For rail the **only** slope effect is the uphill
drag above and the `+0.35*slopeUp` term in engine load. `$MOVEMENT_HILL_SPEED` (0x862c) and
`$MOVEMENT_HILL_ACCELERATION` (0x8630) are **dead for rail** (used only by road/airplane
updaters). Track/loco speed limits: limit = min(consist loco top speed, infrastructure limit
`FUN_1403bceb0`, signal targets). Infrastructure call site in the rail update: **0x1406a7abb**.

### Mass **[C]**

Total train mass (`FUN_140698e70`, 0x140698e70) = Σ over loco + wagons (recursive) of
`type->0x867c` (empty weight) + Σ cargo entry weights (instance+0x3b8 vector, per-resource
precomputed tonnes) + 0.07 t × passenger count. It enters acceleration/braking/slope-drag
only through the `divisor` above. There is no tractive-effort/adhesion model, no curve
resistance, no air drag.

---

## Q3 — Fuel / energy consumption

Gate: global int **`DAT_1409d54c0`** must be ≥ 2 (game difficulty/setting "fuel
consumption"); otherwise `FUN_1406b3a40` returns 1.0 immediately (no burn, full power). **[C]**

### Fuel function `FUN_1406b3a40(vehicle, float load, char flag)` (0x1406b3a40) **[C]**

```
lo_tot = Σ_{consist, active} type->0x8684      // tank capacities
hi_tot = Σ_{consist, active} type->0x8688      // burn-rate coefficients
w_tot  = Σ (veh->0x5f0 + type->0x8684 * 0.5)   // DAT_140909df4=0.5 — stored fuel + half tank as weight
if hi_tot <= 0: return 1.0
if load < 0.001 and lo_tot > 0: return 1.0     // DAT_140909b14=0.001 — no burn at idle

if lead type->0x8680 == 0:   // ---- DIESEL / STEAM (burn from onboard tanks)
    if lo_tot <= 0: return 1.0
    for each consist vehicle v (index -1 = self) with type->0x8684 > 0, not electric, active:
        // if v is a powered loco with multiple engine configs, scale by engine count ratio (lines 4230-4246)
        share = ((v.lo*0.5 + v->0x5f0) / w_tot) * hi_tot
        if share > 0:
            burn = PowerTime(0.001) * share * load        // = share*load per second (t)
            if v->0x5f0 - burn < v.lo * -0.5: burn = v.lo*0.5 + v->0x5f0
            resource = DAT_1409e11f0                       // "fuel" (diesel)
            if horse(0x8682) or has cargo cap or type->0x7c70>0: resource = DAT_1409e1208
            if steam(0x8681): resource = DAT_1409e11f8     // "coal"
            FUN_1402fda60(&registry, &resource, &DAT_1409e6348, 0,0,1.0,0)  // statistics/economy record
            v->0x5f0 -= burn
        // clamp tank to ≥ -0.5*lo (allows slight negative reserve)
        if v->0x5f0 <= v.lo*(-0.5): v->0x5f0 = v.lo*(-0.5)
    return 1.0

else:                          // ---- ELECTRIC (draw from grid)
    demand = PowerTime(0.001) * hi_tot * load              // energy this frame
    seg = vehicle->0x798 (track segment power link)
    if seg == 0:
        if type->0x294 != 1 and vehicle->0x67d == 0: return 0.0   // no grid → no power
        <special cases: 0xc80/0x528/0x4f0 → return 1.0 or 0.0>
    else:
        distribute demand over substation sources (src fields: +0x10e0 stored charge*,
          +0x10e8 voltage, +0x10f8 capacity, +0x1100, +0x10f0/+0x112c consumption stats);
          per source draws min(charge*voltage, share) if voltage ≥ 0.1
        smooth gauges into vehicle->0x1780/0x1784 (Power(0.1)/Power(0.3) slew)
        return fVar28 = delivered fraction of demand (0..1)     // ← scales acceleration
```

So consumption is **computed purely from engine power** (via the parser-derived 0x8684/0x8688);
`$MOVEMENT_CONSPUMPTION` is never used. Burn rate ≈ `hi_tot × load` per second where
`load = (1-sr²)*0.85 + 0.15 + 0.35*slopeUp` (rail). Diesel: ~0.00467·P_MW t/s at full load;
steam ×3; electric: 0.49·P_MW units/s from the grid. **[C]**

### Refueling **[C]**

`FUN_140142440` (station/depot servicing, 0x140142440), entry block: if fuel mode active
(hidden arg `[rcx+0x5b8/4]==2`) and `veh->0x5f0 < type->0x8684` (tank capacity): pull
`min(deficit, available)` from the station building's storage (fuel resource id
`in_RCX[0x185c]`; steam locos take coal `in_RCX[0x185d]` + a second resource) into `0x5f0`.
The dispatcher (`14066d460`, lines 268-399) makes a low-fuel rail loco
(`fuel < 0.4*tank`, DAT_140909da0=0.4) search the route for a station stocking fuel
(DAT_1409e11f0) / coal+water (DAT_1409e11f8, DAT_1409e1298) and sets the refuel target
(vehicle+0x17a, flag 0xd3a). **Tank capacity = type->0x8684 = P_MW×5.2 (×3 steam).** **[C]**

---

## Q4 — Hook feasibility (bytes recorded, SOVIET64.exe 1.1.1.7)

All sites verified against `out/train_update.asm` / `out/hook_sites.asm`. 14-byte absolute
jump assumed. "RIP-safe" = stolen window contains no RIP-relative instruction.

### H1 — Rail acceleration hook (best single point) — `0x1406a83b7` in FUN_1406a7410

```
1406a83b7  f3 44 0f 10 9d 30 02 00 00   MOVSS XMM11,[RBP+0x230]   ; powerAvail
1406a83c0  f3 41 0f 59 c3               MULSS XMM0,XMM11
```
Exactly 14 bytes, RIP-safe. At entry: XMM0 = raw accel increment this frame (km/h),
XMM15 = divisor, XMM9 = limit, XMM14 = consist top speed, RSI = vehicle instance.
Recompute/replace XMM0, then `jmp 0x1406a83c5`.

### H2 — Brake hook — `0x1406a8443`

```
1406a8443  f3 0f 5c f0                  SUBSS XMM6,XMM0
1406a8447  f3 0f 11 b6 24 0d 00 00      MOVSS [RSI+0xd24],XMM6
1406a844f  44 0f 2f ce                  COMISS XMM9,XMM6
```
16 bytes, RIP-safe. XMM0 = decel this frame, XMM6 = speed after braking.
(Also covers emergency braking; check flag byte [RSI+0x5df].)

### H3 — Mass/power divisor hook — `0x140698b40` (FUN_140698b40 entry)

```
140698b40  40 53                        PUSH RBX
140698b42  48 83 ec 30                  SUB  RSP,0x30
140698b46  0f 29 74 24 20               MOVAPS [RSP+0x20],XMM6
140698b4b  48 8b d9                     MOV  RBX,RCX
```
Exactly 14 bytes, RIP-safe (next insn at 140698b4e is a direct CALL, not stolen).
RCX = vehicle. Return the divisor in XMM0 — or ignore the ABI and return your own
physics denominator. Called by rail/road/airplane updaters (filter by
`((type*)[RCX+0x1708])->0x294 == 4` for rail locos).
Call-site alternative: 5-byte direct call at `0x1406a7860` (`e8 db 12 ff ff`) — rail-only.

### H4 — Fuel/energy hook — `0x1406b3a40` (FUN_1406b3a40 entry)

```
1406b3a40  48 8b c4                     MOV RAX,RSP
1406b3a43  55                           PUSH RBP
1406b3a44  41 54                        PUSH R12
1406b3a46  41 55                        PUSH R13
1406b3a48  41 56                        PUSH R14
1406b3a4a  41 57                        PUSH R15
1406b3a4c  48 8d 68 b8                  LEA RBP,[RAX-0x48]
```
16 bytes, RIP-safe. RCX = vehicle, XMM1 = load factor, R8B = flag; return power
availability (0..1) in XMM0. Used by rail + road + ship + heli updaters — filter by
category. Rail-only call sites: `0x1406a773f` / `0x1406a7786` (5-byte direct calls
`e8 fc c2 00 00` / `e8 b5 c2 00 00`).

### H5 — Slope-drag hook — `0x1406a851d` / `0x1406a8566`

```
1406a851d  f3 41 0f 59 c2               MULSS XMM0,XMM10
1406a8522  f3 0f 58 86 24 0d 00 00      ADDSS XMM0,[RSI+0xd24]
```
9+8 = 17 bytes from 0x1406a851d (XMM0 = slope delta this frame, XMM10 = slopeAvg×−0.5,
XMM15 = divisor). RIP-safe. (1406a84fe dead-branch accel variant at 1406a8517's twin.)

### Data-constant rewrite options (all .rdata, float unless noted) **[C values]**

| Address | Value | Effect | Shared? |
|---|---|---|---|
| 0x140909d2c | 0.277 | divisor scale (↓ = faster accel) | check |
| 0x14090ab88 | 100.0 | global accel/brake scale | **yes, widely used — avoid** |
| 0x14090a338 | 5.0 (double) | mass divisor inside sqrt | likely unique |
| 0x140909be4 | 0.025 | min power/mass floor | likely unique |
| 0x14090a4b8 | 3.2 | electric ROAD power bonus | likely unique |
| 0x14090a6d4 | 5.2 | fuel tank per MW (parser) | few users |
| 0x14090ac38 | 150.0 | burn-rate denominator per MW | few users |
| 0x140909e6c | 0.7 | burn-rate numerator | **shared** |
| 0x14090a45c | 3.0 | steam burn ×3 / brake ÷3 | **shared** |
| 0x140909eec / 0x140909cb8 / 0x140909d7c | 0.85 / 0.15 / 0.35 | engine load curve | shared |
| 0x14090b2e8 | −0.5 | uphill slope drag factor | shared |

Prefer code hooks; most float constants are pooled and reused elsewhere.

---

## Open items / not verified

- FUN_1403f03b0 (called by TotalPower FUN_140698be0 when the vehicle has a hill-config list
  or type->0x7c70>0) — adjusts power by an unidentified factor. **[?]**
- Where instance+0x5f0 is initialized at purchase (tank presumably starts full); many
  writers exist (see out/offset_uses2.txt). **[?]**
- The rail gearbox sub-object (instance+0x1770, FUN_14067e220) only feeds
  throttle smoothing (0x1778) and RPM/sound for cat 4; no effect on tractive physics. **[I]**
- DAT_1409d54c0 exact semantics (setting id; 2 = fuel on). Written in FUN_14028c700. **[I]**
- FUN_1406c1540 (forces limit 0) and FUN_1406ac5d0 (stop-at-position check) — condition
  helpers not fully decoded. **[?]**
