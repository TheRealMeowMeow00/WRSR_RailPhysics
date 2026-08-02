# Rail physics — design: traction, braking, gravity, curves, stations, fuel

The vanilla rail update (`FUN_1406a7410`, per frame, speed in km/h at
`instance+0xD24`) drives a train with one scalar — the "divisor" from
`FUN_140698b40`:

```
accel       = 100 / divisor            km/h per second
brake       = 100 / (3 * divisor)      km/h per second  (x6.67 emergency)
uphill drag = 50 / divisor * slopeAvg  km/h per second  (uphill only)

divisor = 27.7 / sqrt(max(0.025*mass, power) / (5*mass))
```

i.e. a constant-by-speed acceleration of `3.61*sqrt(P/(5m))` km/h/s, braking
coupled to it at 1/3, and gravity that only pulls backward. Fuel burn comes
from engine power through a speed-ratio load curve
`(1-(v/vlim)^2)*0.85 + 0.15 + 0.35*slope`; `$MOVEMENT_CONSPUMPTION` is parsed
but never read anywhere. Full map of the vanilla code:
[findings/01-vanilla-rail-physics.md](findings/01-vanilla-rail-physics.md).

The plugin (`plugins/railphysics/`) replaces all four pieces with a physical
model, tunable from `railphysics.ini`:

```
traction    F = min(P*power_scale / v, mu * m_adhesive * g)   power, adhesion
resistance  R = m * (A + B*v) + C*v^2                         Davis, v in km/h
gravity     G = m * g * grade                                 signed, downhill pulls
accel       a = (F - R - G) / m
brake       independent m/s^2 rates, service and emergency
fuel        load factor follows mechanical power F*v, not the speed curve
```

Mass and power are the game's own aggregates: `FUN_140698e70` (empty weight +
cargo + 0.07 t/passenger, recursive over the consist) and `FUN_140698be0`
(wear-derated). Adhesive mass is the sum of empty weights of powered units
(type category 4 with power > 0), walked over the wagon vector at
`instance+0x3A0`.

## Mechanics

* **Acceleration — call-site rewrite**, `0x1406A7860` (the only call to the
  divisor function in the rail update) now reaches `RailDivisor`, which
  computes the model and returns `100/a`. The vanilla `0.001/D*100`
  arithmetic, its `powerAvail` multiplier and its clamp to the speed limit
  are untouched and apply our acceleration. A negative net force below
  walking speed reports a huge divisor (stall — vanilla cannot reverse); at
  speed it returns a negative divisor so the train settles onto its
  drag/gravity equilibrium instead of pinning to the limit.
* **Fuel — call-site rewrite**, `0x1406A773F` (cruise) and `0x1406A7786`
  (station). `RailFuel` converts mechanical power into a 0..1 load factor
  (`idle_load + (1-idle_load) * P_mech/P_rated`) and forwards to the original
  function, so tank bookkeeping, refuelling, the electric grid draw and the
  economy statistics stay vanilla. One deliberate change: station loading
  burns `idle_load` rather than the vanilla flat 0.65.
* **Braking — mid-function inline hook** at `0x1406A8443` (16 bytes). A stub
  (`stubs.S`) saves the volatile registers, passes vehicle/speed/vanilla
  increment/divisor/limit to `rp_brake_helper`, replays the stolen
  instructions with the returned decrement and jumps back. The helper scales
  the vanilla increment by rate ratio, so the frame-dt factor cancels and no
  timer access is needed. Two regimes: *planned* braking — the binding limit
  is a curve, station zone or stop parabola — uses exactly
  `service_brake_ms2`, so the train rides the braking parabola the lookahead
  computed; *protective* braking (signals, obstacles, station stops) keeps
  the `brake_min_vanilla_ratio` floor, because the game's own approach logic
  is tuned for vanilla-grade deceleration.
* **Gravity — branch patch + inline hook.** The slope block at `0x1406A8547`
  is gated `if (s < 0)` (uphill only); the `JBE` at `0x1406A8545` becomes a
  `JMP` so downhill executes it too (flat still skips on its own), and the
  apply site at `0x1406A8566` (21 bytes) is hooked to add a signed
  `g*grade*3.6*dt` delta instead of the power-scaled vanilla drag. The
  vanilla clamp to >= 0 at the continuation is kept — a train cannot roll
  backward, it can only be stopped by a grade.
* **Curves — mid-function inline hook at `0x1406A8326` (site C)**, the point
  where the effective limit in XMM9 is final on every control path. The stub
  clamps XMM9 to `min(limit, curveLimit)` and replays the stolen block; the
  stolen `JZ rel8` (roll-out path) is routed through a second absolute
  address slot. The clamp flows into `d28` downstream, so the throttle/fuel
  estimator treats the curve limit as "the limit".

### Curve scanning

The game has no curve speed limits and stores no curvature; it is derived
from the route geometry. A train's route is a segment vector in travel order
(`instance+0x6B8`, entered at the authoritative route index `+0x700` — a
route may pass a segment twice, so a pointer match is only a fallback), each
segment a polyline of `(x, y, z, cumdist)` points (stride 0x18, at `seg+0x08`)
between two junction nodes (`+0x20/+0x28`, positions at node+0x04/0x08/0x0C).
The sampler walks the polyline ahead; the window is the larger of
`curve_lookahead_m` and the full-stop braking distance from the current
speed plus 150 m, capped at 8 km (a 270 km/h train scans ~4.5 km — a fixed
window discovers a curve only when braking is already hopeless). Curvature
is measured over a **±10 m chord window** around each point
(`R = window / |Δheading|`), because single-interval derivatives spike on
the 1-3 m jags that tunnel portals and construction artifacts leave in the
polyline; heading steps over ~57° across a window are treated as route
reversals and skipped. Speed per curve: `v_c = sqrt(curve_lateral_ms2 * R)`,
and the applied limit is the braking curve over every point ahead:

```
v_allow(i) = sqrt(v_c(i)^2 + 2 * (service_brake/curve_brake_margin) * dist(i))
```

i.e. the train starts braking exactly as far ahead of the curve as its own
service brake needs, with a margin. Leg orientation comes from the dir byte
on the first leg and the shared junction node afterwards, so junctions follow
the *routed* branch — not the geometric one. Results are cached per train and
recomputed at most every 750 ms; the per-frame helper is a cache lookup under
a critical section. Yaw continuity across leg boundaries is kept, so a
junction's diverging curve is measured correctly. Known v1 limitation: a
mid-route reversal (dead-end station) is sampled as if the path continued —
station stop logic overrides it anyway.

### Electric grid feed

Electric traction demand is `hi_tot * load` from substations linked to the
current track segment; the delivered fraction scales acceleration. Vanilla's
load curve rarely asked for full power, so the vanilla feed chain — trafo
buffer with voltage sag `V = min(1, 2A/C)`, refilled as an ordinary class-9
receiver under the shared plant outflow ceiling `dt * capacity` — sufficed;
at honest full-power demand it delivers a stable fraction well under 1.
Two levers are provided: `grid_boost` multiplies that ceiling via a pure-asm
inline hook at `0x1401BDF1C` (replay the 16 stolen bytes, one extra `mulss`),
raising total grid throughput; `electric_load_scale` shrinks the request
instead. Demand is also divided by the number of substations linked to the
segment, so several trafos along one acceleration stretch genuinely add up.

### Station zones

Stations are chain objects in the global chain vector (`exe+0x9E6A18`); the
type id lives at `chain+0x318` → `+0x360` (0 = cargo, 1 = passenger,
0x60 = waiting), and the chain's membership table (`+0xA10`, stride 0x60,
entry+0x20 = segment ptr) is the station's track extent. The plugin rebuilds
a sorted segment set every 5 s (so stations built or bulldozed mid-game are
picked up) and the route sampler treats a station segment as a zone capped
at `station_limit_kmh`, fed through the same braking-curve budget as curves:
the train slows to the limit before the boundary and accelerates after its
head leaves it — including pass-through trains, which vanilla never slows at
all. Tail overhang is not tracked (head-based constraint).

### Smooth stops

A scheduled stop's stop point is the end of the route's last leg, and the
train advertises stop intent at `instance+0xD39` (next station) / `+0xD3C`
(further stops follow). Distance-to-stop is therefore just the remaining
route length, computed in the same leg walk. The plugin adds a braking
parabola to zero, `v_allow = sqrt(2*a*(dist-25m))`, which hands over to
vanilla's own last-leg ramp (d28 = 0.5, d24 rescale inside 20 m) at 25 m
out — vanilla's window only fires on the last leg, so the handover is clean.
This also smooths customhouse and end-of-line stops. Planned braking toward
curves/stations/stops uses exactly `service_brake_ms2` (no vanilla-strength
floor); the floor is kept only for protective braking (signals, obstacles),
which the game's own approach logic is tuned for.

The divisor/fuel rewrites redirect a `call rel32` at a near-cave thunk
(`mov rax, fn / jmp rax`, from `allocNear`); the original functions are
untouched and called by absolute address as fallbacks. Every site is checked
against 1.1.1.7's exact bytes before anything is written.

## Building on Linux

`build-linux.sh` (repo root) compiles the plugin with llvm-mingw/clang:

```
LLVM_MINGW=~/toolchains/llvm-mingw-20251118-ucrt-ubuntu-22.04-x86_64 ./build-linux.sh
```

It assembles `stubs.S` alongside the C++ and copies the ini beside the DLL.
The tesmioloader itself is a separate project and is not built here.

## Testing

Every subsystem above has been verified in game against the telemetry it
logs — see [03-testing.md](03-testing.md) for the full protocol, the exact
log lines that prove each piece, and the regression traps found the hard
way (late curve horizons, portal curvature spikes, stub replay
verification, route re-entry). Highlights from the validation runs: a
2314 t freight crawling at 0.34 m/s² and stalling on a 5.2 % effective
grade, a 380 t / 10 MW trainset reaching 330 km/h (the track limit) with
`power_scale = 1.35`, and `pa = 1.00` grid delivery under a full 13.5 MW
request with `grid_boost = 2.0`.
