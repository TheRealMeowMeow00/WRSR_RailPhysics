# Testing — how every subsystem was verified, and how to re-verify

The only real oracle is a running game. This is the verification protocol
used during development, with the exact log lines that prove each piece.
`log_physics = 1`, `log_curves = 1` and `log_decisions = 1` while testing;
off for normal play. Record a whole run before changing anything: half of
what looked wrong mid-run in 2.0 development was the next stage of a
correct approach.

The logic that does not need the game — the approach core, curvature,
bogies, tilt — also runs in the offline harness (section 9) before any
build goes into the game.

## 0. Smoke test

Start the game. In `tesmioloader.log` (build 1.1.1.9):

```
railphysics  curve site at +006A8436 (sig CURVE)
railphysics  brake site at +006A8553 (sig BRAKE)
railphysics  slope site A at +006A862D (sig SLOPE_A)
railphysics  slope site B at +006A8676 (sig SLOPE_B)
railphysics  grid site at +001BDF8C (sig GRID)
railphysics  divisor call at +006A7970 -> fn +00698C50 (sig CALL_DIV)
railphysics  fuel calls at +006A784F/+006A7896 -> fn +006B3B50 (sig CALL_FUEL)
railphysics  car transform call at +006AFED6 (sig NODE_CALL)
railphysics  chain vector at +009E6A18 (sig CHAIN_VEC)
railphysics  accel/divisor call site redirected
railphysics  fuel (cruise) call site redirected
railphysics  fuel (station) call site redirected
hook ok      rail brake ...
hook ok      rail slope ...
hook ok      rail slope (downhill assist) ...
railphysics  slope branch opened both ways
hook ok      rail approach limit ...
hook ok      grid boost ...          (only if grid_boost > 1.0)
railphysics  tilt: car transform call at +006AFED6 redirected (...)
railphysics  10 subsystem(s) patched
```

A site whose signature is missing or not unique is logged as not located,
and only its subsystem is off; the rest still works.

## 1. Traction / mass

Empty locomotive on a straight: brisk start, acceleration visibly decaying
past ~60 km/h (the P/v cap). Then 20-40 loaded wagons (1500-3000 t): crawl
start at 0.05-0.15 m/s². In the log:

```
railphysics  v=43.2 km/h mass=2314 t P=10020 kW F=834 kN R=5 kN G=35 kN a=0.34 m/s2
```

Check `mass` matches the consist, `a` is small, `F` falls as v rises.

## 2. Gravity

Heavy train over a hill: uphill it bleeds speed; a weak consist stalls.
Downhill the train accelerates on its own — vanilla never did that. With
`grade_scale = 0.06`, a 10 % in-game ramp is an effective 6 % grade and a
wall for most consists. `G` in the physics line is the lead's grade times
the consist mass — a long train over a crest shows large swings while the
applied force, car by car, is much smaller.

## 3. Braking

Service braking from 100 km/h at 0.8 m/s² ≈ 480 m. No station/signal
overshoot with `brake_min_vanilla_ratio = 1`. Planned braking (curve,
station, stop) is visibly gentle, and softer for a consist without
passengers (`freight_brake_ms2`); protective braking stays vanilla-strong.

## 4. Curves

`log_decisions` shows a curve taking over (`engage curve limit=… v=…`) and
every short brake pulse at its limit. `log_curves` prints each curve slower
than 100 km/h once:

```
railphysics  curve T…3830: 43 km/h, R 50 m (window 18.0 m), 220 m ahead, run of 86 m
```

Reference points at `curve_lateral_ms2 = 2.8`: R=300 → ~104, R=650 → ~154,
R=1000 → ~190, R=2400 → ~295 km/h. Watch for:

- **no braking on straight track**; bridge joins may log
  `curve: N kink point(s) ignored ahead` — that is the filter working;
- **no `R 10 m`** — the artifact floor; report it with the window size;
- the limit held until the tail has left the curve (the train accelerates
  one consist length after the curve's end).

## 5. Stations, customs and stops

With `log_decisions = 1`, a customs stop reads:

```
decision T…8cb0 engage customs limit=335 v=277 len=209 passenger
decision T…8cb0 stop armed 98 m ahead (customs)
decision T…8cb0 engage stop limit=50 v=50 len=209 passenger
decision T…8cb0 snap v=15 limit=15 stop=6 odo=49185
decision T…8cb0 stop served 0 m from the target
decision T…8cb0 engage customs limit=50 v=0 len=209 passenger
decision T…8cb0 release customs v=50
```

— a parabola down to 50 at the zone, 50 held to the stop, the last metres
at 15, served at the target, 50 held on the way out for the consist's
length (+5 m), released. A station stop reads the same with `(station)` and
60; a pass-through train logs `engage station limit=60` and is released
one consist length after the platform. `stop dropped: the route continues
past it` is normal when a train stood before a customs and got its
clearance. What must never appear: a stop served tens of metres short, a
limit of 0 before the target, a snap with `v` far above `limit` inside a
zone.

## 6. Fuel / electric grid

Fuel difficulty on. Diesel: at a standstill the tank creeps down
(`idle_load`), at full climb it nears `fuel_load_max`. Electric:

```
railphysics  fuel: v=130 km/h Pmech=10000 kW load=1.00 pa=1.00 (electric)
```

`pa` is the grid's delivered fraction. `pa < 0.95` under hard acceleration
means the grid — not the physics — is throttling the train: raise
`grid_boost` or add substations on that stretch.

## 7. Bogies

On the first scan of a train, one line per car type put on its wheelsets:

```
railphysics  bogies: workshop_subscribed/3019370041/CR400AF_Tc: pivots 5.78 m / 4.13 m from the ends (were 3.00 / 3.00)
```

Base-game cars never appear (one-object models). Look at a long Workshop
car on a tight curve (R < 100 m): the wheels sit on the rails; on mainline
radii the difference is a few centimetres.

## 8. Tilt

Every 30 s with `log_physics`: `tilt: cars leaned up to 5.0 deg in the last
30 s`. A passenger train on a mainline curve above `tilt_min_kmh` leans
inward, rolls in and out smoothly, stands upright on straights and bridges;
a car coupled back to front leans the same way as its neighbours; freight
stays upright unless `tilt_freight = 1`.

## 9. The offline harness

`test/host_test14.cpp` maps `SOVIET64.exe` as an image — none of its code
runs — so `TsmPluginInit` resolves every signature against the real bytes.
It then builds track, facility and train objects at the offsets the plugin
reads, moves a train along them with a simple speed controller, and checks:

- the odometer against the metres actually travelled, at x1 and x4, across
  reversed segments;
- a two-zone customs run (entry at 50, never above 50 between the zones,
  never below 15 before the stop, the stop reached, 50 held for the tail),
  at x1 and x4 and as freight;
- a train standing short of its stop never pinned at 0;
- a station pass-through held at 60 until the tail leaves;
- curves: an R 500 arc, a 1.2° kink, a 0.15 m jog, a 190 m switch, and
  straight lines at 0/8/15/40° with node-only segments;
- a torn read of the route, the slot table under churn;
- bogies from the real CR400AF model files (skipped when not subscribed);
- tilt: sign, size, ramp, reversed cars, freight, low speed, x4, kinks, jogs.

```bash
LLVM_MINGW=/c/path/to/llvm-mingw GAME=/c/path/to/SovietRepublic ./build-windows.sh
```

It ends with `ALL PASS (0 failure(s))`. It tests the plugin's logic, not
the game: a pass says nothing about whether a hook fires in game.

## What was learned the hard way (regression traps)

- A curve discovered too late is a curve missed: the scan horizon must
  cover the full braking distance (adaptive; watch 250+ km/h trains).
- Mid-function hook stubs must be disassembled and compared to the site's
  expected bytes — a SIB-addressing replay bug (`[rdi+rbx+0x8c]` read as
  `[rdi+0x18c]`) crashed every grid connection until caught this way.
- `+0x6B8` route segments are index-parallel to the route nodes: start the
  walk at `+0x700`, not at a pointer match — a route passing a segment
  twice reads backwards otherwise.
- Never time anything with the wall clock: the game runs at x1–x4 and
  plugins rescale time. 1.x's stop timing froze trains at x4.
- `+0x7A4` counts from the entry end in both directions. A harness whose
  segments all run node0 → node1 cannot see that mistake — it now reverses
  every other segment.
- Test tracks along an axis hide heading bugs: `atan2(0, 0)` is exactly
  right along +z. The curve tests now include lines at an angle.
- A test map without the facility you changed tests nothing: the 2.0
  station logic went untested through two maps without a station.
