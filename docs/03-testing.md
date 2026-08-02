# Testing — how every subsystem was verified, and how to re-verify

There is no test suite: the only oracle is a running game. This is the
verification protocol used during development, with the exact log lines
that prove each piece. `log_physics = 1` and `log_curves = 1` while
testing; off for normal play.

## 0. Smoke test

Start the game. In `tesmioloader.log`:

```
railphysics  accel/divisor call site redirected
railphysics  fuel (cruise) call site redirected
railphysics  fuel (station) call site redirected
hook ok      rail brake ...
hook ok      rail slope ...
railphysics  slope branch opened both ways
hook ok      rail curve limit ...
hook ok      grid boost ...          (only if grid_boost > 1.0)
railphysics  8 subsystem(s) patched
```

Any `does not match 1.1.1.7` line means a game update moved that site; the
subsystem is safely off, the rest still works.

## 1. Traction / mass

Empty locomotive on a straight: brisk start, acceleration visibly decaying
past ~60 km/h (the P/v cap). Then 20-40 loaded wagons (1500-3000 t): crawl
start at 0.05-0.15 m/s². In the log:

```
railphysics  v=43.2 km/h mass=2314 t P=10020 kW F=834 kN R=5 kN G=35 kN a=0.34 m/s2
```

Check `mass` matches the consist, `a` is small, `F` falls as v rises.

## 2. Gravity

Heavy train over a hill: uphill it bleeds speed; a weak consist stalls
(`stalled: ... grade=...%` in the log). Downhill the train accelerates on
its own — vanilla never did that. With `grade_scale = 0.06`, a 10 %
in-game ramp is an effective 6 % grade and a wall for most consists:
`F=848 kN G=1183 kN grade=5.2%`.

## 3. Braking

Service braking from 100 km/h at 0.8 m/s² ≈ 480 m. No station/signal
overshoot with `brake_min_vanilla_ratio = 1`. Planned braking (curve,
station, stop) is visibly gentle; protective braking stays vanilla-strong.

## 4. Curves

On a winding stretch, `log_curves` shows e.g.
`curve: v=110 km/h limit=94 km/h R=574 m` and the train enters the curve
at its limit. Reference points: R=300 → ~68, R=574 → ~94, R=1000 → ~124,
R=2400 → ~210 km/h (at `curve_lateral_ms2 = 1.4`). Watch for:

- **No braking on straight track** (no `curve:` lines at all),
- **No `R=10 m`** — that is the artifact floor; report if you see it,
- junction limits matching the *diverging* route,
- tunnel portals causing no phantom braking.

## 5. Stations and stops

Pass-through train: brakes in advance, holds `station_limit_kmh` through
the platform, accelerates after. Stopping train: smooth parabolic approach
(`(stop ahead)` in the log), crawling the last metres — no 20 m slam.
End-of-line and customhouse stops are smooth too.

## 6. Fuel / electric grid

Fuel difficulty on. Diesel: at a standstill the tank creeps down
(`idle_load`), at full climb it nears `fuel_load_max`. Electric:

```
railphysics  fuel: v=130 km/h Pmech=10000 kW load=1.00 pa=1.00 (electric)
```

`pa` is the grid's delivered fraction. `pa < 0.95` under hard acceleration
means the grid — not the physics — is throttling the train: raise
`grid_boost` or add substations on that stretch. Development history: a
10 MW trainset at vanilla-sized grid delivered `pa=0.48` (2.35 MW of the
requested 4.9), and it showed exactly as the in-game wattage gauge.

## What was learned the hard way (regression traps)

- A curve discovered too late is a curve missed: the scan horizon must
  cover the full braking distance (adaptive now; watch 250+ km/h trains).
- Single-interval curvature spikes on construction jags (tunnel portals):
  the 20 m window fixed it; if `R=10 m` returns, the window missed a case.
- Mid-function hook stubs must be disassembled and compared to the site's
  expected bytes — a SIB-addressing replay bug (`[rdi+rbx+0x8c]` read as
  `[rdi+0x18c]`) crashed every grid connection until caught this way.
- `+0x6B8` route segments are index-parallel to the route nodes: start the
  walk at `+0x700`, not at a pointer match — a route passing a segment
  twice reads backwards otherwise.
