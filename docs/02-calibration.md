# Calibration — railphysics.ini reference

Every key, what it does, and how to tune it with telemetry. The file is
UTF-8 **without BOM** — never save it with PowerShell's `-Encoding UTF8`
(the BOM silently resets everything to defaults).

All subsystems default to on and can be disabled independently; anything
disabled behaves exactly like vanilla.

## Subsystem switches

| Key | Effect |
|---|---|
| `accel` | traction/resistance/gravity acceleration model |
| `brake` | independent brake rates + planned/protective regimes |
| `slope` | two-sided gravity (downhill pushes) |
| `fuel` | burn follows mechanical power |
| `curves` | curve speed limits + braking parabola |
| `stations` | station zone cap |
| `smoothstop` | braking parabola to scheduled stops |

## Validated presets

Set these as a group, not piecemeal:

| Key | Strict realism | **Balanced (default)** | Arcade |
|---|---|---|---|
| `power_scale` | 1.0 | **1.35** | 1.6 |
| `curve_lateral_ms2` | 1.0 | **1.4** | 1.6 |
| `grid_boost` | 1.0 | **2.0** | 2.5 |

Strict realism is honest but demanding: a 10 MW high-speed trainset needs
~25 km of clear straight for its top speed and a grid that vanilla never
required. Balanced keeps heavy trains heavy and starts honest while letting
fast trains actually run. Arcade is for "trains but fun".

## Physics keys

- **`adhesion_mu`** (default 0.30) — wheel/rail adhesion. Caps tractive
  effort at low speed. 0.20 wet rail, 0.35 generous sanding. This is what
  stalls a 2300 t freight on a steep grade; raise slightly if your map is
  all mountains and you refuse to reroute.

  Stall behaviour, so it does not surprise you: when grade force exceeds
  adhesion the train does **not** freeze dead — the game's own ~0.5 km/h
  crawl floor takes over and the consist inches up the hill until the
  grade eases and traction wins. It looks (and is) dramatic: a weak
  locomotive with 5500 t crawls through a pass at walking pace instead of
  being hard-locked. Verified in game: Ov steam loco (445 kW, 52 t →
  153 kN adhesion) vs 5577 t — accelerates fine on flat, stalls to a
  0.5 km/h crawl on a 0.6 % effective grade, climbs out when the grade
  eases below ~0.3 %. If you want true deadlock (helper-loco gameplay),
  that is a separate switch we have not added — ask.
- **`power_scale`** (1.35) — engine power multiplier. Only affects speeds
  above the adhesion/power crossover, i.e. the mid/high-speed acceleration
  tail. Fuel demand scales along.
- **`davis_a/b/c`** (1.5 / 0.006 / 0.40) — resistance
  `R = m_t(A + B·v) + C·v²` newtons, v in km/h. A and B per tonne
  (rolling/bearings), C whole-train aero. For sleek HSR sets C ≈ 0.25,
  for open freight up to 0.6.
- **`grade_scale`** (0.06) — converts the game's slope unit to a real
  grade: in-game slope × (scale/0.12) = effective grade. 0.12 is physical;
  0.06 halves every grade (validated default — WRSR terrain is much
  steeper than any real mainline); 0.04 for very mountainous maps.

## Curve keys

- **`curve_lateral_ms2`** (1.4) — lateral acceleration limit, the single
  "how much curves slow trains" knob. Includes a superelevation allowance
  the track geometry cannot express. See the preset table in the ini.
- **`curve_brake_margin`** (1.25) — braking budget divisor; larger = brake
  earlier. 1.0 brakes at the theoretical last moment (do not).
- **`curve_lookahead_m`** (1200) — floor of the scan window; the real
  horizon auto-extends to the full-stop braking distance + 150 m (cap 8 km).
- **`log_curves`** (0) — writes `curve: v=… limit=… R=…` per train when a
  limit is active. Your calibration tool: R is the tightest radius the
  scanner found.

## Station / stop keys

- **`station_limit_kmh`** (60) — cap through station track, pass-through
  included. 30 for yard flavour, 60 mainline (validated).
- **`smoothstop`** (1) — braking parabola to scheduled stops ending 25 m
  out, where vanilla's own ramp takes over. Also smooths end-of-line stops.
- **`customstop`** (1) — customs/border stops: the route ends at the
  border until clearance, and its terminal node sits at the same world
  position as a customhouse track node — the scan matches by position and
  brakes to **`customs_entry_kmh`** (50) at the entry; the zone-run
  zero-parabola owns the final ~115 m. A static corridor graph remains as
  fallback. Needed because the stop-intent flags never fire for customs.

## Braking keys

- **`service_brake_ms2`** (0.8) — service deceleration.
- **`emergency_brake_ms2`** (1.3) — emergency deceleration.
- **`brake_min_vanilla_ratio`** (1.0) — protective-braking floor: never
  brake weaker than vanilla × ratio. Planned braking (to curves/stations/
  stops) ignores the floor and rides the parabola. Set below 1 only if you
  accept overshoots at signals as a gameplay feature.

## Fuel / grid keys

- **`idle_load`** (0.05) — load factor at standstill (engine idling).
- **`fuel_load_max`** (1.0) — cap of the load factor; 1.0 keeps the game's
  own burn scale.
- **`grid_boost`** (2.0) — multiplier on the plant outflow ceiling that
  caps total grid throughput. Needed when honest electric traction sags
  the grid (delivered fraction < 1 in the `fuel:` log line). Global: the
  town's electricity moves faster too.
- **`electric_load_scale`** (1.0) — shrinks the catenary demand instead.
  Cheaper than infrastructure, less honest.

## Telemetry

- **`log_physics`** (0) — every 5 s per train:
  `v / mass / P / F / R / G / a` plus `fuel: v / Pmech / load / pa`.
  Read `pa` as "grid delivered fraction" — below ~0.95 your grid is
  throttling the train, not the physics.
- **`log_curves`** (0) — curve/station/stop limits as computed.
- **`log_decisions`** (0) — the per-train flight recorder: `decision
  T…xxxx` event lines when a limiter engages/changes/releases (with
  distance, limit, geo-match gap, route-left), brake ON/OFF flips against
  the active limit (a pulsing approach shows as repeated pairs), and a 1 s
  snapshot while a stop/customs limiter is active. The tool for "why did
  that train not brake".

Both are write-only diagnostics with no gameplay effect; turn them off for
normal play.
