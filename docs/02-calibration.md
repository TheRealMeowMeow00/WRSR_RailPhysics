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
| `smoothstop` | smooth approach to scheduled stops |
| `customstop` | customs zones and stops |
| `bogies` | pivots of long Workshop cars moved onto their wheelsets |
| `tilt` | cars leaning into curves at speed (visual) |

## Validated presets

Set these as a group, not piecemeal:

| Key | Strict realism | **Balanced (default)** | Arcade |
|---|---|---|---|
| `power_scale` | 1.0 | **1.35** | 1.6 |
| `curve_lateral_ms2` | 1.7 | **2.8** | 3.2 |
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

- **`curve_lateral_ms2`** (2.8) — lateral acceleration limit, the single
  "how much curves slow trains" knob: `v = √(a·R)`. Includes a
  superelevation allowance the track geometry cannot express. 2.0 measures
  radii correctly — 1.x read them twice too large — so the scale doubled:
  2.8 now gives the speeds 1.4 gave. Real railways allow about 1.7 (160 mm
  cant plus 100 mm deficiency).
- **`curve_brake_margin`** (1.25) — braking budget divisor; larger = brake
  earlier. 1.0 brakes at the theoretical last moment (do not).
- **`curve_lookahead_m`** (1200) — floor of the scan window; the real
  horizon auto-extends to the full-stop braking distance + 150 m (cap 8 km).
- **`log_curves`** (0) — logs each curve slower than 100 km/h once, with
  the tightest radius the scanner found and the window it was measured
  over, and a line when track kinks were ignored. Your calibration tool.

## Station / stop keys

Stations and customs follow one rule in 2.0: the zone speed is held from
the zone's first switch to the stop, and on the way out until the tail —
the consist's real length — has left the zone. Zones of one kind less than
1 km apart are one complex.

- **`station_limit_kmh`** (60) — speed through station track, pass-through
  included. 30 for yard flavour, 60 mainline (validated).
- **`customs_entry_kmh`** (50) — the same for customs track. Customs count
  only for a train whose route is bound for that customhouse.
- **`smoothstop`** / **`customstop`** (1) — the approach to a stop: the
  target is where the route ends on the facility's own track, the braking
  parabola never drops below **`stop_release_kmh`** (15), and the game makes
  the final stop. Never 0: a limit of 0 short of the target is what froze
  trains in 1.x.

## Braking keys

- **`service_brake_ms2`** (1.6 shipped) — service deceleration; planned
  braking of passenger trains rides a parabola at this over
  `curve_brake_margin`.
- **`freight_brake_ms2`** (0.8) — planned braking of a consist without
  passengers: softer and earlier.
- **`emergency_brake_ms2`** (2.6 shipped) — emergency deceleration.
- **`brake_min_vanilla_ratio`** (1.0) — protective-braking floor: never
  brake weaker than vanilla × ratio. Planned braking (to curves/stations/
  stops) ignores the floor and rides the parabola. Set below 1 only if you
  accept overshoots at signals as a gameplay feature.

## Bogies and tilt

- **`bogies`** (1) — a car type left on the default pivots (3 m from each
  end, because its ini sets no `$TRAIN_*_AXIS_DISTANCE`) gets its pivots at
  its modelled wheelsets, read once from its `main.nmf`. Base-game models
  and cars that set the values are left alone.
- **`tilt`** (1) — passenger cars lean into curves at speed by
  `tilt_gain · atan(v²/(g·R))`. Visual only: speeds and physics do not
  change.
  - **`tilt_max_deg`** (5) — the most a car leans. Models are rigid: past
    ~6° the wheels visibly lift off one rail.
  - **`tilt_gain`** (0.5) — the share of the lateral-acceleration angle.
  - **`tilt_min_kmh`** (80) — no lean below this; full lean 40 km/h above.
  - **`tilt_rate_deg_s`** (4) — roll rate, degrees per second of game time.
  - **`tilt_freight`** (0) — 1 lets freight lean too.

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
- **`log_curves`** (0) — each slow curve once, with its radius (see above).
- **`log_decisions`** (0) — the per-train flight recorder: `decision
  T…xxxx` lines when the binding limit changes kind (`engage … limit=…
  v=… len=… passenger|freight`, `release …`), when a stop is armed, served
  or dropped, brake ON/OFF flips against the limit, and a 1 s snapshot
  (`snap v=… limit=… stop=… odo=…`) near a station or customs. The tool for
  "why did that train brake / not brake". With `log_physics` it also prints
  the tilt peak every 30 s.

Both are write-only diagnostics with no gameplay effect; turn them off for
normal play.
