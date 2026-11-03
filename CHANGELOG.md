# Changelog

## 2.0.2 - 2026-11-03

### Fixed

- **Settings under Republic Mod Loader.** The plugin reads `railphysics.ini`
  from the folder its DLL is in. Under RML the loader's own lookup found no ini
  and every setting fell back to its default; under tesmioloader it is the
  same file as before. The loader log names the file it read.
- The Workshop item keeps `railphysics.dll` and `railphysics.ini` in
  `plugins/`, where RML looks for a plugin.

## 2.0.1 — 2026-09-27

### Changed

- **Curve speeds by class.** Freight, passenger and tilting trains each have
  their own lateral limit: `curve_lateral_freight_ms2` (2.1),
  `curve_lateral_ms2` (2.8) and `curve_lateral_tilt_ms2` (4.2) — the real
  ratios 0.75 : 1 : 1.5, scaled for the game's compact maps. A tilting train
  is a passenger train while `tilt` is on, and it earns its allowance as it
  leans: none below `tilt_min_kmh`, all of it 40 km/h above, so in station
  throats it is a passenger train like any other. A CR400 with tilt on takes
  an R 2000 m curve at 330 km/h and R 1000 m at 233; freight slows more in
  tight curves. Left out of the ini, the freight and tilting values are 0.75x
  and 1.5x of `curve_lateral_ms2`.

## 2.0.0 — 2026-09-27

A rewrite of how trains approach curves, stations, customs and stops, and two
new visual features. Requires game build 1.1.1.9 and a tesmioloader with
plugin API 4. Replace both files when updating: the ini has new keys and
`curve_lateral_ms2` is on a new scale.

### New

- **Approach core.** Every limit — curves, station and customs zones, the
  stop — is an interval on a per-train odometer measured from the train's
  position along its route, evaluated every frame as the lower of a braking
  parabola before it and its own speed inside it. Nothing is timed with a
  clock, so trains behave the same at any game speed and under plugins that
  rescale time.
- **Stations and customs, one rule.** A zone is held at its speed
  (`station_limit_kmh`, `customs_entry_kmh`) from its first switch to the
  stop, and on the way out until the tail — the consist's real length — has
  left it. Zones of one kind less than 1 km apart are one complex: no
  re-acceleration between the two fences of a border crossing.
- **Stops.** The target is where the route ends on the facility's own track.
  The approach brakes along a parabola that never drops below
  `stop_release_kmh` (15 km/h), and the game makes the final stop. A stop the
  route runs past — customs cleared, a stop the train will not make — is
  dropped at once instead of being crawled towards.
- **Freight braking.** A consist without passengers plans its braking at
  `freight_brake_ms2` (0.8 m/s²): softer, earlier, better to watch.
- **Bogies** (`bogies = 1`). A car type left on the default pivots
  (`$TRAIN_*_AXIS_DISTANCE` not set: 3 m from each end) gets its pivots at its
  modelled wheelsets, read once from its `main.nmf`. The wheels of long
  Workshop cars stay on the rails in tight curves. Base-game models and cars
  whose ini sets the values are left alone.
- **Tilt** (`tilt = 1`). Passenger cars lean into curves at speed by
  `tilt_gain · atan(v²/(g·R))`, capped at `tilt_max_deg` (5°), from
  `tilt_min_kmh` (80), rolling at `tilt_rate_deg_s` (4) of game time.
  Curvature comes from each car's own heading over the last 40 m, so track
  kinks and jogs do not rock the train. Visual only.
- **Decision log** (`log_decisions = 1`): per-train engage/release, stop
  armed/served/dropped and brake events, with a snapshot every second near a
  facility — for bug reports. `log_curves = 1` prints every curve slower than
  100 km/h once, with its radius.

### Fixed

- Some customs houses braked trains, others were crossed at full speed.
- Trains could stop short of a stop and wait there indefinitely; only pausing
  the game released them.
- The position inside a segment crossed backwards (`dir = 1`) was read from
  the wrong end: the odometer froze and jumped, targets receded, and a
  customs zone could be entered at 141 km/h.
- Curve radii were measured twice too large. Bridge-end kinks and small
  lateral jogs braked trains. A straight track at an angle to the map axes
  could read as a sharp curve where a segment without intermediate points
  joined a dense one.
- Gravity: sign fixed, and applied once — the traction divisor no longer
  includes it while the slope hook is live, and the two slope sites no longer
  both apply it.
- The FPS collapse on big maps: memory-check cache thrash, the train slot
  table leaking, and a rescan on every track segment.
- A false brake 60 m before any facility.

### Changed

- `curve_lateral_ms2` is on a new scale, because radii are now measured
  correctly: the default 2.8 gives the same curve speeds 1.4 gave in 1.x.
  Presets: 1.7 strict, 2.8 balanced, 3.2 arcade.
- Removed: the Dijkstra corridors and every wall-clock timer.

### New ini keys

`stop_release_kmh`, `freight_brake_ms2`, `bogies`, `tilt`, `tilt_max_deg`,
`tilt_gain`, `tilt_min_kmh`, `tilt_rate_deg_s`, `tilt_freight`,
`log_decisions`.

## 1.3.0 — 2026-08-29

- Signature-scan re-targeting: every patch site is located at load by a
  unique wildcard byte signature, so a game update that moves code disables
  only the affected subsystem, with a log line.
- Hooks the steep-downhill assist site added in game build 1.1.1.9.

## 1.1.0 — 2026-08-10

- Smooth stops reworked; customs/border stops; arcade preset recalibrated.

## 1.0.0 — 2026-08-02

- First release: traction, braking, gravity, curves, station zones,
  fuel/energy, grid boost.
