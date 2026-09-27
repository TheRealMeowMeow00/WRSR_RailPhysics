# RailPhysics — realistic rail dynamics for Workers & Resources: Soviet Republic

A plugin for [tesmioloader](https://github.com/MaxLegend/TesmioLoader) that
replaces the game's simplified train physics with a physical model — traction,
braking, gravity, curves, stations, stops and fuel/energy consumption — all
tunable from one ini file. No game file is modified; everything is an
in-memory patch installed by the loader.

Russian version: [README_RU.md](README_RU.md). What changed in each version:
[CHANGELOG.md](CHANGELOG.md).

Targets `SOVIET64.exe` **v1.1.1.9** (64-bit) and a current tesmioloader
(plugin API 4). Every patch site is located at load by a unique byte
signature and verified before anything is written: a game update makes the
affected subsystem skip itself and log why — it never writes into changed code.

## What changes

| Vanilla | RailPhysics |
|---|---|
| Constant acceleration `3.61·√(P/5m)` km/h/s at any speed | Traction `F = min(P/v, μ·W_adhesive·g)`, Davis resistance `m(A+Bv)+Cv²` |
| Braking = acceleration / 3, always | Independent service/emergency rates (m/s²); planned braking to curves, stations and stops — softer for freight |
| Gravity only pulls uphill, downhill does nothing | Signed grade force — descents push the train |
| Fuel burn from a speed-ratio curve | Burn follows mechanical power `F·v` actually delivered; idle at standstill |
| No curve speed limits | Route scanned ahead, `v = √(a_lat·R)` per curve, braking parabola started one braking distance out, limit held until the tail leaves the curve; track kinks (bridge joins) ignored |
| Trains fly through stations at full speed | Station track held at `station_limit_kmh` from the first switch until the tail has left |
| Some customs stop, others are crossed at full speed | Customs held at `customs_entry_kmh` the same way; a two-zone border is one complex |
| Stops slam in the last 20 m | The approach to a stop brakes smoothly and never drops below `stop_release_kmh`; the game makes the final stop |
| Long Workshop cars pivot 3 m from their ends — wheels off the rails in tight curves | `bogies`: pivots read from the model's wheelsets |
| Cars stay upright in curves | `tilt`: passenger cars lean into curves at speed (visual only) |
| Electric trains creep at high power draw | `grid_boost` multiplies the plant outflow ceiling the grid's throughput is divided from |

Every limit is planned in metres along the route, never in time, so trains
behave the same at any game speed. Low-speed behaviour stays honest
(adhesion caps the start); the mass of the consist — wagons, cargo,
passengers, wear derating — is taken from the game's own aggregates, so a
2300 t freight and a 380 t CR400 feel like different universes. A
high-speed trainset on decent track reaches 300+ km/h; a heavy freight on a
10 % ramp stalls and needs a helper or a reroute, like the real thing.

## Install

1. Install tesmioloader (the loader this plugin runs on).
2. Copy `railphysics.dll` and `railphysics.ini` into
   `<game>/tesmioloader/build/plugins/`.
3. Start the game. The loader log (`tesmioloader.log`) should say
   `10 subsystem(s) patched` and show no `does not match` lines.

Updating from 1.x: replace both files — the ini has new keys and the curve
setting is on a new scale (see the changelog). Uninstall: delete the two
files. Saves are unaffected — the plugin changes behaviour per frame, nothing
is written to the save format.

## Recommended companion: railspeed

The stock track limit (150 km/h concrete) caps everything RailPhysics can
do. The [railspeed plugin](https://github.com/SosseTurner/Railspeed-Plugin-for-Workers-and-Resources)
raises the *track* speed limit (e.g. 330 km/h) — and does nothing else.
The two plugins are complementary and independent: railspeed opens the
ceiling, RailPhysics makes reaching it an honest engineering effort. Not
required — without it everything works, the ceiling is just vanilla.

## Configure

Everything lives in `railphysics.ini` beside the DLL — each subsystem can be
switched independently, and three presets are documented at the top of the
file (strict realism / balanced / arcade). Highlights:

- `power_scale` — arcade multiplier on engine power (adhesion keeps starts honest)
- `curve_lateral_ms2` — how hard curves bite (1.7 strict … 3.2 arcade, 2.8 default)
- `grade_scale` — how steep the terrain feels (0.06 makes 10 % ramps a wall)
- `station_limit_kmh`, `customs_entry_kmh`, `stop_release_kmh` — stations, customs, stops
- `service_brake_ms2`, `freight_brake_ms2` — planned braking, passenger and freight
- `bogies` — put long Workshop cars on their modelled wheelsets
- `tilt`, `tilt_max_deg`, `tilt_gain`, `tilt_min_kmh` — cars leaning into curves
- `grid_boost`, `electric_load_scale` — electric traction vs the grid
- `log_physics`, `log_curves`, `log_decisions` — telemetry and a per-train decision log in the loader log

See [docs/02-calibration.md](docs/02-calibration.md) for tuning recipes.

## Build from source

With [llvm-mingw](https://github.com/mstorsjo/llvm-mingw) (no MSVC needed):

```bash
LLVM_MINGW=/path/to/llvm-mingw ./build-linux.sh      # Linux
LLVM_MINGW=/c/path/to/llvm-mingw ./build-windows.sh  # Windows, Git Bash
```

Output lands in `build/plugins/`. `build-windows.sh` also builds and runs the
offline harness (`test/host_test14.cpp`, see [docs/03-testing.md](docs/03-testing.md))
when `GAME` points at the game folder. The loader itself is not part of this
project — only the plugin and the vendored plugin-API headers
(`src/tesmio_api.h`, `src/tesmio_plugin.h`, GPLv3, from the tesmioloader
project).

## Documentation

- [CHANGELOG.md](CHANGELOG.md) — what changed in each version
- [docs/01-design.md](docs/01-design.md) — the physics model and the hook
  mechanisms (1.x; the 2.0 approach core is in findings/06)
- [docs/02-calibration.md](docs/02-calibration.md) — ini reference and tuning
- [docs/03-testing.md](docs/03-testing.md) — how each subsystem was verified
  in game, the offline harness, and how to re-verify
- [docs/04-reverse-engineering.md](docs/04-reverse-engineering.md) — the
  toolkit and how to re-check patch sites after a game update
- [docs/findings/](docs/findings/) — the reverse-engineering reports: vanilla
  rail physics, route/curve representation, station internals, stop logic,
  electric grid feed, and the 2.0 findings (positions, consists, bogies, the
  car transform)

## License

GPL v3 (same as tesmioloader). See [LICENSE](LICENSE).
