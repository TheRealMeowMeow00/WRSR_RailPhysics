# RailPhysics — realistic rail dynamics for Workers & Resources: Soviet Republic

A plugin for [tesmioloader](https://github.com/MaxLegend/TesmioLoader) that
replaces the game's simplified train physics with a physical model — traction,
braking, gravity, curves, stations, stops and fuel/energy consumption — all
tunable from one ini file. No game file is modified; everything is an
in-memory patch installed by the loader.

Russian version: [README_RU.md](README_RU.md).

Targets `SOVIET64.exe` **v1.1.1.7** (64-bit). Every patch site is verified
byte-for-byte at load time: a game update makes the affected subsystem skip
itself and log why — it never writes into changed code.

## What changes

| Vanilla | RailPhysics |
|---|---|
| Constant acceleration `3.61·√(P/5m)` km/h/s at any speed | Traction `F = min(P/v, μ·W_adhesive·g)`, Davis resistance `m(A+Bv)+Cv²` |
| Braking = acceleration / 3, always | Independent service/emergency rates (m/s²), gentle planned braking to curves/stations/stops |
| Gravity only pulls uphill, downhill does nothing | Signed grade force — descents push the train |
| Fuel burn from a speed-ratio curve | Burn follows mechanical power `F·v` actually delivered; idle at standstill |
| No curve speed limits | Route scanned ahead, `v = √(a_lat·R)` per curve, ETCS-style braking parabola started exactly one braking distance out |
| Trains fly through stations at full speed | Station track zone capped (`station_limit_kmh`), entered through the braking curve |
| Stops slam in the last 20 m | Braking parabola to zero ending 25 m out, vanilla's ramp takes over for the final crawl |
| Electric trains creep at high power draw | `grid_boost` multiplies the plant outflow ceiling that the whole grid's throughput is divided from |

Low-speed behaviour stays honest (adhesion caps the start); mass of the
consist — wagons, cargo, passengers, wear derating — is taken from the
game's own aggregates, so a 2300 t freight and a 380 t CR400 feel like
different universes. A high-speed trainset on decent track reaches 300+
km/h; a heavy freight on a 10 % ramp stalls and needs a helper or a
reroute, exactly like the real thing.

## Install

1. Install tesmioloader (the loader this plugin runs on).
2. Copy `railphysics.dll` and `railphysics.ini` into
   `<game>/tesmioloader/build/plugins/`.
3. Start the game. The loader log (`tesmioloader.log`) should say
   `8 subsystem(s) patched` and show no `does not match` lines.

Uninstall: delete the two files. Saves are unaffected — the plugin changes
behaviour per frame, nothing is written to the save format.

## Recommended companion: railspeed

The stock track limit (150 km/h concrete) caps everything RailPhysics can
do. The [railspeed plugin](https://github.com/SosseTurner/Railspeed-Plugin-for-Workers-and-Resources)
raises the *track* speed limit (e.g. 330 km/h) — and does nothing else.
The two plugins are complementary and independent: railspeed opens the
ceiling, RailPhysics makes reaching it an honest engineering effort. Not
required — without it everything works, the ceiling is just vanilla.

## Configure

Everything lives in `railphysics.ini` beside the DLL — each subsystem can be
switched independently, and three validated presets are documented at the
top of the file (strict realism / balanced / arcade). The defaults are the
balanced preset. Highlights:

- `power_scale` — arcade multiplier on engine power (adhesion keeps starts honest)
- `curve_lateral_ms2` — how hard curves bite (0.7 strict … 1.6 arcade)
- `grade_scale` — how steep the terrain feels (0.06 makes 10 % ramps a wall)
- `station_limit_kmh`, `smoothstop`, `service_brake_ms2`
- `grid_boost`, `electric_load_scale` — for electric traction vs the grid
- `log_physics`, `log_curves` — calibration telemetry into the loader log

See [docs/02-calibration.md](docs/02-calibration.md) for the full key
reference and tuning recipes.

## Build from source

Linux, with llvm-mingw (no MSVC needed):

```bash
LLVM_MINGW=/path/to/llvm-mingw ./build-linux.sh
```

Output lands in `build/plugins/`. The loader itself is not part of this
project — only the plugin and the vendored plugin-API headers
(`src/tesmio_api.h`, `src/tesmio_plugin.h`, GPLv3, from the tesmioloader
project).

## Documentation

- [docs/01-design.md](docs/01-design.md) — the physics model and every hook
  mechanism, with addresses
- [docs/02-calibration.md](docs/02-calibration.md) — ini reference and tuning
- [docs/03-testing.md](docs/03-testing.md) — how each subsystem was verified
  in game, and how to re-verify
- [docs/04-reverse-engineering.md](docs/04-reverse-engineering.md) — the
  toolkit and how to re-check patch sites after a game update
- [docs/findings/](docs/findings/) — the raw reverse-engineering reports:
  vanilla rail physics, route/curve representation, station internals, stop
  logic, electric grid feed

## License

GPL v3 (same as tesmioloader). See [LICENSE](LICENSE).
