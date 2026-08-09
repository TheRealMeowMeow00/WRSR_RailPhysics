# HANDOFF — RailPhysics on soviet-mod-loader (SML), customs/station geo-stops

Written 2026-08-05 before a PC reboot. Everything below is the current, verified
state of the work. Nothing is git-committed anywhere — all changes live in
working trees only.

## 1. The moving parts

| Piece | Path | Notes |
|---|---|---|
| Game | `/run/media/meow/Games/steamapps/common/SovietRepublic/` | `SOVIET64.exe` v1.1.1.7 (64-bit); runs under Proton, prefix `compatdata/784150` |
| Mod loader | `~/soviet-mod-loader` | clone of github.com/PresFox/soviet-mod-loader; our additions: `build-linux.sh` (llvm-mingw build), `launch-steam.sh` (Steam launch-options wrapper), `build/` (runtime root, gitignored) |
| Runtime root | `~/soviet-mod-loader/build/` | `SovietBootloader.exe`, `loader/SovietModLoader.dll`, `mods/<id>/{mod.ini,bin/*.dll,plugins/*.ini}`, `profiles/enabled-mods.ini`, `logs/loader.log` |
| RailPhysics (our mod) | `~/RailPhysics` | source of truth: `plugins/railphysics/railphysics.cpp` + `stubs.S`; docs in `docs/` (findings = our RE reports, see esp. `docs/findings/04-stops.md` Q6–Q8) |
| SML package: RailPhysics | `~/SML_RailPhysics` | `shim/sml_host.cpp` (TsmHost ABI → `SovietModInitialize`), `include/` (vendored Tsm ABI headers), `config/railphysics.ini` (DEPLOYED config), `build.sh` |
| SML package: RailSpeed | `~/SML_RailSpeed` | third-party railspeed plugin adapted; LOCAL ONLY, do not publish; same layout + `src/railspeed.cpp` |
| Toolchain | `~/toolchains/llvm-mingw-20251118-ucrt-ubuntu-22.04` | `bin/clang++ -target x86_64-w64-mingw32 -O2 -Wall -static -shared -fms-extensions` |

Enabled mods (`build/profiles/enabled-mods.ini`): `railphysics = true`,
`railspeed = true`, `example-mod = false`. tesmioloader is DEPRECATED — do not
use it, do not build with it; the game dir still has `tesmioloader/` and
`tesmioloader.backup/` folders pending deletion (after final in-game test).

## 2. How to build / deploy / run

```bash
~/SML_RailPhysics/build.sh     # builds railphysics.cpp + stubs.S + shim →
                               # ~/soviet-mod-loader/build/mods/railphysics/
~/SML_RailSpeed/build.sh       # same for railspeed
```

Run the game via Steam launch options (already set):

```
/home/meow/soviet-mod-loader/launch-steam.sh %command%
```

The wrapper swaps the launch exe for SovietBootloader and targets
`SOVIET64.exe` directly (Steam's default exe is the `SETUPAPPLICATION
SOVIET.exe` stub — injecting into it misses the real game process).

Logs (per mod): `~/soviet-mod-loader/build/mods/<id>/bin/<id>.log`.
Loader log: `~/soviet-mod-loader/build/logs/loader.log`.

## 3. Config state (deployed `~/SML_RailPhysics/config/railphysics.ini`)

Telemetry is ON for debugging: `log_curves = 1`, `log_physics = 1`,
`log_decisions = 1` (the flight recorder). Turn all three OFF before release.
Feature keys: `customstop = 1`, `customs_entry_kmh = 50`, `smoothstop = 1` —
all present in both ini copies (`plugins/railphysics/railphysics.ini` and the
SML config); keep them in sync manually.

## 4. Where we are — feature status

### Done and verified in game
- **SML migration**: railphysics + railspeed run under soviet-mod-loader via
  the shim. Loader confirmed up (`loader.log`: `Loaded railphysics`).
- **Performance fix** (user report: FPS 120→30 with >64 trains): curve cache
  is now a 1024-slot open-addressed hash (was 64 round-robin → thrash → full
  route scan per frame per train); origin-keyed acceptance (route
  vec/seg/idx/pos), stopped trains rescan ≤1/10 s; TLS region cache in front
  of VirtualQuery-backed ReadablePtr; per-(veh,frame) ComputePhysics memo;
  station sweep 5 s→30 s. In-game confirmation: `curve cache: 15582 hits /
  106 scans`. Ready to tell the reporter.
- **Customs stops, forward direction** and **station geo-stops**: perfect on
  the cyclic test map (parabola from kilometres out, 50 km/h at the customs
  fence, zero at the building; stations: `stop ahead(geo)` parabola to zero).

### Just deployed, UNTESTED (iteration 21c) — TEST THIS FIRST
Run 8 (iteration 21) verdict: the pulse customs braked accurately
(fence ~68, clean stop, 4 identical loops) but the /15 band still gave
~full rate on every 15-20 km/h corridor leg-step - the toggling
remained. Iteration 21b widened the band to /30 with a 0.25 floor
(step overshoots get 0.25-0.65 rate and merge into one continuous
application; overshoot >=30 - real demand - still full rate instantly).
Iteration 21c adds a targeted diagnostic for customs B: when the island
handover (corrOk==2) fires farther than 1 km out, one
`customs: FAR HANDOVER seg=… nA=(x,y,z) nB=(…)` line per engagement
(log_curves) dumps the "customs" segment's node coords - to tell a
mis-zoned segment from a route loop. User named the pair: A = OK
(formerly pulsing), B = NOT OK (overestimated entry, both corridor and
geo run ~1.4-3.5 km high until the final route truncation, ~278-285 at
the fence, vanilla grab every loop). User marked both with road signs
in game.

### Previous state (iteration 21, superseded)
Iteration 20 tested 2026-08-06 (run 7): the candidate clamps held (no
300 km/h fly-through), BUT the working customs direction PULSED the
brake [user report]: the corridor graph distance falls in leg-quantized
steps of up to ~700 m, every confirmed step drops the limit 15-20 km/h,
and at 2x arcade brake each step is a full-rate yank. Iteration 21:
proportional band in the PLANNED brake regime - full service rate at
>=15 km/h overshoot, easing to 30% at the limit; bangs fuse into one
continuous application; real demand is never delayed. Emergency and the
vanilla-floor regime unchanged.
KNOWN, NOT FIXED YET (deliberately): the mirror customs direction
overestimates the entry distance by ~1.4 km in BOTH corridor and geo
(corridor graph path and route both extend well past the border;
routeleft churned 3510->3694->1443->0->184->... with gap 0.00) - the
train reached the fence at ~282 and vanilla grabbed it (v=0 release).
Needs its own investigation; candidates broke the geo-takes-over there
because the geo values churned instead of settling.

### Previous state (iteration 20, superseded)
Iteration 19 tested 2026-08-06 (run 6, short): 300 km/h INTO the customs
[user report]. Two compounding holes, both in the customs entry clamp:
1. iter-19's drop rate-limit (traveled+25/scan) rejected LEGITIMATE
   post-void re-truncation steps - the route length falls in leg-sized
   chunks of 150-450 m, not smoothly (1106->668->516->...->49, zone-run
   fired at 115 m while the clamp held ~3900 [C: run 6]).
2. the instant reset corr.dist<0 -> corrClamp=-1 let every torn flicker
   wipe the clamp, and the next (growing) reading adopted fresh:
   1444->3464 up-churn, limit 341, accelerate to 300 [C: run 6].
Iteration 20: PERSISTENCE, not step size, separates torn reads from real
corrections - a torn read is a one-scan blip that bounces back, a real
re-truncation stays low. Both clamps (corr + zero) now: decay by travel
unconditionally; steady readings (within traveled+25) follow; a bigger
drop becomes a CANDIDATE adopted only when the next scan stays low;
jumps to <=25 m are never believed (the stop is only ever reached by
travelling); fresh engagements adopt instantly; growth never adopted
(corr) / only far >50 m steps (zero). The corrClamp instant reset is
GONE - evidence gaps hold+decay, capped at 500 m evidence-free
(corrNoEv), same as the corridor's own rule. corrViaLive removed again
(via-upgrade adoption is subsumed by candidates).

**Acceptance test:** fast train into the customs must start braking
~3.3 km out (arcade brake) and cross the fence at ~50; no target
dist growth inside an engagement; stations keep the run-5 good
behaviour (no frozen dist while accelerating, no hot zone entries).
Watch the `slot init` diag: recurring inits for a live train = hash
eviction (still unconfirmed as a fault source).

### Previous state (iteration 19, superseded)
Iteration 18 tested 2026-08-06 (run 5, ~1.5 h, 7 trains, passengers,
planes; signal stops present - the mod stands politely through them):
customs far approaches textbook; void hold+decay works (973->26 etc.);
the run-4 snail gone. REMAINING, fixed by iteration 19:
1. FROZEN PIN [the big one]: after signal stops / in the void the geo
   distance FREEZES (dist stuck at 2536/2740 while the train accelerates
   0->138, gapstat 0.00). The clamp only decayed when readings were
   ABSENT; a frozen present reading pinned it -> limit stayed ~200 ->
   zone entries at 121-196 km/h -> vanilla platform slam (196->37 in 1 s).
   FIX: the zero clamp now decays by traveled UNCONDITIONALLY; readings
   may only pull it down (<= traveled+25 per scan) or raise it on a far
   re-extension step (>50 m while >250 m out). Frozen readings fail both
   tests and the decay simply stands.
2. corrClamp (customs entry) had NO rate limit: a torn 1312->336
   collapse was adopted -> 126 km/h at the fence. FIX: same rule as the
   zero clamp - within one source a drop > traveled+25 holds the decayed
   value; source UPGRADE (corridor->handover->geo) still adopts instantly
   (the 2071->501 handover). New slot field corrViaLive.
3. DIAG: "slot init" line (log_decisions) on every cache-slot birth -
   recurring inits for a live train = hash eviction resetting clamp
   state mid-approach (suspected path behind 3870->2865-type jumps;
   unconfirmed - watch the log).
Arcade recalibration (user call, small map): service_brake_ms2 0.8->1.6
(braking distances HALVE; all parabolas derive from it), emergency
1.3->2.6 (same 1.625x ratio), power_scale 1.35->1.5 (slightly brisker
acceleration). BOTH ini copies updated (still differ only in log flags).
OPEN: the passenger shuttle's micro-stops (384 m zone, E… addresses)
get no smoothing at all (vanilla 60->1 slams every ~350 m) - chain type
0x60 IS seeded for the zone cap but the geo stop never engages there;
decide with the user whether to pursue. Station names not visible to
the mod (pointers/coords only).

**Acceptance test:** same loaded map. Zero `dist` freeze-lines: a snap
series with v rising must show dist DECAYING every snapshot. No zone
entries above ~70. Braking distances visibly ~2x shorter; fences still
at ~50 km/h (the parabola starts later now). Customs stays perfect.

### Previous state (iteration 18, superseded)
Iteration 17 tested 2026-08-06 (run 4): the unconditional hold fixed the
middle-void slams BUT turned two unverified zRaw sources into poison:
1. THE SNAIL [user report]: the intent-bytes path (+0xD39/+0xD3C ->
   full remaining route length as the stop distance) is wrong by ~430 m
   on re-chunked routes; the phantom parabola crawled the train at
   12-40 km/h for a kilometre, then evaporated mid-nowhere. FIX: the
   intent-bytes path is DELETED - the coordinate-verified geo match is
   the only station-stop distance source now. Watch: any stop type the
   sweep does not seed (waiting station 0x60?) loses smoothing - it
   would show as a vanilla 20 m slam at that stop.
2. THE LOCKOUT: the post-clearance customs residual zone-run (312-348 m,
   documented "benign cap") adopted into zeroClamp, spent, and its LATCH
   then suppressed the real station geo stop right behind it (re-arm
   needed zRaw<0, but geo kept matching) -> 144 km/h into the zone,
   119->0 platform slam. FIX: the spent latch also re-arms on any sighting
   >250 m (platform chunk growth never exceeds ~190 m).

**Acceptance test (Sacoi -> Tabovina via 2 stops + back):** no more
snail crawls (no long non-geo `stop ahead` descents - those lines should
be GONE from the log entirely); stations ride one parabola far->near
through the middle void; no zone entries above ~70 km/h; customs stays
perfect INCLUDING the return crossing right after the terminus turn
(that is where the lockout fired).

### Previous state (iteration 17, superseded)
Iteration 16 tested 2026-08-05 late (long semi-AFK run, line
Sacoi -> Tabovina via 2 stops + back, border crossing on the way):
customs PERFECT both directions; torn-read filter worked (no early
braking, growth adoption walked 425->2139 correctly); BUT the new
far-release rule was a regression: the station geo match genuinely
VANISHES mid-approach (route extends past the stop; lost at 0.9-2 km
out, returns at 200-450 m [C: run 3, every station, every loop]) and
the >500 m sourceless bail dropped the parabola into the 60 km/h zone
cap -> platform slam returned (138->0, 90->0, 71->0 at the platform).
Iteration 17: source lost now ALWAYS holds+decays (no distance cap) -
travel decay tracks the true remaining distance exactly and the source
re-joins close in; a cancelled stop just decays to spent (safe).
Station names are NOT visible to the mod (pointers/coords only;
name-string RE is an open nicety).

**Acceptance test (same line):** stations must ride the parabola
THROUGH the middle void: no `engage station zone` right after a
`stop ahead` descent at v>70; the zone cap must engage only after the
parabola is done (v~20). Watch for: phantom braking if a line is
rerouted mid-approach (expected: train slows to a stop at the OLD
stop point - known trade-off, safe). Customs must stay perfect.

### Previous state (iteration 16, superseded)
Iteration 15 tested 2026-08-05 evening: customs PERFECT both directions;
stations broken BOTH ways ("brake too early AND too late"). Decision-log
evidence: near stops the game re-chunks the route array and our scans read
it mid-rebuild — remaining length collapses (3221->425, 206->0), grows in
steps (425->1185), or vanishes past the horizon, all with the geo match
steady at gap 0.00. Iter-15's naive clamp (instant downward adopt, growth
frozen, reset on route shrink) turned those torn reads into: hard braking
3 km out (3221->425 adopted), parabola kill at 140-206 m out (206->0
adopted → release at v=58-170 → re-acceleration → re-match at 25-85 m →
emergency slam or vanilla 60→0 at the platform).
Iteration 16 fix (zeroClamp is now a physical-consistency filter):
1. Mid-engagement drops > traveled+25 m per scan = torn read → hold
   decayed value (fresh sightings still adopt instantly — customs
   2071->501 handover untouched).
2. Growth adopted only far out (>250 m); refused close-in but still
   decays by travel.
3. Source lost: release when far (>500 m), hold+decay ≤500 m
   evidence-free when close (anti-flap).
4. Spent latch at 25 m: readings ignored until the source vanishes —
   platform chunk growth can't re-engage a handed-over stop.
5. zeroClamp no longer resets on route shrink (that reset was exactly
   what adopted torn readings as fresh truth); corrClamp unchanged
   (customs verified good — don't touch).

**Acceptance test (same cyclic loop):** stations must now show: single
`engage stop ahead(geo)` far out → monotone descent with NO release until
routeleft≈20/v≈19 → no `engage` at dist<100 with v>60 (the re-match slam
signature), no hard braking right after a far engage (v must track limit
±~10). Customs must stay as good as iteration 15.

### Previous state (iteration 15, superseded)
Fixes from the decision-log analysis of the reverse-direction customs slam:
1. The distance clamp was `lo = prev − (traveled+25)` — it *pinned* the stale
   corridor estimate (~2 km) and refused the geo match's true 501→36 m.
   Now a pure running minimum: downward corrections adopt instantly, never
   increase, per engagement, for ALL sources.
2. Zone-run zero-parabola distances were logged wrong (`dist=1719` was the
   corridor estimate, not the zone distance — logging conflation) AND the
   zero source flickered at the fence, letting the weak entry limit (172)
   retake min() at ~100 m out. Now all zero-target sources (customs zone-run,
   station geo, intent bytes) share one `zeroClamp` with hold/decay/spend
   semantics — priority by persistence.
3. Station close-in chunk growth (dist 37→74→107→137 re-inviting
   acceleration near the platform) is clamped by the same mechanism.

**Acceptance test (cyclic map: Customs–Station–Station–Customs–Station–Station loop):**
run one full loop, then check `…/mods/railphysics/bin/railphysics.log`:
- reverse customs approach must show: `engage customs entry(raw) dist=~4800`
  → a single large DOWNWARD `target … 2071->501` jump when geo takes over
  (that's the fix working) → monotone `(customs entry)` descent →
  `engage customs stop dist=~117` at ~55 km/h → zero at the building;
- NO upward `target` moves, NO `customs entry` lines after `customs stop`
  engages, station `stop ahead(geo)` distances never grow;
- forward direction and station stops must stay as good as iteration 14.

### Known remaining roughness (not bugs, decisions pending)
- Vanilla's bang-bang brake controller makes limit-tracking feel pulsed
  (brake ON/OFF pairs ±~5 km/h around the falling limit). Verdict so far:
  vanilla behavior, our brake helper only sets the rate. Do NOT add hysteresis
  where it can delay real braking demand.
- Station-zone 60 km/h cap for PASS-THROUGH trains still can't see station
  islands early (same chain-local geometry); stops are covered by geo-match,
  early zone-cap is documented future work (comment on the station-zone
  branch in railphysics.cpp).

## 5. The core structural facts (hard-won, [C] = confirmed in game)

Full narrative: `docs/findings/04-stops.md` Q6–Q8. Essentials:

- Chain types: station 0/1, depot 0xF, rail constr. office 0x1C, waiting
  station 0x60, **customhouse 0x14** (via `chain+0x318->+0x360`).
- Station/customhouse track is **chain-local island geometry**: its nodes are
  distinct objects from mainline nodes; node adjacency (+0x28/+0x30 vector)
  does NOT bridge to the mainline. The chain's `+0xA10/+0xA18` table (stride
  0x60, entry+0x20 = segment, entry+0x30 = node) enumerates the zone's
  segments; `entry+0x30` is mainline-connected for only ~9 of 37
  customhouses on the user's big map.
- **Route objects** `train+0x680/+0x688` (8-byte ptr entries, type via
  `obj+0x318->+0x360`) list the trip's stops AT ASSIGNMENT TIME — seen 24 km
  out (`[14! 1 1 14! 1 1]` = the whole cyclic route). Stations = type 1.
- **The route array extends to the next stop from departure** (customs
  boundary or station). Near stops it can be re-extended in chunks.
- **Geo-match (the working detector):** when the route is exhausted inside
  the scan horizon, the terminal leg's far node matches a customs/station
  island node BY COORDINATES (`node+0x04/+0x08/+0x0C`, eps 2 m; measured gap
  0.00 m) → distance to stop = remaining route length. Priority: island
  handover > geo > corridor tags (fallback for conn-seeded zones).
- Vanilla stop blocks A–F (20 m/10 m ramps + hard clamp) all require the
  route's last leg — they own the final 25 m; we hand over there.

## 6. Remaining tasks (in order)

1. **Test iteration 15** (acceptance above). If the reverse direction still
   misbehaves, the decision log is the instrument — read the failing
   approach's `decision T…` lines and narrate before touching code.
2. **Cleanup for release**: remove/gate the load-time diagnostics (seed
   dumps, per-zone audit lines, chain-entry hex dumps), then set
   `log_curves/log_physics/log_decisions = 0` in BOTH ini copies.
3. **Delete** `tesmioloader/` and `tesmioloader.backup/` from the game dir.
4. **Workshop release**: update item 3776784867 — new DLL (customstops + FPS
   fix + geo stops), rewrite the description (`modesc.txt` in
   `~/…/SovietRepublic/RailPhysics/`): it still says tesmioloader is
   required — now it's soviet-mod-loader; answer the FPS reporter (cause:
   64-slot curve-cache thrash with >64 trains; fixed).
   NOTE: workshop upload goes through the game's own uploader from
   `media_soviet/workshop_wip/3776784867/railphysics/` — but that pipeline
   packages the tesmioloader layout; the SML layout (mod.ini + bin/) needs
   rethinking for the workshop. Also: SML itself is PresFox's prototype —
   how end users get the loader is an open question (link his repo? bundle?
   discuss with the user before publishing).
5. RailPhysics README/`docs` are current as of iteration 15; keep
   `docs/03-testing.md` updated with the acceptance results.

## 7. Conventions when editing railphysics.cpp

- Single-file plugin, heavy comments, offsets via `#define` with citations to
  findings docs; mark certainty [C]/[I]/[?] like the findings do.
- Every patch site byte-verified at load; runtime struct reads must be
  ReadablePtr-guarded (use ReadableFast in hot paths).
- Build must stay `-Wall` clean (8 pre-existing warnings are known: `y2`
  set-but-unused, 7 unused header helpers).
- The shim (`~/SML_RailPhysics/shim/sml_host.cpp`) implements the Tsm host
  ABI: Logf → `<modname>.log`, configInt/configString (ini at
  `mods/<id>/plugins/<id>.ini`), allocNear, installInlineHook (14-byte
  FF 25 + trampoline), findIatSlot/patchIat, faultFilter; provide/consume are
  inert stubs (deposits/depletion are NOT portable because of this + VFS).
- Never git-commit without the user's explicit say-so.
