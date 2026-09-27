# RailPhysics 2.0 Findings — SOVIET64.exe 1.1.1.9

What the 2.0 rewrite rests on: how a train's position, consist and
facilities read, how a car is put on the track and drawn, and what time is
in this game. Addresses are for build 1.1.1.9 (image base 0x140000000; the
plugin finds every site by signature, so these are for orientation, not for
patching).

Legend: **[C]** confirmed (in game, on live data or on the real files),
**[I]** inferred from the decompiler, **[?]** unknown.

---

## 1. Position on a segment **[C]**

| Field | Meaning |
|---|---|
| instance+0x798 | current track segment |
| instance+0x7A0 u8 | direction: 0 = node0 → node1, 1 = node1 → node0 |
| instance+0x7A4 float | metres travelled **from the segment's entry end**, in either direction |
| instance+0x6B8 {begin,end} | route segments in travel order |
| instance+0x700 int | index of the current segment in that route |
| instance+0x680 {begin,end} | route objects: the facilities the route is bound for |

`+0x7A4` counts from whichever end the train came in by. The vanilla
look-ahead in FUN_1406a7520 turns it into node0-based distance as
`pos + la` for direction 0 and `(len − pos) − la` for direction 1, and the
vanilla stop ramps use `len − pos` as the distance left. 1.x read `len − pos`
as the distance travelled on a reversed segment: the odometer stood still
while a train crossed one and jumped at its end, targets receded, and a
customs zone was entered at 141 km/h (2.0.0 run, 2026-09-27). The offline
harness now builds every other segment reversed.

## 2. The consist **[C]**

| Field | Meaning |
|---|---|
| lead+0x3A0 {begin,end} | the wagon instances behind the lead |
| wagon+0x398 | the vehicle pulling it; 0 on the lead itself |
| instance+0x4C4 u8 | car coupled back to front |
| instance+0xD24 float | speed, km/h (the lead's is the train's) |
| instance+0x1708 | → vehicle type |
| type+0x000 char[512] | the type's folder, as the ini parser FUN_1403eb870 copies it first thing: `workshop_subscribed/<id>/<name>` for a Workshop car |
| type+0x294 int | category: 3 wagon, 4 locomotive |
| type+0x7A78 float | car length; summed over the consist it is the length the UI shows (209 m for an 8-car CR400AF) |
| type+0x8600 int | transport class; 7 = passengers. A consist with any such car is planned as passenger, otherwise as freight |

FUN_1406a7520, the per-frame rail update, returns early into FUN_1406a5410
for a wagon (`+0x398 != 0`): a wagon has no speed logic of its own and is
placed along the lead's trail **[I]**. The railphysics speed hooks therefore
run for leads only.

## 3. Facilities **[C]**

- The chain vector at 0x1409E6A18 {begin,end} holds every station and
  customs complex. chain+0x318 → its descriptor; descriptor+0x360 int is the
  kind: 0 cargo station, 1 passenger station, 0x14 customhouse (0x60 also
  appears; **[?]** a waiting/other station kind).
- chain+0xA10 {begin,end}, stride 0x60, entry+0x20 = segment: the complex's
  own track. Those segments are different objects from the mainline's, but
  their nodes sit on the mainline's nodes to 0.00–0.01 m — so a route is
  matched to a facility by **node coincidence**, through a hash of node
  positions, not by pointer.
- A route that ends on a facility's own track is a stop there. A customs
  zone counts only for a train whose route objects (+0x680) include that
  customhouse: a train running along the border line is not stopped by it.
  Station zones count for every train passing through.

## 4. Track geometry **[C]**

Segment: +0x08 {begin,end} polyline, stride 0x18 (x +0, y +4, z +8,
cumulative distance +0x14); +0x20 node0; +0x28 node1; +0x11C length.
Node: x +4, y (height) +8, z +0xC.

- **Radius.** The chords lo → i and i → hi over a ±10 m window point along
  the tangents at their midpoints, which are span/2 apart along the track:
  `R = span / 2 / Δheading`. 1.x divided the whole span and read every
  radius twice too large — which is why `curve_lateral_ms2 = 1.4` in 1.x
  and 2.8 in 2.0 give the same speeds.
- **Kinks.** Bridge joins carry heading steps of 0.3–1.2° and lateral jogs
  of ~0.15 m (logged in game). They are told from curves by a ±40 m window:
  on an arc the heading change grows with the window, at a kink it does not.
- **Duplicate nodes.** A walk over consecutive segments lists the shared
  node twice. Where a segment has no intermediate points, a window's only
  neighbour on one side can be that duplicate: a zero-length chord, and
  `atan2(0, 0) = 0` reads as due +z — a straight line off that axis measured
  as R 10–40 m. Chords shorter than 1 m are now rejected (harness case
  "straight line off the z axis").
- Real station throats on player maps do reach R 19–50 m (measured with full
  windows); the 26–43 km/h they give at `curve_lateral_ms2 = 2.8` is
  geometry, not a bug.

## 5. Car pivots — "bogies" **[C]**

- type+0x7AF8 / +0x7AFC float: `$TRAIN_FORWARD_AXIS_DISTANCE` /
  `$TRAIN_BACKWARD_AXIS_DISTANCE`, the pivot's distance **inward from the
  car's front / back end** (swapped for a reversed car). The type
  constructor defaults both to 3.0.
- Read every frame by the placement blocks at the end of FUN_1406a7520
  (lead) and in FUN_1406a5410 (wagons): a point at each distance along the
  trail (FUN_14054bee0 / FUN_1406a52a0), the car's heading from one to the
  other.
- type+0x7A7C = max.z of the corrected bounding box (the front extent),
  type+0x7A80 = −min.z (the back extent); +0x7AF0/+0x7AF4 are the bbox
  corrections.
- The C3DDLL64 export `?C3DPath_GetFullPath@@YAXPEADPEBD@Z`
  (`void (char* out, const char* in)`) maps `workshop_subscribed/…` to the
  real folder.
- **NMF**, as read from the files (no spec): `"fromObj\0"`, u32 name count,
  u32 object count, u32 file size; names × 64 bytes; then per object at S:
  u32 block size, name[64] (UTF-8), u32, two 4×4 matrices, bbox min xyz /
  max xyz at S+200; the next object at S + size. `tools/nmfdump.py` lists them.
- Every object matrix seen is identity, and neither executable has a bogie,
  wheel or tire name for trains: the engine draws a car as **one rigid
  body**. Bogies cannot turn under a body without new models; the pivots are
  the one thing that decides whether the wheels sit on the rails.
- CR400AF (Workshop 3019370041): end car 5.78 / 4.13 m from the ends, middle
  cars 4.09 / 4.10. On the 3.0 default the end car's front bogie sat ~0.5 m
  off the rail at R 50 m.

## 6. The car transform — "tilt" **[C]**

- Every rail vehicle, lead and wagon, is put in the world by
  FUN_1406afde0(instance, pos*, yaw, pitch). For a reversed car it adds π to
  the yaw, negates the pitch and shifts pos by type+0x318/+0x324; then
  rot = (pitch, yaw, **0**) and scale = (1, 1, 1) go to
  `C3D_NODE::CreateFromPositionRotationScale(instance + 0xD60, &pos, &rot, &scale)`
  through `call [rip+disp32]` at 0x1406AFED6 (import slot 0x14086C360).
  FUN_1406afde0 has 8 call sites.
- C3DDLL64: CreateFromPositionRotationScale stores pos / rot / scale at
  node+0x80 / +0x8C / +0x98 and tail-calls UpdateFromPositionRotationScale →
  C3DMatrixCompleteTransform → C3DMatrixRotationYawPitchRoll(yaw = rot.y,
  pitch = rot.x, roll = rot.z), which multiplies Rz(roll) · Rx(pitch) ·
  Ry(yaw) — row vectors, D3DX order: the roll turns the model about its own
  z, its length axis, before anything else.
- Signs: Ry turns +z toward +x as yaw grows, so a yaw growing along the
  track is a turn toward the car's +x side; Rz with a negative angle moves
  the top toward +x. A car leans into a curve with
  `roll = −k · sign(dyaw/ds)`, a reversed car with `+`. Seen in game
  (2026-09-27): cars lean inward.
- 2.0 moves only that one call: its disp32 points at a near slot holding the
  plugin's function, which writes rot.z and calls the engine. The import slot,
  which every other node in the game goes through, is untouched.

## 7. Time **[C]**

The game runs at x1–x4, and plugins rescale time further (a day/night
plugin with vehicle and simulation scales is common). 1.x timed its stop
logic with the wall clock and broke at x4: a train stood 24 s before a stop
until the game was paused. 2.0 measures only distance — the odometer, the
scan spacing (40–150 m travelled), a stop served within 25 m at under
1 km/h, and the tilt rate (distance over speed is game time).
