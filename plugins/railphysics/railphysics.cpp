// railphysics - realistic rail vehicle dynamics.
//
// The vanilla rail update (SOVIET64.exe 1.1.1.9, FUN_1406a7520) drives a train
// with one scalar, the "divisor" from FUN_140698c50: acceleration is
// 100/divisor km/h per second, braking a third of that, and the uphill drag
// 50/divisor * slope - a constant-by-speed 3.61*sqrt(P/(5m)) km/h/s, braking
// tied to engine power, and gravity that only ever pulls backward.
//
// This plugin replaces those with a physical model:
//
//     traction   F = min(P / v, mu * m_adhesive * g)   power cap, adhesion cap
//     resistance R = m * (A + B*v) + C*v^2             Davis, v in km/h
//     gravity    G = m * g * grade                     signed, applied by the
//                                                      slope hook only
//     brake      independent rates, m/s^2; freight softer on planned braking
//     fuel       burn follows mechanical power F*v
//
// and adds what the game does not have: curve speed limits, station and
// customs track held at a set speed from the first node to the moment the
// tail clears it, and a smooth approach to a stop that never falls below a
// release speed. All of that is measured in metres along the route, never in
// time (see "approach core").
//
// Mechanics:
//   * The divisor call is rel32-rewritten to RailDivisor, which returns 100/a
//     so the untouched vanilla arithmetic applies our traction.
//   * Both fuel calls are rewritten to RailFuel (load factor from mechanical
//     power; tank, refuel, grid draw and economy stay vanilla).
//   * The brake site is a mid-function inline hook (stubs.S); the helper
//     returns the per-frame decrement.
//   * The slope block's uphill-only JBE becomes a JMP and both apply sites
//     (the classic block and the 1.1.1.9 downhill assist) are hooked, so a
//     signed gravity delta is applied once per frame.
//   * The curve site clamps the final vanilla limit (xmm9) to our limit.
//
// All per-frame dt factors cancel in the helpers (they scale the vanilla
// increment by a ratio of rates), so no timer access is needed.

#include "../../src/tesmio_plugin.h"
#include <float.h>
#include <math.h>

// ---------------------------------------------------------------- game layout
//
// Build 1.1.1.9 (b23935965). NO hardcoded addresses: every code site is
// located at init by a wildcard signature scan of .text (same technique as
// railspeed v2.0). A signature that is missing or ambiguous disables only its
// subsystem with a log line - a game update that reshapes the code degrades
// gracefully instead of corrupting the process. The object layout (vehicle /
// chain / route offsets below) is data, not code, and has been stable across
// 1.1.1.x; the one .data global we need (the chain vector) is located through
// the code that references it and cross-checked for consistency.
//
// Where the sites were on 1.1.1.9 (for reference / future re-signing):
//   curve  0x6A8436 (+7 in SIG_CURVE)   brake 0x6A8553 (SIG_BRAKE @0)
//   slopeA 0x6A862D (+0x29 in SIG_SLOPE_A)  slopeB 0x6A8676 (+0x25 in SIG_SLOPE_B)
//   slope branch 0x6A8655 (SIG_SLOPE_B @+4) grid 0x1BDF8C (SIG_GRID @0)
//   call divisor 0x6A7970 (SIG_CALL_DIV @+3 -> 0x698C50)
//   call fuel    0x6A784F / 0x6A7896 (SIG_CALL_FUEL @+9/+11 -> 0x6B3B50)
//   fn power 0x698CF0, fn mass 0x698F80, chain vector 0x9E6A18

#define W (-1)

// Hook sites. Offsets are from the match start to the site the stub replaces.
static const short SIG_CURVE[] = {   // cmpb r13b,[rsi+0x1798]; cmpb r13b,[rsi+0xd41]; je rel8; movss xmm6,[rsi+0xd24]
    0x44,0x88,0xAE,0x98,0x17,0x00,0x00, 0x44,0x38,0xAE,0x41,0x0D,0x00,0x00,
    0x74,W, 0xF3,0x0F,0x10,0xB6,0x24,0x0D,0x00,0x00, 0x45,0x33,0xC9,0x45,0x33,0xC0
};  // site +7, stolen 17, je at site+7

static const short SIG_BRAKE[] = {   // subss xmm6,xmm0; movss [rsi+0xd24],xmm6; comiss xmm9,xmm6
    0xF3,0x0F,0x5C,0xF0, 0xF3,0x0F,0x11,0xB6,0x24,0x0D,0x00,0x00, 0x44,0x0F,0x2F,0xCE
};  // site +0, stolen 16

static const short SIG_SLOPE_A[] = { // steep-downhill assist block (s > 1.0):
    0x44,0x0F,0x2F,0x15,W,W,W,W,     // comiss xmm10, [rip+1.0]   (guard)
    0x76,W,                           // jbe rel8
    0x0F,0x28,0xCE, 0xF3,0x41,0x0F,0x5E,0xCF, 0xF3,0x0F,0x59,0xCF,
    0x45,0x33,0xC9, 0x45,0x33,0xC0
};  // site +0x29, stolen 21, continuation caps at the effective limit (xmm14)

static const short SIG_SLOPE_B[] = { // classic uphill-drag block (s < 0):
    0x45,0x0F,0x2F,0xE2,             // comiss xmm10, xmm12 (=0)  (guard)
    0x76,W,                           // jbe rel8  (patch site for the JBE->JMP)
    0x0F,0x28,0xCE, 0xF3,0x41,0x0F,0x5E,0xCF, 0xF3,0x0F,0x59,0xCF,
    0x45,0x33,0xC9, 0x45,0x33,0xC0
};  // site +0x25, stolen 21, branch at +4, continuation clamps >= 0

static const short SIG_GRID[] = {    // mov rax,[rbp+0x970]; mulss xmm0,[rdi+rbx+0x8c]
    0x48,0x8B,0x85,0x70,0x09,0x00,0x00, 0xF3,0x0F,0x59,0x84,0x1F,0x8C,0x00,0x00
};  // site +0, stolen 16

// Call sites: the rel32 target IS the function we patch in front of.
static const short SIG_CALL_DIV[] = {  // mov rcx,rsi; call <divisor>; movaps xmm15,xmm0; movaps xmm9,xmm14
    0x48,0x8B,0xCE, 0xE8,W,W,W,W, 0x44,0x0F,0x28,0xF8, 0x45,0x0F,0x28,0xCE, 0x45,0x0F,0x28,0xEC
};  // call +3  -> divisor fn (accel)
static const short SIG_CALL_FUEL1[] = { // xor r8d; movaps xmm1,xmm3; mov rcx,rsi; call <fuel>; movss [rbp+0x230]
    0x45,0x33,0xC0, 0x0F,0x28,0xCB, 0x48,0x8B,0xCE, 0xE8,W,W,W,W,
    0xF3,0x0F,0x11,0x85,0x30,0x02,0x00,0x00
};  // call +9  -> fuel fn (cruise)
static const short SIG_CALL_FUEL2[] = { // movss xmm1,[rip+0.65]; mov rcx,rsi; call <fuel>; mov rax,[rsi+0x1708]
    0xF3,0x0F,0x10,0x0D,W,W,W,W, 0x48,0x8B,0xCE, 0xE8,W,W,W,W, 0x48,0x8B,0x86,0x08,0x17,0x00,0x00
};  // call +11 -> fuel fn (station)

// Helper functions the plugin calls directly.
static const short SIG_FN_POWER[] = {  // power fn prologue + body
    0x48,0x8B,0xC4, 0x48,0x89,0x58,0x18, 0x57, 0x48,0x81,0xEC,0x80,0x00,0x00,0x00,
    0x4C,0x8B,0x89,0x08,0x17,0x00,0x00, 0x0F,0xB6,0xFA
};
static const short SIG_FN_MASS[] = {   // mass fn prologue + body
    0x48,0x89,0x5C,0x24,0x08, 0x48,0x89,0x74,0x24,0x10, 0x57, 0x48,0x83,0xEC,0x30,
    0x48,0x8B,0x81,0x08,0x17,0x00,0x00, 0x33,0xFF
};
static const short SIG_FN_DIVISOR[] = {// divisor fn (cross-check against call target)
    0x40,0x53, 0x48,0x83,0xEC,0x30, 0x0F,0x29,0x74,0x24,0x20, 0x48,0x8B,0xD9,
    0xE8,W,W,W,W, 0xB2,0x01, 0x48,0x8B,0xCB, 0x0F,0x28,0xF0, 0xE8,W,W,W,W,
    0x48,0x8B,0x83,0x08,0x17,0x00,0x00
};

// The global chain vector {begin,end} - a .data global; located through code
// that indexes it: mov rax,[rip+&vec]; mov rcx,[rax+rcx*8]; add rcx,0x320.
// Every match must resolve to the same address or the scan fails.
static const short SIG_CHAIN_VEC[] = {
    0x48,0x8B,0x05,W,W,W,W, 0x48,0x8B,0x0C,0xC8, 0x48,0x81,0xC1,0x20,0x03,0x00,0x00
};

// The tail of the car-transform setter FUN_1406afde0 (see "tilt"): lea rcx,
// [rdi+0xd60]; pos, rot = (pitch, yaw, 0), scale = 1 on the stack; call
// [rip+disp32] -> C3D_NODE::CreateFromPositionRotationScale.
static const short SIG_NODE_CALL[] = {
    0x48,0x8D,0x8F,0x60,0x0D,0x00,0x00,             // lea  rcx,[rdi+0xd60]
    0xF3,0x0F,0x11,0x74,0x24,0x30,                  // movss [rsp+30h],xmm6   rot.x = pitch
    0x4C,0x8D,0x4C,0x24,0x20,                       // lea  r9,[rsp+20h]      &scale
    0xF3,0x0F,0x11,0x7C,0x24,0x34,                  // movss [rsp+34h],xmm7   rot.y = yaw
    0x4C,0x8D,0x44,0x24,0x30,                       // lea  r8,[rsp+30h]      &rot
    0xF3,0x0F,0x11,0x4C,0x24,0x40,                  // movss [rsp+40h],xmm1   pos
    0x48,0x8D,0x54,0x24,0x40,                       // lea  rdx,[rsp+40h]     &pos
    0xF3,0x0F,0x11,0x54,0x24,0x44,
    0xF3,0x0F,0x11,0x5C,0x24,0x48,
    0xC7,0x44,0x24,0x20,0x00,0x00,0x80,0x3F,        // scale = 1, 1, 1
    0xC7,0x44,0x24,0x24,0x00,0x00,0x80,0x3F,
    0xC7,0x44,0x24,0x28,0x00,0x00,0x80,0x3F,
    0xC7,0x44,0x24,0x38,0x00,0x00,0x00,0x00,        // rot.z = 0: the roll
    0xFF,0x15,W,W,W,W                               // call [rip+disp32]
};  // call +0x54

#undef W

// Resolved at init. NULL = not located (subsystem skips itself).
static BYTE*  g_siteCurve;
static BYTE*  g_siteBrake;
static BYTE*  g_siteSlopeA;
static BYTE*  g_siteSlopeB;
static BYTE*  g_siteSlopeBranch;    // the JBE (76) inside SIG_SLOPE_B's guard
static BYTE*  g_siteGrid;
static BYTE*  g_siteCallDiv;
static BYTE*  g_siteCallFuel1;
static BYTE*  g_siteCallFuel2;
static BYTE*  g_siteNodeCall;       // FF 15: the car-transform call (tilt)
static BYTE*  g_fnDivisor;         // from call target, cross-checked with SIG_FN_DIVISOR
static BYTE*  g_fnFuel;             // from call target, both calls must agree
static BYTE*  g_fnPower;
static BYTE*  g_fnMass;
static BYTE*** g_chainVec;          // address of the global chain ptr vector {begin,end}
// vehicle instance
#define V_WAGON_VEC       0x3A0       // ptr begin (end at +8): wagon instances
#define V_FUEL_EMPTY      0x5FC
#define V_EMERGENCY       0x5DF
#define V_ROUTE_SEGS      0x6B8       // ptr begin (end at +8): route segments, travel order
#define V_ROUTE_OBJS      0x680       // ptr begin (end at +8): route objects (stop block B gate, findings/04 Q4)
#define V_ROUTE_IDX       0x700       // int, current route index
#define V_CUR_SEG         0x798       // current track segment
#define V_DIR             0x7A0       // u8: 0 = travelling node0 -> node1, 1 = node1 -> node0
#define V_POS             0x7A4       // float, m travelled into the current segment from
                                      // its ENTRY end, in either direction. The rail
                                      // update FUN_1406a7520 proves it: its look-ahead
                                      // point is pos + la for dir 0 and (len - pos) - la
                                      // (node0-based) for dir 1, and every stop ramp takes
                                      // len - pos as the distance left, whatever the dir.
                                      // 1.3.x and 2.0.0 read len - pos for dir 1: on those
                                      // segments every distance ran backwards [C: 2026-09
                                      // 2.0.0 run, odometer frozen then +1597 m at a join,
                                      // customs target receding, entered at 141 km/h].
#define V_STOP_NEXT       0xD39       // u8: will stop at the upcoming station
#define V_STOP_MORE       0xD3C       // u8: another stop follows further on
#define V_SPEED           0xD24       // float, km/h
#define V_SLOPE           0xD30       // float, pitch/0.12 clamped +-1
#define V_TYPE            0x1708      // -> vehicle type
#define V_INACTIVE        0x1794      // int, wagon excluded when non-zero

// track segment
#define S_POLY            0x08        // polyline point vector (end at +8), stride 0x18
#define S_NODE0           0x20        // start node (polyline distance origin)
#define S_NODE1           0x28        // end node
#define S_LEN             0x11C       // float, m
// polyline point: x +0x00, y +0x04, z +0x08, cumdist +0x14
// node position: x +0x04, y +0x08, z +0x0C

// vehicle type
#define T_CATEGORY        0x294       // int; 3 vagon, 4 locomotive, 5 service
#define T_TOPSPEED        0x8624
#define T_POWER           0x8678      // float kW
#define T_WEIGHT          0x867C      // float tonnes
// T_LENGTH (0x7A78) and T_TRANSPORT (0x8600) are with the consist code below.

typedef float (*t_TotalMass)(void* veh);
typedef float (*t_TotalPower)(void* veh, char one);
typedef float (*t_DivisorFn)(void* veh);
typedef float (*t_FuelFn)(void* veh, float load, unsigned char flag);

static t_TotalMass  g_TotalMass;
static t_TotalPower g_TotalPower;
static t_DivisorFn  g_OrigDivisor;
static t_FuelFn     g_OrigFuel;

extern "C" void  rp_brake_stub(void);
extern "C" void  rp_slope_stub(void);
extern "C" void  rp_slope_stub2(void);
extern "C" void  rp_curve_stub(void);
extern "C" void  rp_grid_stub(void);
extern "C" void* rp_brake_back;      // filled with runtime addresses at init
extern "C" void* rp_slope_back;
extern "C" void* rp_slope_back2;
extern "C" void* rp_curve_back;
extern "C" void* rp_curve_branch;
extern "C" void* rp_grid_back;
extern "C" float rp_grid_k;

// ---------------------------------------------------------------- config

static struct Cfg
{
    int   accel;
    int   brake;
    int   slope;
    int   fuel;
    int   curves;
    int   stations;
    int   smoothStop;
    int   customStop;
    float customsEntry;     // km/h target at the customs zone entry
    float stopRelease;      // km/h the approach to a stop never drops below
    float freightBrake;     // m/s^2, planned braking of a train without passengers
    int   bogies;           // place cars on their modelled bogies (see "bogies")
    int   tilt;             // cars lean into curves at speed (see "tilt")
    float tiltMax;          // radians, the most a car leans
    float tiltGain;         // share of atan(v^2 k / g) the car leans by
    float tiltMinKmh;       // no lean below this; full lean 40 km/h above it
    float tiltRate;         // radians per second of game time
    int   tiltFreight;      // freight leans too
    float stationLimit;     // km/h through station track
    float curveLat;         // lateral accel comfort limit in curves, m/s^2
    float curveMargin;      // braking-budget safety divisor (>1 = brake earlier)
    float curveLookahead;   // meters of path scanned ahead
    int   logCurves;
    float mu;               // adhesion coefficient (steel/steel, sanded)
    float powerScale;       // arcade multiplier on engine power (1.0 = honest)
    float davisA;           // N per tonne, constant
    float davisB;           // N per tonne per km/h
    float davisC;           // N per (km/h)^2, whole train (aero)
    float gradeScale;       // slopeAvg -> grade; vanilla implies 0.12
    float serviceBrake;     // m/s^2
    float emergencyBrake;   // m/s^2
    float brakeFloorRatio;  // never weaker than vanilla*this; 0 disables
    float idleLoad;         // fuel load factor at standstill
    float loadMax;          // fuel load factor cap
    float electricLoadScale;// demand-side scale for electric traction
    float gridBoost;        // plant outflow ceiling multiplier; 1.0 = off
    int   logPhysics;       // periodic per-train log lines
    int   logDecisions;     // per-train decision flight recorder
} g;

// ------------------------------------------------- readable-pointer fast path
//
// ReadablePtr (the host's readablePtr) is VirtualQuery-backed: one syscall
// per call, and the consist walk alone makes two per wagon, 2-3 times per
// train per frame. Game objects live in long-lived committed regions, so
// cache the last verified region and answer hits with two compares; misses
// take the full check and then learn the region extent from VirtualQuery.
// __thread: whether the game's vehicle dispatcher is multi-threaded is not
// established, and this stays correct (and lock-free) either way. A region
// decommitted while cached would be trusted until the next miss - the same
// race the raw checks already have, with a wider window; accepted for the
// syscall saving.
//
// 1.3.1: the single-entry cache above was the FPS collapse on big maps. A
// consist walk alternates wagon instances and wagon TYPES, which live in
// different regions, so every call missed and paid two syscalls - 4 per
// wagon per frame, every train, every frame. Worse, VirtualQuery returns
// the run from the PAGE of p onward, so an object below the first queried
// page missed as well. Now: 16 regions per thread, entries that share an
// end are merged downward (every query into one committed run reports the
// same end), and the whole table is dropped once a second so a decommitted
// region is trusted for at most that long.
#define RP_REGIONS 16
static __thread BYTE*    t_rpBase[RP_REGIONS];
static __thread BYTE*    t_rpEnd[RP_REGIONS];
static __thread unsigned t_rpNext;
static __thread DWORD    t_rpStamp;

static int ReadableFast(const void* p, size_t n)
{
    BYTE* b = (BYTE*)p;
    DWORD now = GetTickCount();
    if ((DWORD)(now - t_rpStamp) > 1000)
    {
        t_rpStamp = now;
        for (int i = 0; i < RP_REGIONS; i++) t_rpBase[i] = t_rpEnd[i] = NULL;
    }
    for (int i = 0; i < RP_REGIONS; i++)
        if (b >= t_rpBase[i] && b + n <= t_rpEnd[i]) return 1;
    if (!ReadablePtr(p, n)) return 0;
    MEMORY_BASIC_INFORMATION mbi;
    if (VirtualQuery(p, &mbi, sizeof(mbi)) && mbi.State == MEM_COMMIT)
    {
        BYTE* base = (BYTE*)mbi.BaseAddress;
        BYTE* end  = base + mbi.RegionSize;
        for (int i = 0; i < RP_REGIONS; i++)
            if (t_rpEnd[i] == end)
            {
                if (base < t_rpBase[i]) t_rpBase[i] = base;   // same run: extend down
                return 1;
            }
        unsigned s = t_rpNext++ % RP_REGIONS;
        t_rpBase[s] = base;
        t_rpEnd[s]  = end;
    }
    return 1;
}

// ---------------------------------------------------------------- the model

struct Physics
{
    int   ok;
    float v_kmh;
    float mass_t;
    float power_kW;
    float slopeAvg;
    float F_tr;             // tractive effort, N
    float R;                // rolling + aero resistance, N
    float F_g;              // grade force, N (signed)
    float a_ms2;            // signed net acceleration, gravity included
    float accel_kmh_s;      // a_ms2 * 3.6
    float trac_kmh_s;       // (F - R) / m * 3.6: what the divisor applies when
                            // the slope hook owns gravity
};

static void ComputePhysics(void* veh, Physics* out)
{
    memset(out, 0, sizeof(*out));
    BYTE* v = (BYTE*)veh;

    BYTE* type = *(BYTE**)(v + V_TYPE);
    if (!ReadableFast(type, T_WEIGHT + 4)) return;
    int cat = *(int*)(type + T_CATEGORY);
    if (cat < 3 || cat > 5) return;                      // not rail, stay vanilla

    float mass  = g_TotalMass(veh);
    float power = g_TotalPower(veh, 1) * g.powerScale;   // wear-derated, arcade-scaled
    if (!isfinite(mass) || !isfinite(power) || mass <= 0.01f) return;

    float slopeSum = *(float*)(v + V_SLOPE);
    float adh = 0.0f;
    if (*(float*)(type + T_POWER) > 0.0f) adh = *(float*)(type + T_WEIGHT);
    int n = 1;

    BYTE** wb = *(BYTE***)(v + V_WAGON_VEC);
    BYTE** we = *(BYTE***)(v + V_WAGON_VEC + 8);
    if (we < wb || we - wb > 512) return;
    for (BYTE** p = wb; p != we; ++p)
    {
        BYTE* w = *p;
        if (!ReadableFast(w, V_INACTIVE + 4)) return;
        if (*(int*)(w + V_INACTIVE)) continue;
        slopeSum += *(float*)(w + V_SLOPE);
        ++n;
        BYTE* wt = *(BYTE**)(w + V_TYPE);
        if (!ReadableFast(wt, T_WEIGHT + 4)) continue;
        if (*(int*)(wt + T_CATEGORY) == 4 && *(float*)(wt + T_POWER) > 0.0f)
            adh += *(float*)(wt + T_WEIGHT);            // coupled powered unit
    }
    if (adh <= 0.0f) adh = *(float*)(type + T_WEIGHT);

    float v_kmh = *(float*)(v + V_SPEED);
    if (!isfinite(v_kmh)) return;
    if (v_kmh < 0.0f) v_kmh = 0.0f;
    float v_ms  = v_kmh / 3.6f;

    float F_adh = g.mu * adh * 1000.0f * 9.81f;
    float F_pow = (v_ms > 0.5f) ? (power * 1000.0f / v_ms) : FLT_MAX;
    float F_tr  = F_pow < F_adh ? F_pow : F_adh;

    float R     = mass * (g.davisA + g.davisB * v_kmh) + g.davisC * v_kmh * v_kmh;
    float grade = (slopeSum / n) * g.gradeScale;
    float F_g   = mass * 1000.0f * 9.81f * grade;
    float a     = (F_tr - R - F_g) / (mass * 1000.0f);

    out->ok          = 1;
    out->v_kmh       = v_kmh;
    out->mass_t      = mass;
    out->power_kW    = power;
    out->slopeAvg    = slopeSum / n;
    out->F_tr        = F_tr;
    out->R           = R;
    out->F_g         = F_g;
    out->a_ms2       = a;
    out->accel_kmh_s = a * 3.6f;
    out->trac_kmh_s  = (F_tr - R) / (mass * 1000.0f) * 3.6f;
}

// Per-frame memo for the above. The divisor call and the two fuel call sites
// all need the same physics within one frame - 2-3 full consist walks per
// train per frame - and nothing in the inputs changes between them, so cache
// on (vehicle, tick). Same __thread reasoning as ReadableFast: correct with
// or without a multi-threaded dispatcher, and no shared state to lock.
static __thread void*   t_phVeh;
static __thread DWORD   t_phTick;
static __thread Physics t_ph;

static void ComputePhysicsCached(void* veh, Physics* out)
{
    DWORD now = GetTickCount();
    if (veh == t_phVeh && now == t_phTick) { *out = t_ph; return; }
    ComputePhysics(veh, &t_ph);
    t_phVeh  = veh;
    t_phTick = now;
    *out = t_ph;
}

// ---------------------------------------------------------------- divisor

// Vanilla computes the accel increment as PowerTime(0.001/D * 100), i.e.
// 100/D km/h per second. Returning 100/our_accel makes the untouched
// arithmetic apply our acceleration instead - including its multipliers
// (powerAvail) and its clamp to the speed limit.
static int g_slopeLive;              // the slope hook applies gravity
static int g_approachLive;           // the curve hook runs the approach core

static float RailDivisor(void* veh)
{
    Physics ph;
    ComputePhysicsCached(veh, &ph);
    if (!ph.ok) return g_OrigDivisor(veh);

    // With the slope hook live, gravity is applied there, every frame, signed;
    // counting it here as well would apply it twice while accelerating.
    float a = g_slopeLive ? ph.trac_kmh_s : ph.accel_kmh_s;
    if (a >= 0.0f && a < 0.001f) a = 0.001f;    // near standstill creep guard
    if (a < 0.0f && ph.v_kmh < 0.5f)
    {
        // Stalled: net force points backward at walking speed. Vanilla cannot
        // run a train in reverse, so report a huge divisor (accel ~ 0) and let
        // the slope hook below hold the train against the grade.
        static DWORD lastStallLog = 0;
        if (g.logPhysics && GetTickCount() - lastStallLog > 5000)
        {
            lastStallLog = GetTickCount();
            Logf("railphysics  stalled: v=%.1f mass=%.0ft P=%.0fkW F=%.0fkN G=%.0fkN grade=%.1f%%",
                 ph.v_kmh, ph.mass_t, ph.power_kW, ph.F_tr / 1000.0f, ph.F_g / 1000.0f,
                 ph.slopeAvg * g.gradeScale * 100.0f);
        }
        return 1e9f;
    }

    float D = 100.0f / a;
    if (D > 1e9f)  D = 1e9f;
    if (D < -1e9f) D = -1e9f;

    static DWORD lastLog = 0;
    if (g.logPhysics && GetTickCount() - lastLog > 5000)
    {
        lastLog = GetTickCount();
        Logf("railphysics  v=%.1f km/h mass=%.0f t P=%.0f kW F=%.0f kN R=%.0f kN G=%.0f kN a=%.2f m/s2",
             ph.v_kmh, ph.mass_t, ph.power_kW,
             ph.F_tr / 1000.0f, ph.R / 1000.0f, ph.F_g / 1000.0f, ph.a_ms2);
    }
    return D;
}

// ---------------------------------------------------------------- fuel

// The vanilla load factor is (1-(v/vlim)^2)*0.85 + 0.15 + 0.35*slope - a
// speed curve, not work. Ours is the mechanical power actually being
// delivered: full tractive effort while accelerating, resistance equilibrium
// in cruise, near-idle in braking and at standstill. Mapped onto the game's
// own 0..1 load so the absolute burn rates (and the economy around them)
// stay vanilla.
static float RailFuel(void* veh, float load, unsigned char flag)
{
    Physics ph;
    ComputePhysicsCached(veh, &ph);
    if (!ph.ok) return g_OrigFuel(veh, load, flag);

    float F_work;
    if (ph.a_ms2 > 0.0f) F_work = ph.F_tr;
    else
    {
        F_work = ph.R + ph.F_g;
        if (F_work < 0.0f) F_work = 0.0f;      // downhill: gravity does the work
    }
    float P_mech = F_work * (ph.v_kmh / 3.6f) / 1000.0f;   // kW
    float rated  = ph.power_kW > 1.0f ? ph.power_kW : 1.0f;

    float l = g.idleLoad + (1.0f - g.idleLoad) * (P_mech / rated);
    if (l > g.loadMax) l = g.loadMax;
    if (l < 0.0f)      l = 0.0f;

    // Demand-side scale for electric traction. The vanilla catenary feed
    // (trafo buffer + shared plant outflow ceiling) was sized for the
    // vanilla load curve, which rarely asked for full power; our mechanical
    // model asks for it every acceleration. This scales the request down for
    // players who prefer that bargain over building more grid.
    int electric = *(BYTE*)(*(BYTE**)((BYTE*)veh + V_TYPE) + 0x8680);
    if (electric && g.electricLoadScale < 1.0f)
        l *= g.electricLoadScale;

    float pa = g_OrigFuel(veh, l, flag);

    static DWORD lastFuelLog = 0;
    if (g.logPhysics && GetTickCount() - lastFuelLog > 5000)
    {
        lastFuelLog = GetTickCount();
        Logf("railphysics  fuel: v=%.0f km/h Pmech=%.0f kW load=%.2f pa=%.2f%s",
             ph.v_kmh, P_mech, l, pa, electric ? " (electric)" : "");
    }
    return pa;
}

// ---------------------------------------------------------------- brake

// Called from the stub with the live registers. Vanilla applies
// decrement = vanillaInc where vanillaInc = (100/(3*D)) * dt. We keep dt
// implicit: new = vanillaInc * ourRate / vanillaRate, always positive (the
// site subtracts it). With our negative-D settle states D may be negative;
// fabsf keeps the rate math honest.
//
// Two braking regimes. Planned braking - one of the plugin's own constraints
// (a curve, station or customs track, a stop) is the binding limit - is done
// at the train's planned rate, the one its parabolas were planned with
// (softer for a train without passengers), inside a proportional band so
// the bang-bang controller around the limit does not pulse. Everything else
// (signals, obstacles, the game's own stops) keeps the service rate and the
// vanilla-strength floor: the game's logic is tuned for that deceleration.
static float CurveLimitFor(void* veh, float* planRate);

extern "C" float rp_brake_helper(void* veh, float speed, float vanillaInc, float divisor,
                                 float limit)
{
    float D = fabsf(divisor);
    float vanillaRate = (D > 1e-6f) ? 100.0f / (3.0f * D) : 0.0f;
    if (vanillaRate <= 1e-4f) return fabsf(vanillaInc);

    int emergency = *(BYTE*)((BYTE*)veh + V_EMERGENCY) != 0;

    float planRate = g.serviceBrake;
    int planned = !emergency && g_approachLive &&
                  CurveLimitFor(veh, &planRate) <= limit + 0.5f;

    float rate = (emergency ? g.emergencyBrake : g.serviceBrake) * 3.6f;
    if (planned)
    {
        // Full planned rate at >= 30 km/h overshoot, easing to 25 % at the
        // limit: the limit moves in steps as scans refine it, and a full-rate
        // yank on every step reads as a pulse train [C: 1.3.0 runs 7-8].
        rate = planRate * 3.6f;
        float band = (speed - limit) / 30.0f;
        if (band < 0.25f) band = 0.25f;
        if (band < 1.0f) rate *= band;
    }
    else if (g.brakeFloorRatio > 0.0f)
    {
        float floorRate = vanillaRate * g.brakeFloorRatio;
        if (rate < floorRate) rate = floorRate;
    }
    return fabsf(vanillaInc) * (rate / vanillaRate);
}

// ---------------------------------------------------------------- slope
//
// s = slopeAvg * -0.5 is in the register at the site. The vanilla classic
// block runs when s < 0 and adds increment * s, i.e. slopeAvg > 0 is uphill
// and slows the train; ComputePhysics agrees (F_g = m g grade resists when
// slopeAvg > 0). The signed gravity rate is therefore -g * grade * 3.6 km/h
// per second: uphill slows, downhill speeds up. (1.3.x returned +g * grade -
// uphill pushed the train - and also counted gravity in the divisor, so the
// two cancelled while accelerating.) dt = increment * D / 100 cancels the
// timer scaling exactly (both operands carry D's sign).
//
// The 1.1.1.9 build split the block: the classic uphill drag (site B) and a
// steep-downhill assist (site A, s > 1.0). Our JMP opens B for every s, so
// on a steep descent both run in one frame - A first. A marks the vehicle,
// B consumes the mark and adds nothing: gravity once per frame, with no
// clock involved (1.3.0 keyed this on GetTickCount, whose 15.6 ms steps
// dropped gravity on every other frame for a lone train above 64 fps).

static __thread void* t_slopeAVeh;

static float GravityDelta(float vanillaInc, float s, float divisor)
{
    float dt = vanillaInc * divisor * 0.01f;
    if (!(dt > 0.0f) || dt > 2.0f) return 0.0f;
    float slopeAvg = s / -0.5f;
    float grade    = slopeAvg * g.gradeScale;
    return -9.81f * grade * 3.6f * dt;
}

extern "C" float rp_slope_helper_a(void* veh, float vanillaInc, float s, float divisor)
{
    t_slopeAVeh = veh;
    return GravityDelta(vanillaInc, s, divisor);
}

extern "C" float rp_slope_helper(void* veh, float vanillaInc, float s, float divisor)
{
    if (t_slopeAVeh == veh)
    {
        t_slopeAVeh = NULL;             // site A already applied this frame
        return 0.0f;
    }
    return GravityDelta(vanillaInc, s, divisor);
}

// ================================================================ approach core
//
// Everything the plugin adds on top of the game's own speed limits - curves,
// station and customs track, and the approach to a stop - is one model, and
// that model works in DISTANCE, never in time:
//
//   odometer     every train carries a metre counter advanced from its own
//                position on the route (segment + position along it). It
//                reads the same at x1 and x4, across a pause, and under
//                daynight's vehicle_scale. 1.3.x integrated speed x wall
//                clock instead, fell behind by the game-speed factor, and
//                that alone put trains into customs at 150 km/h [C: 2026-09
//                run, the game at ~x4].
//   constraints  what a route scan finds is stored as intervals on that
//                counter - [from, to] at v, tail-extended by the train's
//                length where the tail must clear it. The limit applied every
//                frame is the minimum over the list: a braking parabola
//                before 'from', v inside. It is exact between scans, so scans
//                are paced by distance.
//   facilities   stations and customhouses are recognised by NODE
//                COINCIDENCE (see below). One facility kind's runs along the
//                route merge across gaps of FAC_MERGE_M, so a two-zone
//                customs complex is held at one speed end to end instead of
//                re-accelerating between its zones [C: 2026-09 run,
//                50 -> 100 -> 50 at customs B].
//   the stop     is where the route runs onto a facility's own track and
//                ends there. The approach never drops below the release
//                speed; the game's own final ramp stops the train. 1.3.x
//                ended its parabola 25 m short at 0 km/h and could hold a
//                train there indefinitely [C: 24 s at customs B, released
//                only by a pause].

// ---------------------------------------------------------------- facilities
//
// Stations (chain type 0 cargo, 1 passenger, 0x60 waiting) and customhouses
// (0x14) are chain objects in the global chain vector; their track is the
// membership table at +0xA10 (stride 0x60, entry+0x20 = segment). That track
// is chain-local - separate segment and node objects from the mainline the
// route runs on - but its nodes sit at exactly the world positions of the
// mainline nodes where it attaches (gap 0.00-0.01 m [C]). So a facility is
// found on a route by node coincidence, and on its last stretch by the route
// running onto the facility's own segments.

#define CHAIN_TYPE_DESC        0x318   // -> descriptor; +0x360 = type id
#define CHAIN_TABLE            0xA10   // {begin,end}, stride 0x60; +0x20 = segment
#define CHAIN_TYPE_CUSTOMHOUSE 0x14

#define FAC_STATION   1
#define FAC_CUSTOMS   2

#define FAC_EPS_M       2.0f      // node coincidence radius
#define FAC_CELL_M      8.0f      // hash grid cell
#define FAC_MERGE_M     1000.0    // same-kind runs closer than this are one complex
#define FAC_RESCAN_MS   30000     // facilities are rebuilt this often

struct FacNode { float x, z; int chain; BYTE kind; };
struct FacSeg  { void* seg; int chain; BYTE kind; };

static FacNode* g_facNodes;   static int g_facNodeCount, g_facNodeCap;
static FacSeg*  g_facSegs;    static int g_facSegCount,  g_facSegCap;
static void**   g_facChains;  static int g_facChainCap;
static int*     g_facCell;    static int g_facCellN;
static int*     g_facNext;
static DWORD    g_facScanTick;
static int      g_facScanned, g_facStations, g_facCustoms;

// Chain type id, -1 when the object is not readable.
static int ChainType(BYTE* chain)
{
    if (!ReadableFast(chain, CHAIN_TYPE_DESC + 8)) return -1;
    BYTE* desc = *(BYTE**)(chain + CHAIN_TYPE_DESC);
    if (!ReadableFast(desc, 0x370)) return -1;
    return *(int*)(desc + 0x360);
}

static int FacKindOfType(int type)
{
    if (type == 0 || type == 1 || type == 0x60) return FAC_STATION;
    if (type == CHAIN_TYPE_CUSTOMHOUSE)          return FAC_CUSTOMS;
    return 0;
}

static unsigned FacCellHash(int ix, int iz)
{
    return ((unsigned)ix * 73856093u) ^ ((unsigned)iz * 19349663u);
}

static int __cdecl FacSegCmp(const void* a, const void* b)
{
    void* x = ((const FacSeg*)a)->seg;
    void* y = ((const FacSeg*)b)->seg;
    return (x > y) - (x < y);
}

static int GrowTo(void** arr, int* cap, int need, size_t elem)
{
    if (need <= *cap) return 1;
    int ncap = *cap ? *cap : 256;
    while (ncap < need) ncap *= 2;
    void* grown = realloc(*arr, (size_t)ncap * elem);
    if (!grown) return 0;
    *arr = grown;
    *cap = ncap;
    return 1;
}

static void RescanFacilities(void)
{
    DWORD now = GetTickCount();
    if (g_facScanned && (DWORD)(now - g_facScanTick) < FAC_RESCAN_MS) return;
    g_facScanTick = now;
    g_facScanned  = 1;

    int nodes = 0, segs = 0, chains = 0, stations = 0, customs = 0;
    if (g_chainVec)
    {
        BYTE** vb = *g_chainVec;
        BYTE** ve = *(g_chainVec + 1);
        long   nc = ((BYTE*)ve - (BYTE*)vb) / 8;
        if (vb && nc > 0 && nc <= 200000 && ReadableFast(vb, nc * 8))
            for (long i = 0; i < nc; i++)
            {
                BYTE* chain = vb[i];
                int   kind  = FacKindOfType(ChainType(chain));
                if (!kind) continue;
                if (kind == FAC_STATION && !g.stations && !g.smoothStop) continue;
                if (kind == FAC_CUSTOMS && !g.customStop) continue;

                BYTE* tb = *(BYTE**)(chain + CHAIN_TABLE);
                BYTE* te = *(BYTE**)(chain + CHAIN_TABLE + 8);
                long  ne = (te - tb) / 0x60;
                if (ne <= 0 || ne > 4096 || !ReadableFast(tb, ne * 0x60)) continue;

                if (!GrowTo((void**)&g_facChains, &g_facChainCap, chains + 1, sizeof(void*))) break;
                int ci = chains++;
                g_facChains[ci] = chain;
                if (kind == FAC_STATION) ++stations; else ++customs;

                for (long j = 0; j < ne; j++)
                {
                    BYTE* sg = *(BYTE**)(tb + j * 0x60 + 0x20);
                    if (!sg || !ReadableFast(sg, S_LEN + 4)) continue;
                    if (!GrowTo((void**)&g_facSegs, &g_facSegCap, segs + 1, sizeof(FacSeg))) break;
                    g_facSegs[segs].seg   = sg;
                    g_facSegs[segs].chain = ci;
                    g_facSegs[segs].kind  = (BYTE)kind;
                    ++segs;
                    for (int e = 0; e < 2; e++)
                    {
                        BYTE* nd = *(BYTE**)(sg + (e ? S_NODE1 : S_NODE0));
                        if (!nd || !ReadableFast(nd, 0x10)) continue;
                        if (!GrowTo((void**)&g_facNodes, &g_facNodeCap, nodes + 1, sizeof(FacNode))) break;
                        g_facNodes[nodes].x     = *(float*)(nd + 0x04);
                        g_facNodes[nodes].z     = *(float*)(nd + 0x0C);
                        g_facNodes[nodes].chain = ci;
                        g_facNodes[nodes].kind  = (BYTE)kind;
                        ++nodes;
                    }
                }
            }
    }

    if (segs > 1) qsort(g_facSegs, segs, sizeof(FacSeg), FacSegCmp);

    // Hash grid over node positions: a coincidence query touches nine cells.
    int cells = 1024;
    while (cells < nodes * 2) cells *= 2;
    if (cells != g_facCellN)
    {
        free(g_facCell);
        g_facCell  = (int*)malloc((size_t)cells * sizeof(int));
        g_facCellN = g_facCell ? cells : 0;
    }
    free(g_facNext);
    g_facNext = (int*)malloc((size_t)(nodes > 0 ? nodes : 1) * sizeof(int));
    if (!g_facCell || !g_facNext) nodes = 0;
    for (int c = 0; c < g_facCellN; c++) g_facCell[c] = -1;
    for (int k = 0; k < nodes; k++)
    {
        int ix = (int)floorf(g_facNodes[k].x / FAC_CELL_M);
        int iz = (int)floorf(g_facNodes[k].z / FAC_CELL_M);
        unsigned h = FacCellHash(ix, iz) & (unsigned)(g_facCellN - 1);
        g_facNext[k] = g_facCell[h];
        g_facCell[h] = k;
    }

    int changed = nodes != g_facNodeCount || segs != g_facSegCount ||
                  stations != g_facStations || customs != g_facCustoms;
    g_facNodeCount  = nodes;
    g_facSegCount   = segs;
    g_facStations   = stations;
    g_facCustoms    = customs;
    if (changed)
        Logf("railphysics  facilities: %d station(s), %d customhouse(s), %d track segments, %d nodes",
             stations, customs, segs, nodes);
}

// Facility kind (0 = none) of the node at (x, z); *chain = its chain index.
static int FacNodeAt(float x, float z, int* chain)
{
    if (g_facNodeCount <= 0 || g_facCellN <= 0) return 0;
    int ix = (int)floorf(x / FAC_CELL_M);
    int iz = (int)floorf(z / FAC_CELL_M);
    for (int dx = -1; dx <= 1; dx++)
        for (int dz = -1; dz <= 1; dz++)
        {
            unsigned h = FacCellHash(ix + dx, iz + dz) & (unsigned)(g_facCellN - 1);
            for (int k = g_facCell[h]; k >= 0; k = g_facNext[k])
            {
                float ex = g_facNodes[k].x - x, ez = g_facNodes[k].z - z;
                if (ex * ex + ez * ez <= FAC_EPS_M * FAC_EPS_M)
                {
                    *chain = g_facNodes[k].chain;
                    return g_facNodes[k].kind;
                }
            }
        }
    return 0;
}

// Facility kind (0 = none) owning this track segment; *chain = its index.
static int FacSegKind(void* seg, int* chain)
{
    if (g_facSegCount <= 0) return 0;
    FacSeg key;
    key.seg = seg;
    FacSeg* hit = (FacSeg*)bsearch(&key, g_facSegs, g_facSegCount, sizeof(FacSeg), FacSegCmp);
    if (!hit) return 0;
    *chain = hit->chain;
    return hit->kind;
}

// Is a customhouse among the train's route objects (+0x680)? The list names
// the customhouse a train is bound for from assignment on, kilometres out
// (probe U, findings Q6). Customs node coincidences count only for such a
// train: a border line can run past a customhouse it does not use.
static int BoundForCustoms(BYTE* v)
{
    BYTE** ob = *(BYTE***)(v + V_ROUTE_OBJS);
    BYTE** oe = *(BYTE***)(v + V_ROUTE_OBJS + 8);
    long   on = oe - ob;
    if (!ob || on <= 0 || on > 64 || !ReadableFast(ob, on * 8)) return 0;
    for (long j = 0; j < on; j++)
        if (ChainType(ob[j]) == CHAIN_TYPE_CUSTOMHOUSE) return 1;
    return 0;
}

// ---------------------------------------------------------------- the consist
//
// Length and class come from the vehicle types, located in the vehicle
// .ini parser (FUN_1403eb870) against the known +0x8678/+0x867C pair:
//   +0x7A78 float  max.z - min.z of the bounding box, the TRAIN_BBOX
//                  corrections included - the vehicle's length [C]
//   +0x8600 int    $RESOURCE_TRANSPORT_TYPE; 7 = passengers - the parser
//                  itself tests == 7 to pick "vagon_pasanger" [C]
// A train's length is the sum over the head and its active wagons (the
// vehicle window's "Length" figure); a train is a passenger train when any
// of its vehicles carries passengers - an EMU's head does.

#define T_LENGTH          0x7A78
#define T_TRANSPORT       0x8600
#define TRANSPORT_PASSENGER 7

static void BogieFix(BYTE* type);

static void ConsistInfo(BYTE* v, float* length, int* passenger)
{
    float len  = 0.0f;
    int   pass = 0, n = 0;
    BYTE* type = *(BYTE**)(v + V_TYPE);
    if (ReadableFast(type, T_TRANSPORT + 4))
    {
        BogieFix(type);
        float l = *(float*)(type + T_LENGTH);
        if (l > 1.0f && l < 100.0f) { len += l; ++n; }
        if (*(int*)(type + T_TRANSPORT) == TRANSPORT_PASSENGER) pass = 1;
    }
    BYTE** wb = *(BYTE***)(v + V_WAGON_VEC);
    BYTE** we = *(BYTE***)(v + V_WAGON_VEC + 8);
    if (we >= wb && we - wb <= 512)
        for (BYTE** p = wb; p != we; ++p)
        {
            BYTE* w = *p;
            if (!ReadableFast(w, V_INACTIVE + 4) || *(int*)(w + V_INACTIVE)) continue;
            BYTE* wt = *(BYTE**)(w + V_TYPE);
            if (!ReadableFast(wt, T_TRANSPORT + 4)) continue;
            BogieFix(wt);
            float l = *(float*)(wt + T_LENGTH);
            if (l > 1.0f && l < 100.0f) { len += l; ++n; }
            if (*(int*)(wt + T_TRANSPORT) == TRANSPORT_PASSENGER) pass = 1;
        }
    if (n == 0 || len < 5.0f || len > 3000.0f) len = 150.0f;    // unreadable: a plausible middle
    *length    = len;
    *passenger = pass;
}

// ---------------------------------------------------------------- bogies
//
// A rail car is placed on the track by two pivot points, found by walking the
// train's trail back from the car's front end (FUN_1406a7520):
//   front pivot  type+0x7AF8 metres behind the front end
//   back pivot   type+0x7AFC metres in front of the back end
// (swapped for a car coupled reversed, vehicle+0x4C4). The .ini sets them
// with $TRAIN_FORWARD_AXIS_DISTANCE / $TRAIN_BACKWARD_AXIS_DISTANCE and the
// type constructor defaults both to 3.0 - right for the ~15 m base-game cars,
// wrong for a 27 m Workshop car whose bogies sit 4-6 m in: the car pivots
// outboard of its bogies and, on a tight curve, the bogies (modelled rigid
// on the body) swing off the rails [user report 2026-09-27, CR400AF: about
// 0.27 m off at R 100 m].
//
// For a type still on those defaults, read its model once and put the pivots
// where its wheelsets are. The type starts with the full path of its folder
// (type+0x000, char[512] - the .ini parser copies it there first thing);
// C3DPath_GetFullPath maps "workshop_subscribed/..." to the real folder.
// Base-game models are one object and give nothing to measure - they keep
// the defaults they were made for.

#define T_AXIS_FWD        0x7AF8
#define T_AXIS_BWD        0x7AFC
#define T_FRONT_EXT       0x7A7C      // max.z of the corrected bounding box
#define T_BACK_EXT        0x7A80      // -min.z

typedef void (*t_GetFullPath)(char* out, const char* in);
static t_GetFullPath g_GetFullPath;

struct NmfBox { float mn[3], mx[3]; };

// Object bounding boxes of an .nmf model, -1 when it is not one. Format as
// read from the files (no spec): "fromObj\0", u32 names, u32 objects, u32
// file size; names x 64 bytes; then per object at S: u32 size, name[64],
// u32, two 4x4 matrices, bbox min xyz / max xyz at S+200; the next object
// at S + size.
static int NmfBoxes(const BYTE* d, size_t n, NmfBox* out, int max)
{
    if (n < 0x20 || memcmp(d, "fromObj", 8) != 0) return -1;
    unsigned names = *(const unsigned*)(d + 8);
    unsigned objs  = *(const unsigned*)(d + 12);
    if (names > 4096 || objs > 4096) return -1;
    size_t s = 0x14 + (size_t)names * 64 + 4;
    int k = 0;
    for (unsigned o = 0; o < objs && k < max; o++)
    {
        if (s + 232 > n) break;
        unsigned size = *(const unsigned*)(d + s);
        if (size < 232 || d[s + 4] == 0) break;
        memcpy(&out[k], d + s + 200, sizeof(NmfBox));
        ++k;
        s += size;
    }
    return k;
}

// Pivot distances from the ends that put the pivots at the modelled bogies;
// 0 when the model does not show one at each end. A wheelset spans the
// gauge, stands on the rail and is about a wheel diameter long; lacking
// wheelsets, any low, short part (bogie frames) will do.
static int BogieAxes(const NmfBox* b, int nb, float frontExt, float backExt, float length,
                     float* fwd, float* bwd)
{
    for (int pass = 0; pass < 2; pass++)
    {
        double lo[2] = { 1e9, 1e9 }, hi[2] = { -1e9, -1e9 };
        int    cnt[2] = { 0, 0 };
        for (int i = 0; i < nb; i++)
        {
            const NmfBox* x = &b[i];
            double w  = x->mx[0] - x->mn[0], zl = x->mx[2] - x->mn[2];
            double zc = 0.5 * (x->mx[2] + x->mn[2]);
            int ok = pass == 0
                ? (w >= 1.1 && x->mn[1] <= 0.15f && x->mx[1] >= 0.5f && x->mx[1] <= 1.4f &&
                   zl >= 0.5 && zl <= 1.5)
                : (x->mn[1] <= 0.5f && x->mx[1] <= 1.6f && zl >= 0.3 && zl <= 4.5);
            if (!ok || fabs(zc) < 1.0) continue;
            int side = zc > 0.0 ? 0 : 1;
            if (x->mn[2] < lo[side]) lo[side] = x->mn[2];
            if (x->mx[2] > hi[side]) hi[side] = x->mx[2];
            ++cnt[side];
        }
        if (!cnt[0] || !cnt[1]) continue;
        double zf = 0.5 * (lo[0] + hi[0]), zb = 0.5 * (lo[1] + hi[1]);
        double f = frontExt - zf, r = backExt + zb;
        if (f < 0.0 || r < 0.0 || f > 0.5 * length || r > 0.5 * length) return 0;
        *fwd = (float)f;
        *bwd = (float)r;
        return 1;
    }
    return 0;
}

static void*    g_bogieSeen[2048];
static unsigned g_bogieFixed;

static unsigned BogieHash(void* p)
{
    UINT_PTR x = (UINT_PTR)p >> 4;
    x *= 0x9E3779B97F4A7C15ULL;
    return (unsigned)(x >> 32);
}

static void BogieFix(BYTE* type)
{
    if (!g.bogies || !g_GetFullPath || !type) return;
    unsigned h = BogieHash(type) & 2047;
    for (int i = 0; i < 64; i++)
    {
        void** s = &g_bogieSeen[(h + i) & 2047];
        if (*s == type) return;                     // done before
        if (!*s) { *s = type; break; }
        if (i == 63) return;                        // crowded: skip, harmless
    }
    if (!ReadableFast(type, T_TRANSPORT + 4)) return;
    int cat = *(int*)(type + T_CATEGORY);
    if (cat != 3 && cat != 4) return;
    if (*(float*)(type + T_AXIS_FWD) != 3.0f || *(float*)(type + T_AXIS_BWD) != 3.0f) return;

    char folder[512], rel[600], full[1024];
    memcpy(folder, type, sizeof(folder) - 1);
    folder[sizeof(folder) - 1] = 0;
    size_t fl = strlen(folder);
    if (fl == 0 || fl > 480) return;
    for (size_t i = 0; i < fl; i++) if ((unsigned char)folder[i] < 0x20) return;
    sprintf(rel, "%s/main.nmf", folder);
    full[0] = 0;
    g_GetFullPath(full, rel);
    if (!full[0]) return;

    HANDLE f = CreateFileA(full, GENERIC_READ, FILE_SHARE_READ, NULL, OPEN_EXISTING, 0, NULL);
    if (f == INVALID_HANDLE_VALUE) return;
    DWORD size = GetFileSize(f, NULL), got = 0;
    BYTE* data = (size > 0x20 && size < 64u * 1024 * 1024) ? (BYTE*)malloc(size) : NULL;
    int   ok = data && ReadFile(f, data, size, &got, NULL) && got == size;
    CloseHandle(f);
    if (!ok) { free(data); return; }

    static NmfBox boxes[1024];
    int nb = NmfBoxes(data, size, boxes, 1024);
    free(data);
    float fwd, bwd;
    if (nb < 3 || !BogieAxes(boxes, nb, *(float*)(type + T_FRONT_EXT), *(float*)(type + T_BACK_EXT),
                             *(float*)(type + T_LENGTH), &fwd, &bwd))
        return;
    *(float*)(type + T_AXIS_FWD) = fwd;
    *(float*)(type + T_AXIS_BWD) = bwd;
    ++g_bogieFixed;
    Logf("railphysics  bogies: %s: pivots %.2f m / %.2f m from the ends (were 3.00 / 3.00)",
         folder, fwd, bwd);
}

// ---------------------------------------------------------------- trains
//
// Per-train state in an open-addressed table keyed by the vehicle pointer
// (fibonacci hash, 32-slot probe window). A slot silent for SLOT_STALE_MS
// belongs to nobody: a live train calls every frame, so this reclaims
// deleted trains - the 1.3.0 table never did, filled up over a long
// session, and then made every train rescan every frame.

#define SLOT_N         1024
#define SLOT_PROBE     32
#define SLOT_STALE_MS  60000
#define CONS_MAX       48

#define STOP_HAND_M    20.0      // release speed is reached this far before the stop
#define STOP_SERVE_M   25.0      // standing still this close to the stop serves it
#define STOP_KEEP_M    150.0     // the release cap outlives the stop point by this
#define STOP_AGREE_M   30.0      // two stop readings this close are the same stop

enum { K_NONE = 0, K_CURVE, K_STATION, K_CUSTOMS, K_STOP };
static const char* const g_kindNames[] = { "none", "curve", "station", "customs", "stop" };

struct Cons
{
    double from, to;    // odometer metres
    float  v;           // km/h inside [from, to (+ length when tail)]
    BYTE   kind, tail;
    BYTE   miss;        // scans that ended short of it; see ConsBeginScan
};

struct Train
{
    void*    veh;
    DWORD    seen, scanTick;

    double   odo;                       // metres travelled, position-derived
    BYTE*    odoSeg;
    double   odoDS;                     // distance from the leg's entry node
    float    odoLen;
    int      odoIdx;
    void*    odoRoute;
    BYTE     odoDir;
    unsigned odoResyncs;

    double   scanOdo;                   // odometer at the last route scan
    void*    scanRoute;
    long     scanRouteN;

    float    length;                    // consist length, m
    BYTE     passenger;
    float    planRate;                  // m/s^2 the parabolas are braked at
    float    aPlan;                     // planRate / margin: what they are planned with

    Cons     cons[CONS_MAX];
    int      nCons;

    BYTE     stopArmed, stopKind;
    double   stopAt;                    // odometer position of the stop
    double   stopCand;
    int      stopCandN;
    int      stopChain;
    int      servedChain;               // stop readings on this complex are spent...
    double   servedAt;                  // ...until the train is FAC_MERGE_M past here

    float    lastLimit;
    BYTE     lastKind, braking, kinkLog;
    DWORD    snapTick;
    double   tightAt[8];                // odometer of the tight curves logged lately
    int      tightN;
};

static Train         g_trains[SLOT_N];
static unsigned long g_evals, g_scans;

static unsigned SlotHash(void* p)
{
    UINT_PTR x = (UINT_PTR)p >> 4;              // game objects are 16-aligned
    x *= 0x9E3779B97F4A7C15ULL;
    return (unsigned)(x >> 32);
}

static float SegLen(BYTE* sg)
{
    if (!ReadableFast(sg, S_LEN + 4)) return -1.0f;
    float len = *(float*)(sg + S_LEN);
    return (len > 0.0f && len < 100000.0f) ? len : -1.0f;
}

static int SegsAdjacent(BYTE* a, BYTE* b)
{
    if (!ReadableFast(a, S_NODE1 + 8) || !ReadableFast(b, S_NODE1 + 8)) return 0;
    void* a0 = *(void**)(a + S_NODE0); void* a1 = *(void**)(a + S_NODE1);
    void* b0 = *(void**)(b + S_NODE0); void* b1 = *(void**)(b + S_NODE1);
    return a0 == b0 || a0 == b1 || a1 == b0 || a1 == b1;
}

// Advance the odometer from the train's position. Same leg: the change in
// distance from the leg's entry. New leg: the rest of the old one, every leg
// the route index skipped, and the distance into the new one. Anything that
// cannot be related (a reversal, a teleport, a rebuilt route that no longer
// holds the old leg) resyncs without adding - the next scan re-anchors.
static void OdoUpdate(Train* t, BYTE* cur, BYTE dir, float pos, void* route, long routeN, int routeIdx)
{
    float len = SegLen(cur);
    if (len < 0.0f) return;
    double ds = pos;                            // from the entry end, either direction

    if (!t->odoSeg)
    {
        t->odoSeg = cur; t->odoDS = ds; t->odoLen = len; t->odoDir = dir;
        t->odoIdx = routeIdx; t->odoRoute = route;
        return;
    }

    double d = -1.0;
    if (cur == t->odoSeg && dir == t->odoDir)
        d = ds - t->odoDS;
    else
    {
        BYTE** segs = (BYTE**)route;
        if (route == t->odoRoute && routeIdx > t->odoIdx && t->odoIdx >= 0 &&
            routeIdx < routeN && routeIdx - t->odoIdx <= 64 &&
            ReadableFast(segs, (size_t)routeN * 8) &&
            segs[t->odoIdx] == t->odoSeg && segs[routeIdx] == cur)
        {
            d = (double)t->odoLen - t->odoDS + ds;
            for (int i = t->odoIdx + 1; i < routeIdx && d >= 0.0; i++)
            {
                float l = SegLen(segs[i]);
                d = l < 0.0f ? -1.0 : d + l;
            }
        }
        else if (SegsAdjacent(t->odoSeg, cur))
            d = (double)t->odoLen - t->odoDS + ds;
    }

    if (d >= -0.5 && d < 2000.0)
    {
        if (d > 0.0) t->odo += d;
    }
    else
        ++t->odoResyncs;

    t->odoSeg = cur; t->odoDS = ds; t->odoLen = len; t->odoDir = dir;
    t->odoIdx = routeIdx; t->odoRoute = route;
}

// ---------------------------------------------------------------- the scan
//
// Walk the route from the train's leg to the horizon: collect the polyline
// in travel order (for curvature), the facilities on it (for station and
// customs extents), and whether the route ends on a facility's own track (the
// stop). Distances are metres ahead of the train; the caller anchors them on
// the odometer.

struct WalkPt { float x, z; float s; };
static WalkPt* g_pts;
static int     g_ptsCap;

struct FacMark { double s; BYTE kind; };
static FacMark* g_marks;
static int      g_marksCap;

struct ScanResult
{
    double coverage;        // metres of route actually seen
    double stopDist;        // metres to the stop, -1 = none
    int    stopKind, stopChain;
    int    nMarks;
    int    npts;
};

// Curvature over a +/-halfW window around point i: returns the heading change
// (radians, absolute) and the span in metres, 0 when the window is empty.
#define MIN_CHORD_M 1.0

static double WindowYaw(int i, int npts, double halfW, double* span)
{
    double s = g_pts[i].s;
    int lo = i, hi = i;
    while (lo > 0 && g_pts[lo - 1].s >= s - halfW) --lo;
    while (hi < npts - 1 && g_pts[hi + 1].s <= s + halfW) ++hi;
    if (hi <= i || lo >= i) { *span = 0.0; return 0.0; }
    // A chord needs a length to have a heading. The walk lists a node shared
    // by two legs twice, and next to a leg that has only its two nodes the
    // window's one neighbour on that side is the duplicate: atan2(0, 0) = 0
    // reads it as due +z, and a straight line off that axis "turns" by its
    // own bearing - R 10-40 m, 19-36 km/h, held for the train's length. Every
    // test track ran along +z, where that reading happens to be right.
    double cl = hypot(g_pts[i].x - g_pts[lo].x, g_pts[i].z - g_pts[lo].z);
    double cr = hypot(g_pts[hi].x - g_pts[i].x, g_pts[hi].z - g_pts[i].z);
    if (cl < MIN_CHORD_M || cr < MIN_CHORD_M) { *span = 0.0; return 0.0; }
    *span = g_pts[hi].s - g_pts[lo].s;
    double yawL = atan2(g_pts[i].x - g_pts[lo].x, g_pts[i].z - g_pts[lo].z);
    double yawR = atan2(g_pts[hi].x - g_pts[i].x, g_pts[hi].z - g_pts[i].z);
    double d = yawR - yawL;
    while (d >  3.14159265) d -= 2 * 3.14159265;
    while (d < -3.14159265) d += 2 * 3.14159265;
    return fabs(d);
}

static void AddMark(int* n, double s, int kind)
{
    if (!GrowTo((void**)&g_marks, &g_marksCap, *n + 1, sizeof(FacMark))) return;
    g_marks[*n].s    = s;
    g_marks[*n].kind = (BYTE)kind;
    ++*n;
}

static void ScanRoute(BYTE* v, double horizon, int customsBound, ScanResult* out)
{
    out->coverage = 0.0;
    out->stopDist = -1.0;
    out->stopKind = out->stopChain = 0;
    out->nMarks   = 0;
    out->npts     = 0;

    BYTE* cur = *(BYTE**)(v + V_CUR_SEG);
    if (!cur) return;
    BYTE** segs = *(BYTE***)(v + V_ROUTE_SEGS);
    BYTE** se   = *(BYTE***)(v + V_ROUTE_SEGS + 8);
    long   n    = se - segs;
    if (!segs || n <= 0 || n > 4096 || !ReadableFast(segs, (size_t)n * 8)) return;

    // The route index is authoritative (a route may pass a segment twice);
    // a pointer match is the fallback.
    int k = -1;
    int ridx = *(int*)(v + V_ROUTE_IDX);
    if (ridx >= 0 && ridx < n && segs[ridx] == cur) k = ridx;
    else
        for (long i = 0; i < n; i++)
            if (segs[i] == cur) { k = (int)i; break; }
    if (k < 0) return;

    float pos     = *(float*)(v + V_POS);
    BYTE  dir     = *(BYTE*)(v + V_DIR);
    float curLen  = SegLen(cur);
    if (curLen < 0.0f) return;
    double dStart = pos;                        // travelled into the leg, see V_POS

    double sLeg = 0.0;
    int    npts = 0, nMarks = 0, lastLegKind = 0, lastLegChain = 0;
    long   i;
    for (i = k; i < n && sLeg < horizon; i++)
    {
        BYTE* sg  = segs[i];
        float len = SegLen(sg);
        if (len < 0.0f) break;
        BYTE* nA = *(BYTE**)(sg + S_NODE0);
        BYTE* nB = *(BYTE**)(sg + S_NODE1);
        if (!ReadableFast(nA, 16) || !ReadableFast(nB, 16)) break;

        BYTE* entry;
        if (i == k) entry = dir ? nB : nA;
        else
        {
            BYTE* ps = segs[i - 1];
            BYTE* pA = *(BYTE**)(ps + S_NODE0);
            BYTE* pB = *(BYTE**)(ps + S_NODE1);
            entry = (nA == pA || nA == pB) ? nA : nB;
        }
        int   fromStart = (entry == nA);
        BYTE* exitNode  = fromStart ? nB : nA;
        double legIn    = (i == k) ? 0.0 : sLeg;
        double legOut   = sLeg + ((i == k) ? (double)len - dStart : (double)len);

        // Facilities: the route on a facility's own track (always counts),
        // or through a node that coincides with one (customs only for a
        // train bound for a customhouse).
        int chain = 0;
        int legKind = FacSegKind(sg, &chain);
        if (legKind == FAC_CUSTOMS && !g.customStop) legKind = 0;
        if (legKind == FAC_STATION && !g.stations && !g.smoothStop) legKind = 0;
        if (legKind)
        {
            AddMark(&nMarks, legIn, legKind);
            AddMark(&nMarks, legOut, legKind);
        }
        lastLegKind  = legKind;
        lastLegChain = chain;

        int nodeChain = 0;
        int nodeKind  = FacNodeAt(*(float*)(exitNode + 0x04), *(float*)(exitNode + 0x0C), &nodeChain);
        if (nodeKind == FAC_CUSTOMS && (!customsBound || !g.customStop)) nodeKind = 0;
        if (nodeKind == FAC_STATION && !g.stations && !g.smoothStop) nodeKind = 0;
        if (nodeKind) AddMark(&nMarks, legOut, nodeKind);

        // Polyline in travel order: entry node, points, exit node.
        BYTE* pb = *(BYTE**)(sg + S_POLY);
        BYTE* pe = *(BYTE**)(sg + S_POLY + 8);
        long  np = (pe - pb) / 0x18;
        if (np < 0 || np > 100000) break;
        if (np > 0 && !ReadableFast(pb, (size_t)np * 0x18)) break;
        for (long j = 0; j <= np + 1; j++)
        {
            long jj = fromStart ? j : (np + 1 - j);
            float x, z;
            double cum;
            if (jj == 0)           { x = *(float*)(nA + 0x04); z = *(float*)(nA + 0x0C); cum = 0.0; }
            else if (jj == np + 1) { x = *(float*)(nB + 0x04); z = *(float*)(nB + 0x0C); cum = len; }
            else
            {
                BYTE* pt = pb + (jj - 1) * 0x18;
                x = *(float*)(pt + 0x00); z = *(float*)(pt + 0x08);
                cum = *(float*)(pt + 0x14);
            }
            double along = fromStart ? cum : (double)len - cum;
            double sHere = (i == k) ? along - dStart : sLeg + along;
            if (i == k && sHere < -25.0) continue;     // a short window behind, as chord anchor
            if (!GrowTo((void**)&g_pts, &g_ptsCap, npts + 1, sizeof(WalkPt))) break;
            g_pts[npts].x = x;
            g_pts[npts].z = z;
            g_pts[npts].s = (float)sHere;
            ++npts;
        }

        sLeg = legOut;
    }

    out->coverage = sLeg;
    out->nMarks   = nMarks;
    out->npts     = npts;

    // The stop: the route runs onto a facility's own track and ends there.
    // Before that it ends at the facility's boundary node - an entry, which
    // the extent already holds at facility speed.
    if (i == n && lastLegKind &&
        ((lastLegKind == FAC_STATION && g.smoothStop) || (lastLegKind == FAC_CUSTOMS && g.customStop)))
    {
        out->stopDist  = sLeg;
        out->stopKind  = lastLegKind;
        out->stopChain = lastLegChain;
    }
}

// ---------------------------------------------------------------- constraints

static float KindSpeed(int kind)
{
    return kind == K_STATION ? g.stationLimit : kind == K_CUSTOMS ? g.customsEntry : FLT_MAX;
}

static void ConsAdd(Train* t, double from, double to, float v, int kind, int tail)
{
    // A run of the same facility kind that starts within FAC_MERGE_M of one
    // already held extends it - one complex, one speed, no re-acceleration
    // between its zones.
    if (kind == K_STATION || kind == K_CUSTOMS)
        for (int c = 0; c < t->nCons; c++)
        {
            Cons* e = &t->cons[c];
            if (e->kind != kind) continue;
            if (from <= e->to + FAC_MERGE_M && to >= e->from - FAC_MERGE_M)
            {
                if (from < e->from) e->from = from;
                if (to   > e->to)   e->to   = to;
                e->miss = 0;
                return;
            }
        }
    if (t->nCons >= CONS_MAX)
    {
        // Full: fold into the nearest entry of the same kind, conservatively.
        int best = -1;
        for (int c = 0; c < t->nCons; c++)
            if (t->cons[c].kind == kind &&
                (best < 0 || fabs(t->cons[c].to - from) < fabs(t->cons[best].to - from)))
                best = c;
        if (best < 0) return;
        Cons* e = &t->cons[best];
        if (from < e->from) e->from = from;
        if (to > e->to)     e->to = to;
        if (v < e->v)       e->v = v;
        return;
    }
    Cons* e = &t->cons[t->nCons++];
    e->from = from; e->to = to; e->v = v; e->kind = (BYTE)kind; e->tail = (BYTE)tail;
    e->miss = 0;
}

// Replace what the scan saw: entries ahead of the train and inside the
// scan's coverage are rebuilt; entries already entered stay until the tail
// clears them. Entries beyond the coverage survive CONS_MISS_MAX scans that
// ended short of them: a torn read of the route (the game re-chunks it near
// stops, and a scan can catch it mid-rebuild [C: 1.3.0 runs 5-6]) is a
// one-scan event and must not erase what it did not reach, while a real
// reroute's leftovers must not wait for the odometer to arrive under them.
#define CONS_MISS_MAX 3

static void ConsBeginScan(Train* t, double coverage)
{
    int w = 0;
    for (int c = 0; c < t->nCons; c++)
    {
        Cons* e = &t->cons[c];
        if (e->from <= t->odo) { t->cons[w++] = *e; continue; }
        if (e->from > t->odo + coverage && ++e->miss <= CONS_MISS_MAX) t->cons[w++] = *e;
    }
    t->nCons = w;
}

static void ConsExpire(Train* t)
{
    int w = 0;
    for (int c = 0; c < t->nCons; c++)
    {
        Cons* e = &t->cons[c];
        double end = e->to + (e->tail ? t->length : 0.0) + 5.0;
        if (t->odo <= end) t->cons[w++] = *e;
    }
    t->nCons = w;
}

static double StopAllowedKmh(Train* t, double d)
{
    double vr = g.stopRelease / 3.6;
    double r  = d - STOP_HAND_M;
    return sqrt(vr * vr + 2.0 * t->aPlan * (r > 0.0 ? r : 0.0)) * 3.6;
}

// The limit now: a braking parabola to each constraint ahead, its speed
// inside it, and the stop's release-floored parabola.
static float EvalCap(Train* t, int* kindOut)
{
    double best = FLT_MAX;
    int    kind = K_NONE;
    for (int c = 0; c < t->nCons; c++)
    {
        Cons* e = &t->cons[c];
        double allowed;
        if (t->odo < e->from)
        {
            double vms = e->v / 3.6;
            allowed = sqrt(vms * vms + 2.0 * t->aPlan * (e->from - t->odo)) * 3.6;
        }
        else
            allowed = e->v;
        if (allowed < best) { best = allowed; kind = e->kind; }
    }
    if (t->stopArmed)
    {
        double allowed = StopAllowedKmh(t, t->stopAt - t->odo);
        if (allowed < best) { best = allowed; kind = K_STOP; }
    }
    ++g_evals;
    *kindOut = kind;
    return (float)best;
}

// Curvature -> curve intervals. Heading change over a +/-10 m window gives
// the radius: the two chords lo->i and i->hi point along the tangents at
// their midpoints, which are span/2 apart along the track, so R = span / 2 /
// dyaw. (1.3.x divided the whole span and read every radius twice too large
// - its "R = 996 m" curves were ~500 m, and curve_lateral_ms2 = 1.4 acted as
// 2.8. The shipped ini now says 2.8 to keep the same speeds.) A +/-40 m
// window tells a curve from a kink. On a real arc the
// curvature is the same in both windows; at a kink or a small lateral jog -
// the join of a bridge span to its approach track [user report, 2026-09] -
// the heading change does not grow with the window, so its curvature halves
// or worse. Kinks under ~3 degrees are ignored; tighter corners always count.
#define KINK_MAX_RAD  0.05

// One run of curvature -> a tail-extended interval. A slow one is logged once
// (log_curves), keyed on where its tightest point lies - a run's ends move
// with the scan's reach, that point does not - with the geometry behind it.
static void AddCurve(Train* t, double from, double to, float v, double R, double span, double at)
{
    ConsAdd(t, t->odo + from, t->odo + to, v, K_CURVE, 1);
    if (!g.logCurves || v >= 100.0f) return;
    double where = t->odo + at;
    for (int i = 0; i < 8; i++)
        if (fabs(where - t->tightAt[i]) < 30.0) return;
    t->tightAt[t->tightN++ & 7] = where;
    Logf("railphysics  curve T…%04x: %.0f km/h, R %.0f m (window %.1f m), %.0f m ahead, run of %.0f m",
         (unsigned)((UINT_PTR)t->veh & 0xFFFF), v, R, span, at, to - from);
}

static void CurveIntervals(Train* t, int npts, double horizon)
{
    double runFrom = 0.0, runTo = 0.0, runV = FLT_MAX, runR = 0.0, runSpan = 0.0, runAt = 0.0;
    int    haveRun = 0;
    int    kinks = 0;
    double kinkS = 0.0, kinkDeg = 0.0;
    for (int i = 0; i < npts; i++)
    {
        double s = g_pts[i].s;
        if (s < -25.0) continue;
        if (s > horizon) break;
        double spanN, spanW;
        double yawN = WindowYaw(i, npts, 10.0, &spanN);
        if (spanN < 5.0 || yawN < 0.004 || yawN > 1.0) continue;
        if (yawN < KINK_MAX_RAD)
        {
            double yawW = WindowYaw(i, npts, 40.0, &spanW);
            if (spanW > spanN + 10.0 && yawW / spanW < 0.5 * yawN / spanN)
            {
                if (!kinks++) { kinkS = s; kinkDeg = yawN * 57.2958; }
                continue;
            }
        }
        double R = 0.5 * spanN / yawN;
        if (R < 10.0) R = 10.0;
        float vc = (float)(sqrt((double)g.curveLat * R) * 3.6);
        if (vc > 400.0f) continue;
        if (haveRun && s - runTo <= 30.0)
        {
            runTo = s;
            if (vc < runV) { runV = vc; runR = R; runSpan = spanN; runAt = s; }
        }
        else
        {
            if (haveRun) AddCurve(t, runFrom, runTo, (float)runV, runR, runSpan, runAt);
            runFrom = runTo = runAt = s;
            runV = vc; runR = R; runSpan = spanN;
            haveRun = 1;
        }
    }
    if (haveRun) AddCurve(t, runFrom, runTo, (float)runV, runR, runSpan, runAt);

    if (kinks && g.logCurves && !t->kinkLog)
    {
        t->kinkLog = 1;
        Logf("railphysics  curve: %d kink point(s) ignored ahead, first %.0f m out (%.1f deg)",
             kinks, kinkS, kinkDeg);
    }
    else if (!kinks)
        t->kinkLog = 0;
}

// Facility marks -> extents: consecutive marks of one kind within
// FAC_MERGE_M are one complex.
static void FacilityExtents(Train* t, int nMarks)
{
    for (int kind = FAC_STATION; kind <= FAC_CUSTOMS; kind++)
    {
        int ck = kind == FAC_STATION ? K_STATION : K_CUSTOMS;
        if (ck == K_STATION && !g.stations && !g.smoothStop) continue;
        double from = -1.0, to = -1.0;
        for (int m = 0; m < nMarks; m++)
        {
            if (g_marks[m].kind != kind) continue;
            double s = g_marks[m].s;
            if (from >= 0.0 && s - to <= FAC_MERGE_M) { if (s > to) to = s; continue; }
            if (from >= 0.0) ConsAdd(t, t->odo + from, t->odo + to, KindSpeed(ck), ck, 1);
            from = to = s < 0.0 ? 0.0 : s;
        }
        if (from >= 0.0) ConsAdd(t, t->odo + from, t->odo + to, KindSpeed(ck), ck, 1);
    }
}

// The stop target, anchored on the odometer. A reading within STOP_AGREE_M
// refines it; a different one needs a second agreeing reading before it
// moves the target (a torn read of the route is a one-scan event). A stop
// already served is not re-armed by the same complex's track.
static void StopReading(Train* t, double dist, int kind, int chain)
{
    if (dist < 0.0) return;
    double at = t->odo + dist;
    if (t->servedChain == chain + 1 && t->odo < t->servedAt + FAC_MERGE_M) return;

    if (!t->stopArmed)
    {
        t->stopArmed = 1; t->stopAt = at; t->stopKind = (BYTE)kind; t->stopChain = chain;
        t->stopCandN = 0;
        if (g.logDecisions)
            Logf("railphysics  decision T…%04x stop armed %.0f m ahead (%s)",
                 (unsigned)((UINT_PTR)t->veh & 0xFFFF), dist, kind == FAC_CUSTOMS ? "customs" : "station");
        return;
    }
    if (fabs(at - t->stopAt) <= STOP_AGREE_M) { t->stopAt = at; t->stopCandN = 0; return; }
    if (t->stopCandN > 0 && fabs(at - t->stopCand) <= STOP_AGREE_M)
    {
        if (g.logDecisions)
            Logf("railphysics  decision T…%04x stop moved %.0f -> %.0f m ahead",
                 (unsigned)((UINT_PTR)t->veh & 0xFFFF), t->stopAt - t->odo, dist);
        t->stopAt = at; t->stopKind = (BYTE)kind; t->stopChain = chain; t->stopCandN = 0;
        return;
    }
    t->stopCand  = at;
    t->stopCandN = 1;
}

static void StopUpdate(Train* t, float speed)
{
    if (!t->stopArmed) return;
    double d = t->stopAt - t->odo;
    if (speed < 1.0f && fabs(d) <= STOP_SERVE_M)
    {
        t->stopArmed   = 0;
        t->servedChain = t->stopChain + 1;
        t->servedAt    = t->odo;
        if (g.logDecisions)
            Logf("railphysics  decision T…%04x stop served %.0f m from the target",
                 (unsigned)((UINT_PTR)t->veh & 0xFFFF), d);
        return;
    }
    if (d < -STOP_KEEP_M) t->stopArmed = 0;     // passed without stopping
}

// ---------------------------------------------------------------- per frame

static void TrainInit(Train* t, void* veh, DWORD now)
{
    memset(t, 0, sizeof(*t));
    t->veh      = veh;
    t->seen     = now;
    t->length   = 150.0f;
    t->planRate = g.serviceBrake;
    t->aPlan    = g.serviceBrake / (g.curveMargin > 1.0f ? g.curveMargin : 1.0f);
    for (int i = 0; i < 8; i++) t->tightAt[i] = -1e9;
}

static Train* TrainFor(void* veh, DWORD now)
{
    Train* own = NULL;
    Train* spare = NULL;
    unsigned h = SlotHash(veh) & (SLOT_N - 1);
    for (int i = 0; i < SLOT_PROBE; i++)
    {
        Train* s = &g_trains[(h + i) & (SLOT_N - 1)];
        if (s->veh == veh) { own = s; break; }
        if (!spare && (!s->veh || (DWORD)(now - s->seen) > SLOT_STALE_MS)) spare = s;
    }
    if (own && (DWORD)(now - own->seen) > SLOT_STALE_MS)
    {
        TrainInit(own, veh, now);                    // a new train at a reused address
        return own;
    }
    if (own) return own;
    Train* s = spare ? spare : &g_trains[h];
    TrainInit(s, veh, now);
    if (g.logDecisions)
        Logf("railphysics  decision T…%04x slot init", (unsigned)((UINT_PTR)veh & 0xFFFF));
    return s;
}

static void Rescan(Train* t, BYTE* v, void* route, long routeN, float speed, DWORD now)
{
    float len; int pass;
    ConsistInfo(v, &len, &pass);
    t->length    = len;
    t->passenger = (BYTE)pass;
    t->planRate  = pass ? g.serviceBrake : g.freightBrake;
    if (t->planRate < 0.05f) t->planRate = 0.05f;
    t->aPlan     = t->planRate / (g.curveMargin > 1.0f ? g.curveMargin : 1.0f);

    // Horizon: the full-stop distance from this speed plus a margin that
    // exceeds the scan spacing, so nothing new appears inside braking range.
    double vms     = (speed > 0.0f ? speed : 0.0f) / 3.6;
    double brake   = vms * vms / (2.0 * t->aPlan);
    double horizon = brake + 250.0;
    if (horizon < g.curveLookahead) horizon = g.curveLookahead;
    if (horizon > 8000.0) horizon = 8000.0;

    ScanResult r;
    ScanRoute(v, horizon, g.customStop && BoundForCustoms(v), &r);

    ConsBeginScan(t, r.coverage);
    if (g.curves) CurveIntervals(t, r.npts, horizon);
    FacilityExtents(t, r.nMarks);
    StopReading(t, r.stopDist, r.stopKind, r.stopChain);

    // The route now runs on past the stop: the game has moved on (cleared at
    // the customs, or a stop the train will not make). Without this a train
    // that stood a little short of the target would crawl away at the
    // release speed for STOP_KEEP_M.
    if (t->stopArmed && r.stopDist < 0.0 && r.coverage > t->stopAt - t->odo + 50.0)
    {
        t->stopArmed = 0;
        if (g.logDecisions)
            Logf("railphysics  decision T…%04x stop dropped: the route continues past it",
                 (unsigned)((UINT_PTR)t->veh & 0xFFFF));
    }

    t->scanOdo    = t->odo;
    t->scanRoute  = route;
    t->scanRouteN = routeN;
    t->scanTick   = now;
    ++g_scans;
}

static void DecisionLog(Train* t, float speed, float cap, int kind, DWORD now)
{
    unsigned tag = (unsigned)((UINT_PTR)t->veh & 0xFFFF);
    if (kind != t->lastKind)
    {
        if (kind != K_NONE)
            Logf("railphysics  decision T…%04x engage %s limit=%.0f v=%.0f len=%.0f %s",
                 tag, g_kindNames[kind], cap, speed, t->length, t->passenger ? "passenger" : "freight");
        else
            Logf("railphysics  decision T…%04x release %s v=%.0f", tag, g_kindNames[t->lastKind], speed);
        t->lastKind = (BYTE)kind;
    }
    if (kind != K_NONE && cap < FLT_MAX)
    {
        int braking = speed > cap + 0.3f;
        if (braking != t->braking)
        {
            t->braking = (BYTE)braking;
            Logf("railphysics  decision T…%04x brake %s v=%.0f limit=%.0f (%s)",
                 tag, braking ? "ON" : "OFF", speed, cap, g_kindNames[kind]);
        }
        if (kind >= K_STATION && (DWORD)(now - t->snapTick) >= 1000)
        {
            t->snapTick = now;
            Logf("railphysics  decision T…%04x snap v=%.0f limit=%.0f stop=%.0f odo=%.0f",
                 tag, speed, cap, t->stopArmed ? t->stopAt - t->odo : -1.0, t->odo);
        }
    }
    else
        t->braking = 0;
}

// The limit this train is held to right now, km/h (FLT_MAX = none), and the
// deceleration its planned braking uses. Called from the curve hook and the
// brake hook, so every frame.
static float CurveLimitFor(void* veh, float* planRate)
{
    DWORD now = GetTickCount();
    BYTE* v   = (BYTE*)veh;

    // Read like the rest of the hook context: unguarded, inside this train's
    // own rail update, which is writing these same fields.
    void* route    = *(void**)(v + V_ROUTE_SEGS);
    long  routeN   = *(BYTE***)(v + V_ROUTE_SEGS + 8) - (BYTE**)route;
    BYTE* cur      = *(BYTE**)(v + V_CUR_SEG);
    int   routeIdx = *(int*)(v + V_ROUTE_IDX);
    float pos      = *(float*)(v + V_POS);
    BYTE  dir      = *(BYTE*)(v + V_DIR);
    float speed    = *(float*)(v + V_SPEED);
    if (!isfinite(speed) || speed < 0.0f) speed = 0.0f;

    EnterCriticalSection(&g_lock);
    RescanFacilities();

    Train* t = TrainFor(veh, now);
    t->seen = now;
    if (cur) OdoUpdate(t, cur, dir, pos, route, routeN, routeIdx);

    // Rescan by distance (at most the scan spacing into the margin), on a new
    // route vector, and on a wall-clock backstop for route growth.
    double vms   = speed / 3.6;
    double brake = vms * vms / (2.0 * t->aPlan);
    double space = 0.05 * (brake + 250.0);
    if (space < 40.0)  space = 40.0;
    if (space > 150.0) space = 150.0;
    DWORD age = now - t->scanTick;
    if (!t->scanTick || route != t->scanRoute || t->odo - t->scanOdo >= space ||
        (speed > 1.0f && age >= 1000) || age >= 10000)
        Rescan(t, v, route, routeN, speed, now);

    ConsExpire(t);
    StopUpdate(t, speed);

    int   kind;
    float cap = EvalCap(t, &kind);
    if (planRate) *planRate = t->planRate;

    if (g.logDecisions) DecisionLog(t, speed, cap, kind, now);
    if (g.logCurves && kind != K_NONE && cap < FLT_MAX &&
        (t->lastLimit <= 0.0f || fabsf(cap - t->lastLimit) > 5.0f))
    {
        t->lastLimit = cap;
        Logf("railphysics  limit: v=%.0f km/h limit=%.0f km/h (%s) len=%.0f m %s",
             speed, cap, g_kindNames[kind], t->length, t->passenger ? "passenger" : "freight");
    }

    static DWORD lastStats = 0;
    if (g.logPhysics && (DWORD)(now - lastStats) > 30000)
    {
        lastStats = now;
        Logf("railphysics  approach core: %lu evaluations / %lu scans, %u car type(s) put on their bogies",
             g_evals, g_scans, g_bogieFixed);
    }

    LeaveCriticalSection(&g_lock);
    return cap;
}

// Called from the site-C stub with XMM9 (the final vanilla limit) in xmm1.
extern "C" float rp_curve_helper(void* veh, float limit)
{
    float cl = CurveLimitFor(veh, NULL);
    return cl < limit ? cl : limit;
}

// ---------------------------------------------------------------- tilt
//
// Rail cars lean into curves at speed. Every rail vehicle - lead and wagon
// alike - is put in the world by FUN_1406afde0(vehicle, pos, yaw, pitch),
// which ends in
//     C3D_NODE::CreateFromPositionRotationScale(vehicle + 0xD60, &pos, &rot, &scale)
// with rot = (pitch, yaw, 0), the 0 a literal (`mov [rsp+38h], 0`). The
// engine turns that into a matrix in C3DMatrixCompleteTransform through
// C3DMatrixRotationYawPitchRoll(yaw = rot.y, pitch = rot.x, roll = rot.z),
// which multiplies Rz(roll) * Rx(pitch) * Ry(yaw) - row vectors, D3DX order:
// the roll turns the model about its own z, its length axis, before
// anything else. So the one `call [rip+disp32]` at the end of FUN_1406afde0
// is pointed at a slot holding RailNodeTransform, which writes rot.z and
// calls the engine. The model is one rigid body (see "bogies"): the whole
// car rolls, wheels and all, about its reference point at rail level.
//
// Signs, from those matrices: Ry turns the model's +z toward its +x as yaw
// grows, so a yaw growing along the track is a turn toward the car's +x
// side, and Rz with a negative angle moves the top toward +x. A car coupled
// back to front (vehicle+0x4C4) is given yaw + pi: the centre of its turn
// lies on its -x side and its roll is positive.
//
// Curvature is read off each car's own heading as it moves - the yaw it is
// drawn with, unwrapped, sampled every 2.5 m and compared over the four 10 m
// stretches of the last 40 m. The least-turning of the four counts, and
// only when all four turn the same way: an arc turns every stretch alike; a
// kink (a bridge join: a heading step between two straights, spread over the
// car's pivot spacing) turns one or two; a lateral jog turns both ways. At
// 300 km/h a 1.2 degree kink read as a curve would throw the car over to the
// cap and back. Speed is the lead's: a wagon's +0x398 is the vehicle pulling
// it. The car leans by tilt_gain * atan(v^2 k / g), capped at tilt_max_deg,
// faded in over the 40 km/h above tilt_min_kmh, and rolls toward that at
// tilt_rate_deg_s of GAME time - this frame's distance over the speed - so
// no clock is read and game speed changes nothing.

#define V_LEADER      0x398       // wagon -> the vehicle pulling it; 0 on the lead
#define V_REVERSED    0x4C4       // u8: car coupled back to front
#define V_NODE        0xD60       // C3D_NODE the car is drawn with
#define TILT_N        4096
#define TILT_PROBE    16
#define TILT_RING     32          // samples at least 2.5 m apart: 80 m and more
#define TILT_STEP_M   2.5
#define TILT_SPAN_M   10.0        // four spans: the 40 m looked back over
#define TILT_STALE_MS 60000

typedef void (*t_NodeCreate)(void* node, const float* pos, const float* rot, const float* scale);
static t_NodeCreate g_NodeCreate;       // the engine's, read from the call's own slot

struct TiltCar
{
    void*    veh;
    DWORD    seen;
    unsigned calls;
    BYTE     fresh, tilts;
    float    x, z, yaw;                 // at the last call
    float    roll;                      // radians, as drawn
    double   s, cont, since;            // metres moved, unwrapped yaw there, since the last sample
    double   ringS[TILT_RING], ringY[TILT_RING];
    int      head, n;
};

static TiltCar          g_tilt[TILT_N];
static CRITICAL_SECTION g_tiltLock;
static DWORD            g_tiltStats;
static float            g_tiltPeak;     // degrees, since the last stats line

static TiltCar* TiltSlot(void* veh, DWORD now)
{
    unsigned h = SlotHash(veh) & (TILT_N - 1);
    TiltCar* spare = NULL;
    for (int i = 0; i < TILT_PROBE; i++)
    {
        TiltCar* c = &g_tilt[(h + i) & (TILT_N - 1)];
        if (c->veh == veh)
        {
            if ((DWORD)(now - c->seen) > TILT_STALE_MS) c->fresh = 1;   // a new car at an old address
            return c;
        }
        if (!spare && (!c->veh || (DWORD)(now - c->seen) > TILT_STALE_MS)) spare = c;
    }
    TiltCar* c = spare ? spare : &g_tilt[h];
    c->veh   = veh;
    c->fresh = 1;
    return c;
}

// The unwrapped yaw where the car was at distance s, from the ring and the
// current point; 0 when the history does not reach back that far.
static int TiltYawAt(const TiltCar* c, double s, double* y)
{
    double s1 = c->s, y1 = c->cont;
    for (int i = 0; i < c->n; i++)
    {
        int j = (c->head - 1 - i + TILT_RING) % TILT_RING;
        double s0 = c->ringS[j], y0 = c->ringY[j];
        if (s0 <= s)
        {
            *y = s1 > s0 ? y0 + (y1 - y0) * (s - s0) / (s1 - s0) : y0;
            return 1;
        }
        s1 = s0; y1 = y0;
    }
    return 0;
}

// Curvature under the car, 1/m, signed like the yaw: the least-turning of
// the four 10 m stretches of the last 40 m, 0 unless all four turn one way.
static double TiltCurvature(const TiltCar* c)
{
    double y[5];
    y[0] = c->cont;
    for (int k = 1; k <= 4; k++)
        if (!TiltYawAt(c, c->s - k * TILT_SPAN_M, &y[k])) return 0.0;
    double best = 0.0;
    for (int k = 1; k <= 4; k++)
    {
        double kk = (y[k - 1] - y[k]) / TILT_SPAN_M;
        if (kk == 0.0 || (k > 1 && (kk > 0.0) != (best > 0.0))) return 0.0;
        if (k == 1 || fabs(kk) < fabs(best)) best = kk;
    }
    return best;
}

// A consist leans when it carries passengers - ConsistInfo's test, without
// its bogie pass, which is not this thread's to run.
static BYTE TiltConsist(BYTE* v)
{
    if (g.tiltFreight) return 1;
    BYTE* lead = *(BYTE**)(v + V_LEADER);
    if (!lead || !ReadableFast(lead, V_TYPE + 8)) lead = v;
    BYTE* t = *(BYTE**)(lead + V_TYPE);
    if (ReadableFast(t, T_TRANSPORT + 4) && *(int*)(t + T_TRANSPORT) == 7) return 1;
    BYTE** wb = *(BYTE***)(lead + V_WAGON_VEC);
    BYTE** we = *(BYTE***)(lead + V_WAGON_VEC + 8);
    long   n  = we - wb;
    if (!wb || n <= 0 || n > 512 || !ReadableFast(wb, (size_t)n * 8)) return 0;
    for (long i = 0; i < n; i++)
    {
        BYTE* w = wb[i];
        if (!ReadableFast(w, V_TYPE + 8)) continue;
        BYTE* wt = *(BYTE**)(w + V_TYPE);
        if (ReadableFast(wt, T_TRANSPORT + 4) && *(int*)(wt + T_TRANSPORT) == 7) return 1;
    }
    return 0;
}

// The roll to draw car v with, radians.
static float TiltFor(BYTE* v, const float* pos, float yaw)
{
    DWORD now = GetTickCount();
    EnterCriticalSection(&g_tiltLock);
    TiltCar* c = TiltSlot(v, now);
    c->seen = now;

    float  x  = pos[0], z = pos[2];
    double dy = (double)yaw - c->yaw;
    while (dy >  3.14159265) dy -= 2 * 3.14159265;
    while (dy < -3.14159265) dy += 2 * 3.14159265;
    double ds = hypot((double)x - c->x, (double)z - c->z);
    if (c->fresh || ds > 150.0 || fabs(dy) > 1.0)
    {
        // A new car, or a jump - placed, loaded, turned round: upright, no history.
        c->fresh = 0; c->calls = 0; c->tilts = 0;
        c->s = c->cont = c->since = 0.0;
        c->n = c->head = 0;
        c->roll = 0.0f;
        ds = 0.0;
    }
    c->x = x; c->z = z; c->yaw = yaw;

    if (ds > 0.0)
    {
        c->s += ds; c->cont += dy; c->since += ds;
        if (c->since >= TILT_STEP_M)
        {
            c->ringS[c->head] = c->s;
            c->ringY[c->head] = c->cont;
            c->head = (c->head + 1) % TILT_RING;
            if (c->n < TILT_RING) ++c->n;
            c->since = 0.0;
        }
        if ((c->calls++ & 511) == 0) c->tilts = TiltConsist(v);

        BYTE* lead = *(BYTE**)(v + V_LEADER);
        if (!lead || !ReadableFast(lead, V_SPEED + 4)) lead = v;
        float kmh = *(float*)(lead + V_SPEED);
        if (!isfinite(kmh) || kmh < 0.0f) kmh = 0.0f;
        double vms = kmh / 3.6;

        double target = 0.0;
        if (c->tilts && kmh > g.tiltMinKmh)
        {
            double phi  = g.tiltGain * atan(vms * vms * TiltCurvature(c) / 9.81);
            double fade = (kmh - g.tiltMinKmh) / 40.0;
            if (fade < 1.0) phi *= fade;
            if (phi >  g.tiltMax) phi =  g.tiltMax;
            if (phi < -g.tiltMax) phi = -g.tiltMax;
            target = *(BYTE*)(v + V_REVERSED) ? phi : -phi;
        }
        double step = g.tiltRate * ds / (vms > 1.0 ? vms : 1.0);
        double r    = c->roll;
        r = target > r + step ? r + step : target < r - step ? r - step : target;
        c->roll = (float)r;
        float deg = fabsf(c->roll) * 57.2958f;
        if (deg > g_tiltPeak) g_tiltPeak = deg;
    }

    float roll = c->roll;
    if (g.logPhysics && (DWORD)(now - g_tiltStats) > 30000)
    {
        g_tiltStats = now;
        Logf("railphysics  tilt: cars leaned up to %.1f deg in the last 30 s", g_tiltPeak);
        g_tiltPeak = 0.0f;
    }
    LeaveCriticalSection(&g_tiltLock);
    return roll;
}

// What the patched call in FUN_1406afde0 now reaches: the roll the engine
// left at 0, then the engine's own function with the same arguments.
extern "C" void RailNodeTransform(BYTE* node, const float* pos, float* rot, const float* scale)
{
    rot[2] = TiltFor(node - V_NODE, pos, rot[1]);
    g_NodeCreate(node, pos, rot, scale);
}

// ---------------------------------------------------------------- patching
//
// Signature scan (same technique as railspeed v2.0): each cluster is located
// by its byte signature with -1 wildcards; a match must be UNIQUE or the site
// is refused (two matches means the signature no longer identifies what we
// think it does). Site addresses are offsets from the match; continuations
// are derived arithmetically, so nothing hardcoded survives into runtime.

static BYTE*  g_textStart;
static SIZE_T g_textSize;

static bool LocateText(void)
{
    IMAGE_DOS_HEADER* dos = (IMAGE_DOS_HEADER*)g_exeBase;
    if (dos->e_magic != IMAGE_DOS_SIGNATURE) return false;
    IMAGE_NT_HEADERS64* nt = (IMAGE_NT_HEADERS64*)(g_exeBase + dos->e_lfanew);
    if (nt->Signature != IMAGE_NT_SIGNATURE) return false;
    IMAGE_SECTION_HEADER* sec = IMAGE_FIRST_SECTION(nt);
    for (WORD i = 0; i < nt->FileHeader.NumberOfSections; ++i)
        if (memcmp(sec[i].Name, ".text", 5) == 0)
        {
            g_textStart = (BYTE*)g_exeBase + sec[i].VirtualAddress;
            g_textSize  = sec[i].Misc.VirtualSize;
            return true;
        }
    return false;
}

static bool InText(const void* p)
{
    return (BYTE*)p >= g_textStart && (BYTE*)p < g_textStart + g_textSize;
}

// Unique match or nothing. Ambiguity is failure on purpose.
static BYTE* ScanUnique(const short* sig, int len)
{
    BYTE* found = NULL;
    int   count = 0;
    if (len <= 0 || (SIZE_T)len > g_textSize) return NULL;
    const SIZE_T last = g_textSize - (SIZE_T)len;
    for (SIZE_T i = 0; i <= last; ++i)
    {
        BYTE* p = g_textStart + i;
        int   k = 0;
        for (; k < len; ++k)
        {
            const short want = sig[k];
            if (want < 0) continue;
            if (p[k] != (BYTE)want) break;
        }
        if (k != len) continue;
        ++count;
        if (count > 1) return NULL;
        found = p;
    }
    return (count == 1) ? found : NULL;
}

// For a rip-relative `mov reg, [rip+disp]` signature: every match must
// resolve to the SAME target or the scan fails (consistency check).
static BYTE* ScanSameTarget(const short* sig, int len, int dispOff)
{
    BYTE*   target = NULL;
    SIZE_T  last   = g_textSize - (SIZE_T)len;
    for (SIZE_T i = 0; i <= last; ++i)
    {
        BYTE* p = g_textStart + i;
        int   k = 0;
        for (; k < len; ++k)
        {
            const short want = sig[k];
            if (want < 0) continue;
            if (p[k] != (BYTE)want) break;
        }
        if (k != len) continue;
        BYTE* t = p + dispOff + 4 + *(INT32*)(p + dispOff);
        if (!target) target = t;
        else if (t != target) return NULL;      // disagree -> fail
    }
    return target;
}

// A call-site rewrite: verify the `e8 rel32` calls what we think, then point
// it at a near cave holding `mov rax, fn / jmp rax`. The original function is
// untouched and stays callable by absolute address.
static bool PatchCallSite(BYTE* site, BYTE* orig, void* fn, const char* label)
{
    if (!site || !orig)
    {
        Logf("railphysics  %s: site not located by signature, skipped", label);
        return false;
    }
    if (site[0] != 0xE8 || (BYTE*)site + 5 + *(INT32*)(site + 1) != orig)
    {
        Logf("railphysics  %s: call site at %08lX does not match 1.1.1.9 b23935965, skipped",
             label, (unsigned)(site - g_exeBase));
        return false;
    }

    BYTE* thunk = AllocNear(g_exeBase, 16);
    if (!thunk) { Logf("railphysics  %s: allocNear failed", label); return false; }
    thunk[0] = 0x48; thunk[1] = 0xB8;                    // mov rax, imm64
    *(void**)(thunk + 2) = fn;
    thunk[10] = 0xFF; thunk[11] = 0xE0;                  // jmp rax

    INT64 delta = (INT64)thunk - (INT64)(site + 5);
    if (delta > 0x7FFFFFFFLL || delta < -0x7FFFFFFFLL)
    {
        Logf("railphysics  %s: thunk out of rel32 range", label);
        return false;
    }

    DWORD prot = 0;
    if (!VirtualProtect(site, 5, PAGE_EXECUTE_READWRITE, &prot))
    {
        Logf("railphysics  %s: VirtualProtect failed (%lu)", label, GetLastError());
        return false;
    }
    *(INT32*)(site + 1) = (INT32)delta;
    VirtualProtect(site, 5, prot, &prot);
    FlushInstructionCache(GetCurrentProcess(), site, 5);

    Logf("railphysics  %s call site redirected", label);
    return true;
}

// The slope block is guarded by `if (s < 0)` (uphill only). Turn the JBE into
// a JMP so downhill runs it too; the hooked apply site gives the delta its
// sign, and flat (s == 0) still skips on its own. The rel8 displacement is
// wildcarded in the signature - only the JBE opcode itself is verified, and
// the displacement is preserved, so the skip target never changes.
static bool PatchSlopeBranch(void)
{
    BYTE* site = g_siteSlopeBranch;
    if (!site)
    {
        Logf("railphysics  slope branch not located by signature, skipped");
        return false;
    }
    if (site[0] != 0x76 || (site[1] & 0x80))          // JBE, forward rel8
    {
        Logf("railphysics  slope branch at %08X does not match 1.1.1.9 b23935965, skipped",
             (unsigned)(site - g_exeBase));
        return false;
    }
    DWORD prot = 0;
    if (!VirtualProtect(site, 2, PAGE_EXECUTE_READWRITE, &prot)) return false;
    site[0] = 0xEB;
    VirtualProtect(site, 2, prot, &prot);
    FlushInstructionCache(GetCurrentProcess(), site, 2);
    Logf("railphysics  slope branch opened both ways");
    return true;
}

// The car-transform call is `call [rip+disp32]` through the import slot of
// C3D_NODE::CreateFromPositionRotationScale. Only that one call is moved: its
// disp32 is pointed at a near slot of our own holding RailNodeTransform, and
// the import slot, which every other node in the game goes through, is left
// alone. The engine function must be the one the export names - if anything
// else already sits in the slot, tilt stays off rather than chain blindly.
static bool PatchNodeCall(void)
{
    BYTE* site = g_siteNodeCall;
    if (!site)
    {
        Logf("railphysics  tilt: car transform call not located by signature, tilt off");
        return false;
    }
    if (site[0] != 0xFF || site[1] != 0x15)
    {
        Logf("railphysics  tilt: call at %08X does not match 1.1.1.9, tilt off", (unsigned)(site - g_exeBase));
        return false;
    }
    void** slot  = (void**)(site + 6 + *(INT32*)(site + 2));
    void*  real  = ReadablePtr(slot, 8) ? *slot : NULL;
    void*  named = g_engine ? (void*)GetProcAddress(g_engine,
                       "?CreateFromPositionRotationScale@C3D_NODE@@QEAAXVC3DVECTOR3@@00@Z") : NULL;
    if (!real || real != named)
    {
        Logf("railphysics  tilt: the call does not reach C3D_NODE::CreateFromPositionRotationScale (%p, export %p), tilt off",
             real, named);
        return false;
    }
    BYTE* mine = AllocNear(g_exeBase, 16);
    if (!mine) { Logf("railphysics  tilt: allocNear failed, tilt off"); return false; }
    INT64 delta = (INT64)mine - (INT64)(site + 6);
    if (delta > 0x7FFFFFFFLL || delta < -0x7FFFFFFFLL)
    {
        Logf("railphysics  tilt: slot out of rel32 range, tilt off");
        return false;
    }
    g_NodeCreate  = (t_NodeCreate)real;
    *(void**)mine = (void*)RailNodeTransform;

    DWORD prot = 0;
    if (!VirtualProtect(site + 2, 4, PAGE_EXECUTE_READWRITE, &prot))
    {
        Logf("railphysics  tilt: VirtualProtect failed (%lu), tilt off", GetLastError());
        return false;
    }
    *(INT32*)(site + 2) = (INT32)delta;
    VirtualProtect(site + 2, 4, prot, &prot);
    FlushInstructionCache(GetCurrentProcess(), site, 6);
    Logf("railphysics  tilt: car transform call at +%08X redirected (max %.1f deg, gain %.2f, from %.0f km/h%s)",
         (unsigned)(site - g_exeBase), g.tiltMax * 57.2958f, g.tiltGain, g.tiltMinKmh,
         g.tiltFreight ? ", freight too" : ", passenger trains");
    return true;
}

// Resolve every site by signature. Each subsystem keeps working unless its
// own signature is missing; the global cross-checks (divisor/fuel call
// targets, chain vector) fail closed with a log line.
static void ResolveSites(void)
{
    BYTE* m;

    if ((m = ScanUnique(SIG_CURVE, (int)(sizeof(SIG_CURVE)/sizeof(SIG_CURVE[0])))))
    {
        g_siteCurve = m + 7;
        Logf("railphysics  curve site at +%08X (sig CURVE)",
             (unsigned)(g_siteCurve - g_exeBase));
    }
    if ((m = ScanUnique(SIG_BRAKE, (int)(sizeof(SIG_BRAKE)/sizeof(SIG_BRAKE[0])))))
    {
        g_siteBrake = m;
        Logf("railphysics  brake site at +%08X (sig BRAKE)",
             (unsigned)(g_siteBrake - g_exeBase));
    }
    if ((m = ScanUnique(SIG_SLOPE_A, (int)(sizeof(SIG_SLOPE_A)/sizeof(SIG_SLOPE_A[0])))))
    {
        g_siteSlopeA = m + 0x29;
        Logf("railphysics  slope site A at +%08X (sig SLOPE_A)",
             (unsigned)(g_siteSlopeA - g_exeBase));
    }
    if ((m = ScanUnique(SIG_SLOPE_B, (int)(sizeof(SIG_SLOPE_B)/sizeof(SIG_SLOPE_B[0])))))
    {
        g_siteSlopeB      = m + 0x25;
        g_siteSlopeBranch = m + 4;
        Logf("railphysics  slope site B at +%08X (sig SLOPE_B)",
             (unsigned)(g_siteSlopeB - g_exeBase));
    }
    if ((m = ScanUnique(SIG_GRID, (int)(sizeof(SIG_GRID)/sizeof(SIG_GRID[0])))))
    {
        g_siteGrid = m;
        Logf("railphysics  grid site at +%08X (sig GRID)",
             (unsigned)(g_siteGrid - g_exeBase));
    }

    if ((m = ScanUnique(SIG_CALL_DIV, (int)(sizeof(SIG_CALL_DIV)/sizeof(SIG_CALL_DIV[0])))))
    {
        g_siteCallDiv = m + 3;
        BYTE* t = g_siteCallDiv + 5 + *(INT32*)(g_siteCallDiv + 1);
        if (InText(t)) g_fnDivisor = t;
    }
    if ((m = ScanUnique(SIG_CALL_FUEL1, (int)(sizeof(SIG_CALL_FUEL1)/sizeof(SIG_CALL_FUEL1[0])))))
    {
        g_siteCallFuel1 = m + 9;
        BYTE* t = g_siteCallFuel1 + 5 + *(INT32*)(g_siteCallFuel1 + 1);
        if (InText(t)) g_fnFuel = t;
    }
    if ((m = ScanUnique(SIG_CALL_FUEL2, (int)(sizeof(SIG_CALL_FUEL2)/sizeof(SIG_CALL_FUEL2[0])))))
    {
        g_siteCallFuel2 = m + 11;
        BYTE* t = g_siteCallFuel2 + 5 + *(INT32*)(g_siteCallFuel2 + 1);
        if (!g_fnFuel) g_fnFuel = t;                 // first one wins...
        else if (t != g_fnFuel) g_fnFuel = NULL;     // ...disagreement fails
    }
    if (g_siteCallDiv)
        Logf("railphysics  divisor call at +%08X -> fn +%08X (sig CALL_DIV)",
             (unsigned)(g_siteCallDiv - g_exeBase), (unsigned)(g_fnDivisor - g_exeBase));
    if (g_siteCallFuel1 || g_siteCallFuel2)
        Logf("railphysics  fuel calls at +%08X/+%08X -> fn +%08X (sig CALL_FUEL)",
             (unsigned)((g_siteCallFuel1 ? g_siteCallFuel1 - g_exeBase : 0)),
             (unsigned)((g_siteCallFuel2 ? g_siteCallFuel2 - g_exeBase : 0)),
             (unsigned)(g_fnFuel - g_exeBase));

    if ((m = ScanUnique(SIG_FN_POWER, (int)(sizeof(SIG_FN_POWER)/sizeof(SIG_FN_POWER[0])))))
        g_fnPower = m;
    if ((m = ScanUnique(SIG_FN_MASS, (int)(sizeof(SIG_FN_MASS)/sizeof(SIG_FN_MASS[0])))))
        g_fnMass = m;

    // Divisor cross-check: the signature-derived entry point must equal the
    // call-derived one. Two independent ways to the same address.
    BYTE* divSig = ScanUnique(SIG_FN_DIVISOR, (int)(sizeof(SIG_FN_DIVISOR)/sizeof(SIG_FN_DIVISOR[0])));
    if (g_fnDivisor && divSig && divSig != g_fnDivisor)
    {
        Logf("railphysics  divisor fn cross-check failed (+%08X sig vs +%08X call) - accel disabled",
             (unsigned)(divSig - g_exeBase), (unsigned)(g_fnDivisor - g_exeBase));
        g_fnDivisor = NULL;
        g_siteCallDiv = NULL;
    }

    if ((m = ScanUnique(SIG_NODE_CALL, (int)(sizeof(SIG_NODE_CALL)/sizeof(SIG_NODE_CALL[0])))))
    {
        g_siteNodeCall = m + 0x54;
        Logf("railphysics  car transform call at +%08X (sig NODE_CALL)",
             (unsigned)(g_siteNodeCall - g_exeBase));
    }

    // Chain vector: located through the code that indexes it; every match
    // must resolve to the same .data address.
    g_chainVec = (BYTE***)ScanSameTarget(SIG_CHAIN_VEC, (int)(sizeof(SIG_CHAIN_VEC)/sizeof(SIG_CHAIN_VEC[0])), 3);
    if (!g_chainVec)
        Logf("railphysics  chain vector not located - station/customs zones disabled");
    else
        Logf("railphysics  chain vector at +%08X (sig CHAIN_VEC)",
             (unsigned)((BYTE*)g_chainVec - g_exeBase));
}

// ---------------------------------------------------------------- setup

static float CfgFloat(const char* ini, const char* key, float fallback)
{
    char v[64];
    if (H->configString(ini, "railphysics", key, v, sizeof(v), "") && v[0])
        return (float)atof(v);
    return fallback;
}

extern "C" __declspec(dllexport) unsigned TsmPluginApiVersion(void)
{
    return TSM_API_VERSION;
}

extern "C" __declspec(dllexport) int TsmPluginInit(const TsmHost* host, TsmPluginInfo* info)
{
    TsmBind(host);

    info->name    = "railphysics";
    info->version = "2.0.0";

    const char* ini = "plugins\\railphysics.ini";

    if (!H->configInt(ini, "railphysics", "enabled", 1))
    {
        Logf("railphysics  enabled = 0, nothing to do");
        return 1;
    }

    g.accel           = H->configInt(ini, "railphysics", "accel", 1);
    g.brake           = H->configInt(ini, "railphysics", "brake", 1);
    g.slope           = H->configInt(ini, "railphysics", "slope", 1);
    g.fuel            = H->configInt(ini, "railphysics", "fuel", 1);
    g.curves          = H->configInt(ini, "railphysics", "curves", 1);
    g.stations        = H->configInt(ini, "railphysics", "stations", 1);
    g.smoothStop      = H->configInt(ini, "railphysics", "smoothstop", 1);
    g.customStop      = H->configInt(ini, "railphysics", "customstop", 1);
    g.customsEntry    = CfgFloat(ini, "customs_entry_kmh", 50.0f);
    g.stopRelease     = CfgFloat(ini, "stop_release_kmh", 15.0f);
    if (g.stopRelease < 3.0f) g.stopRelease = 3.0f;     // 0 would re-create the stall
    g.stationLimit    = CfgFloat(ini, "station_limit_kmh", 30.0f);
    g.curveLat        = CfgFloat(ini, "curve_lateral_ms2", 2.8f);
    g.curveMargin     = CfgFloat(ini, "curve_brake_margin", 1.25f);
    g.curveLookahead  = CfgFloat(ini, "curve_lookahead_m", 1200.0f);
    g.logCurves       = H->configInt(ini, "railphysics", "log_curves", 0);
    g.mu              = CfgFloat(ini, "adhesion_mu", 0.30f);
    g.powerScale      = CfgFloat(ini, "power_scale", 1.0f);
    if (g.powerScale < 0.1f || g.powerScale > 5.0f)
    {
        Logf("railphysics  power_scale %.2f refused, sane range is 0.1-5.0", g.powerScale);
        g.powerScale = 1.0f;
    }
    g.davisA          = CfgFloat(ini, "davis_a", 1.5f);
    g.davisB          = CfgFloat(ini, "davis_b", 0.006f);
    g.davisC          = CfgFloat(ini, "davis_c", 0.40f);
    g.gradeScale      = CfgFloat(ini, "grade_scale", 0.12f);
    g.serviceBrake    = CfgFloat(ini, "service_brake_ms2", 0.8f);
    g.emergencyBrake  = CfgFloat(ini, "emergency_brake_ms2", 1.3f);
    g.freightBrake    = CfgFloat(ini, "freight_brake_ms2", 0.8f);
    g.bogies          = H->configInt(ini, "railphysics", "bogies", 1);
    g.tilt            = H->configInt(ini, "railphysics", "tilt", 1);
    g.tiltMax         = CfgFloat(ini, "tilt_max_deg", 5.0f);
    g.tiltGain        = CfgFloat(ini, "tilt_gain", 0.5f);
    g.tiltMinKmh      = CfgFloat(ini, "tilt_min_kmh", 80.0f);
    g.tiltRate        = CfgFloat(ini, "tilt_rate_deg_s", 4.0f);
    g.tiltFreight     = H->configInt(ini, "railphysics", "tilt_freight", 0);
    if (g.tiltMax < 0.0f)  g.tiltMax = 0.0f;
    if (g.tiltMax > 12.0f) g.tiltMax = 12.0f;           // a rigid car past this lifts off the rail
    if (g.tiltRate < 0.5f) g.tiltRate = 0.5f;
    g.tiltMax  /= 57.2958f;
    g.tiltRate /= 57.2958f;
    g.brakeFloorRatio = CfgFloat(ini, "brake_min_vanilla_ratio", 1.0f);
    g.idleLoad        = CfgFloat(ini, "idle_load", 0.05f);
    g.loadMax         = CfgFloat(ini, "fuel_load_max", 1.0f);
    g.electricLoadScale = CfgFloat(ini, "electric_load_scale", 1.0f);
    g.gridBoost       = CfgFloat(ini, "grid_boost", 1.0f);
    g.logPhysics      = H->configInt(ini, "railphysics", "log_physics", 0);
    g.logDecisions    = H->configInt(ini, "railphysics", "log_decisions", 0);

    if (g_engine)
        g_GetFullPath = (t_GetFullPath)GetProcAddress(g_engine, "?C3DPath_GetFullPath@@YAXPEADPEBD@Z");
    if (g.bogies && !g_GetFullPath)
        Logf("railphysics  bogies: C3DPath_GetFullPath not exported by the engine - bogie placement off");

    if (!LocateText())
    {
        Logf("railphysics  could not find .text, aborting");
        return 1;
    }

    ResolveSites();

    g_TotalMass   = (t_TotalMass)g_fnMass;
    g_TotalPower  = (t_TotalPower)g_fnPower;
    g_OrigDivisor = (t_DivisorFn)g_fnDivisor;
    g_OrigFuel    = (t_FuelFn)g_fnFuel;
    if (!g_TotalMass || !g_TotalPower || !g_OrigDivisor || !g_OrigFuel)
    {
        Logf("railphysics  helper functions not fully located - aborting");
        return 1;
    }

    rp_brake_back   = g_siteBrake   ? g_siteBrake   + 16 : NULL;
    rp_slope_back   = g_siteSlopeB  ? g_siteSlopeB  + 21 : NULL;
    rp_slope_back2  = g_siteSlopeA  ? g_siteSlopeA  + 21 : NULL;
    rp_curve_back   = g_siteCurve   ? g_siteCurve   + 17 : NULL;
    rp_curve_branch = g_siteCurve   ? g_siteCurve + 9 + (INT8)g_siteCurve[8] : NULL;
    rp_grid_back    = g_siteGrid    ? g_siteGrid    + 16 : NULL;
    rp_grid_k       = g.gridBoost;

    int patched = 0;

    // The expect arrays below are the exact stolen bytes; the host re-checks
    // them before hooking. The real verification is the unique signature scan
    // (ResolveSites) - the jz displacement in the curve site is read from the
    // found site so a rel8 drift cannot strand a correctly located cluster.
    BYTE expect[21];

    if (g.accel)
        patched += PatchCallSite(g_siteCallDiv, g_fnDivisor, (void*)RailDivisor, "accel/divisor");

    if (g.fuel)
    {
        patched += PatchCallSite(g_siteCallFuel1, g_fnFuel, (void*)RailFuel, "fuel (cruise)");
        patched += PatchCallSite(g_siteCallFuel2, g_fnFuel, (void*)RailFuel, "fuel (station)");
    }

    if (g.brake && g_siteBrake)
    {
        memcpy(expect, g_siteBrake, 16);
        void* tramp = NULL;
        if (InstallInlineHook(g_siteBrake, (void*)rp_brake_stub, &tramp,
                              expect, 16, "rail brake"))
            ++patched;
    }

    if (g.slope)
    {
        if (g_siteSlopeB)
        {
            memcpy(expect, g_siteSlopeB, 21);
            void* tramp = NULL;
            if (InstallInlineHook(g_siteSlopeB, (void*)rp_slope_stub, &tramp,
                                  expect, 21, "rail slope"))
            {
                ++patched;
                g_slopeLive = 1;
            }
        }
        // The 1.1.1.9 update added a second apply site (steep-downhill
        // assist, s > 1.0, capped at the effective limit). Hook it with the
        // same 21-byte pattern so the vanilla assist cannot fight the model;
        // the shared helper is idempotent per (vehicle, tick).
        if (g_siteSlopeA)
        {
            memcpy(expect, g_siteSlopeA, 21);
            void* tramp = NULL;
            if (InstallInlineHook(g_siteSlopeA, (void*)rp_slope_stub2, &tramp,
                                  expect, 21, "rail slope (downhill assist)"))
                ++patched;
        }
        patched += PatchSlopeBranch();               // (uphill-only degradation, logged)
    }

    if ((g.curves || g.stations || g.customStop || g.smoothStop) && g_siteCurve)
    {
        memcpy(expect, g_siteCurve, 17);
        void* tramp = NULL;
        if (InstallInlineHook(g_siteCurve, (void*)rp_curve_stub, &tramp,
                              expect, 17, "rail approach limit"))
        {
            ++patched;
            g_approachLive = 1;
        }
    }

    // Grid boost: scale the plant outflow ceiling (dt * storage capacity)
    // that the whole grid's transfer rate is divided from. Vanilla sized it
    // for a town plus the vanilla traction load curve; full-power electric
    // traction needs more. Pure-asm stub, no helper.
    if (g.gridBoost > 1.0f)
    {
        if (g.gridBoost > 10.0f)
        {
            Logf("railphysics  grid_boost %.1f refused, sanity cap is 10", g.gridBoost);
        }
        else if (g_siteGrid)
        {
            memcpy(expect, g_siteGrid, 16);
            void* tramp = NULL;
            if (InstallInlineHook(g_siteGrid, (void*)rp_grid_stub, &tramp,
                                  expect, 16, "grid boost"))
                ++patched;
        }
    }

    if (g.tilt && g.tiltMax > 0.0f)
    {
        InitializeCriticalSectionAndSpinCount(&g_tiltLock, 4000);
        patched += PatchNodeCall();
    }

    if (!patched)
    {
        Logf("railphysics  nothing patched, unloading");
        return 1;
    }

    Logf("railphysics  %d subsystem(s) patched", patched);
    return 0;
}
