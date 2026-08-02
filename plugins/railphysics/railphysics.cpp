// railphysics - realistic rail vehicle dynamics.
//
// The vanilla rail update (SOVIET64.exe 1.1.1.7, FUN_1406a7410) drives a train
// with one scalar, the "divisor" from FUN_140698b40:
//
//     accel        = 100 / divisor            km/h per second
//     brake        = 100 / (3 * divisor)      km/h per second
//     uphill drag  = 50 / divisor * slopeAvg  km/h per second (uphill only)
//
// with divisor = 27.7 / sqrt(max(0.025*mass, power) / (5*mass)) - i.e. a
// constant-by-speed acceleration of 3.61*sqrt(P/(5m)) km/h/s, braking that is
// always one third of it, and gravity that only ever pulls backward. Fuel burn
// is derived from engine power via a speed-ratio curve, not from work done.
//
// This plugin replaces all four pieces with a physical model:
//
//     traction   F = min(P / v, mu * m_adhesive * g)   power cap, adhesion cap
//     resistance R = m * (A + B*v) + C*v^2             Davis, v in km/h
//     gravity    G = m * g * grade                     signed - downhill pulls
//     accel      a = (F - R - G) / m
//     brake      independent rates, m/s^2, configurable
//     fuel       burn follows mechanical power F*v, not a speed-ratio curve
//
// Mechanics, in order of preference (see docs/01-architecture.md):
//
//   * The divisor call at 0x1406A7860 (the only call to FUN_140698b40 in the
//     rail update) is rel32-rewritten to RailDivisor, which returns 100/a so
//     the vanilla `0.001/D*100` arithmetic produces exactly our acceleration.
//   * Both fuel calls (0x1406A773F cruise, 0x1406A7786 station loading) are
//     rewritten to RailFuel, which converts real mechanical power into a load
//     factor and forwards to the original function - tank bookkeeping, refuel
//     logic, the electric grid draw and the economy stats all stay vanilla.
//   * The brake site at 0x1406A8443 is a mid-function inline hook; a stub in
//     stubs.S captures the register context, asks RailBrakeHelper for a new
//     per-frame decrement, replays the stolen instructions and jumps back.
//   * The slope block at 0x1406A8547 is gated behind `if (s < 0)` (uphill
//     only); the JBE at 0x1406A8545 becomes a JMP so downhill executes it too,
//     and the apply site at 0x1406A8566 is inline-hooked to apply a signed
//     gravity delta instead of the vanilla power-scaled drag.
//
// All per-frame dt factors cancel in the helpers (they scale the vanilla
// increment by the ratio of rates), so no timer access is needed.

#include "../../src/tesmio_plugin.h"
#include <float.h>
#include <math.h>

// ---------------------------------------------------------------- game layout
// SOVIET64.exe 1.1.1.7, RVAs from the image base.

#define RVA_TOTAL_MASS    0x698E70    // float fn(vehicle)                 -> tonnes
#define RVA_TOTAL_POWER   0x698BE0    // float fn(vehicle, char)           -> kW
#define RVA_DIVISOR_FN    0x698B40    // original divisor fn (fallback)
#define RVA_FUEL_FN       0x6B3A40    // float fn(vehicle, float load, u8) -> powerAvail

#define RVA_CALL_DIVISOR  0x6A7860    // e8 db 12 ff ff   call 0x140698b40
#define RVA_CALL_FUEL_1   0x6A773F    // e8 fc c2 00 00   call 0x1406b3a40
#define RVA_CALL_FUEL_2   0x6A7786    // e8 b5 c2 00 00   call 0x1406b3a40

#define RVA_BRAKE_SITE    0x6A8443    // mid-function, 16 bytes
#define RVA_BRAKE_BACK    0x6A8453
#define RVA_SLOPE_BRANCH  0x6A8545    // 76 41  JBE -> eb 41 JMP
#define RVA_SLOPE_SITE    0x6A8566    // mid-function, 21 bytes
#define RVA_SLOPE_BACK    0x6A857B
#define RVA_CURVE_SITE    0x6A8326    // mid-function, 17 bytes, XMM9 = final limit
#define RVA_CURVE_BACK    0x6A8337    // fall-through continuation
#define RVA_CURVE_BRANCH  0x6A8380    // the JZ target (roll-out path)
#define RVA_GRID_SITE     0x1BDF1C    // plant outflow ceiling, 16 bytes
#define RVA_GRID_BACK     0x1BDF2C

// vehicle instance
#define V_WAGON_VEC       0x3A0       // ptr begin (end at +8): wagon instances
#define V_FUEL_EMPTY      0x5FC
#define V_EMERGENCY       0x5DF
#define V_ROUTE_SEGS      0x6B8       // ptr begin (end at +8): route segments, travel order
#define V_ROUTE_IDX       0x700       // int, current route index
#define V_CUR_SEG         0x798       // current track segment
#define V_DIR             0x7A0       // u8: 0 = pos measured from +0x20 node end
#define V_POS             0x7A4       // float, m along current segment (travel measure)
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
extern "C" void  rp_curve_stub(void);
extern "C" void  rp_grid_stub(void);
extern "C" void* rp_brake_back;      // filled with runtime addresses at init
extern "C" void* rp_slope_back;
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
} g;

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
    float a_ms2;            // signed net acceleration
    float accel_kmh_s;      // a_ms2 * 3.6
};

static void ComputePhysics(void* veh, Physics* out)
{
    memset(out, 0, sizeof(*out));
    BYTE* v = (BYTE*)veh;

    BYTE* type = *(BYTE**)(v + V_TYPE);
    if (!ReadablePtr(type, T_WEIGHT + 4)) return;
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
        if (!ReadablePtr(w, V_INACTIVE + 4)) return;
        if (*(int*)(w + V_INACTIVE)) continue;
        slopeSum += *(float*)(w + V_SLOPE);
        ++n;
        BYTE* wt = *(BYTE**)(w + V_TYPE);
        if (!ReadablePtr(wt, T_WEIGHT + 4)) continue;
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
}

// ---------------------------------------------------------------- divisor

// Vanilla computes the accel increment as PowerTime(0.001/D * 100), i.e.
// 100/D km/h per second. Returning 100/our_accel makes the untouched
// arithmetic apply our acceleration instead - including its multipliers
// (powerAvail) and its clamp to the speed limit.
static float RailDivisor(void* veh)
{
    Physics ph;
    ComputePhysics(veh, &ph);
    if (!ph.ok) return g_OrigDivisor(veh);

    float a = ph.accel_kmh_s;
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
    ComputePhysics(veh, &ph);
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
// Two braking regimes. Planned braking - the binding limit is a curve or a
// station zone (limit == our curve limit) - uses exactly service_brake, so
// the braking parabola the lookahead computed is the one the train rides:
// gentle and early. Anything else (signals, obstacles, station stops, roll
// protection) keeps the vanilla-strength floor, because the game's own logic
// is tuned for that deceleration.
static float CurveLimitFor(void* veh);

extern "C" float rp_brake_helper(void* veh, float speed, float vanillaInc, float divisor,
                                 float limit)
{
    float D = fabsf(divisor);
    float vanillaRate = (D > 1e-6f) ? 100.0f / (3.0f * D) : 0.0f;
    if (vanillaRate <= 1e-4f) return fabsf(vanillaInc);

    int emergency = *(BYTE*)((BYTE*)veh + V_EMERGENCY) != 0;

    // planned = the curve/station lookahead is the binding constraint
    int planned = !emergency && g.curves && CurveLimitFor(veh) <= limit + 0.5f;

    float rate = (emergency ? g.emergencyBrake : g.serviceBrake) * 3.6f;
    if (!planned && g.brakeFloorRatio > 0.0f)
    {
        float floorRate = vanillaRate * g.brakeFloorRatio;
        if (rate < floorRate) rate = floorRate;
    }
    return fabsf(vanillaInc) * (rate / vanillaRate);
}

// ---------------------------------------------------------------- slope

// s = slopeAvg * -0.5 is in the register at the site; the signed gravity rate
// is g*grade*3.6 km/h per second. dt = vanillaInc * D / 100 cancels the timer
// scaling exactly (both operands carry D's sign, so the product is positive).
extern "C" float rp_slope_helper(void* veh, float vanillaInc, float s, float divisor)
{
    float dt       = vanillaInc * divisor * 0.01f;
    if (dt <= 0.0f) return 0.0f;
    float slopeAvg = s / -0.5f;
    float grade    = slopeAvg * g.gradeScale;
    float rate     = 9.81f * grade * 3.6f;     // signed: uphill < 0
    return rate * dt;
}

// ---------------------------------------------------------------- stations
//
// A station (cargo 0, passenger 1, waiting 0x60) is a chain object in the
// global chain vector; its track extent is the membership table at +0xA10
// (stride 0x60, entry+0x20 = segment ptr). We rebuild a sorted segment set
// every few seconds and the route sampler treats station segments as a zone
// with v = station_limit, fed through the same braking-curve budget as
// curves - so the train slows to the limit BEFORE the station boundary and
// accelerates after the head leaves it. (v1 limitation: the constraint
// tracks the head of the train; the tail may still be inside by up to a
// consist length when the limit lifts.)

#define RVA_CHAIN_VECTOR  0x9E6A18    // {begin,end} of the global chain ptr vector
#define CHAIN_TYPE_DESC   0x318       // -> descriptor; +0x360 = type id
#define CHAIN_TABLE       0xA10       // {begin,end}, stride 0x60; +0x20 = segment

static void** g_statSegs;
static int    g_statSegCount;
static DWORD  g_statScanTick;

static int __cdecl PtrCmp(const void* a, const void* b)
{
    void* x = *(void**)a;
    void* y = *(void**)b;
    return (x > y) - (x < y);
}

static int ChainIsStation(BYTE* chain)
{
    if (!ReadablePtr(chain, CHAIN_TYPE_DESC + 8)) return 0;
    BYTE* desc = *(BYTE**)(chain + CHAIN_TYPE_DESC);
    if (!ReadablePtr(desc, 0x370)) return 0;
    int type = *(int*)(desc + 0x360);
    return type == 0 || type == 1 || type == 0x60;
}

static void RescanStations(void)
{
    DWORD now = GetTickCount();
    if ((DWORD)(now - g_statScanTick) < 5000) return;
    g_statScanTick = now;

    g_statSegCount = 0;

    BYTE*** pvec = (BYTE***)(g_exeBase + RVA_CHAIN_VECTOR);
    BYTE**  vb   = *pvec;
    BYTE**  ve   = *(BYTE***)(g_exeBase + RVA_CHAIN_VECTOR + 8);
    long    nc   = ((BYTE*)ve - (BYTE*)vb) / 8;
    if (!vb || nc <= 0 || nc > 100000 || !ReadablePtr(vb, nc * 8)) return;

    int cap = 0;
    for (long i = 0; i < nc; i++)
    {
        BYTE* chain = (BYTE*)vb[i];
        if (!ChainIsStation(chain)) continue;

        BYTE** tb = *(BYTE***)(chain + CHAIN_TABLE);
        BYTE** te = *(BYTE***)(chain + CHAIN_TABLE + 8);
        long  ne = ((BYTE*)te - (BYTE*)tb) / 0x60;
        if (ne <= 0 || ne > 100000) continue;
        if (!ReadablePtr(tb, ne * 0x60)) continue;

        if (g_statSegCount + ne > cap)
        {
            cap = (g_statSegCount + (int)ne) * 2 + 64;
            g_statSegs = (void**)realloc(g_statSegs, cap * sizeof(void*));
            if (!g_statSegs) { g_statSegCount = 0; return; }
        }
        for (long j = 0; j < ne; j++)
        {
            void* seg = *(void**)((BYTE*)tb + j * 0x60 + 0x20);
            if (seg) g_statSegs[g_statSegCount++] = seg;
        }
    }

    if (g_statSegCount > 1)
        qsort(g_statSegs, g_statSegCount, sizeof(void*), PtrCmp);

    static int lastLogged = -1;
    if (g_statSegCount != lastLogged)
    {
        lastLogged = g_statSegCount;
        Logf("railphysics  stations: %d track segments in station zones", g_statSegCount);
    }
}

static int SegmentIsStation(void* seg)
{
    if (!g_statSegs || g_statSegCount <= 0) return 0;
    return bsearch(&seg, g_statSegs, g_statSegCount, sizeof(void*), PtrCmp) != NULL;
}

// ---------------------------------------------------------------- curves
//
// The game has no curve speed limits at all; we compute them from the route
// geometry. A train's route is a vector of track segments in travel order
// (V_ROUTE_SEGS), each segment a polyline of (x,y,z,cumdist) points between
// two junction nodes. Walking it forward, curvature per polyline interval is
// |delta yaw| / interval length, and a radius becomes a speed through the
// lateral-acceleration limit: v = sqrt(a_lat * R). The limit applied now is
// the ETCS-style braking curve over every interval ahead:
//
//     v_allow(i) = sqrt(v_c(i)^2 + 2 * a_brake * dist(i))
//
// so the train starts braking exactly as far before the curve as its own
// service brake needs. Recomputed per train at most every ~0.75 s and cached;
// the per-frame hook helper is a cache lookup.

#define CURVE_CACHE_N  64
#define CURVE_RECALC_MS 750

struct WalkPt { float x, z; float s; };
static WalkPt* g_pts;
static int     g_ptsCap;

struct CurveSlot
{
    void* veh;
    float limit;        // km/h, FLT_MAX = no constraint
    float minR;         // for the log
    float lastLogged;   // last limit written to the log
    DWORD tick;
};

static CurveSlot g_curveCache[CURVE_CACHE_N];
static int       g_curveNext;

// One route leg point source: node endpoints wrap the polyline vector.
// Returns FLT_MAX when the walk cannot be done (no route, stale pointers).
static float ComputeCurveLimit(void* veh, float* outMinR)
{
    BYTE* v   = (BYTE*)veh;
    BYTE* cur = *(BYTE**)(v + V_CUR_SEG);
    if (!cur) return FLT_MAX;

    BYTE** segs = *(BYTE***)(v + V_ROUTE_SEGS);
    BYTE** se   = *(BYTE***)(v + V_ROUTE_SEGS + 8);
    long   n    = se - segs;
    if (!segs || n <= 0 || n > 4096 || !ReadablePtr(segs, n * 8)) return FLT_MAX;

    int k = -1;
    // The route index (+0x700) is authoritative: the route node vector
    // (+0x6A0) is index-parallel to the segment vector (+0x6B8), and a route
    // may pass the same segment twice - a first-occurrence pointer match can
    // then start the walk at the wrong visit and the sampler sees the path
    // double back on itself (a fake 180-degree "curve"). Pointer match stays
    // as a fallback only.
    int ridx = *(int*)(v + 0x700);
    if (ridx >= 0 && ridx < n && segs[ridx] == cur)
        k = ridx;
    else
        for (long i = 0; i < n; i++)
            if (segs[i] == cur) { k = (int)i; break; }
    if (k < 0) return FLT_MAX;

    float pos   = *(float*)(v + V_POS);
    BYTE  dir   = *(BYTE*)(v + V_DIR);
    float curLen = *(float*)(cur + S_LEN);
    float dStart = dir ? curLen - pos : pos;        // our distance from leg entry

    double aBudget = (g.serviceBrake > 0.05f ? g.serviceBrake : 0.05f) /
                     (g.curveMargin > 1.0f ? g.curveMargin : 1.0f);
    double vSt     = g.stationLimit / 3.6;          // station zone speed, m/s

    // Adaptive horizon: at 270 km/h the braking distance alone is over 4 km,
    // so a fixed lookahead would discover the curve only when it is already
    // too late (that is exactly the overshoot this code exists to prevent).
    // Scan at least the full-stop distance from the current speed.
    float speedKmh = *(float*)(v + V_SPEED);
    if (!isfinite(speedKmh) || speedKmh < 0.0f) speedKmh = 0.0f;
    double vMs     = speedKmh / 3.6;
    double horizon = g.curveLookahead;
    double need    = vMs * vMs / (2.0 * aBudget) + 150.0;
    if (need > horizon) horizon = need;
    if (horizon > 8000.0) horizon = 8000.0;

    double best   = FLT_MAX;
    double bestR  = FLT_MAX;
    double sLeg   = 0.0;                            // metres from train to leg entry
    int    npts   = 0;

    // Smooth stop. The stop point of a scheduled stop is the END of the
    // route's last leg, and the train advertises its intent in +0xD39/+0xD3C,
    // so the distance to the stop is just the remaining route length. Brake
    // on a parabola to zero ending 25 m out; inside that window vanilla's
    // own 20 m ramp (d28 = 0.5, d24 rescale) takes over - it only fires on
    // the last leg, so the handover is clean. Bonus: this also smooths
    // customhouse and end-of-line stops, which no vanilla curve covers
    // either.
    if (g.smoothStop &&
        (*(BYTE*)(v + V_STOP_NEXT) || *(BYTE*)(v + V_STOP_MORE)))
    {
        double dist = 0.0;
        for (int i = k; i < n && dist < horizon; i++)
        {
            BYTE* sg = segs[i];
            if (!ReadablePtr(sg, S_LEN + 4)) break;
            float len = *(float*)(sg + S_LEN);
            dist += (i == k) ? (len - dStart) : len;
        }
        if (dist > 25.0 && dist < horizon)
        {
            double allowed = sqrt(2.0 * aBudget * (dist - 25.0)) * 3.6;
            if (allowed < best) { best = allowed; bestR = -2.0; }
        }
    }

    for (int i = k; i < n && sLeg < horizon; i++)
    {
        BYTE* sg = segs[i];
        if (!ReadablePtr(sg, S_LEN + 4)) break;
        float len = *(float*)(sg + S_LEN);
        if (len <= 0.0f || len > 100000.0f) break;

        BYTE* nA = *(BYTE**)(sg + S_NODE0);
        BYTE* nB = *(BYTE**)(sg + S_NODE1);
        if (!ReadablePtr(nA, 16) || !ReadablePtr(nB, 16)) break;

        // Travel orientation: the first leg's entry end comes from the dir
        // byte; every later leg enters through the node it shares with the
        // previous leg.
        BYTE* entry;
        if (i == k)
        {
            entry = dir ? nB : nA;
        }
        else
        {
            BYTE* ps = segs[i - 1];
            BYTE* pA = *(BYTE**)(ps + S_NODE0);
            BYTE* pB = *(BYTE**)(ps + S_NODE1);
            entry = (nA == pA || nA == pB) ? nA : nB;
        }
        int fromStart = (entry == nA);              // travel in +cumdist direction

        // Station zone: same braking-curve budget as curves - be at
        // station_limit by the boundary, hold it inside.
        if (g.stations && SegmentIsStation(sg))
        {
            double x = (i == k) ? 0.0 : sLeg;       // current leg: we are inside
            double allowed = sqrt(vSt * vSt + 2.0 * aBudget * x) * 3.6;
            if (allowed < best) { best = allowed; bestR = -1.0; }
        }

        BYTE* pb = *(BYTE**)(sg + S_POLY);
        BYTE* pe = *(BYTE**)(sg + S_POLY + 8);
        long  np = (pe - pb) / 0x18;
        if (np < 0 || np > 100000) break;
        if (np > 0 && !ReadablePtr(pb, np * 0x18)) break;

        // Point sequence: entry node, polyline (in travel order), exit node.
        for (long j = 0; j <= np + 1; j++)
        {
            long jj = fromStart ? j : (np + 1 - j);
            float x, y2, z;
            double cum;
            if (jj == 0)
            {
                x = *(float*)(nA + 0x04); y2 = *(float*)(nA + 0x08); z = *(float*)(nA + 0x0C);
                cum = 0.0;
            }
            else if (jj == np + 1)
            {
                x = *(float*)(nB + 0x04); y2 = *(float*)(nB + 0x08); z = *(float*)(nB + 0x0C);
                cum = len;
            }
            else
            {
                BYTE* pt = pb + (jj - 1) * 0x18;
                x = *(float*)(pt + 0x00); y2 = *(float*)(pt + 0x04); z = *(float*)(pt + 0x08);
                cum = *(float*)(pt + 0x14);
            }

            double along = fromStart ? cum : (double)len - cum;   // m from leg entry
            double sHere;
            if (i == k)
            {
                sHere = sLeg + along - dStart;
                if (sHere < -25.0) continue;      // keep ~a window behind as chord anchor
            }
            else
                sHere = sLeg + along;

            if (npts >= g_ptsCap)
            {
                int cap = g_ptsCap ? g_ptsCap * 2 : 4096;
                WalkPt* grown = (WalkPt*)realloc(g_pts, (size_t)cap * sizeof(WalkPt));
                if (!grown) break;
                g_pts = grown; g_ptsCap = cap;
            }
            g_pts[npts].x = x;
            g_pts[npts].z = z;
            g_pts[npts].s = (float)sHere;
            ++npts;
        }

        sLeg += (i == k) ? (len - dStart) : len;
    }

    // Window curvature: heading change over +/-W chords around each point.
    // A single-interval derivative cannot survive the 1-3 m jags that
    // tunnel portals and construction artifacts leave in the polyline; a
    // 20 m window absorbs them while real curves (tens of metres) stand.
    const double W = 10.0;
    int lo = 0, hi = 0;
    for (int i2 = 0; i2 < npts; i2++)
    {
        double s = g_pts[i2].s;
        if (s < 0.0) continue;
        if (s > horizon) break;
        if (hi < i2) hi = i2;
        while (lo < i2 && g_pts[lo].s < s - W) ++lo;
        while (hi < npts - 1 && g_pts[hi + 1].s < s + W) ++hi;
        if (hi <= i2 || lo >= i2) continue;
        double span = g_pts[hi].s - g_pts[lo].s;
        if (span < 5.0) continue;

        double yawL = atan2(g_pts[i2].x - g_pts[lo].x, g_pts[i2].z - g_pts[lo].z);
        double yawR = atan2(g_pts[hi].x - g_pts[i2].x, g_pts[hi].z - g_pts[i2].z);
        double dyaw = yawR - yawL;
        while (dyaw > 3.14159265)  dyaw -= 2 * 3.14159265;
        while (dyaw < -3.14159265) dyaw += 2 * 3.14159265;
        double adyaw = fabs(dyaw);
        if (adyaw > 1.0) continue;      // reversal / broken join, not a curve
        if (adyaw < 0.004) continue;    // ~0.2 deg over the window: straight

        double R = span / adyaw;
        if (R < 10.0) R = 10.0;
        double vc = sqrt((double)g.curveLat * R);
        double allowed = sqrt(vc * vc + 2.0 * aBudget * s) * 3.6;
        if (allowed < best) { best = allowed; bestR = R; }
    }

    if (outMinR) *outMinR = (float)bestR;
    return (float)best;
}

static float CurveLimitFor(void* veh)
{
    DWORD now = GetTickCount();
    EnterCriticalSection(&g_lock);

    if (g.stations) RescanStations();

    CurveSlot* slot = NULL;
    for (int i = 0; i < CURVE_CACHE_N; i++)
        if (g_curveCache[i].veh == veh) { slot = &g_curveCache[i]; break; }

    if (slot && (DWORD)(now - slot->tick) < CURVE_RECALC_MS)
    {
        float r = slot->limit;
        LeaveCriticalSection(&g_lock);
        return r;
    }

    if (!slot)
    {
        slot = &g_curveCache[g_curveNext];
        g_curveNext = (g_curveNext + 1) % CURVE_CACHE_N;
        slot->lastLogged = -1.0f;
    }

    float minR = FLT_MAX;
    float lim  = ComputeCurveLimit(veh, &minR);
    slot->veh   = veh;
    slot->limit = lim;
    slot->minR  = minR;
    slot->tick  = now;

    if (g.logCurves && lim < FLT_MAX &&
        (slot->lastLogged < 0.0f || fabsf(lim - slot->lastLogged) > 2.0f))
    {
        slot->lastLogged = lim;
        float speed = *(float*)((BYTE*)veh + V_SPEED);
        if (minR < -1.5f)
            Logf("railphysics  curve: v=%.0f km/h limit=%.0f km/h (stop ahead)", speed, lim);
        else if (minR < 0.0f)
            Logf("railphysics  curve: v=%.0f km/h limit=%.0f km/h (station zone)", speed, lim);
        else
            Logf("railphysics  curve: v=%.0f km/h limit=%.0f km/h R=%.0f m", speed, lim, minR);
    }

    LeaveCriticalSection(&g_lock);
    return lim;
}

// Called from the site-C stub with XMM9 (the final vanilla limit) in xmm1.
extern "C" float rp_curve_helper(void* veh, float limit)
{
    if (!g.curves) return limit;
    float cl = CurveLimitFor(veh);
    return cl < limit ? cl : limit;
}


// ---------------------------------------------------------------- patching

// A call-site rewrite: verify the `e8 rel32` calls what we think, then point
// it at a near cave holding `mov rax, fn / jmp rax`. The original function is
// untouched and stays callable by absolute address.
static bool PatchCallSite(DWORD siteRva, DWORD originalRva, void* fn, const char* label)
{
    BYTE* site = g_exeBase + siteRva;
    BYTE* orig = g_exeBase + originalRva;

    if (site[0] != 0xE8 || (BYTE*)site + 5 + *(INT32*)(site + 1) != orig)
    {
        Logf("railphysics  %s: call site at %08lX does not match 1.1.1.7, skipped", label, siteRva);
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
// sign, and flat (s == 0) still skips on its own.
static bool PatchSlopeBranch(void)
{
    BYTE* site = g_exeBase + RVA_SLOPE_BRANCH;
    if (site[0] != 0x76 || site[1] != 0x41)
    {
        Logf("railphysics  slope branch at %08X does not match 1.1.1.7, skipped", RVA_SLOPE_BRANCH);
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
    info->version = "1.0";

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
    g.stationLimit    = CfgFloat(ini, "station_limit_kmh", 30.0f);
    g.curveLat        = CfgFloat(ini, "curve_lateral_ms2", 0.9f);
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
    g.brakeFloorRatio = CfgFloat(ini, "brake_min_vanilla_ratio", 1.0f);
    g.idleLoad        = CfgFloat(ini, "idle_load", 0.05f);
    g.loadMax         = CfgFloat(ini, "fuel_load_max", 1.0f);
    g.electricLoadScale = CfgFloat(ini, "electric_load_scale", 1.0f);
    g.gridBoost       = CfgFloat(ini, "grid_boost", 1.0f);
    g.logPhysics      = H->configInt(ini, "railphysics", "log_physics", 0);

    g_TotalMass   = (t_TotalMass)(g_exeBase + RVA_TOTAL_MASS);
    g_TotalPower  = (t_TotalPower)(g_exeBase + RVA_TOTAL_POWER);
    g_OrigDivisor = (t_DivisorFn)(g_exeBase + RVA_DIVISOR_FN);
    g_OrigFuel    = (t_FuelFn)(g_exeBase + RVA_FUEL_FN);

    rp_brake_back   = g_exeBase + RVA_BRAKE_BACK;
    rp_slope_back   = g_exeBase + RVA_SLOPE_BACK;
    rp_curve_back   = g_exeBase + RVA_CURVE_BACK;
    rp_curve_branch = g_exeBase + RVA_CURVE_BRANCH;
    rp_grid_back    = g_exeBase + RVA_GRID_BACK;
    rp_grid_k       = g.gridBoost;

    int patched = 0;

    if (g.accel)
        patched += PatchCallSite(RVA_CALL_DIVISOR, RVA_DIVISOR_FN, (void*)RailDivisor, "accel/divisor");

    if (g.fuel)
    {
        patched += PatchCallSite(RVA_CALL_FUEL_1, RVA_FUEL_FN, (void*)RailFuel, "fuel (cruise)");
        patched += PatchCallSite(RVA_CALL_FUEL_2, RVA_FUEL_FN, (void*)RailFuel, "fuel (station)");
    }

    if (g.brake)
    {
        static const BYTE expect[16] =
        {
            0xF3, 0x0F, 0x5C, 0xF0,                         // subss xmm6, xmm0
            0xF3, 0x0F, 0x11, 0xB6, 0x24, 0x0D, 0x00, 0x00, // movss [rsi+0xd24], xmm6
            0x44, 0x0F, 0x2F, 0xCE,                         // comiss xmm9, xmm6
        };
        void* tramp = NULL;
        if (InstallInlineHook(g_exeBase + RVA_BRAKE_SITE, (void*)rp_brake_stub, &tramp,
                              expect, sizeof(expect), "rail brake"))
            ++patched;
    }

    if (g.slope)
    {
        static const BYTE expect[21] =
        {
            0xF3, 0x41, 0x0F, 0x59, 0xC2,                   // mulss xmm0, xmm10
            0xF3, 0x0F, 0x58, 0x86, 0x24, 0x0D, 0x00, 0x00, // addss xmm0, [rsi+0xd24]
            0xF3, 0x0F, 0x11, 0x86, 0x24, 0x0D, 0x00, 0x00, // movss [rsi+0xd24], xmm0
        };
        void* tramp = NULL;
        if (InstallInlineHook(g_exeBase + RVA_SLOPE_SITE, (void*)rp_slope_stub, &tramp,
                              expect, sizeof(expect), "rail slope"))
            ++patched;                                   // live even if the branch patch fails
        patched += PatchSlopeBranch();                   // (uphill-only degradation, logged)
    }

    if (g.curves)
    {
        static const BYTE expect[17] =
        {
            0x44, 0x38, 0xAE, 0x41, 0x0D, 0x00, 0x00,       // cmp [rsi+0xd41], r13b
            0x74, 0x51,                                     // jz 0x1406a8380
            0xF3, 0x0F, 0x10, 0xB6, 0x24, 0x0D, 0x00, 0x00, // movss xmm6, [rsi+0xd24]
        };
        void* tramp = NULL;
        if (InstallInlineHook(g_exeBase + RVA_CURVE_SITE, (void*)rp_curve_stub, &tramp,
                              expect, sizeof(expect), "rail curve limit"))
            ++patched;
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
        else
        {
            static const BYTE expect[16] =
            {
                0x48, 0x8B, 0x85, 0x70, 0x09, 0x00, 0x00,       // mov rax, [rbp+0x970]
                0xF3, 0x0F, 0x59, 0x84, 0x1F, 0x8C, 0x00, 0x00, // mulss xmm0, [rdi+rbx+0x8c]
            };
            void* tramp = NULL;
            if (InstallInlineHook(g_exeBase + RVA_GRID_SITE, (void*)rp_grid_stub, &tramp,
                                  expect, sizeof(expect), "grid boost"))
                ++patched;
        }
    }

    if (!patched)
    {
        Logf("railphysics  nothing patched, unloading");
        return 1;
    }

    Logf("railphysics  %d subsystem(s) patched", patched);
    return 0;
}
