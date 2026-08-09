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
#define V_ROUTE_OBJS      0x680       // ptr begin (end at +8): route objects (stop block B gate, findings/04 Q4)
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
    int   customStop;
    float customsEntry;     // km/h target at the customs zone entry
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
static __thread BYTE* t_rpBase;
static __thread BYTE* t_rpEnd;

static int ReadableFast(const void* p, size_t n)
{
    BYTE* b = (BYTE*)p;
    if (b >= t_rpBase && b + n <= t_rpEnd) return 1;
    if (!ReadablePtr(p, n)) return 0;
    MEMORY_BASIC_INFORMATION mbi;
    if (VirtualQuery(p, &mbi, sizeof(mbi)) && mbi.State == MEM_COMMIT)
    {
        t_rpBase = (BYTE*)mbi.BaseAddress;
        t_rpEnd  = (BYTE*)mbi.BaseAddress + mbi.RegionSize;
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
    float a_ms2;            // signed net acceleration
    float accel_kmh_s;      // a_ms2 * 3.6
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
static float RailDivisor(void* veh)
{
    Physics ph;
    ComputePhysicsCached(veh, &ph);
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
    if (planned && !emergency)
    {
        // Proportional band: full service rate at >=30 km/h overshoot,
        // easing to 25% at the limit itself. The game's brake controller
        // is bang-bang around the limit, and our corridor distance falls
        // in leg-quantized steps of up to ~700 m - each step dropped the
        // limit 15-20 km/h and produced a full-rate brake yank; at 2x
        // arcade brake that is a pulse train [C: run 7, customs entry].
        // The /30 band keeps step-overshoots (15-20) at 0.25-0.65 rate,
        // so the yanks stretch and merge into one continuous application
        // [run 8: the /15 band still gave ~full rate on every step].
        // A real demand (overshoot >= 30) still gets the full rate.
        float over = speed - limit;             // km/h
        float band = over / 30.0f;
        if (band < 0.25f) band = 0.25f;
        if (band < 1.0f) rate *= band;
    }
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
// Type ids from the building ini parser's $TYPE strcmp chain
// (findings/03-stations.md Q2) [C]: 0 cargo, 1 passenger, 0x60 waiting
// station, 0x14 customhouse.
#define CHAIN_TYPE_CUSTOMHOUSE 0x14

static void** g_statSegs;
static int    g_statSegCount;
static float* g_statNodePos;        // x,y,z per station segment endpoint node
static int    g_statNodeCount;
static void** g_custSegs;           // customhouse track, for border stops
static int    g_custSegCount;
static DWORD  g_statScanTick;

struct CustConn { void* node; void* seg; }; // chain entry+0x30 node + its segment

static int __cdecl PtrCmp(const void* a, const void* b)
{
    void* x = *(void**)a;
    void* y = *(void**)b;
    return (x > y) - (x < y);
}

static int __cdecl CustConnCmp(const void* a, const void* b)
{
    void* x = ((const CustConn*)a)->node;
    void* y = ((const CustConn*)b)->node;
    return (x > y) - (x < y);
}

// Chain type id, -1 when the object is not readable.
static int ChainType(BYTE* chain)
{
    if (!ReadablePtr(chain, CHAIN_TYPE_DESC + 8)) return -1;
    BYTE* desc = *(BYTE**)(chain + CHAIN_TYPE_DESC);
    if (!ReadablePtr(desc, 0x370)) return -1;
    return *(int*)(desc + 0x360);
}

static int SegSetAdd(void*** set, int* count, int* cap, void* seg)
{
    if (*count >= *cap)
    {
        *cap = *count * 2 + 64;
        void** grown = (void**)realloc(*set, *cap * sizeof(void*));
        if (!grown) return 0;
        *set = grown;
    }
    (*set)[(*count)++] = seg;
    return 1;
}

// ------------------------------------------------- customs components
//
// The route never contains customhouse legs until the train is ~100 m out
// (findings/04-stops.md Q6 telemetry): from afar it is capped at the zone's
// boundary node. To recognize that cap early, the customs segment set is
// grouped into connected components over shared endpoint nodes (union-find;
// distinct customhouses share no track nodes, so they stay separate - and
// if two genuinely shared track, merging is still geometrically sound). A
// component's boundary nodes are its degree-1 nodes (where open track
// meets the zone); its depth is the summed segment length through the run
// [I: simple path assumed - a branched run would overestimate depth, which
// errs toward braking earlier]. Symmetric: works from either boundary.

struct CustComp { float depth; };      // one contiguous customhouse run
struct CustBnd  { void* node; int comp; };  // (node, segment) ref, build temp

static CustComp* g_custComps;
static int       g_custCompCount;
static void**    g_custSeeds;          // distinct customs-segment nodes
static int*      g_custSeedComp;       // ... and their component ids
static float*    g_custSeedPos;        // ... and world coords (x,y,z per node)
static int       g_custSeedCount;
static CustConn* g_custConn;           // customhouse chain-table nodes (+0x30)
static int*      g_custConnComp;       // ... resolved to component ids
static int       g_custConnCount;
static float     g_custMaxDepth;

static int __cdecl BndCmp(const void* a, const void* b)
{
    void* x = ((const CustBnd*)a)->node;
    void* y = ((const CustBnd*)b)->node;
    return (x > y) - (x < y);
}

static int UfFind(int* parent, int x)
{
    while (parent[x] != x) { parent[x] = parent[parent[x]]; x = parent[x]; }
    return x;
}

static void BuildCustomsComponents(void)
{
    g_custCompCount = 0;
    g_custSeedCount = 0;
    g_custMaxDepth  = 0.0f;

    int m = g_custSegCount;
    if (m <= 0) return;

    float*   len    = (float*)malloc(m * sizeof(float));
    int*     parent = (int*)malloc(m * sizeof(int));
    int*     compOf = (int*)malloc(m * sizeof(int));
    CustBnd* refs   = (CustBnd*)malloc(2 * m * sizeof(CustBnd));
    if (!len || !parent || !compOf || !refs)
    {
        free(len); free(parent); free(compOf); free(refs);
        return;
    }

    int refN = 0;
    for (int i = 0; i < m; i++)
    {
        BYTE* sg = (BYTE*)g_custSegs[i];
        parent[i] = i;
        len[i]    = 0.0f;
        if (!ReadablePtr(sg, S_LEN + 4)) continue;   // isolated, no refs
        BYTE* a = *(BYTE**)(sg + S_NODE0);
        BYTE* b = *(BYTE**)(sg + S_NODE1);
        if (!a || !b) continue;
        len[i] = *(float*)(sg + S_LEN);
        refs[refN].node = a; refs[refN].comp = i; ++refN;
        refs[refN].node = b; refs[refN].comp = i; ++refN;
    }
    if (refN > 1) qsort(refs, refN, sizeof(CustBnd), BndCmp);

    // Segments sharing an endpoint node belong to the same run.
    for (int i = 0, j; i < refN; i = j)
    {
        for (j = i + 1; j < refN && refs[j].node == refs[i].node; j++)
        {
            int ra = UfFind(parent, refs[i].comp);
            int rb = UfFind(parent, refs[j].comp);
            if (ra != rb) parent[rb] = ra;
        }
    }

    int nc = 0;
    for (int i = 0; i < m; i++)
        if (UfFind(parent, i) == i) compOf[i] = nc++;
    if (nc > 0)
        g_custComps = (CustComp*)realloc(g_custComps, nc * sizeof(CustComp));
    if (!g_custComps)
    {
        free(len); free(parent); free(compOf); free(refs);
        return;
    }
    for (int c = 0; c < nc; c++) g_custComps[c].depth = 0.0f;
    for (int i = 0; i < m; i++)
        g_custComps[compOf[UfFind(parent, i)]].depth += len[i];
    g_custCompCount = nc;
    for (int c = 0; c < nc; c++)
        if (g_custComps[c].depth > g_custMaxDepth)
            g_custMaxDepth = g_custComps[c].depth;

    // Corridor seeds: every DISTINCT node of the customs segments; the
    // whole zone is corridor distance 0. Seeding only degree-1 "boundary"
    // nodes produced ZERO seeds on the user's map (2026-08 test): 74
    // segments in exactly 37 zones = two segments per customhouse, and a
    // double-track customhouse is two PARALLEL segments sharing both
    // endpoint nodes - every node has degree 2, no boundary exists. [C]
    // Each seed carries its component id so corridor tags know WHICH
    // customhouse they measure to (37 of them crowd the border; the
    // graph-nearest is not always the route's target). The chain-table
    // nodes are resolved to components through their entry's segment.
    g_custSeeds    = (void**)realloc(g_custSeeds, (refN > 0 ? refN : 1) * sizeof(void*));
    g_custSeedComp = (int*)realloc(g_custSeedComp, (refN > 0 ? refN : 1) * sizeof(int));
    g_custSeedPos  = (float*)realloc(g_custSeedPos, (refN > 0 ? refN : 1) * 3 * sizeof(float));
    g_custConnComp = (int*)realloc(g_custConnComp,
                        (g_custConnCount > 0 ? g_custConnCount : 1) * sizeof(int));
    if (!g_custSeeds || !g_custSeedComp || !g_custSeedPos || !g_custConnComp)
    {
        free(len); free(parent); free(compOf); free(refs);
        return;
    }
    for (int i = 0, j; i < refN; i = j)
    {
        for (j = i + 1; j < refN && refs[j].node == refs[i].node; j++) {}
        BYTE* u = (BYTE*)refs[i].node;
        g_custSeeds[g_custSeedCount]    = u;
        g_custSeedComp[g_custSeedCount] = compOf[UfFind(parent, refs[i].comp)];
        // Coords for the spatial bridge: the route's terminal node is a
        // MAINLINE node, but it sits at the same world position as an
        // island node - the match is by position, not pointer (probe T/U
        // verdict, findings Q6). FLT_MAX = unreadable, never matches.
        float* pp = &g_custSeedPos[g_custSeedCount * 3];
        if (ReadablePtr(u, 0x10))
        {
            pp[0] = *(float*)(u + 0x04);
            pp[1] = *(float*)(u + 0x08);
            pp[2] = *(float*)(u + 0x0C);
        }
        else
            pp[0] = pp[1] = pp[2] = FLT_MAX;
        ++g_custSeedCount;
    }
    int skipNoSeg = 0, skipMiss = 0;
    for (int i = 0; i < g_custConnCount; i++)
    {
        void* seg = g_custConn[i].seg;
        g_custConnComp[i] = -1;
        if (!seg) { ++skipNoSeg; continue; }    // chain with no segments
        void** hit = (void**)bsearch(&seg, g_custSegs, g_custSegCount,
                                     sizeof(void*), PtrCmp);
        if (hit)
            g_custConnComp[i] = compOf[UfFind(parent, (int)(hit - g_custSegs))];
        else
            ++skipMiss;                         // repSeg not in g_custSegs
    }


    // Coverage audit (load-time, while corridors are being debugged): per
    // zone, segment count and resolved conn seeds. 2026-08 telemetry: 35
    // conn seeds for 37 zones - unseeded zones got their mainline tagged
    // by the NEIGHBOUR's corridor, a constant overshoot equal to the
    // inter-customhouse distance, all the way to the fence. [C]
    static int lastAuditComp = -1, lastAuditConn = -1;
    if (g_custCompCount != lastAuditComp || g_custConnCount != lastAuditConn)
    {
        lastAuditComp = g_custCompCount;
        lastAuditConn = g_custConnCount;
        int* zsegs = (int*)calloc((size_t)nc, sizeof(int));
        int* zconn = (int*)calloc((size_t)nc, sizeof(int));
        if (zsegs && zconn)
        {
            for (int i = 0; i < m; i++)
                ++zsegs[compOf[UfFind(parent, i)]];
            for (int i = 0; i < g_custConnCount; i++)
                if (g_custConnComp[i] >= 0)
                    ++zconn[g_custConnComp[i]];
            for (int c = 0; c < nc; c++)
                Logf("railphysics  customs: zone %d: %d segs, %d conn seeds%s",
                     c, zsegs[c], zconn[c],
                     zconn[c] ? "" : "  <-- NO MAINLINE SEED");
        }
        free(zsegs); free(zconn);
        if (skipNoSeg || skipMiss)
            Logf("railphysics  customs: conn resolution: %d no-segment chain, %d segment not in set",
                 skipNoSeg, skipMiss);
    }

    free(len); free(parent); free(compOf); free(refs);
}

static unsigned CurveHash(void* p)
{
    UINT_PTR x = (UINT_PTR)p >> 4;              // game objects are 16-aligned
    x *= 0x9E3779B97F4A7C15ULL;
    return (unsigned)(x >> 32);
}

// ------------------------------------------------- customs corridors
//
// The route array never reaches the customs from afar (findings/04-stops.md
// Q6 telemetry: capped kilometres short on ordinary nodes, customhouse legs
// enter it only ~115 m out), so no route-content detector can brake early.
// Corridors are the way out: static world geometry, built at sweep time -
// a multi-source Dijkstra over the track graph (node+0x28/+0x30 = attached
// segments [C], findings/02-route-and-curves.md Q1), up to CORRIDOR_MAX_M
// out, seeded at distance 0 from every customs-related node we can get:
// the customs segments' endpoint nodes AND the chain table's +0x30 nodes.
// (2026-08 tests [C]: the endpoint nodes are chain-local islands - exactly
// 1 attached segment each, the walk cannot escape onto the mainline
// through them; degree-1 boundary seeding is empty on double-track
// customhouses. The +0x30 entry node is the mainline-bridge candidate.)
// Every reached segment is tagged with the graph distance from the nearest
// customs track to each of its endpoints (FLT_MAX = beyond reach). The
// scan then only needs the route legs' tags: descending in travel
// direction = heading for the border.

#define N_SEGS              0x28    // node: {begin,end} attached segment ptrs [C]
#define CORRIDOR_MAX_M      8000.0f // corridor reach; 330->50 km/h needs ~6.4 km
#define CORRIDOR_MAX_NODES  65536   // sanity cap on one build

struct CorrSeg { void* seg; float d0, d1; int c0, c1; };  // per-end dist + component id
static CorrSeg* g_corrSegs;
static int      g_corrSegCount;
static float    g_corrMaxLen;

static int __cdecl CorrSegCmp(const void* a, const void* b)
{
    void* x = ((const CorrSeg*)a)->seg;
    void* y = ((const CorrSeg*)b)->seg;
    return (x > y) - (x < y);
}

// Open-addressed insert-only map for the build. Nodes use d0 = best
// distance, c0 = source component; segments use d0/d1 + c0/c1 per endpoint.
struct CorrMap { void* key; float d0, d1; int c0, c1; };

// Lookup-or-insert; grows at 70 % load. Use the returned slot immediately -
// the next insert may rehash. NULL on allocation failure.
static CorrMap* CorrMapSlot(CorrMap** pm, int* pcap, int* pcount, void* key)
{
    if (*pcount >= *pcap * 7 / 10)
    {
        int ncap = *pcap ? *pcap * 2 : 1024;
        if (ncap > CORRIDOR_MAX_NODES * 4) return NULL;
        CorrMap* nm = (CorrMap*)calloc((size_t)ncap, sizeof(CorrMap));
        if (!nm) return NULL;
        for (int i = 0; i < *pcap; i++)
            if ((*pm)[i].key)
            {
                unsigned h = CurveHash((*pm)[i].key) & (ncap - 1);
                while (nm[h].key) h = (h + 1) & (ncap - 1);
                nm[h] = (*pm)[i];
            }
        free(*pm);
        *pm = nm; *pcap = ncap;
    }
    unsigned h = CurveHash(key) & (*pcap - 1);
    while ((*pm)[h].key && (*pm)[h].key != key) h = (h + 1) & (*pcap - 1);
    if (!(*pm)[h].key)
    {
        (*pm)[h].key = key;
        (*pm)[h].d0  = FLT_MAX;
        (*pm)[h].d1  = FLT_MAX;
        (*pm)[h].c0  = -1;
        (*pm)[h].c1  = -1;
        ++*pcount;
    }
    return &(*pm)[h];
}

struct CorrHeap { float dist; void* node; };

static int CorrPush(CorrHeap** ph, int* pcap, int* pn, float d, void* node)
{
    if (*pn >= *pcap)
    {
        int cap = *pcap ? *pcap * 2 : 1024;
        CorrHeap* grown = (CorrHeap*)realloc(*ph, (size_t)cap * sizeof(CorrHeap));
        if (!grown) return 0;
        *ph = grown; *pcap = cap;
    }
    int i = (*pn)++;
    while (i > 0)
    {
        int p = (i - 1) / 2;
        if ((*ph)[p].dist <= d) break;
        (*ph)[i] = (*ph)[p];
        i = p;
    }
    (*ph)[i].dist = d; (*ph)[i].node = node;
    return 1;
}

static int CorrPop(CorrHeap* h, int* pn, float* d, void** node)
{
    if (*pn <= 0) return 0;
    *d = h[0].dist; *node = h[0].node;
    CorrHeap last = h[--*pn];
    int i = 0;
    for (;;)
    {
        int c = i * 2 + 1;
        if (c >= *pn) break;
        if (c + 1 < *pn && h[c + 1].dist < h[c].dist) ++c;
        if (h[c].dist >= last.dist) break;
        h[i] = h[c];
        i = c;
    }
    if (*pn > 0) h[i] = last;
    return 1;
}

// Hex dump of a game structure, one log line per 16 bytes - for locating
// a vector by eye when a documented offset reads as empty (the node's
// coords at +0x04/+0x08/+0x0C are known-good anchors to sanity-check).
static void LogHex(const char* what, const void* p, int n)
{
    for (int off = 0; off < n; off += 16)
    {
        char buf[16 * 3 + 1];
        int  w = 0;
        int  c = n - off < 16 ? n - off : 16;
        for (int i = 0; i < c; i++)
            w += sprintf(buf + w, "%02x ", ((const BYTE*)p)[off + i]);
        Logf("railphysics  customs: %s +0x%02x: %s", what, off, buf);
    }
}

static void BuildCorridors(void)
{
    g_corrSegCount = 0;
    g_corrMaxLen   = 0.0f;

    // Build diagnostics, always-on at load while this feature is being
    // debugged: per seed (first 5 of each set) the raw adjacency pair, the
    // computed segment count and the guard verdict; after the walk, the
    // totals and why it stopped. Logged when the seed sets change, not
    // every sweep. "zone" seeds are customs-segment endpoint nodes
    // (chain-local islands: 1 attached segment each [C, 2026-08 test]);
    // "conn" seeds are the chain-table +0x30 nodes - the mainline-bridge
    // candidates.
    static int lastDiagSeeds = -1, lastDiagConn = -1, lastDiagCorr = -1;
    int diag = (g_custSeedCount != lastDiagSeeds || g_custConnCount != lastDiagConn);
    if (diag)
    {
        lastDiagSeeds = g_custSeedCount;
        lastDiagConn  = g_custConnCount;
        for (int set = 0; set < 2; set++)
        {
            int count = set ? g_custConnCount : g_custSeedCount;
            int show  = count < 5 ? count : 5;
            for (int i = 0; i < show; i++)
            {
                BYTE* u = (BYTE*)(set ? g_custConn[i].node : g_custSeeds[i]);
                int   rd = ReadablePtr(u, N_SEGS + 8);
                void* rb = rd ? *(void**)(u + N_SEGS) : NULL;
                void* re = rd ? *(void**)(u + N_SEGS + 8) : NULL;
                long  ns = ((BYTE*)re - (BYTE*)rb) / 8;
                const char* why = !rd        ? "guard reject" :
                                  ns <= 0    ? "empty" :
                                  ns > 64    ? "cap reject" : "ok";
                Logf("railphysics  customs: %s seed[%d] %p: +28=%p +30=%p -> %ld segs (%s)",
                     set ? "conn" : "zone", i, (void*)u, rb, re, ns, why);
            }
        }
    }

    int seedTotal = g_custSeedCount + g_custConnCount;
    if (seedTotal <= 0)
    {
        if (diag)
            Logf("railphysics  customs: corridor build: 0 seeds (no customs nodes collected)");
        return;
    }

    CorrMap*  nodes = NULL; int nodeCap = 0, nodeCount = 0;
    CorrMap*  segs  = NULL; int segCap = 0, segCount = 0;
    CorrHeap* heap  = NULL; int heapN = 0, heapCap = 0;
    const char* stop = "complete";

    for (int set = 0; set < 2; set++)
    {
        int count = set ? g_custConnCount : g_custSeedCount;
        for (int i = 0; i < count; i++)
        {
            void* seed = set ? g_custConn[i].node : g_custSeeds[i];
            int   comp = set ? g_custConnComp[i] : g_custSeedComp[i];
            if (comp < 0) continue;             // unresolved component
            CorrMap* s = CorrMapSlot(&nodes, &nodeCap, &nodeCount, seed);
            if (!s || !CorrPush(&heap, &heapCap, &heapN, 0.0f, seed))
            { stop = "alloc/cap"; goto done; }
            s->d0 = 0.0f;
            s->c0 = comp;                       // tags remember their source
        }
    }

    for (;;)
    {
        float du; void* u;
        if (!CorrPop(heap, &heapN, &du, &u)) break;
        CorrMap* un = CorrMapSlot(&nodes, &nodeCap, &nodeCount, u);
        if (!un) { stop = "alloc/cap"; goto done; }
        if (du > un->d0) continue;              // stale heap entry
        if (!ReadablePtr(u, N_SEGS + 8)) continue;
        void** sb = *(void***)((BYTE*)u + N_SEGS);
        void** se = *(void***)((BYTE*)u + N_SEGS + 8);
        long  ns  = se - sb;
        if (ns <= 0 || ns > 64 || !ReadablePtr(sb, ns * 8)) continue;
        for (long j = 0; j < ns; j++)
        {
            BYTE* sg = (BYTE*)sb[j];
            if (!ReadablePtr(sg, S_LEN + 4)) continue;
            float len = *(float*)(sg + S_LEN);
            if (len <= 0.0f || len > 100000.0f) continue;
            BYTE* a = *(BYTE**)(sg + S_NODE0);
            BYTE* b = *(BYTE**)(sg + S_NODE1);
            if (!a || !b) continue;

            CorrMap* st = CorrMapSlot(&segs, &segCap, &segCount, sg);
            if (!st) { stop = "alloc/cap"; goto done; }
            if ((BYTE*)u == a && du < st->d0) { st->d0 = du; st->c0 = un->c0; }
            if ((BYTE*)u == b && du < st->d1) { st->d1 = du; st->c1 = un->c0; }

            float cand = du + len;
            if (cand > CORRIDOR_MAX_M) continue;
            CorrMap* vn = CorrMapSlot(&nodes, &nodeCap, &nodeCount,
                                      (BYTE*)u == a ? b : a);
            if (!vn) { stop = "alloc/cap"; goto done; }
            if (cand < vn->d0)
            {
                vn->d0 = cand;
                vn->c0 = un->c0;
                if (!CorrPush(&heap, &heapCap, &heapN, cand, (BYTE*)u == a ? b : a))
                { stop = "alloc/cap"; goto done; }
            }
        }
    }

done:
    if (segCount > 0)
    {
        CorrSeg* out = (CorrSeg*)realloc(g_corrSegs, segCount * sizeof(CorrSeg));
        if (out)
        {
            g_corrSegs = out;
            int n = 0;
            for (int i = 0; i < segCap; i++)
                if (segs[i].key)
                {
                    g_corrSegs[n].seg = segs[i].key;
                    g_corrSegs[n].d0  = segs[i].d0;
                    g_corrSegs[n].d1  = segs[i].d1;
                    g_corrSegs[n].c0  = segs[i].c0;
                    g_corrSegs[n].c1  = segs[i].c1;
                    float best = segs[i].d0 < segs[i].d1 ? segs[i].d0 : segs[i].d1;
                    if (best > g_corrMaxLen) g_corrMaxLen = best;
                    ++n;
                }
            g_corrSegCount = n;
            if (n > 1) qsort(g_corrSegs, n, sizeof(CorrSeg), CorrSegCmp);
        }
    }
    free(nodes); free(segs); free(heap);

    if (diag || g_corrSegCount != lastDiagCorr)
    {
        lastDiagCorr = g_corrSegCount;
        Logf("railphysics  customs: corridor build: %d zone + %d conn seeds, %d nodes visited, %d segs tagged (%s)",
             g_custSeedCount, g_custConnCount, nodeCount, g_corrSegCount, stop);
    }
}

// Corridor distances at the segment's +0x20/+0x28 ends and the component
// each end measures to; FLT_MAX = beyond reach (not in any corridor, or
// that end past the 8 km fringe).
static void CorridorDist(void* seg, float* d0, float* d1, int* c0, int* c1)
{
    *d0 = *d1 = FLT_MAX;
    *c0 = *c1 = -1;
    if (!g_corrSegs || g_corrSegCount <= 0) return;
    CorrSeg key;
    key.seg = seg;
    CorrSeg* hit = (CorrSeg*)bsearch(&key, g_corrSegs, g_corrSegCount,
                                     sizeof(CorrSeg), CorrSegCmp);
    if (hit) { *d0 = hit->d0; *d1 = hit->d1; *c0 = hit->c0; *c1 = hit->c1; }
}

// Any customhouse chain in the train's route-object list? (probe U,
// findings Q6: type 0x14 entries appear at assignment time, kilometres out)
static int RouteObjHasCustomhouse(BYTE* v)
{
    BYTE** ob = *(BYTE***)(v + V_ROUTE_OBJS);
    BYTE** oe = *(BYTE***)(v + V_ROUTE_OBJS + 8);
    long  on  = oe - ob;
    if (!ob || on <= 0 || on > 64 || !ReadableFast(ob, on * 8)) return 0;
    for (long j = 0; j < on; j++)
    {
        BYTE* obj = ob[j];
        if (!ReadableFast(obj, CHAIN_TYPE_DESC + 8)) continue;
        BYTE* desc = *(BYTE**)(obj + CHAIN_TYPE_DESC);
        if (ReadableFast(desc, 0x370) &&
            *(int*)(desc + 0x360) == CHAIN_TYPE_CUSTOMHOUSE)
            return 1;
    }
    return 0;
}

static void CollectStationNodes(void);

static void RescanStations(void)
{
    DWORD now = GetTickCount();
    // The sweep walks the whole global chain vector with two guard checks
    // per chain - a periodic hitch on big maps. 30 s: stations/customhouses
    // appear rarely, and a newly built one just starts applying its zone up
    // to half a minute late. There is no event that forces an early rescan.
    if ((DWORD)(now - g_statScanTick) < 30000) return;
    g_statScanTick = now;

    g_statSegCount = 0;
    g_custSegCount = 0;
    g_custConnCount = 0;

    BYTE*** pvec = (BYTE***)(g_exeBase + RVA_CHAIN_VECTOR);
    BYTE**  vb   = *pvec;
    BYTE**  ve   = *(BYTE***)(g_exeBase + RVA_CHAIN_VECTOR + 8);
    long    nc   = ((BYTE*)ve - (BYTE*)vb) / 8;
    if (!vb || nc <= 0 || nc > 100000 || !ReadablePtr(vb, nc * 8)) return;

    int statCap = 0, custCap = 0, connCap = 0;
    static int diagChainsLeft = 3;      // load-time structure diagnostics
    static int diagRawDone    = 0;
    for (long i = 0; i < nc; i++)
    {
        BYTE* chain = (BYTE*)vb[i];
        int   type  = ChainType(chain);
        int   station = (type == 0 || type == 1 || type == 0x60);
        int   customs = (type == CHAIN_TYPE_CUSTOMHOUSE);
        if (!station && !customs) continue;

        BYTE** tb = *(BYTE***)(chain + CHAIN_TABLE);
        BYTE** te = *(BYTE***)(chain + CHAIN_TABLE + 8);
        long  ne = ((BYTE*)te - (BYTE*)tb) / 0x60;
        if (ne <= 0 || ne > 100000) continue;
        if (!ReadablePtr(tb, ne * 0x60)) continue;

        // Representative segment for conn-node component resolution: the
        // walker notes +0xA10 entries with a NULL segment (+0x20), so the
        // entry's own segment cannot tie the node to the component - but
        // the CHAIN is the customhouse, and any of its segments resolves
        // it. (2026-08: entry-segment resolution left customhouses with
        // segment-less entries seedless; their mainline was then tagged by
        // the NEIGHBOUR's corridor - a constant inter-customhouse
        // overshoot. [C])
        void* repSeg = NULL;
        if (customs)
            for (long j = 0; j < ne; j++)
            {
                repSeg = *(void**)((BYTE*)tb + j * 0x60 + 0x20);
                if (repSeg) break;
            }

        for (long j = 0; j < ne; j++)
        {
            BYTE* entry = (BYTE*)tb + j * 0x60;
            void* seg   = *(void**)(entry + 0x20);
            if (customs)
            {
                // The table entry's second pointer (+0x30) is documented as
                // the section's node (findings/02-route-and-curves.md Q1)
                // and proved mainline-connected - the corridor bridge.
                void* cn = *(void**)(entry + 0x30);
                if (cn)
                {
                    if (g_custConnCount >= connCap)
                    {
                        connCap = g_custConnCount * 2 + 64;
                        CustConn* grown = (CustConn*)realloc(g_custConn,
                                            connCap * sizeof(CustConn));
                        if (!grown) { g_custConnCount = 0; return; }
                        g_custConn = grown;
                    }
                    g_custConn[g_custConnCount].node = cn;
                    g_custConn[g_custConnCount].seg  = repSeg;
                    ++g_custConnCount;
                }

                if (diagChainsLeft > 0 && j < 2)
                {
                    BYTE* n0 = NULL, *n1 = NULL;
                    const char* rel = "seg unreadable";
                    if (seg && ReadablePtr(seg, S_LEN + 4))
                    {
                        n0 = *(BYTE**)((BYTE*)seg + S_NODE0);
                        n1 = *(BYTE**)((BYTE*)seg + S_NODE1);
                        rel = (BYTE*)cn == n0 ? "== seg n0" :
                              (BYTE*)cn == n1 ? "== seg n1" : "other object";
                    }
                    long cnSegs = -1;               // -1 unreadable, -2 garbage
                    if (ReadablePtr(cn, N_SEGS + 8))
                    {
                        void** sb = *(void***)((BYTE*)cn + N_SEGS);
                        void** se = *(void***)((BYTE*)cn + N_SEGS + 8);
                        cnSegs = se - sb;
                        if (cnSegs < 0 || cnSegs > 64) cnSegs = -2;
                    }
                    Logf("railphysics  customs: chain %p entry[%ld]: seg=%p n0=%p n1=%p e30=%p (%s) e30segs=%ld",
                         (void*)chain, j, seg, (void*)n0, (void*)n1, cn, rel, cnSegs);
                    if (!diagRawDone)
                    {
                        diagRawDone = 1;
                        LogHex("chain entry raw", entry, 0x60);
                        if (cnSegs <= 1 && ReadablePtr(cn, 0x40))
                            LogHex("e30 node raw", cn, 0x40);
                    }
                }
            }
            if (!seg) continue;
            if (station)
            {
                if (!SegSetAdd(&g_statSegs, &g_statSegCount, &statCap, seg))
                { g_statSegCount = 0; return; }
            }
            else if (!SegSetAdd(&g_custSegs, &g_custSegCount, &custCap, seg))
            { g_custSegCount = 0; return; }
        }
        if (customs && diagChainsLeft > 0) --diagChainsLeft;
    }

    // Seed candidates deduped by node (first segment wins - same node, same
    // chain, so any of its segments resolves the same component).
    if (g_custConnCount > 1)
    {
        qsort(g_custConn, g_custConnCount, sizeof(CustConn), CustConnCmp);
        int w = 0;
        for (int r = 0; r < g_custConnCount; r++)
            if (w == 0 || g_custConn[r].node != g_custConn[w - 1].node)
                g_custConn[w++] = g_custConn[r];
        g_custConnCount = w;
    }

    if (g_statSegCount > 1)
        qsort(g_statSegs, g_statSegCount, sizeof(void*), PtrCmp);
    if (g_custSegCount > 1)
        qsort(g_custSegs, g_custSegCount, sizeof(void*), PtrCmp);

    if (g.smoothStop) CollectStationNodes();
    if (g.customStop)
    {
        BuildCustomsComponents();
        BuildCorridors();
    }

    static int lastLogged = -1, lastLoggedCust = -1, lastLoggedComp = -1;
    static int lastLoggedNodes = -1;
    if (g_statSegCount != lastLogged || g_statNodeCount != lastLoggedNodes)
    {
        lastLogged      = g_statSegCount;
        lastLoggedNodes = g_statNodeCount;
        Logf("railphysics  stations: %d track segments in station zones, %d stop nodes",
             g_statSegCount, g_statNodeCount);
    }
    if (g_custSegCount != lastLoggedCust || g_custCompCount != lastLoggedComp)
    {
        lastLoggedCust = g_custSegCount;
        lastLoggedComp = g_custCompCount;
        Logf("railphysics  customs: %d segments in %d customhouse zones, max depth %.0f m, corridors %d segs / max %.0f m",
             g_custSegCount, g_custCompCount, g_custMaxDepth,
             g_corrSegCount, g_corrMaxLen);
    }
}

static int SegmentIsStation(void* seg)
{
    if (!g_statSegs || g_statSegCount <= 0) return 0;
    return bsearch(&seg, g_statSegs, g_statSegCount, sizeof(void*), PtrCmp) != NULL;
}

static int SegmentIsCustoms(void* seg)
{
    if (!g_custSegs || g_custSegCount <= 0) return 0;
    return bsearch(&seg, g_custSegs, g_custSegCount, sizeof(void*), PtrCmp) != NULL;
}

// Station stop nodes: endpoint coords of every station segment. Station
// track is chain-local exactly like the customhouses (findings Q7), so the
// route's terminal node only meets a station by position, not by pointer.
static void CollectStationNodes(void)
{
    g_statNodeCount = 0;
    int m = g_statSegCount;
    if (m <= 0) return;

    void** tmp = (void**)malloc(2 * (size_t)m * sizeof(void*));
    if (!tmp) return;
    int t = 0;
    for (int i = 0; i < m; i++)
    {
        BYTE* sg = (BYTE*)g_statSegs[i];
        if (!ReadablePtr(sg, S_NODE1 + 8)) continue;
        void* a = *(void**)(sg + S_NODE0);
        void* b = *(void**)(sg + S_NODE1);
        if (a) tmp[t++] = a;
        if (b) tmp[t++] = b;
    }
    if (t > 1) qsort(tmp, t, sizeof(void*), PtrCmp);
    g_statNodePos = (float*)realloc(g_statNodePos,
                                    (t > 0 ? t : 1) * 3 * sizeof(float));
    if (!g_statNodePos) { free(tmp); return; }
    for (int i = 0; i < t; i++)
    {
        if (i > 0 && tmp[i] == tmp[i - 1]) continue;
        BYTE* u = (BYTE*)tmp[i];
        float* pp = &g_statNodePos[g_statNodeCount * 3];
        if (ReadablePtr(u, 0x10))
        {
            pp[0] = *(float*)(u + 0x04);
            pp[1] = *(float*)(u + 0x08);
            pp[2] = *(float*)(u + 0x0C);
        }
        else
            pp[0] = pp[1] = pp[2] = FLT_MAX;    // never matches
        ++g_statNodeCount;
    }
    free(tmp);
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
// service brake needs. The result is cached per train in an open-addressed
// table: a cached limit is accepted while the scan origin (route vector,
// current leg, direction) is unchanged and the train has moved < 25 m - so
// a standing train never rescans - with the 0.75 s cadence kept as the
// bound for moving trains and a 10 s absolute bound as a safety net (a
// line reassignment can rewrite the route vector in place). The per-frame
// hook helper is a table lookup.

// Open addressing, power-of-two size, fibonacci hash on the vehicle
// pointer, bounded linear probe, home-slot eviction. The previous 64-slot
// round-robin thrashed as soon as a map had more trains than slots: every
// lookup missed and the full route scan ran per train per frame.
#define CURVE_CACHE_N       1024
#define CURVE_PROBE_MAX     32
#define CURVE_RECALC_MS     750      // rescan cadence for a moving train
#define CURVE_RESCAN_MAX_MS 10000    // absolute bound, even standing still

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
    // Scan origin the limit was computed from. A cached limit is accepted
    // only while these still match the train, so a vehicle pointer reused
    // by a different train can never inherit its predecessor's limit -
    // the key is the route data itself, not the pointer alone.
    void* routeVec;     // V_ROUTE_SEGS begin
    long  routeN;
    void* curSeg;
    int   routeIdx;
    float pos;
    BYTE  dir;
    BYTE  custSeen;     // customs diag: corridor engagement already logged
    BYTE  custGuard;    // customs diag: direction-guard rejection logged
    BYTE  custGeo;      // customs diag: geo no-match witness already logged
    BYTE  custFarLog;   // customs diag: far handover (>1 km) already logged
    BYTE  statSeen;     // station diag: geo station stop already logged
    DWORD custObjSig;   // probe U: last-logged route-object list signature
    // Decision flight-recorder state (log_decisions).
    int   decSrc;       // active limiter: 0 none 1 curve 2 station 3 stop 4 customs-stop 5 customs-entry
    int   decVia;       // customs estimate provenance: 1 corridor 2 handover 3 geo
    int   decStatGeo;   // stop-ahead came from the geo match
    int   decBraking;   // brake currently applied relative to the limit
    double decDist;     // last target distance logged
    double decRouteLeft;
    float decGapC, decGapS;
    DWORD decSnapTick;
    double corrClamp;   // monotone-clamped corridor distance, -1 = no engagement
    double corrCand;    // unconfirmed big-drop candidate, -1 = none
    double corrNoEv;    // metres decayed without descending-leg evidence
    double zeroClamp;   // monotone-clamped zero-target stop distance, -1 = none
    double zeroCand;    // unconfirmed big-drop candidate, -1 = none
    int   zeroCustoms;  // the zero target is a customhouse (vs a station stop)
    BYTE  zeroSpent;    // final-25m handover done: ignore readings until the
                        // source vanishes (platform chunk re-extension must
                        // not re-engage the stop we just handed over)
};

static CurveSlot     g_curveCache[CURVE_CACHE_N];
static unsigned long g_curveHits, g_curveScans;

// Corridor report for the customs diagnostics: dist >= 0 = engaged, the
// customs entry is that many metres ahead; seen = at least one scanned
// route leg carried a corridor tag; viaZone/viaGeo = estimate provenance
// (island-handover route geometry / terminal-node spatial match; neither =
// corridor tags, lockId = winning candidate's customhouse id). routeEnd =
// remaining route length when the route ends inside the horizon, else -1;
// gap = terminal node's distance to the nearest customs island node.
// statDist/statGap = the same spatial match against STATION island nodes
// (a station stop ahead; findings Q7).
struct CorrInfo
{
    double dist;
    int    seen, viaZone, lockId, viaGeo;
    float  gap;
    double routeEnd;
    double statDist;
    float  statGap;
    void*  term;        // terminal node when the route ends in the horizon
    double horizon;     // scan horizon used
    double zeroDist;    // applied zero-target stop distance (clamped), -1 none
    int    zeroCustoms; // the zero target is a customhouse
};

// One route leg point source: node endpoints wrap the polyline vector.
// Returns FLT_MAX when the walk cannot be done (no route, stale pointers).
// slot/traveled feed the corridor estimate's temporal monotone clamp.
static float ComputeCurveLimit(void* veh, float* outMinR, CorrInfo* outCust,
                               CurveSlot* slot, double traveled)
{
    if (outCust) { outCust->dist = -1.0; outCust->seen = 0;
                   outCust->viaZone = 0; outCust->lockId = -1;
                   outCust->viaGeo = 0; outCust->gap = FLT_MAX;
                   outCust->routeEnd = -1.0;
                   outCust->statDist = -1.0; outCust->statGap = FLT_MAX;
                   outCust->term = NULL; outCust->horizon = 0.0;
                   outCust->zeroDist = -1.0; outCust->zeroCustoms = 0; }

    BYTE* v   = (BYTE*)veh;
    BYTE* cur = *(BYTE**)(v + V_CUR_SEG);
    if (!cur) return FLT_MAX;

    BYTE** segs = *(BYTE***)(v + V_ROUTE_SEGS);
    BYTE** se   = *(BYTE***)(v + V_ROUTE_SEGS + 8);
    long   n    = se - segs;
    if (!segs || n <= 0 || n > 4096 || !ReadableFast(segs, n * 8)) return FLT_MAX;

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
    double custFar = -1.0;                          // first customs run ahead
    int    custState = 0;                           // 0 none, 1 in run, 2 closed
    int    corrOk   = 1;    // 1 ok, 0 rejected (own leg ascends), 2 handed over
    int    corrSeen = 0;    // any scanned leg carried a corridor tag
    int    corrSelf = 0;    // the current leg itself descends (any id)
    double corrMin = -1.0;  // first-zero rule: min over descending legs
    int    corrMinId = -1;  // ... and its customhouse id (diagnostics)
    double corrZoneEntry = -1.0;                    // distance to the zone edge
    int    i;                                       // main loop index, hoisted
    BYTE*  termNode = NULL;                         // far node of last leg scanned
    double zRaw = -1.0;                             // zero-target stop candidates
    int    zCustoms = 0;                            // ... and which family

    // Smooth stop: geo match only (below). The intent-bytes path
    // (+0xD39/+0xD3C -> remaining route length) was REMOVED in iteration
    // 18: on re-chunked routes the array end is NOT the stop point - it
    // ended ~430 m short on the Sacoi-Tabovina line and the parabola to
    // that phantom point crawled the train at 12-40 km/h for a kilometre
    // before evaporating [C: 2026-08-06 run 4, the "snail" report]. The
    // geo match verifies the route end by island-node coordinates and is
    // the only trustworthy stop-distance source; exotic stop types the
    // sweep does not seed fall back to vanilla's own final-25m blocks.

    for (i = k; i < n && sLeg < horizon; i++)
    {
        BYTE* sg = segs[i];
        if (!ReadableFast(sg, S_LEN + 4)) break;
        float len = *(float*)(sg + S_LEN);
        if (len <= 0.0f || len > 100000.0f) break;

        BYTE* nA = *(BYTE**)(sg + S_NODE0);
        BYTE* nB = *(BYTE**)(sg + S_NODE1);
        if (!ReadableFast(nA, 16) || !ReadableFast(nB, 16)) break;

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
        termNode = fromStart ? nB : nA;             // this leg's exit node
        int isCust = g.customStop && SegmentIsCustoms(sg);

        // Corridor targeting: FIRST-ZERO rule. Tags measure graph distance
        // to the NEAREST customs entry, but in a chain of border
        // customhouses the graph-nearest is not always the one the route
        // stops at [C: iteration 8 overshot by 1.6 km, iteration 9 by
        // 3.4 km - both targeted the NEXT customhouse along the fence].
        // Every border-crossing train stops at the FIRST customhouse along
        // its route, so the estimate is the MINIMUM over descending legs of
        // (distance to the leg's exit + exit tag): a leg tagged to the
        // near customhouse measures exactly the true distance; legs tagged
        // to a farther one always measure longer and lose the min; a leg
        // at the zone's edge has tag ~0 and pins the min exactly. Untagged
        // legs (chain-local islands) and MIXED-ID legs (Voronoi boundary
        // straddlers) are tolerated - unknown, not evidence. Ascent
        // rejection applies only to the train's OWN leg (it is then
        // physically moving away from every customhouse); mid-route
        // ascents are tolerated as boundary artifacts. The first customs
        // island leg hands the approach to the zone run by pure route
        // geometry - the island IS the true target, final authority.
        if (g.customStop && corrOk == 1)
        {
            if (isCust)
            {
                corrZoneEntry = (i == k) ? 0.0 : sLeg;
                corrOk = 2;
                // Diag for customs B (2026-08-06): the island handover
                // fires kilometres before the physical border there
                // (corrZoneEntry 4988 with the fence really ~1.4 km out).
                // Dump the segment's node coords once per engagement so
                // we can tell a mis-zoned segment from a route loop.
                if (corrZoneEntry > 1000.0 && slot && !slot->custFarLog &&
                    g.logCurves)
                {
                    slot->custFarLog = 1;
                    Logf("railphysics  customs: FAR HANDOVER seg=%p sLeg=%.0f "
                         "nA=(%.1f,%.1f,%.1f) nB=(%.1f,%.1f,%.1f)",
                         (void*)sg, corrZoneEntry,
                         *(float*)(nA + 0x04), *(float*)(nA + 0x08), *(float*)(nA + 0x0C),
                         *(float*)(nB + 0x04), *(float*)(nB + 0x08), *(float*)(nB + 0x0C));
                }
            }
            else
            {
                float d0, d1; int c0, c1;
                CorridorDist(sg, &d0, &d1, &c0, &c1);
                float entryD = fromStart ? d0 : d1;
                float exitD  = fromStart ? d1 : d0;
                int   entryC = fromStart ? c0 : c1;
                int   exitC  = fromStart ? c1 : c0;
                if (exitD < FLT_MAX || entryD < FLT_MAX) corrSeen = 1;
                if (exitD < FLT_MAX && entryD < FLT_MAX && entryC != exitC)
                {
                    // Voronoi boundary straddler: tolerate, no evidence.
                }
                else if (exitD < entryD)
                {
                    double cand = ((i == k) ? (double)(len - dStart)
                                            : sLeg + len) + exitD;
                    if (corrMin < 0.0 || cand < corrMin)
                    {
                        corrMin   = cand;
                        corrMinId = exitC;
                    }
                    if (i == k) corrSelf = 1;
                }
                else if (i == k && (exitD < FLT_MAX || entryD < FLT_MAX))
                    corrOk = 0;         // own leg rises: moving away
            }
        }

        // Station zone: same braking-curve budget as curves - be at
        // station_limit by the boundary, hold it inside. Note this only
        // sees IN-ROUTE station segments; station track is chain-local
        // like the customhouses, so pass-through zones still bite late.
        // The STOP case is covered early by the geo match below (route
        // ends at the station); early zone caps are future work.
        if (g.stations && SegmentIsStation(sg))
        {
            double x = (i == k) ? 0.0 : sLeg;       // current leg: we are inside
            double allowed = sqrt(vSt * vSt + 2.0 * aBudget * x) * 3.6;
            if (allowed < best) { best = allowed; bestR = -1.0; }
        }

        // Customs run tracking: the first contiguous run of customhouse
        // segments along the route; far = distance from the train to the
        // run's far end (extended while consecutive legs stay in the zone).
        if (custState == 0 && isCust)
            custState = 1;
        if (custState == 1)
        {
            if (isCust)
                custFar = sLeg + ((i == k) ? (double)(len - dStart) : (double)len);
            else
                custState = 2;
        }

        BYTE* pb = *(BYTE**)(sg + S_POLY);
        BYTE* pe = *(BYTE**)(sg + S_POLY + 8);
        long  np = (pe - pb) / 0x18;
        if (np < 0 || np > 100000) break;
        if (np > 0 && !ReadableFast(pb, np * 0x18)) break;

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

    // Customs (border) stops. Two complementary detectors, same parabola
    // (aBudget, 25 m handover to vanilla's late blocks):
    //
    // Zone run: customhouse segments that ARE in the route. Telemetry
    // (findings/04-stops.md Q6) shows that happens only ~115 m out, so this
    // covers the final approach after the route extends into the zone; the
    // stop point is the run's far end (block A stops at a segment END [I:
    // the run's last segment coincides with the extended route's last
    // leg]). After clearance the train stands inside the run: the residual
    // cap to the far end is a few dozen km/h and lifts at the boundary.
    // Fed into the zero-target clamp like every other stop distance.
    if (g.customStop && custFar >= 0.0 && custFar < horizon &&
        (zRaw < 0.0 || custFar < zRaw))
    {
        zRaw     = custFar;
        zCustoms = 1;
    }

    // Primary detector (probe U verdict, findings Q6/Q7): the route ENDS at
    // the next stop from assignment (customhouse 24 km out, station 2 km
    // out - "route left" shrinks smoothly as the train approaches), and its
    // terminal node is a MAINLINE node sitting at the same world position
    // as a chain-local island node. Pointer identity failed (H3), so match
    // by COORDS against the island node positions stored at sweep time.
    // Customs match = the route ends at a border: distance to the entry =
    // remaining route length, exact. After clearance the route extends
    // past, the terminal node sits beyond it, and the match releases by
    // itself. Station match = the route ends at a station: vanilla's stop
    // blocks all require the last route leg, so route-terminal-at-station
    // IS the stop intent (the +0xD39/+0xD3C bytes never fired on the
    // cyclic test) - same parabola as a scheduled stop.
    double geoDist  = -1.0;                 // customs boundary match
    float  geoGap   = FLT_MAX;
    double statDist = -1.0;                 // station boundary match
    float  statGap  = FLT_MAX;
    if (i == n && termNode && ReadableFast(termNode, 0x10))
    {
        float tx = *(float*)(termNode + 0x04);
        float ty = *(float*)(termNode + 0x08);
        float tz = *(float*)(termNode + 0x0C);
        // Both gaps are computed whenever the route ends in the horizon -
        // the near-miss distance above the epsilon is exactly what the
        // decision log needs. Engagement semantics unchanged: customs
        // first, station only when customs didn't match.
        if (g.customStop && g_custSeedPos)
        {
            float best2 = FLT_MAX;
            for (int q = 0; q < g_custSeedCount; q++)
            {
                const float* p = &g_custSeedPos[q * 3];
                if (p[0] == FLT_MAX) continue;
                float dx = p[0] - tx, dy = p[1] - ty, dz = p[2] - tz;
                float g2 = dx * dx + dy * dy + dz * dz;
                if (g2 < best2) best2 = g2;
            }
            if (best2 < FLT_MAX) geoGap = sqrtf(best2);
            if (geoGap <= 2.0f)             // [C: customs gap measured 0.00 m]
                geoDist = sLeg;
        }
        if (g.smoothStop && g_statNodePos)
        {
            float best2 = FLT_MAX;
            for (int q = 0; q < g_statNodeCount; q++)
            {
                const float* p = &g_statNodePos[q * 3];
                if (p[0] == FLT_MAX) continue;
                float dx = p[0] - tx, dy = p[1] - ty, dz = p[2] - tz;
                float g2 = dx * dx + dy * dy + dz * dz;
                if (g2 < best2) best2 = g2;
            }
            if (best2 < FLT_MAX) statGap = sqrtf(best2);
            if (geoDist < 0.0 && statGap <= 2.0f)
                statDist = sLeg;
        }
    }

    // Station stop, geo trigger: the remaining route length IS the stop
    // distance, with the same parabola and 25 m handover the removed
    // intent-bytes path used. Recorded into the zero-target clamp like
    // every other stop distance.
    if (g.smoothStop && statDist >= 0.0 &&
        (zRaw < 0.0 || statDist < zRaw))
    {
        zRaw     = statDist;
        zCustoms = 0;
    }

    // Priority: island handover (route contains customs legs - final
    // approach) > geo match (route ends at the boundary) > corridor tags
    // (fallback, covers what the geo epsilon might miss). All feed the
    // same parabola; the zone run owns the stop itself regardless.
    double corrDist = -1.0;
    int    corrVia  = 0;                    // 1 corridor, 2 handover, 3 geo
    if (g.customStop && corrOk == 2 && corrSeen)
    {
        corrDist = corrZoneEntry;
        corrVia  = 2;
    }
    else if (geoDist >= 0.0)
    {
        corrDist = geoDist;
        corrVia  = 3;
        if (slot) slot->corrNoEv = 0.0;
    }
    else if (g.customStop && corrOk == 1)
    {
        int engaged = slot && slot->corrClamp >= 0.0;
        if (corrMin >= 0.0 && (corrSelf || engaged))
        {
            corrDist = corrMin;
            corrVia  = 1;
            if (slot) slot->corrNoEv = 0.0;
        }
        else if (engaged)
        {
            // Engaged but no evidence in this window (an island gap):
            // decay the estimate by the travel. Hysteresis strictly within
            // an active engagement, capped at 500 m evidence-free (a
            // station island is shorter); beyond that, release.
            slot->corrNoEv += traveled;
            if (slot->corrNoEv <= 500.0)
            {
                corrDist = slot->corrClamp - traveled;
                corrVia  = 1;
            }
        }
    }

    // Sanity cap: nothing beyond the corridor's reach plus the zone itself
    // is a live estimate - belt and braces against tag noise.
    double corrCap = (double)g_corrMaxLen + g_custMaxDepth + 250.0;
    if (corrDist > corrCap) corrDist = -1.0;

    if (slot)
    {
        // Corridor clamp. The route length signal is noisy BOTH ways at
        // all times: torn reads collapse it (1312->336 [C: run 5]) and
        // legitimate re-truncation after a void drops it in leg-sized
        // steps FASTER than the travel (1106->668->516->...->49, then the
        // zone-run fired at 115 m [C: run 6]). Step size cannot separate
        // the two; PERSISTENCE can - a torn read is a one-scan blip, a
        // real correction repeats. So: the clamp decays by the travel
        // unconditionally, steady readings (within travel+25 of the last
        // value) follow, and a bigger drop becomes a CANDIDATE adopted
        // only when the next scan stays low (a torn read bounces back to
        // ~the clamp; a real correction persists - it may itself keep
        // falling in leg-sized steps). Growth is never adopted (re-invites acceleration
        // [C: findings Q8]). A fresh engagement adopts instantly. Jumps
        // to <=25 m are never believed - the fence can only be REACHED by
        // travelling there.
        if (corrDist >= 0.0)
        {
            if (slot->corrClamp < 0.0)
                slot->corrClamp = corrDist;             // fresh: adopt
            else
            {
                double prev = slot->corrClamp;
                slot->corrClamp -= traveled;            // unconditional
                if (corrDist >= prev - (traveled + 25.0))
                {
                    if (corrDist < slot->corrClamp)
                        slot->corrClamp = corrDist;     // steady: follow
                    slot->corrCand = -1.0;
                }
                else if (corrDist > 25.0 && slot->corrCand >= 0.0 &&
                         corrDist <= slot->corrCand + 50.0)
                {
                    slot->corrClamp = corrDist;         // confirmed: adopt
                    slot->corrCand  = -1.0;
                }
                else if (corrDist > 25.0)
                    slot->corrCand = corrDist;          // candidate: wait
            }
        }
        else if (slot->corrClamp >= 0.0)
        {
            // Evidence gap (void flicker): hold decayed, bounded at 500 m
            // evidence-free like the corridor's own corrNoEv rule. The
            // INSTANT reset this replaces let every torn flicker wipe the
            // clamp, after which ANY next reading adopted fresh - the
            // 1444->3464 up-churn that carried 300 km/h into the customs
            // [C: run 6].
            slot->corrNoEv += traveled;
            slot->corrClamp -= traveled;
            if (slot->corrClamp <= 25.0 || slot->corrNoEv > 500.0)
                slot->corrClamp = -1.0;
        }
        if (slot->corrClamp >= 0.0)
        {
            corrDist = slot->corrClamp;
            if (corrDist > 25.0)
            {
                double vE = g.customsEntry / 3.6;
                double allowed = sqrt(vE * vE + 2.0 * aBudget * (corrDist - 25.0)) * 3.6;
                if (allowed < best) { best = allowed; bestR = -4.0; }
            }
        }
        else
            corrDist = -1.0;
    }

    // Zero-target stop distances (customs zone run, station geo match)
    // share one clamp. The route array is RE-CHUNKED by the game near
    // stops and our scan reads it mid-rebuild: the remaining length
    // collapses (3221->425, 206->0), grows in steps (425->1185), FREEZES
    // (stuck at 2536/2740 through a whole post-signal acceleration
    // 0->138 km/h [C: run 5]), or vanishes past the horizon - all with
    // the geo match itself steady at gap 0.00. So the clamp trusts
    // PHYSICS first and readings second: it decays by the travel
    // unconditionally (a fixed stop gets closer by exactly that much,
    // readings or not - the void hold of iter 17 and the frozen pin of
    // run 5 are the same rule), and a reading may only
    //  - adopt instantly on a FRESH engagement (zeroClamp < 0);
    //  - follow within the travel +25 m per scan (steady approaches
    //    track exactly);
    //  - for a BIGGER drop: become a CANDIDATE, adopted only if the next
    //    scan stays low. Step size cannot tell a torn read from a real
    //    re-truncation (legit post-void steps run 150-450 m [C: run 6
    //    1106->668->...->49]); persistence can - a torn read bounces
    //    back to ~the clamp next scan. Jumps to <=25 m are never
    //    believed: the platform is only ever REACHED by travelling;
    //  - raise it on a FAR re-extension STEP (>50 m jump while >250 m
    //    out - the route growing back toward the truth). Close-in growth
    //    is the platform chunk wobble (37->74->107->137 [C: findings
    //    Q8]) and a frozen reading is the post-signal pin - both fail
    //    these tests and the decay simply stands.
    //  - at 25 m the stop is spent: vanilla blocks A-F own the rest, and
    //    the latch ignores readings so platform chunk growth cannot
    //    re-engage the stop we just handed over. Re-arm on the source
    //    vanishing OR on a FAR sighting (>250 m): platform chunk growth
    //    never reaches that (37-190 m observed), and without it a spent
    //    phantom (e.g. the post-clearance customs residual cap, 312-348 m)
    //    latches out the REAL station stop behind it [C: 2026-08-06 run 4,
    //    144 km/h through the zone into a platform slam].
    if (slot)
    {
        if (slot->zeroSpent && (zRaw < 0.0 || zRaw > 250.0))
            slot->zeroSpent = 0;                    // stop passed: re-arm
        if (zRaw >= 0.0 && !slot->zeroSpent)
        {
            if (slot->zeroClamp < 0.0)
                slot->zeroClamp = zRaw;             // fresh sighting: adopt
            else
            {
                double prev = slot->zeroClamp;
                slot->zeroClamp -= traveled;        // unconditional tracking
                if (zRaw >= prev - (traveled + 25.0))
                {
                    if (zRaw < slot->zeroClamp)
                        slot->zeroClamp = zRaw;     // steady step: follow
                    else if (zRaw > prev + 50.0 && prev > 250.0)
                        slot->zeroClamp = zRaw;     // far growth: re-extension
                    slot->zeroCand = -1.0;
                }
                else if (zRaw > 25.0 && slot->zeroCand >= 0.0 &&
                         zRaw <= slot->zeroCand + 50.0)
                {
                    slot->zeroClamp = zRaw;         // confirmed: adopt
                    slot->zeroCand  = -1.0;
                }
                else if (zRaw > 25.0)
                    slot->zeroCand = zRaw;          // candidate: stays low?
                // else: torn one-scan collapse, a jump to <=25 m, a frozen
                // pin, close-in growth - the decay stands.
            }
            if (slot->zeroClamp <= 25.0)
            {
                slot->zeroClamp  = -1.0;            // spent
                slot->zeroSpent  = 1;
            }
            slot->zeroCustoms = zCustoms;
        }
        else if (slot->zeroClamp >= 0.0)
        {
            slot->zeroClamp -= traveled;            // ride out the void
            if (slot->zeroClamp <= 25.0)
                slot->zeroClamp = -1.0;             // spent
        }

        if (slot->zeroClamp > 25.0)
        {
            double allowed = sqrt(2.0 * aBudget * (slot->zeroClamp - 25.0)) * 3.6;
            if (allowed < best) { best = allowed; bestR = slot->zeroCustoms ? -3.0 : -2.0; }
        }
        if (outCust)
        {
            outCust->zeroDist    = slot->zeroClamp;
            outCust->zeroCustoms = slot->zeroCustoms;
        }
    }

    if (outCust)
    {
        outCust->dist     = corrDist;
        outCust->seen     = corrSeen;
        outCust->viaZone  = (corrVia == 2);
        outCust->lockId   = corrMinId;
        outCust->viaGeo   = (corrVia == 3);
        outCust->gap      = geoGap;
        outCust->routeEnd = (i == n) ? sLeg : -1.0;
        outCust->statDist = statDist;
        outCust->statGap  = statGap;
        outCust->term     = (i == n) ? termNode : NULL;
        outCust->horizon  = horizon;
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

// ------------------------------------------------- decision flight recorder
//
// Per-train event log (log_decisions): what each limiter sees and decides,
// printed on TRANSITIONS, not per frame, plus a 1 s snapshot while a
// non-curve limiter is active. Built to narrate an approach from the log
// alone: when the geo match appeared, with what gap, which estimate drove
// the parabola, and exactly when/why the brake pulsed. Called with g_lock
// held; corr is non-NULL only on scan frames (cache hits still get brake
// flips and snapshots - that is where pulsing shows).

static const char* const g_decSrcNames[] =
    { "none", "curve", "station zone", "stop ahead", "customs stop",
      "customs entry" };

static void DecisionTick(CurveSlot* slot, BYTE* v, DWORD now,
                         const CorrInfo* corr)
{
    unsigned tag = (unsigned)((UINT_PTR)v & 0xFFFF);
    float speed = *(float*)(v + V_SPEED);
    float lim   = slot->limit;

    int src = slot->minR < -3.5f ? 5 : slot->minR < -2.5f ? 4 :
              slot->minR < -1.5f ? 3 : slot->minR <  0.0f ? 2 :
              slot->minR < FLT_MAX ? 1 : 0;

    if (corr)                                   // scan frame: transitions
    {
        slot->decVia     = corr->viaZone ? 2 : corr->viaGeo ? 3 : 1;
        slot->decStatGeo = (corr->statDist >= 0.0);
        slot->decGapC    = corr->gap;
        slot->decGapS    = corr->statGap;

        // Report the WINNING source's own distance: the zero parabola's
        // clamped distance for the stop sources, the corridor estimate for
        // customs entry. (The 2026-08 "engage customs stop dist=1719" line
        // conflated the two - the winning 117 m zone-run with the stale
        // corridor estimate.)
        double dist = corr->dist;
        if (src == 4 || src == 3)
            dist = corr->zeroDist;
        else if (dist < 0.0)
            dist = corr->statDist;

        char srcBuf[48];
        const char* srcName = g_decSrcNames[src];
        if (src == 5)
        {
            sprintf(srcBuf, "customs entry(%s)",
                    slot->decVia == 3 ? "geo" : slot->decVia == 2 ? "zone" : "raw");
            srcName = srcBuf;
        }
        else if (src == 3 && slot->decStatGeo)
        {
            sprintf(srcBuf, "stop ahead(geo)");
            srcName = srcBuf;
        }

        if (src != slot->decSrc)
        {
            if (src != 0)
                Logf("railphysics  decision T…%04x engage %s dist=%.0f limit=%.0f v=%.0f gap=%.2f term=%p routeleft=%.0f horizon=%.0f",
                     tag, srcName, dist, lim, speed,
                     src == 3 ? corr->statGap : corr->gap,
                     corr->term, corr->routeEnd, corr->horizon);
            else if (slot->decSrc != 0)
                Logf("railphysics  decision T…%04x release %s v=%.0f (routeleft=%.0f gapcust=%.2f gapstat=%.2f)",
                     tag, g_decSrcNames[slot->decSrc], speed,
                     corr->routeEnd, corr->gap, corr->statGap);
            slot->decSrc  = src;
            slot->decDist = dist;
        }
        else if (src >= 2 && dist >= 0.0 && slot->decDist > 0.0 &&
                 fabs(dist - slot->decDist) > slot->decDist * 0.1 + 1.0)
        {
            Logf("railphysics  decision T…%04x target %s dist %.0f->%.0f limit=%.0f v=%.0f",
                 tag, srcName, slot->decDist, dist, lim, speed);
            slot->decDist = dist;
        }

        if (corr->routeEnd >= 0.0)
        {
            if (slot->decRouteLeft < 0.0 ||
                fabs(corr->routeEnd - slot->decRouteLeft) >
                    50.0 + 0.1 * slot->decRouteLeft)
                Logf("railphysics  decision T…%04x routeleft=%.0f term=%p gapcust=%.2f gapstat=%.2f horizon=%.0f corr=%.0f",
                     tag, corr->routeEnd, corr->term, corr->gap,
                     corr->statGap, corr->horizon, corr->dist);
            slot->decRouteLeft = corr->routeEnd;
        }
        else
            slot->decRouteLeft = -1.0;
    }

    // Brake state vs the active limit: on every frame, cache hits included.
    // A pulsing approach shows up as repeated ON/OFF pairs.
    if (src != 0 && lim < FLT_MAX)
    {
        int braking = speed > lim + 0.3f;
        if (braking != slot->decBraking)
        {
            slot->decBraking = braking;
            Logf("railphysics  decision T…%04x brake %s v=%.0f limit=%.0f (%s)",
                 tag, braking ? "ON" : "OFF", speed, lim, g_decSrcNames[src]);
        }
        if (src >= 2 && (DWORD)(now - slot->decSnapTick) >= 1000)
        {
            slot->decSnapTick = now;
            Logf("railphysics  decision T…%04x snap v=%.0f limit=%.0f dist=%.0f src=%s gapcust=%.2f gapstat=%.2f",
                 tag, speed, lim, slot->decDist, g_decSrcNames[src],
                 slot->decGapC, slot->decGapS);
        }
    }
    else
        slot->decBraking = 0;
}

static float CurveLimitFor(void* veh)
{
    DWORD now  = GetTickCount();
    BYTE*  v   = (BYTE*)veh;

    // Scan origin, read exactly as ComputeCurveLimit reads it. Unguarded
    // like there: the hooks fire inside this train's own rail update,
    // which is actively writing these same fields.
    void* routeVec = *(void**)(v + V_ROUTE_SEGS);
    long  routeN   = *(BYTE***)(v + V_ROUTE_SEGS + 8) - (BYTE**)routeVec;
    void* curSeg   = *(void**)(v + V_CUR_SEG);
    int   routeIdx = *(int*)(v + V_ROUTE_IDX);
    float pos      = *(float*)(v + V_POS);
    BYTE  dir      = *(BYTE*)(v + V_DIR);

    EnterCriticalSection(&g_lock);

    if (g.stations || g.customStop || g.smoothStop) RescanStations();

    CurveSlot* slot = NULL;
    unsigned h = CurveHash(veh) & (CURVE_CACHE_N - 1);
    for (int i = 0; i < CURVE_PROBE_MAX; i++)
    {
        CurveSlot* s = &g_curveCache[(h + i) & (CURVE_CACHE_N - 1)];
        if (s->veh == veh || !s->veh) { slot = s; break; }
    }
    if (!slot)
    {
        // Probe chain full: evict the home slot. It stays occupied, so
        // other vehicles' probe chains through it are not broken; the
        // evicted owner just rescans and reinserts on its next frame.
        slot = &g_curveCache[h];
        slot->veh = NULL;
    }

    if (slot->veh == veh)
    {
        int   sameRoute = slot->routeVec == routeVec && slot->routeN == routeN &&
                          slot->curSeg == curSeg && slot->routeIdx == routeIdx &&
                          slot->dir == dir;
        DWORD age = now - slot->tick;
        if (sameRoute &&
            (age < CURVE_RECALC_MS ||
             (fabsf(pos - slot->pos) < 25.0f && age < CURVE_RESCAN_MAX_MS)))
        {
            float r = slot->limit;
            ++g_curveHits;
            if (g.logDecisions) DecisionTick(slot, v, now, NULL);
            LeaveCriticalSection(&g_lock);
            return r;
        }
    }
    else
    {
        // First sighting of this vehicle - OR an eviction under a crowded
        // hash. One line per slot birth: a live train reappearing here
        // means its clamp state was silently reset mid-approach.
        if (g.logDecisions)
            Logf("railphysics  decision T…%04x slot init",
                 (unsigned)((UINT_PTR)veh & 0xFFFF));
        slot->lastLogged = -1.0f;
        slot->custSeen   = 0;
        slot->custGuard  = 0;
        slot->custGeo    = 0;
        slot->custFarLog = 0;
        slot->statSeen   = 0;
        slot->custObjSig = 0;
        slot->corrClamp  = -1.0;
        slot->corrCand   = -1.0;
        slot->corrNoEv   = 0.0;
        slot->zeroClamp  = -1.0;
        slot->zeroCand   = -1.0;
        slot->zeroCustoms = 0;
        slot->zeroSpent  = 0;
        slot->decSrc       = 0;
        slot->decVia       = 1;
        slot->decStatGeo   = 0;
        slot->decBraking   = 0;
        slot->decDist      = -1.0;
        slot->decRouteLeft = -1.0;
        slot->decGapC      = FLT_MAX;
        slot->decGapS      = FLT_MAX;
        slot->decSnapTick  = 0;
    }

    // The clamp survives leg transitions (curSeg/routeIdx change
    // constantly) AND route extensions - the game grows the route in chunks
    // as the train advances, and resetting on every growth pulsed the
    // estimate mid-approach [C: iteration-8 sawtooth]. Only a SHRINKING
    // route, or a pointer swap without growth, is a real reroute - for the
    // CORRIDOR estimate. The zero clamp must NOT reset here: near stops the
    // route shrinks precisely when the game re-chunks it and our scan reads
    // a torn state (3221->425, 206->0 mid-approach [C: 2026-08-05 log]) -
    // resetting adopted the torn reading as a fresh truth. The zero clamp's
    // own hold/decay outlives both the torn reads and a real reroute,
    // which dies on its own when the source stays gone.
    if (routeN < slot->routeN ||
        (routeVec != slot->routeVec && routeN <= slot->routeN))
    {
        slot->corrClamp = -1.0;
        slot->corrCand  = -1.0;
        slot->corrNoEv  = 0.0;
    }

    // Travel since the previous scan, for the clamp's decrease bound.
    // Speed read unguarded like the other hook-context fields above.
    float  spdNow   = *(float*)(v + V_SPEED);
    double traveled = (spdNow > 0.0f ? spdNow / 3.6 : 0.0) *
                      ((double)(now - slot->tick) / 1000.0);

    CorrInfo corr;
    float minR = FLT_MAX;
    float lim  = ComputeCurveLimit(veh, &minR, &corr, slot, traveled);
    // NOTE: no corrClamp reset on corr.dist<0 here anymore - the clamp
    // holds and decays through evidence gaps inside ComputeCurveLimit;
    // an instant wipe let every torn flicker re-arm a fresh adoption
    // [C: run 6, 300 km/h through the border].
    slot->veh      = veh;
    slot->limit    = lim;
    slot->minR     = minR;
    slot->tick     = now;
    slot->routeVec = routeVec;
    slot->routeN   = routeN;
    slot->curSeg   = curSeg;
    slot->routeIdx = routeIdx;
    slot->pos      = pos;
    slot->dir      = dir;
    ++g_curveScans;

    if (g.logDecisions) DecisionTick(slot, v, now, &corr);

    // Customs diagnostics (one-shot per train per approach, log_curves).
    // Engaged: the corridor distance at engagement start - expect it
    // kilometres out, and the (customs entry) limit lines right after.
    // Seen-but-not-engaged is the direction-guard witness: corridor tags on
    // the route, but the train is moving away or the route turns off - it
    // must NOT brake. Both reset when no corridor leg is on the route.
    // Probe U (diagnostic, log_curves): stop block B's gate uses a "front
    // route obj" list at train+0x680 (findings/04 Q4) - objects with a
    // +0x5D0 master path, i.e. chains. If customhouse chains appear there
    // LONG before the route legs reach them (the legs are chunk-extended,
    // findings Q6), this list is the early detector that route content
    // never gave. One-shot per list change, fast trains only.
    if (g.logCurves)
    {
        BYTE** ob = *(BYTE***)(v + V_ROUTE_OBJS);
        BYTE** oe = *(BYTE***)(v + V_ROUTE_OBJS + 8);
        long  on  = oe - ob;
        if (spdNow > 100.0f && ob && on > 0 && on <= 64 &&
            ReadableFast(ob, on * 8))
        {
            DWORD sig = (DWORD)on;
            char  tbuf[8 * 6];
            int   tw = 0;
            tbuf[0] = 0;
            for (long j = 0; j < on; j++)
            {
                int t = -1;
                BYTE* obj = ob[j];
                if (ReadableFast(obj, CHAIN_TYPE_DESC + 8))
                {
                    BYTE* desc = *(BYTE**)(obj + CHAIN_TYPE_DESC);
                    if (ReadableFast(desc, 0x370))
                        t = *(int*)(desc + 0x360);
                }
                sig = sig * 31 + (DWORD)t;
                if (j < 8)
                    tw += sprintf(tbuf + tw, "%x%s ", t < 0 ? 0 : t,
                                  t < 0 ? "?" : (t == CHAIN_TYPE_CUSTOMHOUSE ? "!" : ""));
            }
            if (sig != slot->custObjSig)
            {
                slot->custObjSig = sig;

                // Remaining route length for correlation (capped).
                double routeLeft = -1.0;
                BYTE** segs = (BYTE**)routeVec;
                int    kk   = -1;
                if (routeIdx >= 0 && routeIdx < routeN &&
                    segs[routeIdx] == (BYTE*)curSeg)
                    kk = routeIdx;
                else
                    for (long q = 0; q < routeN; q++)
                        if (segs[q] == (BYTE*)curSeg) { kk = (int)q; break; }
                if (kk >= 0 && ReadableFast(segs, routeN * 8))
                {
                    float curLen = *(float*)((BYTE*)curSeg + S_LEN);
                    float dStart = dir ? curLen - pos : pos;
                    routeLeft = 0.0;
                    for (long q = kk; q < routeN && routeLeft < 50000.0; q++)
                    {
                        BYTE* s2 = segs[q];
                        if (!ReadableFast(s2, S_LEN + 4)) break;
                        float L = *(float*)(s2 + S_LEN);
                        routeLeft += (q == kk) ? (L - dStart) : L;
                    }
                }
                Logf("railphysics  customs: route objs: %ld [%s], route left %.0f m, v=%.0f km/h",
                     on, tbuf, routeLeft, spdNow);
            }
        }
    }

    // Customs diagnostics (one-shot per train per approach, log_curves).
    // Engaged: the entry distance and its provenance - geo (route ends at
    // the customs boundary, spatial terminal-node match), zone (island
    // handover), or raw corridor tags. The no-match witness fires when a
    // customhouse is in the route objects and the route ends in the
    // horizon but the terminal node matches no island node - it calibrates
    // the 2 m epsilon. The corridor guard witness confirms non-border
    // traffic is left alone.
    if (g.logCurves)
    {
        if (corr.dist >= 0.0)
        {
            if (!slot->custSeen)
            {
                slot->custSeen = 1;
                float speed = *(float*)((BYTE*)veh + V_SPEED);
                if (corr.viaZone)
                    Logf("railphysics  customs: corridor entry %.0f m ahead (zone), v=%.0f km/h",
                         corr.dist, speed);
                else if (corr.viaGeo)
                    Logf("railphysics  customs: route obj customhouse %.0f m ahead (geo, gap %.2f m), v=%.0f km/h",
                         corr.dist, corr.gap, speed);
                else
                    Logf("railphysics  customs: corridor entry %.0f m ahead (raw, id %d), v=%.0f km/h",
                         corr.dist, corr.lockId, speed);
            }
            slot->custGeo = 0;
        }
        else if (corr.routeEnd >= 0.0 && corr.gap < FLT_MAX &&
                 RouteObjHasCustomhouse(v))
        {
            if (!slot->custGeo)
            {
                slot->custGeo = 1;
                float speed = *(float*)((BYTE*)veh + V_SPEED);
                Logf("railphysics  customs: route end %.0f m, nearest customs node gap %.1f m (no match), v=%.0f km/h",
                     corr.routeEnd, corr.gap, speed);
            }
        }
        else if (corr.seen)
        {
            if (!slot->custGuard)
            {
                slot->custGuard = 1;
                float speed = *(float*)((BYTE*)veh + V_SPEED);
                Logf("railphysics  customs: corridor on route, not descending toward entry (no brake), v=%.0f km/h",
                     speed);
            }
        }
        else
            slot->custSeen = slot->custGuard = slot->custGeo = 0;
        if (corr.dist < 0.0)
            slot->custFarLog = 0;   // re-arm the far-handover dump
    }

    // Station-stop diagnostic (one-shot per approach, log_curves): the
    // geo-matched stop distance and the positional gap. The (stop ahead)
    // limit lines follow as for any scheduled stop.
    if (g.logCurves)
    {
        if (corr.statDist >= 0.0)
        {
            if (!slot->statSeen)
            {
                slot->statSeen = 1;
                float speed = *(float*)((BYTE*)veh + V_SPEED);
                Logf("railphysics  station stop %.0f m ahead (geo, gap %.2f m), v=%.0f km/h",
                     corr.statDist, corr.statGap, speed);
            }
        }
        else
            slot->statSeen = 0;
    }

    if (g.logCurves && lim < FLT_MAX &&
        (slot->lastLogged < 0.0f || fabsf(lim - slot->lastLogged) > 2.0f))
    {
        slot->lastLogged = lim;
        float speed = *(float*)((BYTE*)veh + V_SPEED);
        if (minR < -3.5f)
            Logf("railphysics  curve: v=%.0f km/h limit=%.0f km/h (customs entry)", speed, lim);
        else if (minR < -2.5f)
            Logf("railphysics  curve: v=%.0f km/h limit=%.0f km/h (customs stop)", speed, lim);
        else if (minR < -1.5f)
            Logf("railphysics  curve: v=%.0f km/h limit=%.0f km/h (stop ahead)", speed, lim);
        else if (minR < 0.0f)
            Logf("railphysics  curve: v=%.0f km/h limit=%.0f km/h (station zone)", speed, lim);
        else
            Logf("railphysics  curve: v=%.0f km/h limit=%.0f km/h R=%.0f m", speed, lim, minR);
    }

    // Cache effectiveness under log_physics: hits should dwarf scans once
    // the map is warm; a 1:1 ratio means the cache is not working.
    static DWORD lastCacheLog = 0;
    if (g.logPhysics && (DWORD)(now - lastCacheLog) > 30000)
    {
        lastCacheLog = now;
        Logf("railphysics  curve cache: %lu hits / %lu scans", g_curveHits, g_curveScans);
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
    g.customStop      = H->configInt(ini, "railphysics", "customstop", 1);
    g.customsEntry    = CfgFloat(ini, "customs_entry_kmh", 50.0f);
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
    g.logDecisions    = H->configInt(ini, "railphysics", "log_decisions", 0);

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
