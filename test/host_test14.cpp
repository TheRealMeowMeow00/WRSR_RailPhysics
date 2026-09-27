// Offline harness for railphysics 2.0: maps SOVIET64.exe as an image (no code
// from it runs) so TsmPluginInit resolves every signature against the real
// bytes, then drives the approach core through scenarios built at the object
// offsets the plugin reads: a moving train, a route the "game" extends in
// chunks, customhouse and station chains with chain-local track whose nodes
// coincide with the mainline's. The train's speed follows the plugin's limit
// through a simple controller; nothing here depends on wall-clock time.
#include "../plugins/railphysics/railphysics.cpp"

static unsigned long g_vq;
static const char* g_iniPath;
static int g_fails;

static void HLog(const char* fmt, ...)
{
    va_list ap; va_start(ap, fmt);
    if (getenv("RP_VERBOSE")) { vprintf(fmt, ap); printf("\n"); }
    va_end(ap);
}
static int HCfgInt(const char*, const char* sec, const char* key, int fb)
{
    return (int)GetPrivateProfileIntA(sec, key, fb, g_iniPath);
}
static int HCfgStr(const char*, const char* sec, const char* key, char* out, int n, const char* fb)
{
    GetPrivateProfileStringA(sec, key, fb, out, n, g_iniPath);
    for (char* c = out; *c; c++) if (*c == ';') { *c = 0; break; }
    Trim(out);
    return 1;
}
static int HHook(void* t, void*, void** tr, const unsigned char* expect, size_t n, const char*)
{
    if (tr) *tr = NULL;
    return memcmp(t, expect, n) == 0;
}
static unsigned char* HAllocNear(unsigned char* anchor, size_t size)
{
    SYSTEM_INFO si; GetSystemInfo(&si);
    UINT_PTR g = si.dwAllocationGranularity;
    for (UINT_PTR a = ((UINT_PTR)anchor & ~(g - 1)) - g; a > (UINT_PTR)anchor - 0x70000000ULL; a -= g)
        if (void* p = VirtualAlloc((void*)a, size, MEM_COMMIT | MEM_RESERVE, PAGE_EXECUTE_READWRITE))
            return (unsigned char*)p;
    return NULL;
}
static int HReadable(const void* p, size_t n)
{
    ++g_vq;
    MEMORY_BASIC_INFORMATION mbi;
    if (!p || !VirtualQuery(p, &mbi, sizeof(mbi))) return 0;
    if (mbi.State != MEM_COMMIT || (mbi.Protect & (PAGE_NOACCESS | PAGE_GUARD))) return 0;
    return (BYTE*)p + n <= (BYTE*)mbi.BaseAddress + mbi.RegionSize;
}

static void Check(int ok, const char* fmt, ...)
{
    va_list ap; va_start(ap, fmt);
    printf("  [%s] ", ok ? "PASS" : "FAIL");
    vprintf(fmt, ap);
    printf("\n");
    va_end(ap);
    if (!ok) ++g_fails;
}

// ---------------------------------------------------------------- world

static BYTE* Obj(size_t n) { return (BYTE*)calloc(1, n); }
static float NX(BYTE* n) { return *(float*)(n + 0x04); }
static float NZ(BYTE* n) { return *(float*)(n + 0x0C); }

static BYTE* Node(double x, double z)
{
    BYTE* n = Obj(0x80);
    *(float*)(n + 0x04) = (float)x; *(float*)(n + 0x0C) = (float)z;
    return n;
}

struct P2 { double x, z; };

// A segment from node a to node b through interior points.
static BYTE* Seg(BYTE* a, BYTE* b, const P2* pts, int np)
{
    BYTE* s = Obj(0x200);
    *(BYTE**)(s + S_NODE0) = a;
    *(BYTE**)(s + S_NODE1) = b;
    BYTE* poly = Obj((size_t)(np > 0 ? np : 1) * 0x18);
    double cx = NX(a), cz = NZ(a), len = 0.0;
    for (int i = 0; i < np; i++)
    {
        len += hypot(pts[i].x - cx, pts[i].z - cz);
        BYTE* p = poly + i * 0x18;
        *(float*)(p + 0x00) = (float)pts[i].x;
        *(float*)(p + 0x08) = (float)pts[i].z;
        *(float*)(p + 0x14) = (float)len;
        cx = pts[i].x; cz = pts[i].z;
    }
    len += hypot(NX(b) - cx, NZ(b) - cz);
    *(BYTE**)(s + S_POLY) = poly;
    *(BYTE**)(s + S_POLY + 8) = poly + np * 0x18;
    *(float*)(s + S_LEN) = (float)len;
    return s;
}

// A path of dense points (every ~step m) cut into segments every segLen m.
// Every other segment is built REVERSED (node0 at its far end), as the game's
// track often is: a train crosses it with dir = 1 while its position still
// counts from the end it entered - the case 2.0.0 got wrong.
struct Path { BYTE** segs; int n; BYTE** nodes; BYTE* rev; };

static Path BuildPath(const P2* pts, int npts, double segLen)
{
    Path p;
    p.segs  = (BYTE**)calloc(npts, sizeof(BYTE*));
    p.nodes = (BYTE**)calloc(npts, sizeof(BYTE*));
    p.rev   = (BYTE*)calloc(npts, 1);
    p.n = 0;
    double acc = 0.0;
    int start = 0;
    BYTE* startNode = Node(pts[0].x, pts[0].z);
    p.nodes[0] = startNode;
    for (int i = 1; i < npts; i++)
    {
        acc += hypot(pts[i].x - pts[i - 1].x, pts[i].z - pts[i - 1].z);
        if (acc >= segLen - 1e-6 || i == npts - 1)
        {
            BYTE* endNode = Node(pts[i].x, pts[i].z);
            int   np = i - start - 1;
            if (p.n % 2 == 1)
            {
                P2* rp = (P2*)calloc(np > 0 ? np : 1, sizeof(P2));
                for (int j = 0; j < np; j++) rp[j] = pts[i - 1 - j];
                p.segs[p.n] = Seg(endNode, startNode, rp, np);
                p.rev[p.n]  = 1;
            }
            else
                p.segs[p.n] = Seg(startNode, endNode, pts + start + 1, np);
            p.nodes[++p.n] = endNode;
            startNode = endNode;
            start = i;
            acc = 0.0;
        }
    }
    return p;
}

static P2* StraightPts(double z0, double z1, double step, int* n, double x = 0.0)
{
    int m = (int)((z1 - z0) / step + 0.5) + 1;
    P2* pts = (P2*)calloc(m, sizeof(P2));
    for (int i = 0; i < m; i++) { pts[i].x = x; pts[i].z = z0 + i * step; }
    *n = m;
    return pts;
}

// A chain (station or customhouse) owning the given chain-local segments.
static BYTE* Chain(int type, BYTE** segs, int n)
{
    BYTE* chain = Obj(0x1000);
    BYTE* desc  = Obj(0x400);
    *(int*)(desc + 0x360) = type;
    *(BYTE**)(chain + CHAIN_TYPE_DESC) = desc;
    BYTE* tab = Obj((size_t)n * 0x60);
    for (int i = 0; i < n; i++) *(BYTE**)(tab + i * 0x60 + 0x20) = segs[i];
    *(BYTE**)(chain + CHAIN_TABLE) = tab;
    *(BYTE**)(chain + CHAIN_TABLE + 8) = tab + n * 0x60;
    return chain;
}

// An island segment of its own, with nodes coinciding with (x, z0) and (x, z1).
static BYTE* Island(double z0, double z1)
{
    int n; P2* pts = StraightPts(z0, z1, 5.0, &n);
    return Seg(Node(0, z0), Node(0, z1), pts + 1, n - 2);
}

static void SetChains(BYTE** chains, int n)
{
    static void* vec[64];
    for (int i = 0; i < n; i++) vec[i] = chains[i];
    DWORD old;
    VirtualProtect(g_chainVec, 16, PAGE_READWRITE, &old);
    g_chainVec[0] = (BYTE**)vec;
    g_chainVec[1] = (BYTE**)(vec + n);
    g_facScanned = 0;                       // rebuild on the next call
}

// ---------------------------------------------------------------- trains

static BYTE* VType(float len, int transport)
{
    BYTE* t = Obj(0x9000);
    *(float*)(t + T_LENGTH)   = len;
    *(int*)(t + T_TRANSPORT)  = transport;
    *(int*)(t + T_CATEGORY)   = 4;
    return t;
}

struct Tr
{
    BYTE*  v;
    BYTE** route;           // backing array; the "game" moves the end pointer
    BYTE*  rev;             // per route index: crossed node1 -> node0 (dir 1)
    int    routeN, idx;
    double pos;             // along the current leg, from node0 (dir 0)
    double speed;           // km/h
    double truth;           // metres actually travelled
};

static Tr MakeTrain(int passenger, BYTE** route, int n, int nWagons, void** objs, int nObjs)
{
    Tr t;
    memset(&t, 0, sizeof(t));
    t.v = Obj(0x2000);
    *(BYTE**)(t.v + V_TYPE) = passenger ? VType(25.0f, 7) : VType(20.0f, 0);
    BYTE** wag = (BYTE**)calloc(nWagons > 0 ? nWagons : 1, sizeof(BYTE*));
    for (int i = 0; i < nWagons; i++)
    {
        wag[i] = Obj(0x2000);
        *(BYTE**)(wag[i] + V_TYPE) = passenger ? VType(25.0f, 7) : VType(15.0f, 2);
    }
    *(BYTE***)(t.v + V_WAGON_VEC) = wag;
    *(BYTE***)(t.v + V_WAGON_VEC + 8) = wag + nWagons;
    void** ob = (void**)calloc(nObjs > 0 ? nObjs : 1, sizeof(void*));
    for (int i = 0; i < nObjs; i++) ob[i] = objs[i];
    *(void***)(t.v + V_ROUTE_OBJS) = ob;
    *(void***)(t.v + V_ROUTE_OBJS + 8) = ob + nObjs;
    t.route = route;
    t.routeN = n;
    *(BYTE***)(t.v + V_ROUTE_SEGS) = route;
    *(BYTE***)(t.v + V_ROUTE_SEGS + 8) = route + n;
    return t;
}

static void Sync(Tr* t)
{
    *(BYTE***)(t->v + V_ROUTE_SEGS + 8) = t->route + t->routeN;
    *(int*)(t->v + V_ROUTE_IDX) = t->idx;
    *(BYTE**)(t->v + V_CUR_SEG) = t->route[t->idx];
    *(BYTE*)(t->v + V_DIR) = t->rev ? t->rev[t->idx] : 0;
    *(float*)(t->v + V_POS) = (float)t->pos;
    *(float*)(t->v + V_SPEED) = (float)t->speed;
}

static double SegL(BYTE* s) { return *(float*)(s + S_LEN); }

// Where the head is, in metres from the start of route[0].
static double HeadAt(Tr* t)
{
    double d = 0.0;
    for (int i = 0; i < t->idx; i++) d += SegL(t->route[i]);
    return d + t->pos;
}

// Move ds metres along the route; stops dead at the route's end (the game
// stops a train at the end of its route).
static int Advance(Tr* t, double ds)
{
    t->pos += ds;
    t->truth += ds;
    while (t->pos > SegL(t->route[t->idx]))
    {
        if (t->idx + 1 >= t->routeN)
        {
            t->truth -= t->pos - SegL(t->route[t->idx]);
            t->pos = SegL(t->route[t->idx]);
            return 1;
        }
        t->pos -= SegL(t->route[t->idx]);
        ++t->idx;
    }
    return 0;
}

// One frame: two plugin calls (curve hook, brake hook), then the speed
// follows the limit - braking at the planned rate, accelerating at 0.5 m/s^2.
static float Frame(Tr* t, double dtGame, double vmax)
{
    Sync(t);
    float rate = 1.6f;
    float cap = CurveLimitFor(t->v, NULL);
    CurveLimitFor(t->v, &rate);
    double target = cap < vmax ? cap : vmax;
    if (t->speed > target) { t->speed -= rate * 3.6 * dtGame; if (t->speed < target) t->speed = target; }
    else                   { t->speed += 0.5 * 3.6 * dtGame; if (t->speed > target) t->speed = target; }
    if (t->speed < 0.0) t->speed = 0.0;
    return cap;
}

static Train* SlotOf(BYTE* v)
{
    unsigned h = SlotHash(v) & (SLOT_N - 1);
    for (int i = 0; i < SLOT_PROBE; i++)
        if (g_trains[(h + i) & (SLOT_N - 1)].veh == v) return &g_trains[(h + i) & (SLOT_N - 1)];
    return NULL;
}

// ---------------------------------------------------------------- scenarios

// Customs B, as run on 2026-09-27: a border line with a first customs zone at
// 3000-3050 that trains pass through, a second at 3500 where they stop at
// 3615 after running onto the customhouse's own track. The "game" extends
// the route in chunks: first to the first zone's node, then to the second's,
// and onto the island track only ~200 m before the stop.
struct Profile { double at[12]; double v[12]; int n; double maxInside, stopAt, minNearStop; int stopped; double lenExitFree; };

static Profile CustomsRun(int passenger, double k, int verbose)
{
    int n; P2* pts = StraightPts(0, 8000, 5.0, &n);
    Path main = BuildPath(pts, n, 50.0);                  // nodes every 50 m
    BYTE* zone1 = Island(3000, 3050);
    BYTE* zone2 = Island(3500, 3615);
    // departure: from the island's end node onward
    int m; P2* dp = StraightPts(3615, 8000, 5.0, &m);
    Path dep = BuildPath(dp, m, 50.0);
    dep.segs[0] = Seg(*(BYTE**)(zone2 + S_NODE1), dep.nodes[1], NULL, 0);

    BYTE* zsegs[2] = { zone1, zone2 };
    BYTE* customs = Chain(CHAIN_TYPE_CUSTOMHOUSE, zsegs, 2);
    BYTE* chains[1] = { customs };
    SetChains(chains, 1);

    BYTE** route = (BYTE**)calloc(512, sizeof(BYTE*));
    BYTE*  rev   = (BYTE*)calloc(512, 1);
    for (int i = 0; i < main.n; i++) { route[i] = main.segs[i]; rev[i] = main.rev[i]; }
    void* objs[1] = { customs };
    Tr t = MakeTrain(passenger, route, 60, passenger ? 8 : 20, objs, 1);   // to the zone-1 node at 3000
    t.rev = rev;
    t.speed = 200.0;

    double checks[] = { 1500, 2000, 2500, 2900, 3000, 3200, 3450, 3550, 3590, 3605 };
    Profile pr; memset(&pr, 0, sizeof(pr));
    pr.n = 10;
    for (int i = 0; i < pr.n; i++) pr.at[i] = checks[i];
    pr.minNearStop = 1e9;
    int ci = 0, phase = 0, hold = 0;
    double dt = 0.016 * k;

    for (int f = 0; f < 200000; f++)
    {
        double head = t.idx < 160 && phase < 3 ? HeadAt(&t) : 0.0;
        if (phase == 0 && head > 2800) { t.routeN = 70; phase = 1; }          // chunk to the zone-2 node
        if (phase == 1 && head > 3300)                                         // onto the island track
        {
            route[70] = zone2; rev[70] = 0; t.routeN = 71; phase = 2;
        }
        float cap = Frame(&t, dt, 200.0);
        head = phase < 3 ? HeadAt(&t) : 3615.0 + t.truth - pr.stopAt;
        while (ci < pr.n && head >= pr.at[ci]) pr.v[ci++] = t.speed;
        if (phase >= 1 && phase < 3 && head >= 3000 && head < 3615 && t.speed > pr.maxInside) pr.maxInside = t.speed;
        if (phase == 2 && head > 3560 && t.speed < pr.minNearStop) pr.minNearStop = t.speed;
        if (verbose && f % 40 == 0 && phase < 3)
            printf("      head %7.1f  v %6.1f  cap %7.1f\n", head, t.speed, cap > 999 ? 999.0 : cap);

        int atEnd = Advance(&t, t.speed / 3.6 * dt);
        if (phase == 2 && atEnd)
        {
            t.speed = 0.0; pr.stopped = 1; pr.stopAt = HeadAt(&t);
            phase = 3; hold = 0;
        }
        if (phase == 3)
        {
            t.speed = 0.0;
            if (++hold == 120)                                                  // clearance: depart
            {
                for (int i = 0; i < dep.n; i++) { route[71 + i] = dep.segs[i]; rev[71 + i] = i ? dep.rev[i] : 0; }
                t.routeN = 71 + dep.n;
                phase = 4;
            }
        }
        if (phase == 4)
        {
            double past = t.truth - pr.stopAt;                                  // metres since the stop
            if (t.speed > 52.0 && pr.lenExitFree == 0.0) pr.lenExitFree = past;
            if (past > 1200.0) break;
        }
    }
    return pr;
}

static void TestCustoms(void)
{
    printf("\n== customs B, passenger CR400-like (225 m), game speed x1\n");
    Profile a = CustomsRun(1, 1.0, getenv("RP_TRACE") != NULL);
    for (int i = 0; i < a.n; i++) printf("      at %5.0f m: %6.1f km/h\n", a.at[i], a.v[i]);
    Check(a.v[4] <= 52.0, "enters the first zone at <= 50 (%.1f km/h at 3000 m)", a.v[4]);
    Check(a.maxInside <= 52.0, "never above 50 between the zones (max %.1f km/h)", a.maxInside);
    Check(a.minNearStop >= 13.0, "never below the release speed before the stop (min %.1f km/h)", a.minNearStop);
    Check(a.v[9] <= 20.0, "crawls into the stop (%.1f km/h 10 m out)", a.v[9]);
    Check(a.stopped, "reaches the stop point and the game stops it (at %.0f m)", a.stopAt);
    Check(a.lenExitFree >= 225.0, "leaves at <= 50 until the tail clears the zone (free after %.0f m)", a.lenExitFree);

    printf("\n== same run at game speed x4\n");
    Profile b = CustomsRun(1, 4.0, 0);
    double worst = 0.0;
    for (int i = 0; i < a.n; i++)
    {
        double d = fabs(a.v[i] - b.v[i]);
        if (d > worst) worst = d;
    }
    for (int i = 0; i < b.n; i++) printf("      at %5.0f m: %6.1f km/h  (x1 %6.1f)\n", b.at[i], b.v[i], a.v[i]);
    Check(worst <= 6.0, "x4 profile matches x1 at every checkpoint (worst %.1f km/h)", worst);
    Check(b.maxInside <= 52.0 && b.stopped, "x4 still holds 50 inside and stops (max %.1f)", b.maxInside);

    printf("\n== same run, freight (320 m, no passengers)\n");
    Profile c = CustomsRun(0, 1.0, 0);
    for (int i = 0; i < c.n; i++) printf("      at %5.0f m: %6.1f km/h  (passenger %6.1f)\n", c.at[i], c.v[i], a.v[i]);
    Check(c.v[2] < a.v[2] - 10.0, "freight is already slower 500 m out - softer, earlier braking (%.1f vs %.1f)",
          c.v[2], a.v[2]);
    Check(c.maxInside <= 52.0 && c.stopped, "freight holds 50 inside and stops (max %.1f)", c.maxInside);
}

static void TestDeadlock(void)
{
    printf("\n== stop approach: a train standing 40 m short of its stop\n");
    int n; P2* pts = StraightPts(0, 2000, 5.0, &n);
    Path main = BuildPath(pts, n, 50.0);
    BYTE* zone = Island(1500, 1615);
    BYTE* zs[1] = { zone };
    BYTE* customs = Chain(CHAIN_TYPE_CUSTOMHOUSE, zs, 1);
    BYTE* chains[1] = { customs };
    SetChains(chains, 1);
    BYTE** route = (BYTE**)calloc(64, sizeof(BYTE*));
    for (int i = 0; i < 30; i++) route[i] = main.segs[i];
    route[30] = zone;
    void* objs[1] = { customs };
    Tr t = MakeTrain(1, route, 31, 8, objs, 1);
    t.rev = main.rev;
    t.speed = 50.0;
    while (HeadAt(&t) < 1575.0) { Frame(&t, 0.016, 60.0); Advance(&t, t.speed / 3.6 * 0.016); }
    double minCap = 1e9;
    for (int f = 0; f < 3000; f++)                         // 48 s of game time standing
    {
        t.speed = 0.0;
        float cap = Frame(&t, 0.016, 60.0);
        t.speed = 0.0;
        if (cap < minCap) minCap = cap;
    }
    Train* s = SlotOf(t.v);
    Check(minCap >= 14.0 && minCap < 45.0, "the limit never pins it at 0: lowest %.1f km/h, stop still armed: %s (1.3.1 held ~0-3)",
          minCap, s && s->stopArmed ? "yes" : "no");
}

static void TestStation(void)
{
    printf("\n== station on the line, train passing without stopping\n");
    int n; P2* pts = StraightPts(0, 7000, 5.0, &n);
    Path main = BuildPath(pts, n, 50.0);
    BYTE* plat = Island(5000, 5250);
    BYTE* ps[1] = { plat };
    BYTE* station = Chain(1, ps, 1);
    BYTE* chains[1] = { station };
    SetChains(chains, 1);
    BYTE** route = (BYTE**)calloc(256, sizeof(BYTE*));
    for (int i = 0; i < main.n; i++) route[i] = main.segs[i];
    Tr t = MakeTrain(1, route, main.n, 8, NULL, 0);
    t.rev = main.rev;
    t.speed = 150.0;
    double atEntry = -1, maxIn = 0, freeAt = -1;
    while (HeadAt(&t) < 6800.0)
    {
        Frame(&t, 0.016, 150.0);
        double h = HeadAt(&t);
        if (atEntry < 0 && h >= 5000) atEntry = t.speed;
        if (h >= 5000 && h <= 5250 + 225 && t.speed > maxIn) maxIn = t.speed;
        if (h > 5000 && freeAt < 0 && t.speed > 62.0) freeAt = h;
        if (Advance(&t, t.speed / 3.6 * 0.016)) break;
    }
    Check(atEntry <= 62.0, "reaches the station at <= 60 (%.1f km/h)", atEntry);
    Check(maxIn <= 62.0, "holds 60 until the tail clears (max %.1f km/h)", maxIn);
    Check(freeAt >= 5250 + 225 - 5, "accelerates only after the tail leaves the platform (at %.0f m, tail clears at %.0f)",
          freeAt, 5250.0 + 225.0);
}

static double CurveRun(P2* pts, int n, double* minV, double* freeAt, double arcEnd)
{
    SetChains(NULL, 0);
    Path p = BuildPath(pts, n, 50.0);
    Tr t = MakeTrain(1, p.segs, p.n, 8, NULL, 0);
    t.rev = p.rev;
    t.speed = 200.0;
    *minV = 1e9; *freeAt = -1;
    double atArc = -1;
    while (1)
    {
        Frame(&t, 0.016, 200.0);
        double h = HeadAt(&t);
        if (t.speed < *minV) *minV = t.speed;
        if (arcEnd > 0 && h > arcEnd && *freeAt < 0 && t.speed > *minV + 3.0) *freeAt = h;
        if (atArc < 0 && h >= 1000) atArc = t.speed;
        if (Advance(&t, t.speed / 3.6 * 0.016)) break;
    }
    return atArc;
}

static void TestCurves(void)
{
    printf("\n== curves\n");
    // Straight 1000 m, a 500 m radius arc over 400 m (the "R = 996" curves of
    // the 2026-09 run, measured correctly), straight again.
    P2* pts = (P2*)calloc(1000, sizeof(P2));
    int n = 0;
    for (double z = 0; z <= 1000; z += 5) { pts[n].x = 0; pts[n].z = z; ++n; }
    double R = 500, th = 0;
    for (double s = 5; s <= 400; s += 5)
    {
        th = s / R;
        pts[n].x = R - R * cos(th); pts[n].z = 1000 + R * sin(th); ++n;
    }
    double ex = pts[n - 1].x, ez = pts[n - 1].z;
    for (double s = 5; s <= 1500; s += 5) { pts[n].x = ex + s * sin(th); pts[n].z = ez + s * cos(th); ++n; }
    double minV, freeAt;
    CurveRun(pts, n, &minV, &freeAt, 1400.0);
    double want = sqrt(g.curveLat * 500.0) * 3.6;
    Check(fabs(minV - want) <= 4.0, "R = 500 m arc holds sqrt(%.1f R) = %.1f km/h (min %.1f; 1.3.x gave 135 here too)",
          g.curveLat, want, minV);
    Check(freeAt >= 1400 + 225 - 10, "stays at it until the tail leaves the arc (free at %.0f, tail clears %.0f)",
          freeAt, 1400.0 + 225.0);

    // A 1.2 degree kink at 1000 m - two straights, dense points.
    n = 0;
    for (double z = 0; z <= 1000; z += 5) { pts[n].x = 0; pts[n].z = z; ++n; }
    double k = 1.2 / 57.2958;
    for (double s = 5; s <= 1500; s += 5) { pts[n].x = s * sin(k); pts[n].z = 1000 + s * cos(k); ++n; }
    CurveRun(pts, n, &minV, &freeAt, -1);
    Check(minV >= 195.0, "a 1.2 deg kink (a bridge-end join) does not brake (min %.1f km/h)", minV);

    // A 0.15 m lateral jog at 1000 m.
    n = 0;
    for (double z = 0; z <= 1000; z += 5) { pts[n].x = 0; pts[n].z = z; ++n; }
    for (double z = 1005; z <= 2500; z += 5) { pts[n].x = 0.15; pts[n].z = z; ++n; }
    CurveRun(pts, n, &minV, &freeAt, -1);
    Check(minV >= 195.0, "a 0.15 m lateral jog does not brake (min %.1f km/h)", minV);

    // A real switch: R = 190 m over 30 m must still count.
    n = 0;
    for (double z = 0; z <= 1000; z += 5) { pts[n].x = 0; pts[n].z = z; ++n; }
    R = 190;
    for (double s = 5; s <= 30; s += 5) { th = s / R; pts[n].x = R - R * cos(th); pts[n].z = 1000 + R * sin(th); ++n; }
    ex = pts[n - 1].x; ez = pts[n - 1].z;
    for (double s = 5; s <= 1500; s += 5) { pts[n].x = ex + s * sin(th); pts[n].z = ez + s * cos(th); ++n; }
    CurveRun(pts, n, &minV, &freeAt, -1);
    want = sqrt(g.curveLat * 190.0) * 3.6;
    Check(minV <= want + 3.0, "a 190 m switch curve still brakes hard (min %.1f km/h, sqrt(%.1f*190) = %.1f)",
          minV, g.curveLat, want);
}

// A straight line off the z axis where dense spline points meet legs that
// have only their two nodes (as a building's own track may). The walk lists a
// shared node twice; a window whose only neighbour on one side is that
// duplicate measures a zero-length chord, and atan2(0, 0) = 0 reads it as a
// heading of due +z. Every other test here runs along +z, where that is true.
static P2* SparseLine(double deg, int* n)
{
    P2* pts = (P2*)calloc(2000, sizeof(P2));
    double sx = sin(deg / 57.2958), cz = cos(deg / 57.2958);
    int k = 0;
    for (double s = 0; s <= 1000; s += 5)     { pts[k].x = s * sx; pts[k].z = s * cz; ++k; }
    for (double s = 1050; s <= 1200; s += 50) { pts[k].x = s * sx; pts[k].z = s * cz; ++k; }
    for (double s = 1205; s <= 2700; s += 5)  { pts[k].x = s * sx; pts[k].z = s * cz; ++k; }
    *n = k;
    return pts;
}

static void TestSparseJoin(void)
{
    printf("\n== straight line off the z axis: dense spline meeting node-only legs\n");
    double degs[] = { 0.0, 8.0, 15.0, 40.0 };
    for (int d = 0; d < 4; d++)
    {
        double minV, freeAt;
        int n; P2* pts = SparseLine(degs[d], &n);
        CurveRun(pts, n, &minV, &freeAt, -1);
        Check(minV >= 195.0, "heading %4.1f deg: a straight line does not brake (min %.1f km/h)", degs[d], minV);
    }

    // A real R = 300 m arc off the axis, entered straight from node-only legs:
    // the guard must not hide it.
    P2* pts = (P2*)calloc(2000, sizeof(P2));
    double h = 25.0 / 57.2958, R = 300.0, x = 0, z = 0;
    int k = 0;
    for (double s = 0; s <= 1000; s += 5)     { pts[k].x = s * sin(h); pts[k].z = s * cos(h); ++k; }
    for (double s = 1050; s <= 1200; s += 50) { pts[k].x = s * sin(h); pts[k].z = s * cos(h); ++k; }
    x = pts[k - 1].x; z = pts[k - 1].z;
    double cx = x + R * cos(h), cz = z - R * sin(h);          // centre to the right
    double th = 0.0;
    for (double s = 5; s <= 200; s += 5)
    {
        th = h + s / R;
        pts[k].x = cx - R * cos(th); pts[k].z = cz + R * sin(th); ++k;
    }
    x = pts[k - 1].x; z = pts[k - 1].z;
    for (double s = 5; s <= 1500; s += 5) { pts[k].x = x + s * sin(th); pts[k].z = z + s * cos(th); ++k; }
    double minV, freeAt;
    CurveRun(pts, k, &minV, &freeAt, -1);
    double want = sqrt(g.curveLat * R) * 3.6;
    Check(fabs(minV - want) <= 4.0, "a 300 m arc at 25 deg after node-only legs holds %.1f km/h (min %.1f)", want, minV);
}

// ---------------------------------------------------------------- tilt
//
// The car-transform hook fed the way FUN_1406afde0 feeds it: the car's
// position and its yaw (forward = (sin yaw, cos yaw), + pi for a car coupled
// back to front), one call per frame; the engine's function is a fake that
// keeps the roll it was handed.

static float g_seenRoll;
static void FakeNodeCreate(void*, const float*, const float* rot, const float*) { g_seenRoll = rot[2]; }

static BYTE* TiltCarObj(int passenger, int reversed)
{
    BYTE* v = Obj(0x2000);
    BYTE* t = Obj(0x9000);
    *(int*)(t + T_TRANSPORT) = passenger ? 7 : 2;
    *(BYTE**)(v + V_TYPE) = t;
    *(BYTE*)(v + V_REVERSED) = (BYTE)reversed;
    return v;
}

static double g_h0 = 0.5, g_R = 650.0, g_arcFrom = 500.0, g_arcLen = 400.0;
static double YawArc(double s)
{
    if (s < g_arcFrom) return g_h0;
    if (s < g_arcFrom + g_arcLen) return g_h0 + (s - g_arcFrom) / g_R;
    return g_h0 + g_arcLen / g_R;
}
static double YawKink(double s)             // 1.2 deg, turned over a car's 17 m pivot spacing
{
    double k = 1.2 / 57.2958;
    if (s < 500.0) return g_h0;
    if (s < 517.0) return g_h0 + k * (s - 500.0) / 17.0;
    return g_h0 + k;
}
static double YawJog(double s)              // 0.15 m sideways and back
{
    double a = 0.15 / 17.0;
    if (s < 500.0 || s >= 534.0) return g_h0;
    return s < 517.0 ? g_h0 + a : g_h0 - a;
}

// Roll in degrees at each checkpoint; the largest |roll| between lo and hi.
static void TiltDrive(BYTE* v, double kmh, double (*yawOf)(double), double total, double step,
                      const double* at, double* out, int nat, double lo, double hi, double* peak)
{
    *(float*)(v + V_SPEED) = (float)kmh;
    int rev = *(BYTE*)(v + V_REVERSED);
    double x = 3000.0, z = -2000.0, s = 0.0;
    int ci = 0;
    *peak = 0.0;
    while (s <= total)
    {
        double yaw = yawOf(s);
        float pos[3]   = { (float)x, 0.0f, (float)z };
        float rot[3]   = { 0.0f, (float)(yaw + (rev ? 3.14159265 : 0.0)), 0.0f };
        float scale[3] = { 1.0f, 1.0f, 1.0f };
        RailNodeTransform(v + V_NODE, pos, rot, scale);
        double deg = g_seenRoll * 57.2958;
        if (s >= lo && s <= hi && fabs(deg) > *peak) *peak = fabs(deg);
        while (ci < nat && s >= at[ci]) out[ci++] = deg;
        x += sin(yaw) * step; z += cos(yaw) * step; s += step;
    }
}

static void TestTilt(void)
{
    printf("\n== tilt: max %.1f deg, gain %.2f, from %.0f km/h, %.1f deg/s\n",
           g.tiltMax * 57.2958, g.tiltGain, g.tiltMinKmh, g.tiltRate * 57.2958);
    Check(g_siteNodeCall != NULL, "the car-transform call is located by signature (+%08X)",
          g_siteNodeCall ? (unsigned)(g_siteNodeCall - g_exeBase) : 0u);
    g_NodeCreate = FakeNodeCreate;

    double at[] = { 480.0, 560.0, 600.0, 700.0, 1100.0 };
    double r[5], peak;
    double capDeg = g.tiltMax * 57.2958;

    // R 650 m right-hand arc at 155 km/h: a = 2.85 m/s^2, half of atan(a/g) is 8.1 deg -> the cap.
    TiltDrive(TiltCarObj(1, 0), 155.0, YawArc, 1300.0, 2.0, at, r, 5, 0, 0, &peak);
    printf("      R 650, 155 km/h: %.2f before, %.2f / %.2f / %.2f into the arc, %.2f after\n", r[0], r[1], r[2], r[3], r[4]);
    Check(fabs(r[0]) < 0.05, "upright on the straight before the arc (%.2f deg)", r[0]);
    Check(fabs(r[3] + capDeg) < 0.2, "leans into a right-hand arc: %.2f deg (want -%.1f, the cap)", r[3], capDeg);
    Check(r[1] > r[3] + 0.5, "leans in gradually, not at the first metre (%.2f at 60 m in)", r[1]);
    Check(fabs(r[4]) < 0.1, "upright again 200 m after the arc (%.2f deg)", r[4]);

    TiltDrive(TiltCarObj(1, 1), 155.0, YawArc, 1300.0, 2.0, at, r, 5, 0, 0, &peak);
    Check(fabs(r[3] - capDeg) < 0.2, "a car coupled back to front leans the same way in the world: roll %+.2f deg", r[3]);

    TiltDrive(TiltCarObj(0, 0), 155.0, YawArc, 1300.0, 2.0, at, r, 5, 0, 0, &peak);
    Check(fabs(r[3]) < 0.01, "freight stays upright (tilt_freight = 0): %.2f deg", r[3]);

    TiltDrive(TiltCarObj(1, 0), 60.0, YawArc, 1300.0, 2.0, at, r, 5, 0, 0, &peak);
    Check(fabs(r[3]) < 0.01, "no lean below tilt_min_kmh (60 km/h: %.2f deg)", r[3]);

    // R 2000 m at 160 km/h: a = 0.99 m/s^2 -> 0.5 * atan(a/g) = 2.87 deg, under the cap.
    g_R = 2000.0;
    TiltDrive(TiltCarObj(1, 0), 160.0, YawArc, 1300.0, 2.0, at, r, 5, 0, 0, &peak);
    double want = g.tiltGain * atan(44.444 * 44.444 / 2000.0 / 9.81) * 57.2958;
    Check(fabs(r[3] + want) < 0.15, "a gentle curve gives a gentle lean: %.2f deg (want -%.2f)", r[3], want);
    g_R = 650.0;

    // The same arc at game speed x4: four times the metres per frame.
    double r4[5];
    TiltDrive(TiltCarObj(1, 0), 155.0, YawArc, 1300.0, 8.0, at, r4, 5, 0, 0, &peak);
    TiltDrive(TiltCarObj(1, 0), 155.0, YawArc, 1300.0, 2.0, at, r, 5, 0, 0, &peak);
    double worst = 0.0;
    for (int i = 0; i < 5; i++) if (fabs(r4[i] - r[i]) > worst) worst = fabs(r4[i] - r[i]);
    Check(worst < 1.0, "x4 game speed leans the same along the track (worst %.2f deg apart)", worst);

    TiltDrive(TiltCarObj(1, 0), 300.0, YawKink, 1000.0, 2.0, at, r, 5, 450.0, 800.0, &peak);
    Check(peak < 0.3, "a 1.2 deg kink at 300 km/h does not throw the car over (peak %.2f deg)", peak);

    TiltDrive(TiltCarObj(1, 0), 300.0, YawJog, 1000.0, 2.0, at, r, 5, 450.0, 800.0, &peak);
    Check(peak < 0.3, "a 0.15 m lateral jog at 300 km/h does not either (peak %.2f deg)", peak);
}

static void TestOdometer(void)
{
    printf("\n== odometer\n");
    SetChains(NULL, 0);
    int n; P2* pts = StraightPts(0, 6000, 5.0, &n);
    Path p = BuildPath(pts, n, 37.0);                     // odd leg length
    for (int k = 1; k <= 4; k *= 4)
    {
        Tr t = MakeTrain(1, p.segs, p.n, 0, NULL, 0);
        t.rev = p.rev;
        t.speed = 300.0;
        while (t.truth < 5000.0) { Frame(&t, 0.016 * k, 300.0); Advance(&t, 300.0 / 3.6 * 0.016 * k); }
        Sync(&t);
        CurveLimitFor(t.v, NULL);
        Train* s = SlotOf(t.v);
        Check(s && fabs(s->odo - t.truth) <= 1.0, "x%d: odometer %.1f m vs travelled %.1f m, %u resync(s)",
              k, s ? s->odo : -1.0, t.truth, s ? s->odoResyncs : 0);
    }
}

static void TestTornRead(void)
{
    printf("\n== a torn read of the route does not erase what is ahead\n");
    int n; P2* pts = StraightPts(0, 5000, 5.0, &n);
    Path main = BuildPath(pts, n, 50.0);
    BYTE* zone = Island(3000, 3050);
    BYTE* zs[1] = { zone };
    BYTE* customs = Chain(CHAIN_TYPE_CUSTOMHOUSE, zs, 1);
    BYTE* chains[1] = { customs };
    SetChains(chains, 1);
    BYTE** route = (BYTE**)calloc(128, sizeof(BYTE*));
    for (int i = 0; i < main.n; i++) route[i] = main.segs[i];
    void* objs[1] = { customs };
    Tr t = MakeTrain(1, route, main.n, 8, objs, 1);
    t.rev = main.rev;
    t.speed = 120.0;
    while (HeadAt(&t) < 2200.0) { Frame(&t, 0.016, 120.0); Advance(&t, t.speed / 3.6 * 0.016); }
    float before = Frame(&t, 0.016, 120.0);
    int keep = t.routeN;
    t.routeN = t.idx + 2;                                 // the game's route collapses to 100 m
    Train* s = SlotOf(t.v);
    s->scanTick = 0;                                      // force a scan of the torn route
    float torn = Frame(&t, 0.016, 120.0);
    t.routeN = keep;
    Check(fabs(torn - before) <= 3.0, "limit before %.1f, during the torn scan %.1f km/h", before, torn);
}

static void TestSlots(void)
{
    printf("\n== slot table under churn\n");
    SetChains(NULL, 0);
    int n; P2* pts = StraightPts(0, 1000, 5.0, &n);
    Path p = BuildPath(pts, n, 50.0);
    for (int i = 0; i < 4000; i++)
    {
        Tr t = MakeTrain(1, p.segs, p.n, 0, NULL, 0);
        Sync(&t);
        CurveLimitFor(t.v, NULL);
        Train* s = SlotOf(t.v);
        if (s) s->seen -= 2 * SLOT_STALE_MS;             // deleted long ago
    }
    Tr live = MakeTrain(1, p.segs, p.n, 0, NULL, 0);
    live.speed = 80.0;
    unsigned long s0 = g_scans;
    for (int f = 0; f < 200; f++) { Frame(&live, 0.016, 80.0); Advance(&live, live.speed / 3.6 * 0.016); }
    Check(SlotOf(live.v) != NULL && g_scans - s0 < 20, "a live train keeps its slot (%lu scans in 200 frames)",
          g_scans - s0);
}

// ---------------------------------------------------------------- bogies

// The game folder is where SOVIET64.exe (argv[1]) lives; Workshop items are in
// <library>/steamapps/workshop/content/784150, two levels up from it.
static char g_gameDir[512];

static void FakeFullPath(char* out, const char* in)
{
    const char* ws = "workshop_subscribed/";
    if (!strncmp(in, ws, strlen(ws)))
        sprintf(out, "%s/../../workshop/content/784150/%s", g_gameDir, in + strlen(ws));
    else
        sprintf(out, "%s/%s", g_gameDir, in);
}

static int ReadAll(const char* path, BYTE** data, size_t* n)
{
    FILE* f = fopen(path, "rb");
    if (!f) return 0;
    fseek(f, 0, SEEK_END); long sz = ftell(f); fseek(f, 0, SEEK_SET);
    *data = (BYTE*)malloc(sz); *n = fread(*data, 1, sz, f); fclose(f);
    return *n == (size_t)sz;
}

// A type as the parser leaves it: folder path at +0, category, default
// pivots, and the ends / length from the model's bbox.bin.
static BYTE* ModelType(const char* rel, int* okOut)
{
    char full[1024], bb[1100];
    FakeFullPath(full, rel);
    sprintf(bb, "%s/bbox.bin", full);
    BYTE* d; size_t n;
    *okOut = ReadAll(bb, &d, &n) && n >= 24;
    BYTE* t = Obj(0x9000);
    strcpy((char*)t, rel);
    *(int*)(t + T_CATEGORY) = 4;
    *(float*)(t + T_AXIS_FWD) = 3.0f;
    *(float*)(t + T_AXIS_BWD) = 3.0f;
    if (*okOut)
    {
        float* v = (float*)d;
        *(float*)(t + T_FRONT_EXT) = v[5];
        *(float*)(t + T_BACK_EXT)  = -v[2];
        *(float*)(t + T_LENGTH)    = v[5] - v[2];
    }
    return t;
}

static void TestBogies(void)
{
    printf("\n== bogies from the model (CR400AF, Workshop 3019370041)\n");
    g.bogies = 1;
    g_GetFullPath = FakeFullPath;
    struct { const char* rel; float fwd, bwd; } cases[] = {
        { "workshop_subscribed/3019370041/CR400AF_Tc", 5.78f, 4.14f },
        { "workshop_subscribed/3019370041/CR400AF_m1", 4.10f, 4.10f },
    };
    for (auto& c : cases)
    {
        int ok;
        BYTE* t = ModelType(c.rel, &ok);
        if (!ok) { printf("  [SKIP] %s: not subscribed (Workshop 3019370041)\n", c.rel); continue; }
        BogieFix(t);
        float f = *(float*)(t + T_AXIS_FWD), b = *(float*)(t + T_AXIS_BWD);
        Check(fabsf(f - c.fwd) <= 0.15f && fabsf(b - c.bwd) <= 0.15f,
              "%s: pivots %.2f / %.2f m from the ends (want %.2f / %.2f; length %.2f)",
              c.rel + 30, f, b, c.fwd, c.bwd, *(float*)(t + T_LENGTH));
    }
    int ok;
    BYTE* van = ModelType("media_soviet/cwc/vehicles/2200reverse", &ok);
    BogieFix(van);
    Check(ok && *(float*)(van + T_AXIS_FWD) == 3.0f && *(float*)(van + T_AXIS_BWD) == 3.0f,
          "a one-object base-game model keeps the 3.0 defaults (%.2f / %.2f)",
          *(float*)(van + T_AXIS_FWD), *(float*)(van + T_AXIS_BWD));
    BYTE* set = ModelType("workshop_subscribed/3019370041/CR400AF_Tc", &ok);
    if (!ok) return;
    *(float*)(set + T_AXIS_FWD) = 1.3f;                   // the .ini set it: leave it alone
    BogieFix(set);
    Check(*(float*)(set + T_AXIS_FWD) == 1.3f, "an explicit $TRAIN_FORWARD_AXIS_DISTANCE is left alone (%.2f)",
          *(float*)(set + T_AXIS_FWD));
}

int main(int argc, char** argv)
{
    g_iniPath = argv[2];
    snprintf(g_gameDir, sizeof(g_gameDir), "%s", argv[1]);
    for (char* c = g_gameDir; *c; c++) if (*c == '\\') *c = '/';
    if (char* slash = strrchr(g_gameDir, '/')) *slash = 0;
    HMODULE m = LoadLibraryExA(argv[1], NULL, DONT_RESOLVE_DLL_REFERENCES);
    if (!m) { printf("map failed %lu\n", GetLastError()); return 1; }
    IMAGE_NT_HEADERS64* nt = (IMAGE_NT_HEADERS64*)((BYTE*)m + ((IMAGE_DOS_HEADER*)m)->e_lfanew);

    static TsmHost h;
    h.apiVersion = TSM_API_VERSION; h.structSize = sizeof(TsmHost);
    h.exeModule = m; h.exeBase = (unsigned char*)m; h.exeSize = nt->OptionalHeader.SizeOfImage;
    h.baseDir = "."; h.pluginDir = ".";
    h.log = HLog; h.installInlineHook = HHook; h.allocNear = HAllocNear;
    h.readablePtr = HReadable; h.configInt = HCfgInt; h.configString = HCfgStr;

    TsmPluginInfo info = {};
    int rc = TsmPluginInit(&h, &info);
    printf("init: rc=%d %s %s, approach core %s, slope %s, chain vector %s\n", rc, info.name, info.version,
           g_approachLive ? "live" : "OFF", g_slopeLive ? "live" : "OFF", g_chainVec ? "found" : "MISSING");
    if (rc != 0 || !g_chainVec || !g_approachLive) return 1;
    printf("config: service %.2f  freight %.2f  margin %.2f  a_lat %.2f  customs %.0f  station %.0f  release %.0f\n",
           g.serviceBrake, g.freightBrake, g.curveMargin, g.curveLat, g.customsEntry, g.stationLimit, g.stopRelease);

    TestOdometer();
    TestCustoms();
    TestDeadlock();
    TestStation();
    TestCurves();
    TestSparseJoin();
    TestTornRead();
    TestSlots();
    TestBogies();
    TestTilt();

    printf("\n%lu evaluations, %lu scans, %lu host readablePtr calls\n", g_evals, g_scans, g_vq);
    printf("%s (%d failure(s))\n", g_fails ? "FAILED" : "ALL PASS", g_fails);
    return g_fails != 0;
}
