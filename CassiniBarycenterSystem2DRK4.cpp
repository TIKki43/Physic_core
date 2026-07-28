// COMPILE:
//   g++ -std=c++20 -O2 -I . -I deps/cspice/include \
//       CassiniBarycenterSystem2DRK4.cpp \
//       PreInitializer.cpp WorldPhysics.cpp Body.cpp Vec3.cpp Gravity.cpp \
//       Integrator.cpp Mat3.cpp \
//       deps/cspice/lib/libcspice.a \
//       -lraylib -lopengl32 -lgdi32 -lwinmm -static-libgcc -static-libstdc++ \
//       -o CassiniBarycenterSystem2DRK4.exe
//
//   g++ -std=c++20 -O2 -I . -I deps/cspice/include CassiniBarycenterSystem2DRK4.cpp PreInitializer.cpp WorldPhysics.cpp Body.cpp Vec3.cpp Gravity.cpp Integrator.cpp Mat3.cpp deps/cspice/lib/libcspice.a -lraylib -lopengl32 -lgdi32 -lwinmm -static-libgcc -static-libstdc++  -o CassiniBarycenterSystem2DRK4.exe
// RUN:
//   ./CassiniBarycenterSystem2DRK4.exe


#include "WorldPhysics.h"
#include "PreInitializer.h"
#include "Body.h"
#include "Vec3.h"
#include "Constants.h"
#include "SpiceUsr.h"

#include <string>
#include <vector>
#include <deque>
#include <cmath>
#include <cstdio>
#include <cctype>
#include <algorithm>
#include <fstream>

#include "raylib.h"


static const char*  META_KERNEL          = "cas_2005_physicalcoords.tm";
static const char*  INITIAL_EPOCH        = "2005-01-01T00:00:00";
static const double PHYSICS_STEP_SECONDS = 450.0;             // seconds per physics step
static double       simDaysPerSecond     = 8.0;                // simulation speed
static const double TRAIL_SAMPLE_SECONDS = 3600.0;            // 1 h: fine enough to trace MOON orbits in focus view
static const double ERROR_SAMPLE_SECONDS = 86400.0;           // CSV error sample exactly once per simulated day (independent of trails)
static const int    MAX_TRAIL_POINTS     = 1200;              // 1200 h = 50 days of trail history per body
static const double MAX_ELAPSED_SECONDS  = 1825.0 * 86400.0;  // 5 years

static const char*  CSV_NAME             = "IntegrationErrors_Physical_RK4_450.csv";

static const int    WINDOW               = 1000;

static const double LIN_PX_PER_METER  = 450.0 / 4.6e12;
 // log reference radius 1e9 m
static const double LOG_R0_EXP        = 9.0;
static const double LOG_PX_PER_DECADE = 115.0;

static bool   logMode = false;
static double zoom    = 1.0;
static double panX    = 0.0;
static double panY    = 0.0;


static int    focusIndex = -1;

// World(meters, J2000, SSB) -> screen. Same transform for sim AND SPICE
static Vector2 worldToScreen(const Vec3& p) {
    double sx, sy;
    if (logMode) {
        double r = std::sqrt(p.x * p.x + p.y * p.y);   // 2D projected radius
        if (r < 1.0) { sx = 0.0; sy = 0.0; }
        else {
            double decades = std::log10(r) - LOG_R0_EXP;
            if (decades < 0.0) decades = 0.0;
            double px = decades * LOG_PX_PER_DECADE;    // direction preserved
            sx = (p.x / r) * px;
            sy = (p.y / r) * px;
        }
    } else {
        sx = p.x * LIN_PX_PER_METER;
        sy = p.y * LIN_PX_PER_METER;
    }
    double cx = WINDOW * 0.5 + panX;
    double cy = WINDOW * 0.5 + panY;
    return Vector2{ (float)(cx + sx * zoom), (float)(cy - sy * zoom) }; // invert screen Y
}

// Planet-relative projection used in focus mode (linear, no log): the body is
// drawn at (p - ref), so a moon's orbit around its planet becomes visible
static Vector2 focusScreen(const Vec3& p, const Vec3& ref, double scale) {
    double sx = (p.x - ref.x) * scale * zoom;
    double sy = (p.y - ref.y) * scale * zoom;
    double cx = WINDOW * 0.5 + panX;
    double cy = WINDOW * 0.5 + panY;
    return Vector2{ (float)(cx + sx), (float)(cy - sy) }; // invert screen Y
}

struct BodyView {
    std::string      name;
    std::string      spiceTarget;
    std::deque<Vec3> simTrail;
    std::deque<Vec3> spiceTrail;
};


static bool sameName(const std::string& a, const char* b) {
    std::size_t i = 0;
    for (; i < a.size() && b[i]; ++i)
        if (std::toupper((unsigned char)a[i]) != std::toupper((unsigned char)b[i])) return false;
    return i == a.size() && b[i] == '\0';
}

// A "primary" is the Sun or a planet; everything listed after it by
// SetPhysicalSolarSystem belongs to that primary's system
static bool isPrimary(const std::string& n) {
    static const char* primaries[] = { "SUN","MERCURY","VENUS","EARTH","MARS",
                                       "JUPITER","SATURN","URANUS","NEPTUNE","PLUTO" };
    for (const char* p : primaries) if (sameName(n, p)) return true;
    return false;
}

// One SPICE state query -> position in METERS
static bool querySpice(const std::string& target, double et, Vec3& outMeters, std::string& errOut) {
    SpiceDouble state[6];
    SpiceDouble lt = 0.0;
    spkezr_c(target.c_str(), et, "J2000", "NONE", "SOLAR SYSTEM BARYCENTER", state, &lt);
    if (failed_c()) {
        SpiceChar shortmsg[64];
        getmsg_c("SHORT", sizeof(shortmsg), shortmsg);
        reset_c();
        errOut = shortmsg;
        return false;
    }
    outMeters = Vec3(state[0] * 1000.0, state[1] * 1000.0, state[2] * 1000.0);
    return true;
}


static void initScene(WorldPhysics& world, std::vector<BodyView>& views, double& initialET) {
    world = WorldPhysics{};  // fresh, empty world

    kclear_c();
    erract_c("SET", 0, (SpiceChar*)"RETURN");
    errprt_c("SET", 0, (SpiceChar*)"NONE");

    PreInitializer::Missions::Cassini::SetPhysicalSolarSystem(
        world,
        META_KERNEL,
        INITIAL_EPOCH,
        "J2000",
        "NONE",
        "SOLAR SYSTEM BARYCENTER"
    );

    str2et_c(INITIAL_EPOCH, &initialET);

    views.clear();
    views.reserve(world.Bodies.size());
    for (const Body& b : world.Bodies) {
        BodyView v;
        v.name        = b.GetBodyType();
        v.spiceTarget = v.name;   // physical body compared against the SAME SPICE body
        views.push_back(std::move(v));
    }
}

int main() {
    WorldPhysics world;
    std::vector<BodyView> views;
    double initialET = 0.0;
    initScene(world, views, initialET);


    int         n               = (int)views.size();
    // simulated seconds past initialET(double)
    double      elapsed         = 0.0;
    double      accumulator     = 0.0;
    double      lastTrailSample = 0.0;
    double      nextErrorSample = ERROR_SAMPLE_SECONDS;
    bool        paused          = false;
    // non-empty -> freeze and show error
    std::string coverageError;

    auto findIndex = [&](const char* nm) {
        for (int i = 0; i < (int)views.size(); ++i) if (views[i].name == nm) return i;
        return -1;
    };
    int idxSun    = findIndex("SUN");
    int idxEarth  = findIndex("EARTH");
    int idxSaturn = findIndex("SATURN");

    std::vector<Vec3> spiceNow(n);
    std::vector<bool> spiceNowOk(n, false);

    std::vector<int> parentOf;
    std::vector<int> focusable;   // primaries that actually have moons
    auto buildTopology = [&]() {
        parentOf.assign(n, -1);
        int current = -1;
        for (int i = 0; i < n; ++i) {
            if (isPrimary(views[i].name)) { current = i; parentOf[i] = -1; }
            else                          { parentOf[i] = current; }
        }
        focusable.clear();
        for (int i = 0; i < n; ++i) {
            if (parentOf[i] >= 0) continue;
            for (int j = 0; j < n; ++j) if (parentOf[j] == i) { focusable.push_back(i); break; }
        }
    };
    buildTopology();

    std::ofstream errorCsv;
    auto openCsv = [&]() {
        errorCsv.close();
        errorCsv.clear();
        errorCsv.open(CSV_NAME, std::ios::out | std::ios::trunc);
        if (!errorCsv.is_open()) std::fprintf(stderr, "Cannot open %s\n", CSV_NAME);
        errorCsv.precision(15);
        errorCsv << "elapsed_seconds,elapsed_days,earth_ssb_error_km,earth_heliocentric_error_km,"
                    "saturn_ssb_error_km,saturn_heliocentric_error_km\n";
    };
    openCsv();

    // Push one trail point for every body at a given ET (used at t=0 and per day)
    auto sampleTrails = [&](double et) {
        for (int i = 0; i < n; ++i) {
            views[i].simTrail.push_back(world.Bodies[i].GetPosition());
            if ((int)views[i].simTrail.size() > MAX_TRAIL_POINTS) views[i].simTrail.pop_front();

            Vec3 sp; std::string err;
            if (querySpice(views[i].spiceTarget, et, sp, err)) {
                views[i].spiceTrail.push_back(sp);
                if ((int)views[i].spiceTrail.size() > MAX_TRAIL_POINTS) views[i].spiceTrail.pop_front();
            } else if (coverageError.empty()) {
                coverageError = std::string("Trail SPICE query failed | target=") + views[i].spiceTarget +
                                "  ET=" + std::to_string(et) + " s  | " + err;
            }
        }
    };

    auto sampleError = [&](double et) {
        if (idxSun < 0 || idxEarth < 0 || idxSaturn < 0) return;

        Vec3 sunSp, earthSp, saturnSp; std::string err;
        if (!querySpice(views[idxSun].spiceTarget,    et, sunSp,    err)) return;
        if (!querySpice(views[idxEarth].spiceTarget,  et, earthSp,  err)) return;
        if (!querySpice(views[idxSaturn].spiceTarget, et, saturnSp, err)) return;

        const Vec3 sunSim    = world.Bodies[idxSun].GetPosition();
        const Vec3 earthSim  = world.Bodies[idxEarth].GetPosition();
        const Vec3 saturnSim = world.Bodies[idxSaturn].GetPosition();

        const double earthSsbKm    = (earthSim  - earthSp ).Length() / 1000.0;
        const double saturnSsbKm   = (saturnSim - saturnSp).Length() / 1000.0;
        const double earthHelioKm  = ((earthSim  - sunSim) - (earthSp  - sunSp)).Length() / 1000.0;
        const double saturnHelioKm = ((saturnSim - sunSim) - (saturnSp - sunSp)).Length() / 1000.0;

        errorCsv << elapsed << ','
                 << elapsed / 86400.0 << ','
                 << earthSsbKm << ','
                 << earthHelioKm << ','
                 << saturnSsbKm << ','
                 << saturnHelioKm << '\n';
        errorCsv.flush();
    };

    // First trail point really is at elapsed = 0.
    sampleTrails(initialET);

    InitWindow(WINDOW, WINDOW, "Cassini Physical Solar System [RK4]: simulation vs SPICE");
    SetTargetFPS(60);

    while (!WindowShouldClose()) {
        if (IsKeyPressed(KEY_SPACE)) paused = !paused;
        if (IsKeyPressed(KEY_L))     logMode = !logMode;
        if (IsKeyPressed(KEY_R)) {
            initScene(world, views, initialET);
            n = (int)views.size();
            idxSun = findIndex("SUN");
            idxEarth = findIndex("EARTH");
            idxSaturn = findIndex("SATURN");
            elapsed = 0.0; accumulator = 0.0;
            lastTrailSample = 0.0;
            nextErrorSample = ERROR_SAMPLE_SECONDS;
            coverageError.clear(); paused = false;
            spiceNow.assign(n, Vec3()); spiceNowOk.assign(n, false);
            buildTopology();           // body list can change -> rebuild parent/focus tables
            focusIndex = -1;
            openCsv();                 // fresh file + header, do not append to the previous run
            sampleTrails(initialET);   // t = 0 point again
        }

        if (IsKeyPressed(KEY_F) && !focusable.empty()) {
            int slot = -1;
            for (int k = 0; k < (int)focusable.size(); ++k) if (focusable[k] == focusIndex) { slot = k; break; }
            ++slot;
            focusIndex = (slot >= (int)focusable.size()) ? -1 : focusable[slot];
            zoom = 1.0; panX = 0.0; panY = 0.0;   // reset view when switching
        }
        if (IsKeyPressed(KEY_EQUAL) || IsKeyPressed(KEY_KP_ADD))      simDaysPerSecond *= 1.5;
        if (IsKeyPressed(KEY_MINUS) || IsKeyPressed(KEY_KP_SUBTRACT)) simDaysPerSecond /= 1.5;
        double wheel = GetMouseWheelMove();
        if (wheel != 0.0) { zoom *= std::pow(1.1, wheel); if (zoom < 0.05) zoom = 0.05; if (zoom > 80.0) zoom = 80.0; }
        if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) { Vector2 d = GetMouseDelta(); panX += d.x; panY += d.y; }

        if (!paused && coverageError.empty()) {
            double dtReal = GetFrameTime();
            // avoid spiral-of-death after a stall
            if (dtReal > 0.1) dtReal = 0.1;
            // real seconds -> simulated seconds
            accumulator += dtReal * simDaysPerSecond * 86400.0;

            int guard = 0;
            while (accumulator >= PHYSICS_STEP_SECONDS &&
                   elapsed < MAX_ELAPSED_SECONDS &&
                   coverageError.empty() && guard < 5000) {

                // RK4 step
                world.Euler.RungeKutta4Integration(world.Bodies, world.GravityModel, (float)PHYSICS_STEP_SECONDS);
                elapsed     += PHYSICS_STEP_SECONDS;
                accumulator -= PHYSICS_STEP_SECONDS;
                ++guard;

                // Error sampling is driven by simulated time, not by frames:
                // 86400 / 450 = 192 steps, so rows land exactly on day
                if (elapsed >= nextErrorSample) {
                    sampleError(initialET + elapsed);
                    nextErrorSample += ERROR_SAMPLE_SECONDS;
                }

                if (elapsed - lastTrailSample >= TRAIL_SAMPLE_SECONDS) {
                    sampleTrails(initialET + elapsed);
                    lastTrailSample = elapsed;
                }
            }
        }

        double etNow = initialET + elapsed;
        for (int i = 0; i < n; ++i) {
            Vec3 sp; std::string err;
            spiceNowOk[i] = querySpice(views[i].spiceTarget, etNow, sp, err);
            if (spiceNowOk[i]) spiceNow[i] = sp;
            else if (coverageError.empty())
                coverageError = std::string("SPICE query failed | target=") + views[i].spiceTarget +
                                "  ET=" + std::to_string(etNow) + " s  | " + err;
        }

        double earthErrKm  = -1.0, saturnErrKm = -1.0;
        if (idxEarth  >= 0 && spiceNowOk[idxEarth])
            earthErrKm  = (world.Bodies[idxEarth ].GetPosition() - spiceNow[idxEarth ]).Length() / 1000.0;
        if (idxSaturn >= 0 && spiceNowOk[idxSaturn])
            saturnErrKm = (world.Bodies[idxSaturn].GetPosition() - spiceNow[idxSaturn]).Length() / 1000.0;

        char utc[40] = "??";
        et2utc_c(etNow, "ISOC", 3, sizeof(utc), utc);
        if (failed_c()) reset_c();

        double focusScale = 1.0;
        if (focusIndex >= 0) {
            const Vec3 fp = world.Bodies[focusIndex].GetPosition();
            double maxR = 1.0;
            for (int i = 0; i < n; ++i) {
                if (parentOf[i] != focusIndex) continue;
                const double r = (world.Bodies[i].GetPosition() - fp).Length();
                if (r > maxR) maxR = r;
            }
            focusScale = (WINDOW * 0.38) / maxR;
        }
        auto drawn = [&](int i) {
            return (focusIndex < 0) ? (parentOf[i] < 0)
                                    : (i == focusIndex || parentOf[i] == focusIndex);
        };

        BeginDrawing();
        ClearBackground(Color{ 10, 10, 18, 255 });

        if (focusIndex < 0) DrawCircleV(worldToScreen(Vec3(0, 0, 0)), 2.0f, GRAY);   // SSB

        for (int i = 0; i < n; ++i) {
            if (!drawn(i)) continue;

            const auto& st  = views[i].simTrail;
            const auto& fst = views[focusIndex < 0 ? i : focusIndex].simTrail;
            size_t nst = (focusIndex < 0) ? st.size() : std::min(st.size(), fst.size());
            for (size_t k = 1; k < nst; ++k) {
                float a = (float)k / (float)nst;
                Vector2 p0 = (focusIndex < 0) ? worldToScreen(st[k - 1]) : focusScreen(st[k - 1], fst[k - 1], focusScale);
                Vector2 p1 = (focusIndex < 0) ? worldToScreen(st[k])     : focusScreen(st[k],     fst[k],     focusScale);
                DrawLineV(p0, p1, Fade(SKYBLUE, 0.12f + 0.6f * a));
            }
            const auto& ot  = views[i].spiceTrail;
            const auto& fot = views[focusIndex < 0 ? i : focusIndex].spiceTrail;
            size_t notn = (focusIndex < 0) ? ot.size() : std::min(ot.size(), fot.size());
            for (size_t k = 1; k < notn; ++k) {
                float a = (float)k / (float)notn;
                Vector2 p0 = (focusIndex < 0) ? worldToScreen(ot[k - 1]) : focusScreen(ot[k - 1], fot[k - 1], focusScale);
                Vector2 p1 = (focusIndex < 0) ? worldToScreen(ot[k])     : focusScreen(ot[k],     fot[k],     focusScale);
                DrawLineV(p0, p1, Fade(ORANGE, 0.12f + 0.6f * a));
            }
        }

        for (int i = 0; i < n; ++i) {
            if (!drawn(i)) continue;
            const Vec3 fSim   = (focusIndex < 0) ? Vec3() : world.Bodies[focusIndex].GetPosition();
            const Vec3 fSpice = (focusIndex < 0) ? Vec3() : spiceNow[focusIndex];
            Vector2 simS = (focusIndex < 0) ? worldToScreen(world.Bodies[i].GetPosition())
                                            : focusScreen(world.Bodies[i].GetPosition(), fSim, focusScale);
            DrawCircleV(simS, 4.0f, SKYBLUE);
            if (spiceNowOk[i]) {
                Vector2 spS = (focusIndex < 0) ? worldToScreen(spiceNow[i])
                                               : focusScreen(spiceNow[i], fSpice, focusScale);
                DrawCircleV(spS, 4.0f, ORANGE);
            }
            DrawText(views[i].name.c_str(), (int)simS.x + 6, (int)simS.y - 6, 12, RAYWHITE);
        }

        DrawRectangle(0, 0, 470, 210, Color{ 0, 0, 0, 170 });
        int y = 8; const int dy = 18;
        DrawText("Legend:  SIM = cyan   SPICE reference = orange   [RK4]", 10, y, 14, RAYWHITE); y += dy;
        DrawText(TextFormat("Start epoch : %s UTC   Bodies: %d (physical)", INITIAL_EPOCH, n), 10, y, 14, RAYWHITE); y += dy;
        DrawText(TextFormat("Elapsed     : %.3f days   (%.0f s)", elapsed / 86400.0, elapsed), 10, y, 14, RAYWHITE); y += dy;
        DrawText(TextFormat("SPICE epoch : %s  (ET %.1f)", utc, etNow), 10, y, 14, RAYWHITE); y += dy;
        DrawText(TextFormat("Integrator  : RK4    Physics step: %.0f s   Speed: %.2f sim-days/s", PHYSICS_STEP_SECONDS, simDaysPerSecond), 10, y, 14, RAYWHITE); y += dy;
        DrawText(TextFormat("View: %s   Scale: %s   Zoom: %.2f   %s",
                            focusIndex < 0 ? "SOLAR SYSTEM" : views[focusIndex].name.c_str(),
                            logMode ? "LOG" : "LINEAR", zoom, paused ? "[PAUSED]" : "[running]"),
                 10, y, 14, focusIndex < 0 ? (paused ? YELLOW : RAYWHITE) : GREEN); y += dy;
        DrawText(earthErrKm  >= 0 ? TextFormat("EARTH  sim-vs-SPICE error : %12.1f km", earthErrKm)  : "EARTH  error : n/a",
                 10, y, 14, SKYBLUE); y += dy;
        DrawText(saturnErrKm >= 0 ? TextFormat("SATURN sim-vs-SPICE error : %12.1f km", saturnErrKm) : "SATURN error : n/a",
                 10, y, 14, ORANGE); y += dy;
        DrawText("F focus planet system (see moons) | SPACE pause | R restart | L lin/log | +/- speed | wheel zoom | drag pan | ESC",
                 10, y, 12, LIGHTGRAY); y += dy;

        if (!coverageError.empty()) {
            DrawRectangle(0, WINDOW - 46, WINDOW, 46, Color{ 60, 0, 0, 200 });
            DrawText("SPICE COVERAGE / ERROR (simulation frozen):", 10, WINDOW - 42, 14, RED);
            DrawText(coverageError.c_str(), 10, WINDOW - 22, 12, Color{ 255, 180, 180, 255 });
        }

        EndDrawing();
    }

    kclear_c();
    CloseWindow();
    return 0;
}
