// g++ -std=c++17 -O2 -o LightTravelDemo2D.exe LightTravelDemo2D.cpp Body.cpp Vec3.cpp Mat3.cpp Gravity.cpp Integrator.cpp WorldPhysics.cpp PreInitializer.cpp Ray.cpp RayTracer.cpp -lraylib -lopengl32 -lgdi32 -lwinmm

#include "Vec3.h"
#include "Body.h"
#include "PreInitializer.h"
#include "WorldPhysics.h"
#include "Ray.h"
#include "RayTracer.h"
#include "Constants.h"

#include <iostream>
#include <algorithm>
#include <array>

#define Ray RaylibRay
#include "raylib.h"
#undef Ray

namespace Demo {
    constexpr float SemiMajorAxis = 150.0f;
    constexpr float SunMass = 1000.0f;
    constexpr float EarthMass = 1.0f;

    // WaveSpeed vs orbital speed gives a ~10 sim-second Sun->Earth crossing,
    // during which Earth visibly advances ~10 degrees along its orbit.
    constexpr float RayWaveSpeed = 15.0f;

    // Sim-seconds advanced per wall-clock second: crossing ~1s real-time, orbit ~36.5s real-time.
    constexpr float TimeScale = 1.0f;

    constexpr int PhysicsSubsteps = 8;
    constexpr float MaxFrameDt = 0.05f;

    constexpr int WindowSize = 800;
    constexpr float PixelsPerUnit = 2.0f;
    constexpr int TrailLength = 240;

    float ScreenX(float simX) { return WindowSize / 2.0f + simX * PixelsPerUnit; }
    float ScreenY(float simY) { return WindowSize / 2.0f - simY * PixelsPerUnit; }
}

int main() {
    Body Sun(Vec3(0.0f, 0.0f, 0.0f), Vec3(0.0f, 0.0f, 0.0f), Demo::SunMass, Vec3(), "Sun");
    Body Earth(Vec3(0.0f, 0.0f, 0.0f), Vec3(0.0f, 0.0f, 0.0f), Demo::EarthMass, Vec3(), "Earth");

    PreInitializer Initializer;
    Initializer.SetOrbit(Earth, Sun, Demo::SemiMajorAxis, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

    const Vec3 SunStartPos = Sun.GetPosition();
    const Vec3 EarthStartPos = Earth.GetPosition();
    const float InitialDistance = SunStartPos.Distance(EarthStartPos);

    WorldPhysics World;
    World.AddBody(Sun);
    World.AddBody(Earth);

    Vec3 RayPos = SunStartPos;
    Vec3 RayDir = EarthStartPos - SunStartPos;
    RayDir.Normalize();
    Ray LightRay(1.0f, RayPos, RayDir, 1.0f, Demo::RayWaveSpeed);

    RayTracer Tracer;
    Tracer.AddRay(LightRay);

    std::array<Vec3, Demo::TrailLength> EarthTrail{};
    std::array<Vec3, Demo::TrailLength> RayTrail{};
    int TrailCount = 0;
    int TrailHead = 0;
    int TrailCount1 = 0;
    int TrailHead1 = 0;

    InitWindow(Demo::WindowSize, Demo::WindowSize, "Light Travel Demo 2D");
    SetTargetFPS(60);

    while (!WindowShouldClose()) {
        const float RawDt = std::min(GetFrameTime(), Demo::MaxFrameDt);
        const float FrameDt = RawDt * Demo::TimeScale;

        for (int Substep = 0; Substep < Demo::PhysicsSubsteps; ++Substep) {
            World.SimulationStep(FrameDt / Demo::PhysicsSubsteps);
        }

        if (Tracer.Rays[0].Active) {
            Tracer.RaySimulationStep(FrameDt);

            // if (Tracer.Rays[0].GetTravelDistance() >= InitialDistance) {
            //     Tracer.Rays[0].Active = false;
            //     std::cout << "Ray arrived. TravelTime=" << Tracer.Rays[0].GetTravelTime()
            //               << " TravelDistance=" << Tracer.Rays[0].GetTravelDistance() << "\n";
            // }
        }

        EarthTrail[TrailHead] = World.Bodies[1].GetPosition();
        TrailHead = (TrailHead + 1) % Demo::TrailLength;
        TrailCount = std::min(TrailCount + 1, Demo::TrailLength);

        RayTrail[TrailHead1] = Tracer.Rays[0].GetPosition();
        TrailHead1 = (TrailHead1 + 1) % Demo::TrailLength;
        TrailCount1 = std::min(TrailCount1 + 1, Demo::TrailLength); 

        BeginDrawing();
        ClearBackground(BLACK);

        for (int i = 0; i < TrailCount; ++i) {
            const Vec3& TrailPos = EarthTrail[i];
            DrawCircleV(Vector2{ Demo::ScreenX(TrailPos.x), Demo::ScreenY(TrailPos.y) }, 1.5f, DARKGRAY);
        }
         
        for (int i = 0; i < TrailCount1; ++i) {
            const Vec3& RayPos = RayTrail[i];
            DrawCircleV(Vector2{ Demo::ScreenX(RayPos.x), Demo::ScreenY(RayPos.y) }, 1.5f, GREEN);
        }

        const Vec3 SunPos = World.Bodies[0].GetPosition();
        DrawCircleV(Vector2{ Demo::ScreenX(SunPos.x), Demo::ScreenY(SunPos.y) }, 20.0f, YELLOW);

        const Vec3 EarthPos = World.Bodies[1].GetPosition();
        DrawCircleV(Vector2{ Demo::ScreenX(EarthPos.x), Demo::ScreenY(EarthPos.y) }, 6.0f, SKYBLUE);

        if (Tracer.Rays[0].Active) {
            const Vec3 RayNowPos = Tracer.Rays[0].GetPosition();
            DrawCircleV(Vector2{ Demo::ScreenX(RayNowPos.x), Demo::ScreenY(RayNowPos.y) }, 4.0f, RED);
        }

        DrawText(TextFormat("Ray active: %s", Tracer.Rays[0].Active ? "true" : "false"), 10, 10, 20, RAYWHITE);
        DrawText(TextFormat("Ray travel time: %.2f", Tracer.Rays[0].GetTravelTime()), 10, 35, 20, RAYWHITE);

        EndDrawing();
    }

    CloseWindow();
    return 0;
}
