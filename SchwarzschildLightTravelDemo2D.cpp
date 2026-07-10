// g++ -std=c++17 -O2 -static-libgcc -static-libstdc++ -o SchwarzschildLightTravelDemo2D.exe SchwarzschildLightTravelDemo2D.cpp Ray.cpp RayTracer.cpp Body.cpp Vec3.cpp -lraylib -lopengl32 -lgdi32 -lwinmm


#include "Vec3.h"
#include "Body.h"
#include "Ray.h"
#include "RayTracer.h"
#include "Constants.h"

#include <iostream>
#include <algorithm>
#include <array>
#include <fstream>
#include <cmath>

#define Ray RaylibRay
#include "raylib.h"
#undef Ray

namespace Demo {
    constexpr int WindowSize = 800;
    constexpr float PixelsPerUnit = 2.0f; // visible half-width ~200 sim units

    // Black hole: M_geo = G*M/c^2 = M (G=c=1) Rs = 2*M_geo
    constexpr float BlackHoleMass = 8.0f;   // M_geo = 8 -> Rs = 16 units (32px)

    // Photon conserved quantities. Impact parameter b = L/E
    constexpr float RayEnergy = 1.0f;
    constexpr float RayAngularMomentum = 55.0f;

    constexpr float RayDLambda = 0.05f;
    constexpr int RaySubsteps = 10;

    constexpr int TrailLength = 400;

    float ScreenX(float simX) { return WindowSize / 2.0f + simX * PixelsPerUnit; }
    float ScreenY(float simY) { return WindowSize / 2.0f - simY * PixelsPerUnit; }
}

int main() {
    // Static central black hole at the origin
    Body BlackHole(Vec3(0.0f, 0.0f, 0.0f), Vec3(0.0f, 0.0f, 0.0f), Demo::BlackHoleMass, Vec3(), "BlackHole");

    // Photon starts far to the left, aimed toward the hole(+x). Ray ctor takes Vec3&,
    // so use named lvalues, not temporaries.
    Vec3 RayPos(-190.0f, 0.0f, 0.0f);
    Vec3 RayDir(1.0f, 0.0f, 0.0f);
    RayDir.Normalize();
    Ray Photon(1.0f, RayPos, RayDir, 1.0f, 1.0f);

    RayTracer Tracer;
    Tracer.AddRay(Photon);

    Tracer.Rays[0].InitSchwarzschildMetricState(
        BlackHole,
        Demo::RayEnergy,
        Demo::RayAngularMomentum,
        0.0f,
        Constants::c,
        Constants::G,
        Demo::RayDLambda);

    const float Rs = 2.0f * (Constants::G * Demo::BlackHoleMass / (Constants::c * Constants::c));

    std::array<Vec3, Demo::TrailLength> RayTrail{};
    int TrailCount = 0;
    int TrailHead = 0;

    // std::ofstream CsvOut("schwarzschild_ray.csv");
    // CsvOut << "step,x,y,r,phi\n";
    int StepCounter = 0;

    InitWindow(Demo::WindowSize, Demo::WindowSize, "Schwarzschild Light Travel Demo 2D");
    SetTargetFPS(60);

    while (!WindowShouldClose()) {
        for (int Substep = 0; Substep < Demo::RaySubsteps; ++Substep) {
            if (Tracer.Rays[0].Active) {
                Tracer.SchwarzschildSimulationStep(BlackHole);
            }
        }

        const Vec3 RayNowPos = Tracer.Rays[0].GetPosition();
        const float RayR = RayNowPos.Distance(BlackHole.GetPosition());

        RayTrail[TrailHead] = RayNowPos;
        TrailHead = (TrailHead + 1) % Demo::TrailLength;
        TrailCount = std::min(TrailCount + 1, Demo::TrailLength);

        // CsvOut << StepCounter++ << "," << RayNowPos.x << "," << RayNowPos.y
        //        << "," << RayR << "," << std::atan2(RayNowPos.y, RayNowPos.x) << "\n";

        BeginDrawing();
        ClearBackground(BLACK);

        const Vec3 BhPos = BlackHole.GetPosition();
        const Vector2 BhScreen{ Demo::ScreenX(BhPos.x), Demo::ScreenY(BhPos.y) };

        // Photon sphere (1.5*Rs = 3*M_geo) as a faint reference ring
        DrawCircleLines((int)BhScreen.x, (int)BhScreen.y, 1.5f * Rs * Demo::PixelsPerUnit, DARKGRAY);

        DrawCircleV(BhScreen, Rs * Demo::PixelsPerUnit, BLACK);
        DrawCircleLines((int)BhScreen.x, (int)BhScreen.y, Rs * Demo::PixelsPerUnit, ORANGE);

        // Ray trail.
        for (int i = 0; i < TrailCount; ++i) {
            const Vec3& TrailPos = RayTrail[i];
            DrawCircleV(Vector2{ Demo::ScreenX(TrailPos.x), Demo::ScreenY(TrailPos.y) }, 1.5f, GREEN);
        }

        // Current photon position + direction indicator
        const Vector2 RayScreen{ Demo::ScreenX(RayNowPos.x), Demo::ScreenY(RayNowPos.y) };
        DrawCircleV(RayScreen, 4.0f, RED);
        const Vec3 RayDirNow = Tracer.Rays[0].GetDirection();
        DrawLineV(RayScreen,
                  Vector2{ RayScreen.x + RayDirNow.x * 25.0f, RayScreen.y - RayDirNow.y * 25.0f },
                  YELLOW);

        DrawText(TextFormat("r to hole: %.1f (Rs=%.1f)", RayR, Rs), 10, 10, 20, RAYWHITE);
        DrawText(TextFormat("Status: %s", Tracer.Rays[0].Active ? "TRAVELLING" : "CAPTURED"), 10, 35, 20, RAYWHITE);
        DrawText(TextFormat("Travel time: %.2f", Tracer.Rays[0].GetTravelTime()), 10, 60, 20, RAYWHITE);

        EndDrawing();
    }

    // CsvOut.close();
    CloseWindow();
    return 0;
}
