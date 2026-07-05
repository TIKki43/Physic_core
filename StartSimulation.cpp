#include "WorldPhysics.h"
#include "PreInitializer.h"
#include "Body.h"
#include "Vec3.h"

#include <iostream>

// g++ -std=c++17 -o StartSimulation.exe StartSimulation.cpp Body.cpp Vec3.cpp Mat3.cpp Gravity.cpp Integrator.cpp WorldPhysics.cpp PreInitializer.cpp

int main() {
    Body Sun(Vec3(0.0f, 0.0f, 0.0f), Vec3(0.0f, 0.0f, 0.0f), 1000.0f, Vec3(), "Sun");
    Body Planet(Vec3(0.0f, 0.0f, 0.0f), Vec3(0.0f, 0.0f, 0.0f), 1.0f, Vec3(), "Planet");

    PreInitializer Initializer;
    Initializer.SetOrbit(Planet, Sun, 10.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

    WorldPhysics World;
    World.AddBody(Sun);
    World.AddBody(Planet);

    const int TotalSteps = 20000;
    const int PrintEvery = 2000;

    for (int Step = 1; Step <= TotalSteps; ++Step) {
        World.SimulationStep();

        if (Step % PrintEvery == 0) {
            const Vec3 PlanetPos = World.Bodies[1].GetPosition();
            std::cout << "Step " << Step << " Planet Position: ("
                      << PlanetPos.x << ", " << PlanetPos.y << ", " << PlanetPos.z << ")\n";
        }
    }

    return 0;
}
