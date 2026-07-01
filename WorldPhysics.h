#pragma once

#include "Body.h"
#include "Gravity.h"
#include "Integrator.h"
#include <vector>

class WorldPhysics{
public:
    WorldPhysics();
    WorldPhysics(int Steps) : Steps(Steps) {}

    void StartSimulation();
    void SimulationStep();

    void AddBody(const Body& Some);
    void AddBody(Body&& Some);

    Integrator Euler;
    Gravity GravityModel;
    std::vector<Body> Bodies;
    int Steps = 10;
};