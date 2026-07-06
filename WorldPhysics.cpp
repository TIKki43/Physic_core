#include "WorldPhysics.h"
#include "Body.h"
#include <vector>

// Main simulation's func
void WorldPhysics::StartSimulation(){ 
    for (int CurrentStep = 1; CurrentStep <= Steps; CurrentStep++) {
        SimulationStep();
    }
}

// NBody gravitational simulation step 
void WorldPhysics::SimulationStep(float DeltaTime){
    std::vector<Vec3> Forces(Bodies.size());
    for (int i = 0; i < Bodies.size(); ++i){
        for (int j = i + 1; j < Bodies.size(); ++j){
            Vec3 ForceAB = GravityModel.ComputeGravityForce(Bodies[i], Bodies[j]);
            Forces[i] += ForceAB;
            Forces[j] -= ForceAB;
        }
    }
    for (int i = 0; i < Bodies.size(); i++){
        Euler.EulerIntegration(Bodies[i], Forces[i], DeltaTime);
    }
}

void WorldPhysics::AddBody(const Body& Some){
    Bodies.push_back(Some);
}

void WorldPhysics::AddBody(Body&& Some){
    Bodies.push_back(std::move(Some));
}