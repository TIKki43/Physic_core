#pragma once

#include "Body.h"
#include "Constants.h"
#include "Gravity.h"
#include "Integrator.h"
#include <vector>

class WorldPhysics
{
public:
  WorldPhysics () = default;
  WorldPhysics (int Steps) : Steps (Steps) {}

  void StartSimulation ();
  void SimulationStep (float Deltatime = Constants::DefaultDeltaTime);

  void AddBody (const Body &Some);
  void AddBody (Body &&Some);

  Gravity GravityModel;
  std::vector<Body> Bodies;

  int Steps = 10;
};