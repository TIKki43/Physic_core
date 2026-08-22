#pragma once

#include "Body.h"
#include "Constants.h"
#include "Ray.h"
#include <vector>

class RayTracer
{
public:
  void AddRay (const Ray &Ray);
  void RaySimulationStep (float Deltatime = Constants::DefaultDeltaTime);
  void SchwarzschildSimulationStep (Body &MassiveBody);

  std::vector<Ray> Rays;
};
