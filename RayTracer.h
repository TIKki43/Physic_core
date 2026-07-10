#pragma once

#include "Ray.h"
#include "Body.h"
#include "Constants.h"
#include <vector>


class RayTracer{
public:
    void AddRay(const Ray& Ray);
    void RaySimulationStep(float Deltatime = Constants::DefaultDeltaTime);
    void SchwarzschildSimulationStep(Body& MassiveBody);

    std::vector<Ray> Rays;
};