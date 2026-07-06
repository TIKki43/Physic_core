#pragma once

#include "Ray.h"
#include "Constants.h"
#include <vector>


class RayTracer{
public:
    void AddRay(const Ray& Ray);
    void RaySimulationStep(float Deltatime = Constants::DefaultDeltaTime);

    std::vector<Ray> Rays;
};