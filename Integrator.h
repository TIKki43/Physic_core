#pragma once

#include "Vec3.h"
#include "Body.h"
#include "Gravity.h"
#include "Constants.h"
#include <vector>

namespace Integrator{
    void EulerIntegration(Body& AnyBody, const Vec3& Force, double DeltaTime = Constants::DefaultDeltaTime);

    void RungeKutta4Integration(std::vector<Body>& Bodies, Gravity& GravityModel, double DeltaTime = Constants::DefaultDeltaTime);
    void RungeKutta4Integration1PN(std::vector<Body>& Bodies, Gravity& GravityModel, double DeltaTime = Constants::DefaultDeltaTime);
    void RungeKutta4Integration1PNJ2(std::vector<Body> &Bodies, const Gravity &GravityModel, double DeltaTime = Constants::DefaultDeltaTime);
};