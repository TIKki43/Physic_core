#pragma once

#include "Vec3.h"
#include "Body.h"
#include "Gravity.h"
#include "Constants.h"
#include <vector>

class Integrator{
public:
    Integrator() = default;

    void EulerIntegration(Body& AnyBody, const Vec3& Force, float DeltaTime = Constants::DefaultDeltaTime);

    void RungeKutta4Integration(std::vector<Body>& Bodies, Gravity& GravityModel, float DeltaTime = Constants::DefaultDeltaTime);
    void RungeKutta4Integration1PN(std::vector<Body>& Bodies, Gravity& GravityModel, float DeltaTime = Constants::DefaultDeltaTime);
    void RungeKutta4Integration1PNJ2(std::vector<Body> &Bodies, const Gravity &GravityModel, float DeltaTime = Constants::DefaultDeltaTime);
};