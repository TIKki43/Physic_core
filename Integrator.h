#pragma once 

#include "Vec3.h"
#include "Body.h"

class Integrator{
public:
    Integrator() = default();

    void EulerIntegration(Body& AnyBody, const Vec3& Force);
private:
    float dt = 0.001;

};