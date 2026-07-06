#pragma once 

#include "Vec3.h"
#include "Body.h"
#include "Constants.h"
class Integrator{
public:
    Integrator() = default;

    void EulerIntegration(Body& AnyBody, const Vec3& Force, float DeltaTime = Constants::DefaultDeltaTime);

};