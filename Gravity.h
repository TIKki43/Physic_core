#pragma once

#include "Body.h"
#include "Vec3.h"

class Gravity{
public:
    Gravity() = default;

    Vec3 ComputeGravityForce(const Body& BodyA, const Body& BodyB) const;
private:
    float G = 1.0;
};