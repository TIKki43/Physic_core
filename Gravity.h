#pragma once

#include "Body.h"
#include "Vec3.h"

class Gravity{
public:
    Vec3 ComputeGravityForce(const Body& BodyA, const Body& BodyB) const;
};