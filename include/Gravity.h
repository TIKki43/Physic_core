#pragma once

#include "Body.h"
#include "Vec3.h"

class Gravity
{
public:
  Vec3 ComputeGravityForce (const Body &BodyA, const Body &BodyB) const;

  [[nodiscard]] Vec3 ComputeJ2Acceleration (const Vec3 &DistanceDiff,
                                            const double Mu, const double J2,
                                            const double RefRadius,
                                            const Vec3 &SpinAxis) const;
};