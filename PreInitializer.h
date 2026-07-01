#pragma once

#include "Body.h"
#include "Vec3.h"

struct PreInitializer{

    void SetOrbit(Body& OrbitingBody, Body& CentralBody, const float SemiMajorAxis, const float Eccentricity,
         const float Inclination, const float AscendingLongitude, const float Periapsis, const float MeanAnomaly);


};