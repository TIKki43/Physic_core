#pragma once

#include "Body.h"
#include "Vec3.h"
#include "SpiceUsr.h"
#include "Constants.h"
#include "WorldPhysics.h"

namespace PreInitializer{

    void SetOrbit(Body& OrbitingBody, Body& CentralBody, const float SemiMajorAxis, const float Eccentricity,
         const float Inclination, const float AscendingLongitude, const float Periapsis, const float MeanAnomaly);
    namespace Missions{
        namespace Cassini{
            void SetBarycenterSolarSystem(WorldPhysics& World, const std::string& FilePath,
                 const std::string& Epoch, const std::string& Frame = "J2000"
                 , const std::string& Abcorr = "NONE", const std::string& Observer = "SOLAR SYSTEM BARYCENTER");
        }
    }

};
