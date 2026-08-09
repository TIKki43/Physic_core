#pragma once

#include "Body.h"
#include "Vec3.h"
#include "SpiceUsr.h"
#include "Constants.h"
#include "WorldPhysics.h"

namespace PreInitializer{

    enum class Observers{

    };

    void SetOrbit(Body& OrbitingBody, Body& CentralBody, const float SemiMajorAxis, const float Eccentricity,
         const float Inclination, const float AscendingLongitude, const float Periapsis, const float MeanAnomaly);
    namespace Missions{
        namespace Cassini{
            // set 8 planet system barycenters and sun barycenter
            void SetBarycenterSolarSystem( WorldPhysics& World, const std::string& FilePath
                                         , const std::string& Epoch, const std::string& Frame = "J2000"
                                         , const std::string& Abcorr = "NONE", const std::string& Observer = "SOLAR SYSTEM BARYCENTER");
            
            // set 9 planets, 35 there moons and the sun
            void SetPhysicalSolarSystem( WorldPhysics& World, const std::string& FilePath
                                       , const std::string& Epoch, const std::string& Frame = "J2000"
                                       , const std::string& Abcorr = "None" , const std::string& Observer = "SOLAR SYSTEM BARYCENTER");

            // set 16 mass major asteroids 
            void SetMajorAsteroids     ( WorldPhysics& World, const std::string& FilePath
                                       , const std::string& Epoch, const std::string& Frame = "J2000"
                                       , const std::string& Abcorr = "None" , const std::string& Observer = "SOLAR SYSTEM BARYCENTER");

        }
    }

};
