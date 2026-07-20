#include "PreInitializer.h"
#include "Mat3.h"
#include <cmath>
#include <stdexcept>
#include <iostream>

void PreInitializer::SetOrbit(
      Body& OrbitingBody
    , Body& CentralBody
    , const float a
    , const float e
    , const float i
    , const float w
    , const float om
    , const float M
    )
{
    
    float n;
    n = std::sqrt((Constants::G * CentralBody.GetMass()) / (a * a * a));

    float E = M; 
    // Newton-Raphson method
    // f = E - esin(E) - M
    // f' = 1 - ecos(E)
    for (int j = 0; j < 15; ++j){
        float f = E - e * std::sin(E) - M;
        float fp = 1.0f - e * std::cos(E);
        E = E - f/fp;
    }

    float posx_2d;
    float posy_2d;
    
    float vx_2d;
    float vy_2d;

    posx_2d = a * (std::cos(E) - e);
    posy_2d = a * std::sqrt(1 - e * e) * std::sin(E);
    
    vx_2d = -a * std::sin(E) * (n / (1 - e * std::cos(E)));
    vy_2d = a * std::sqrt(1 - e * e) * std::cos(E) * (n / (1 - e * std::cos(E)));

    Vec3 pos_3d(posx_2d, posy_2d, 0.0f);
    Vec3 v_3d(vx_2d, vy_2d, 0.0f);

    // Rotation matrix: 
    // (cos^2(x) - sin^2(x) cos(x) | -sin(x) cos^2(x) - sin(x) cos(x) | sin^2(x)
    // sin(x) cos^2(x) + sin(x) cos(x) | cos^3(x) - sin^2(x) | sin(x) (-cos(x))
    // sin^2(x) | sin(x) cos(x) | cos(x))
    Mat3 GeneralRotation = Mat3::RotationZ(om) * Mat3::RotationX(i) * Mat3::RotationZ(w);

    OrbitingBody.SetPosition(CentralBody.GetPosition() + GeneralRotation * pos_3d);
    OrbitingBody.SetVelocity(CentralBody.GetVelocity() + GeneralRotation * v_3d);
};

namespace{
    [[noreturn]] void panic(const std::string& message) {
    std::cerr << message << std::endl;
    std::abort();
    };
}

void PreInitializer::Missions::Cassini::SetBarycenterSolarSystem
                                            (      WorldPhysics& World
                                            , const std::string& FilePath
                                            , const std::string& Epoch
                                            , const std::string& Frame
                                            , const std::string& Abcorr
                                            , const std::string& Observer
                                            )
{   
   

    SpiceChar noErrorPrint[] = "NONE";
    SpiceChar returnAction[] = "RETURN";
    
    erract_c("SET", 0, returnAction);
    errprt_c("SET", 0, noErrorPrint);

    // Reading meta-kernel
    furnsh_c(FilePath.c_str());
    if (failed_c()) {
        SpiceChar message[1024];
        getmsg_c("LONG", sizeof(message), message);
        reset_c();
        kclear_c();

        std::printf("Meta-kernel error: %s\n", message);
        panic("Can't read meta-kernel!");
    }
    

    // Epoch: 2005-01-01T00:00:00 UTC  -> ephemeris time(ET, seconds past J2000 TDB)
    SpiceDouble et = 0.0;
    str2et_c(Epoch.c_str(), &et);
    if (failed_c()) {
        SpiceChar message[1024];
        getmsg_c("LONG", sizeof(message), message);
        reset_c();
        kclear_c();

        std::printf("Epoch error: %s\n", message);
        panic("Can't find epoch in time intervals!");
    }

    
    // (J2000) inertial reference frame
    const SpiceChar* frame = Frame.c_str();
    // (None) geometric(no aberration)                 
    const SpiceChar* abcorr = Abcorr.c_str();
    // SSB observer                  
    const SpiceChar* observer = Observer.c_str(); 

    
    struct Planet { 
        const char* name; 
        const char* bary; 

        SpiceDouble GM{}; 
        SpiceDouble state[6]{};  // [0-2]pos km / [3-5]velocity km / s
    };

    Planet planets[9] = {
        {"SUN", "SUN"},
        {"MERCURY", "MERCURY BARYCENTER"},
        {"VENUS",   "VENUS BARYCENTER"},
        {"EARTH",   "EARTH BARYCENTER"},
        {"MARS",    "MARS BARYCENTER"},
        {"JUPITER", "JUPITER BARYCENTER"},
        {"SATURN",  "SATURN BARYCENTER"},
        {"URANUS",  "URANUS BARYCENTER"},
        {"NEPTUNE", "NEPTUNE BARYCENTER"},
    };

    for (int i = 0; i < 9; ++i) {
        SpiceDouble lt = 0.0;
        SpiceInt dimension;
        spkezr_c(planets[i].bary, et, frame, abcorr, observer, planets[i].state, &lt);
        if (failed_c()) {
            SpiceChar shortmsg[26];
            getmsg_c("SHORT", sizeof(shortmsg), shortmsg);
            reset_c();
            std::printf("%-18s : unavailable -> %s\n", planets[i].bary, shortmsg);
            kclear_c();
            panic("No enough data to spkezr_c!");
        }

        std::printf("%-18s : X = %20.3f   Y = %20.3f   Z = %20.3f  [km]\n",
                    planets[i].bary, planets[i].state[0], planets[i].state[1], planets[i].state[2]);

        bodvrd_c(planets[i].bary, "GM", 1, &dimension, &planets[i].GM);
        if (failed_c()) {
            SpiceChar message[1024];
            getmsg_c("LONG", sizeof(message), message);
            reset_c();

            std::printf(
                "%-18s : GM unavailable -> %s\n",
                planets[i].bary,
                message
            );
            kclear_c();
            panic("No enough data to bodvrd_c!");
        }
    }

    for (int i = 0; i < 9; i++){
        Vec3 position(planets[i].state[0] * 1000.0, planets[i].state[1] * 1000.0, planets[i].state[2] * 1000.0); // m
        Vec3 velocity(planets[i].state[3] * 1000.0, planets[i].state[4] * 1000.0, planets[i].state[5] * 1000.0); // m / s
        double MU_SI = planets[i].GM * 1e9; // m^3 * s^-2
        Body Object(velocity, position, MU_SI / Constants::G, Vec3(), planets[i].name);
        World.AddBody(Object);
    }

    kclear_c();
}   
