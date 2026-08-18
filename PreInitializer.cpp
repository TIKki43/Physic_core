#include "PreInitializer.h"
#include "Mat3.h"
#include "Utils.h"
#include <cmath>
#include <stdexcept>
#include <iostream>
#include <array>
#include <span>

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
    for (std::size_t j{}; j < 15; ++j){
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


void PreInitializer::Missions::Cassini::SetBarycenterSolarSystem
                                            (      WorldPhysics& World
                                            , const std::string& FilePath // std::filesystem::path
                                            , const std::string& Epoch   // std::chrono
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

    for (std::size_t i{}; i < 9; ++i) {
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

    for (std::size_t i{}; i < 9; i++){
        Vec3 position(planets[i].state[0] * 1000.0, planets[i].state[1] * 1000.0, planets[i].state[2] * 1000.0); // m
        Vec3 velocity(planets[i].state[3] * 1000.0, planets[i].state[4] * 1000.0, planets[i].state[5] * 1000.0); // m / s
        double MU_SI = planets[i].GM * 1e9; // m^3 * s^-2
        Body Object(velocity, position, MU_SI / Constants::G, Vec3(), 0.0, 0.0, Vec3(), planets[i].bary);
        World.AddBody(Object);
    }

    kclear_c();
}

void PreInitializer::Missions::Cassini::SetPhysicalSolarSystem
                                                    (      WorldPhysics &World
                                                    , const std::string &FilePath
                                                    , const std::string &Epoch
                                                    , const std::string &Frame
                                                    , const std::string &Abcorr
                                                    , const std::string &Observer
                                                    )
                                    
{
    SpiceChar noErrorPrint[]{"NONE"};
    SpiceChar returnAction[]{"RETURN"};

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
    SpiceDouble et{};
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
    
    struct SpaceObject { 
        const char* name; 

        SpiceDouble GM{}; 
        SpiceDouble state[6]{};  // [0-2]pos km / [3-5]velocity km / s

        double J2{};
        double RefRadius{};
    };

    std::array SunSystem{SpaceObject{
        .name="SUN", 
        .J2=Constants::AstroDynamicConstants::SunJ2, 
        .RefRadius=Constants::AstroDynamicConstants::SunRefRadius
    }};
    std::array MercurySystem{SpaceObject{
        .name="MERCURY",
        .J2=Constants::AstroDynamicConstants::MercuryJ2, 
        .RefRadius=Constants::AstroDynamicConstants::MercuryRefRadius
    }};
    std::array VenusSystem{SpaceObject{
        .name="VENUS",
        .J2=Constants::AstroDynamicConstants::VenusJ2, 
        .RefRadius=Constants::AstroDynamicConstants::VenusRefRadius
    }};
    std::array EarthSystem{SpaceObject{
        .name="EARTH",
        .J2=Constants::AstroDynamicConstants::EarthJ2, 
        .RefRadius=Constants::AstroDynamicConstants::EarthRefRadius
    }                       , SpaceObject{"MOON"}};
    std::array MarsSystem{SpaceObject{
        .name="MARS",
        .J2=Constants::AstroDynamicConstants::MarsJ2, 
        .RefRadius=Constants::AstroDynamicConstants::MarsRefRadius
    }                       , SpaceObject{"PHOBOS"}, SpaceObject{"DEIMOS"}};

    std::array JupiterSystem{ SpaceObject{
        .name="JUPITER",
        .J2=Constants::AstroDynamicConstants::JupiterJ2, 
        .RefRadius=Constants::AstroDynamicConstants::JupiterRefRadius
    }                       , SpaceObject{"IO"}
                            , SpaceObject{"EUROPA"}, SpaceObject{"GANYMEDE"}
                            , SpaceObject{"CALLISTO"}, SpaceObject{"AMALTHEA"}
                            , SpaceObject{"THEBE"}, SpaceObject{"ADRASTEA"}
                            , SpaceObject{"METIS"}};

    std::array SaturnSystem{ SpaceObject{
        .name="SATURN",
        .J2=Constants::AstroDynamicConstants::SaturnJ2, 
        .RefRadius=Constants::AstroDynamicConstants::SaturnRefRadius
    }                      , SpaceObject{"MIMAS"}
                           , SpaceObject{"ENCELADUS"}, SpaceObject{"TETHYS"}
                           , SpaceObject{"DIONE"}, SpaceObject{"RHEA"}, SpaceObject{"TITAN"}
                           , SpaceObject{"HYPERION"}, SpaceObject{"IAPETUS"}
                           , SpaceObject{"PHOEBE"}, SpaceObject{"JANUS"}
                           , SpaceObject{"EPIMETHEUS"}, SpaceObject{"HELENE"}
                           , SpaceObject{"ATLAS"}, SpaceObject{"PROMETHEUS"}
                           , SpaceObject{"PANDORA"}};

    std::array UranusSystem{ SpaceObject{
        .name="URANUS",
        .J2=Constants::AstroDynamicConstants::UranusJ2, 
        .RefRadius=Constants::AstroDynamicConstants::UranusRefRadius
    }                      , SpaceObject{"ARIEL"}
                           , SpaceObject{"UMBRIEL"}, SpaceObject{"TITANIA"}
                           , SpaceObject{"OBERON"}, SpaceObject{"MIRANDA"}};

    std::array NeptuneSystem{SpaceObject{
        .name="NEPTUNE",
        .J2=Constants::AstroDynamicConstants::NeptuneJ2, 
        .RefRadius=Constants::AstroDynamicConstants::NeptuneRefRadius
    }, SpaceObject{"TRITON"}};

    std::array PlutoSystem{ SpaceObject{"PLUTO"}, SpaceObject{"CHARON"}
                          , SpaceObject{"NIX"}, SpaceObject{"HYDRA"}
                          , SpaceObject{"KERBEROS"}};

    std::array<std::span<SpaceObject>, 10> Systems{
        std::span{SunSystem},
        std::span{MercurySystem},
        std::span{VenusSystem},
        std::span{EarthSystem},
        std::span{MarsSystem},
        std::span{JupiterSystem},
        std::span{SaturnSystem},
        std::span{UranusSystem},
        std::span{NeptuneSystem},
        std::span{PlutoSystem},
    };
    
    for (auto system: Systems){
        for (SpaceObject& obj: system){
            SpiceDouble lt{};
            SpiceInt dimension;
            spkezr_c(obj.name, et, frame, abcorr, observer, obj.state, &lt);
            if (failed_c()) {
                SpiceChar shortmsg[26];
                getmsg_c("SHORT", sizeof(shortmsg), shortmsg);
                reset_c();
                std::printf("%-18s : unavailable -> %s\n", obj.name, shortmsg);
                kclear_c();
                panic("No enough data to spkezr_c!");
            }

            std::printf("%-18s : X = %20.3f   Y = %20.3f   Z = %20.3f  [km]\n",
                        obj.name, obj.state[0], obj.state[1], obj.state[2]);

            bodvrd_c(obj.name, "GM", 1, &dimension, &obj.GM);
            if (failed_c()) {
                SpiceChar message[1024];
                getmsg_c("LONG", sizeof(message), message);
                reset_c();

                std::printf(
                    "%-18s : GM unavailable -> %s\n",
                    obj.name,
                    message
                );
                kclear_c();
                panic("No enough data to bodvrd_c!");
            }

            Vec3 position(obj.state[0] * 1000.0, obj.state[1] * 1000.0, obj.state[2] * 1000.0); // m
            Vec3 velocity(obj.state[3] * 1000.0, obj.state[4] * 1000.0, obj.state[5] * 1000.0); // m / s
            double MU_SI = obj.GM * 1e9; // m^3 * s^-2
            
            Vec3 SpinAxis{};
            if (obj.J2 != 0.0) {
                SpiceInt frameCode{};
                SpiceChar bodyFrame[64]{};
                SpiceBoolean found{};
                // getting name
                cnmfrm_c(obj.name, sizeof(bodyFrame), &frameCode, bodyFrame, &found);
                if (!found)
                {
                    std::printf("%-18s : body-fixed frame not found\n", obj.name);
                    kclear_c();
                    panic("Can't find body-fixed frame!");
                }
                // getting rotation in et
                SpiceDouble rotation[3][3]{};
                pxform_c(bodyFrame, frame, et, rotation);
                if (failed_c())
                {
                    SpiceChar message[1024];
                    getmsg_c("LONG", sizeof(message), message);
                    reset_c();

                    std::printf(
                        "%-18s : frame transformation unavailable -> %s\n",
                        obj.name,
                        message
                    );

                    kclear_c();
                    panic("Can't transform spin axis!");
                }

                SpiceDouble localSpinAxis[3]{0.0, 0.0, 1.0};

                SpiceDouble SpinAxisFrame[3]{};
                mxv_c(rotation, localSpinAxis,  SpinAxisFrame);

                SpinAxis = Vec3{
                    SpinAxisFrame[0],
                    SpinAxisFrame[1],
                    SpinAxisFrame[2]
                };

                SpinAxis.Normalize();
            }

            Body Object(velocity, position, MU_SI / Constants::G, Vec3(), obj.J2, obj.RefRadius, SpinAxis, obj.name);
            World.AddBody(Object);

        }
    }

}

void PreInitializer::Missions::Cassini::SetMajorAsteroids
                                            (      WorldPhysics& World
                                            , const std::string& FilePath // std::filesystem::path
                                            , const std::string& Epoch   // std::chrono
                                            , const std::string& Frame
                                            , const std::string& Abcorr
                                            , const std::string& Observer
                                            )
{   
   

    SpiceChar noErrorPrint[]{"NONE"};
    SpiceChar returnAction[]{"RETURN"};
    
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
    SpiceDouble et{};
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

    
    struct Asteroid { 
        const SpiceChar* ID; 
        SpiceInt ID_gm; 
        const SpiceChar* Name;

        SpiceDouble GM{}; 
        SpiceDouble State[6]{};  // [0-2]pos km / [3-5]velocity km / s
    };

    std::array Asteroids{
        Asteroid{"20000001", 2000001, "Ceres"},
        Asteroid{"20000002", 2000002, "Pallas"},
        Asteroid{"20000003", 2000003, "Juno"},
        Asteroid{"20000004", 2000004, "Vesta"},
        Asteroid{"20000007", 2000007, "Iris"},
        Asteroid{"20000010", 2000010, "Hygiea"},
        Asteroid{"20000015", 2000015, "Eunomia"},
        Asteroid{"20000016", 2000016, "Psyche"},
        Asteroid{"20000031", 2000031, "Euphrosyne"},
        Asteroid{"20000052", 2000052, "Europa_Asteroid"},
        Asteroid{"20000065", 2000065, "Cybele"},
        Asteroid{"20000087", 2000087, "Sylvia"},
        Asteroid{"20000088", 2000088, "Thisbe"},
        Asteroid{"20000107", 2000107, "Camilla"},
        Asteroid{"20000511", 2000511, "Davida"},
        Asteroid{"20000704", 2000704, "Interamnia"},
    };

    for (std::size_t i{}; i < Asteroids.size(); ++i) {
        SpiceDouble lt{};
        SpiceInt dimension{};
        spkezr_c(Asteroids[i].ID, et, frame, abcorr, observer, Asteroids[i].State, &lt);
        if (failed_c()) {
            SpiceChar shortmsg[26];
            getmsg_c("SHORT", sizeof(shortmsg), shortmsg);
            reset_c();
            std::printf("%-18s : unavailable -> %s\n", Asteroids[i].Name, shortmsg);
            kclear_c();
            panic("No enough data to spkezr_c!");
        }

        std::printf("%-18s : X = %20.3f   Y = %20.3f   Z = %20.3f  [km]\n",
                    Asteroids[i].Name, Asteroids[i].State[0], Asteroids[i].State[1], Asteroids[i].State[2]);

        bodvcd_c(Asteroids[i].ID_gm, "GM", 1, &dimension, &Asteroids[i].GM);
        if (failed_c()) {
            SpiceChar message[1024];
            getmsg_c("LONG", sizeof(message), message);
            reset_c();

            std::printf(
                "%-18s : GM unavailable -> %s\n",
                Asteroids[i].Name,
                message
            );
            kclear_c();
            panic("No enough data to bodvcd_c!");
        }
    }

    for (std::size_t i{}; i < Asteroids.size(); i++){
        Vec3 position(Asteroids[i].State[0] * 1000.0, Asteroids[i].State[1] * 1000.0, Asteroids[i].State[2] * 1000.0); // m
        Vec3 velocity(Asteroids[i].State[3] * 1000.0, Asteroids[i].State[4] * 1000.0, Asteroids[i].State[5] * 1000.0); // m / s
        double MU_SI = Asteroids[i].GM * 1e9; // m^3 * s^-2
        Body Object(velocity, position, MU_SI / Constants::G, Vec3(), 0.0, 0.0, Vec3(), Asteroids[i].ID);
        World.AddBody(Object);
    }

}
