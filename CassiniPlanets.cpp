//   g++ -std=c++17 -I deps/cspice/include CassiniPlanets.cpp deps/cspice/lib/libcspice.a -o CassiniPlanets.exe

#include <cstdio>
#include <iostream>
#include "SpiceUsr.h"
#include "Vec3.h"
#include "Body.h"
#include "Constants.h"

int main() {

    furnsh_c("cas_2005_local.tm");

    erract_c("SET", 0, (SpiceChar*)"RETURN");
    errprt_c("SET", 0, (SpiceChar*)"NONE");

    // Epoch: 2005-01-01T00:00:00 UTC  ->  ephemeris time (ET, seconds past J2000 TDB).
    SpiceDouble et = 0.0;
    str2et_c("2005-01-01T00:00:00", &et);

    const SpiceChar* frame    = "J2000";                 // inertial reference frame
    const SpiceChar* abcorr   = "NONE";                  // geometric(no aberration)
    const SpiceChar* observer = "SOLAR SYSTEM BARYCENTER"; // SSB observer

    struct Planet { 
        const char* name; 
        const char* bary; 

        SpiceDouble GM; 
        SpiceDouble state[6] = {0,0,0,0,0,0};  // km and km / s

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

    std::printf("Cassini/CSPICE planet positions\n");
    std::printf("Epoch    : 2005-01-01T00:00:00 UTC  (ET = %.6f s past J2000)\n", et);
    std::printf("Frame    : %s      Observer : %s      Aberration : %s\n",
                frame, observer, abcorr);
    std::printf("Meta-krnl: cas_2005_local.tm\n");
    std::printf("NOTE: CSPICE returns positions in KILOMETERS (km).\n");
    std::printf("Targets are planet CENTERS(e.g. JUPITER = 599, not barycenter 5).\n\n");

    for (int i = 0; i < 9; ++i) {
        SpiceDouble lt = 0.0;
        SpiceInt dimension;
        spkezr_c(planets[i].bary, et, frame, abcorr, observer, planets[i].state, &lt);
        
        
        if (failed_c()) {
            SpiceChar shortmsg[26];
            getmsg_c("SHORT", sizeof(shortmsg), shortmsg);
            reset_c();
            std::printf("%-18s : unavailable -> %s\n", planets[i].bary, shortmsg);
        } else {
            std::printf("%-18s : X = %20.3f   Y = %20.3f   Z = %20.3f  [km]\n",
                        planets[i].bary, planets[i].state[0], planets[i].state[1], planets[i].state[2]);
        }
        bodvrd_c(planets[i].bary, "GM", 1, &dimension, &planets[i].GM);
    }

    for (int i = 0; i < 9; i++){
        Vec3 position(planets[i].state[0] * 1000.0, planets[i].state[1] * 1000.0, planets[i].state[2] * 1000.0); // m
        Vec3 velocity(planets[i].state[3] * 1000.0, planets[i].state[4] * 1000.0, planets[i].state[5] * 1000.0); // m / s
        double MU_CI = planets[i].GM * 1e9; // m^3 * kg^-1 * s^-2
        Body Object(velocity, position, MU_CI / Constants::G, Vec3(), planets[i].name);
        
    }

    // planet CENTERS -------------------------------------------(No enough data)
    // for (int i = 0; i < 8; ++i) {
    //     SpiceDouble lt = 0.0;
    //     spkpos_c(planets[i].name, et, frame, abcorr, observer, planets[i].pos, &lt);

    //     if (failed_c()) {
    //         SpiceChar shortmsg[26];
    //         getmsg_c("SHORT", sizeof(shortmsg), shortmsg);
    //         reset_c();

    //         SpiceInt      id = 0;
    //         SpiceBoolean  found = SPICEFALSE;
    //         bodn2c_c(planets[i].name, &id, &found);

    //         std::printf("%-8s : NOT AVAILABLE as a center", planets[i].name);
    //         if (found) std::printf(" (NAIF ID %ld)", (long)id);
    //         std::printf(" -> %s\n", shortmsg);
    //         std::printf("           Not substituting the barycenter silently.\n");
    //     } else {
    //         std::printf("%-8s : X = %20.3f   Y = %20.3f   Z = %20.3f  [km]\n",
    //                     planets[i].name, pos[0], pos[1], pos[2]);
    //     }
    // }
    kclear_c();
    return 0;
}
