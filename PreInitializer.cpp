#include "PreInitializer.h"
#include "Mat3.h"
#include <cmath>
#include "Constants.h"

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