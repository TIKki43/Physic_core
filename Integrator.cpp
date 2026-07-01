#include "Integrator.h"
#include "Body.h"
#include "Vec3.h"

void Integrator::EulerIntegration(Body& BodyA, const Vec3& Force){
    const Vec3 Acceleration = Force / BodyA.GetMass();
    BodyA.SetVelocity(BodyA.GetVelocity() + Acceleration * dt); // v = v + a * dt
    BodyA.SetPosition(BodyA.GetPosition() + BodyA.GetVelocity() * dt); // x = x + v * dt
}