#include "Integrator.h"
#include "Body.h"
#include "Vec3.h"

void Integrator::EulerIntegration(Body& BodyA, const Vec3& Force, float DeltaTime){
    const Vec3 Acceleration = Force / BodyA.GetMass();
    BodyA.SetVelocity(BodyA.GetVelocity() + Acceleration * DeltaTime); // v = v + a * dt
    BodyA.SetPosition(BodyA.GetPosition() + BodyA.GetVelocity() * DeltaTime); // x = x + v * dt
}