#include "Gravity.h"
#include "Body.h"
#include "Constants.h"

Vec3 Gravity::ComputeGravityForce(const Body& BodyA, const Body& BodyB) const {
    const Vec3 BodyAPos = BodyA.GetPosition();
    const Vec3 BodyBPos = BodyB.GetPosition();

    Vec3 NormRadialVector = (BodyBPos - BodyAPos); 
    NormRadialVector.Normalize(); // Normalized direction

    float r = (BodyAPos - BodyBPos).LengthSquared(); // Squared distance between two bodies

    float GravityForce = (Constants::G * (BodyA.GetMass() * BodyB.GetMass())) / r; 
    return NormRadialVector * GravityForce;
}