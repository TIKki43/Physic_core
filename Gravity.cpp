#include "Gravity.h"
#include "Body.h"
#include "Constants.h"

Vec3 Gravity::ComputeGravityForce(const Body& BodyA, const Body& BodyB) const {
    const Vec3 BodyAPos = BodyA.GetPosition();
    const Vec3 BodyBPos = BodyB.GetPosition();

    Vec3 NormRadialVector = (BodyBPos - BodyAPos); 
    NormRadialVector.Normalize(); // Normalized direction

    double r = (BodyAPos - BodyBPos).LengthSquared(); // Squared distance between two bodies

    double GravityForce = (BodyA.GetMass() * BodyB.GetMU()) / r; 
    return NormRadialVector * GravityForce;
}