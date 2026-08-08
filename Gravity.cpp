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

Vec3 Gravity::ComputeJ2Acceleration(const Vec3& DistanceDif, const double Mu, const double J2, const double RefRadius, const Vec3 &SpinAxis) const
{
    double r2 = DistanceDif.Dot(DistanceDif);
    double r = std::sqrt(r2);
    double r5 = r2 * r2 * r;

    Vec3 k = SpinAxis;
    k.Normalize();

    double rDotSpin = DistanceDif.Dot(k);

    double aJ2_0 = 1.5 * (Mu * J2 * (RefRadius * RefRadius)) / r5;
    Vec3 aJ2_1 = DistanceDif * (5.0 * (rDotSpin) * (rDotSpin) / r2 - 1.0) - (SpinAxis * rDotSpin * 2.0);
    Vec3 aJ2 = aJ2_1 * aJ2_0;
    return aJ2;
}
