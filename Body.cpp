#include "Body.h"


Vec3 Body::GetVelocity() const { return Velocity; }
void Body::SetVelocity(const Vec3& NewVelocity) { Velocity = NewVelocity; }

double Body::GetMass() const { return Mass; }
void Body::SetMass(double NewMass) { Mass = NewMass; MU = Mass * Constants::G; }

double Body::GetMU() const { return MU; }
void Body::SetMu(double NewMU) { MU = NewMU; Mass = MU / Constants::G; }

Vec3 Body::GetPosition() const { return Position; }
void Body::SetPosition(const Vec3& NewPosition) { Position = NewPosition; }

Vec3 Body::GetForce() const { return Force; }
void Body::SetForce(const Vec3& NewForce) { Force = NewForce; } 

const std::string& Body::GetBodyType() const { return BodyType; }
void Body::SetBodyType(const std::string& NewBodyType) {BodyType = NewBodyType; }
