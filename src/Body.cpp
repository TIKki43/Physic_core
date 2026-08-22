#include "Body.h"

Vec3
Body::GetVelocity () const noexcept
{
  return Velocity;
}
void
Body::SetVelocity (const Vec3 &NewVelocity) noexcept
{
  Velocity = NewVelocity;
}

Vec3
Body::GetSpinAxis () const noexcept
{
  return SpinAxis;
}
void
Body::SetSpinAxis (const Vec3 &NewSpinAxis) noexcept
{
  SpinAxis = NewSpinAxis;
}

double
Body::GetMass () const noexcept
{
  return Mass;
}
void
Body::SetMass (double NewMass) noexcept
{
  Mass = NewMass;
  MU = Mass * Constants::G;
}

double
Body::GetMU () const noexcept
{
  return MU;
}
void
Body::SetMu (double NewMU) noexcept
{
  MU = NewMU;
  Mass = MU / Constants::G;
}

double
Body::GetJ2 () const noexcept
{
  return J2;
}
void
Body::SetJ2 (double NewJ2) noexcept
{
  J2 = NewJ2;
}

double
Body::GetRefRadius () const noexcept
{
  return RefRadius;
}
void
Body::SetRefRadius (double NewRefRadius) noexcept
{
  RefRadius = NewRefRadius;
}

Vec3
Body::GetPosition () const noexcept
{
  return Position;
}
void
Body::SetPosition (const Vec3 &NewPosition) noexcept
{
  Position = NewPosition;
}

Vec3
Body::GetForce () const noexcept
{
  return Force;
}
void
Body::SetForce (const Vec3 &NewForce) noexcept
{
  Force = NewForce;
}

const std::string &
Body::GetBodyType () const noexcept
{
  return BodyType;
}
void
Body::SetBodyType (const std::string &NewBodyType)
{
  BodyType = NewBodyType;
}
