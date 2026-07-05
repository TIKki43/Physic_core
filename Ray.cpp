#include "Ray.h"

float Ray::GetFrequency() const { return WaveSpeed / Wavelength; }

void Ray::SetPosition(const Vec3 NewPosition) { Position = NewPosition; }
Vec3 Ray::GetPosition() const { return Position; }

void Ray::SetDirection(const Vec3 NewDirection) { Direction = NewDirection; Direction.Normalize(); }
Vec3 Ray::GetDirection() const { return Direction; }