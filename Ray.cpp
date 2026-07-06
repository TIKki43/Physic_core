#include "Ray.h"

float Ray::GetFrequency() const { return WaveSpeed / Wavelength; }

void Ray::SetPosition(const Vec3& NewPosition) { Position = NewPosition; }
Vec3 Ray::GetPosition() const { return Position; }

void Ray::SetDirection(const Vec3& NewDirection) { Direction = NewDirection; Direction.Normalize(); }
Vec3 Ray::GetDirection() const { return Direction; }

float Ray::GetTravelDistance() const { return TravelDistance; }
float Ray::GetTravelTime() const { return TravelTime; }

float Ray::AdvanceByTime(const float DeltaTime)
{
    const float DeltaDistance = WaveSpeed * DeltaTime;

    Position += Direction * DeltaDistance;

    TravelDistance += DeltaDistance;
    TravelTime += DeltaTime;
    return DeltaDistance;
}

float Ray::AdvanceByDistance(const float DeltaDistance)
{
    float DeltaTime = DeltaDistance / WaveSpeed;

    Position += Direction * DeltaDistance;

    TravelDistance += DeltaDistance;
    TravelTime += DeltaTime;
    return DeltaTime;
}
