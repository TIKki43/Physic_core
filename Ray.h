#pragma once
#include "Vec3.h"

class Ray{
public:
    Ray() = default;
    Ray(float Wavelength, Vec3& Position, Vec3& Direction, float Intensity, float WaveSpeed) 
            : Wavelength(Wavelength), Position(Position), Direction(Direction), Intensity(Intensity), WaveSpeed(WaveSpeed) {}
    
    
    float GetFrequency() const;

    void SetPosition(const Vec3& Position);
    Vec3 GetPosition() const;

    void SetDirection(const Vec3& Direction);
    Vec3 GetDirection() const;

    float GetTravelDistance() const;
    float GetTravelTime() const;
    
    float AdvanceByTime(const float DeltaTime);
    float AdvanceByDistance(const float DeltaDistance);

    bool Active = true;

private:
    Vec3 Position;
    Vec3 Direction;
    float Wavelength;
    float Intensity;
    float WaveSpeed;

    float DeltaTime;
    float DeltaDistance;
    float TravelTime = 0.0f;
    float TravelDistance = 0.0f;
    
    
};