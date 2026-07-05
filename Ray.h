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


    float TravelDistance = 0.0f;

    bool Active = true;

private:
    Vec3 Position;
    Vec3 Direction;
    float Wavelength;
    float Intensity;
    float WaveSpeed;
    
};