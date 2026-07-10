#pragma once
#include "Vec3.h"
#include "Body.h"
#include "Constants.h"

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

    void InitSchwarzschildMetricState
                            ( Body& MassiveBody
                            , float InitialE
                            , float InitialL
                            , float Initialt
                            , float Initialc = Constants::c
                            , float InitialG = Constants::G
                            , float Initialdlambda = Constants::DefaultDeltaTime
    );
    void SchwarzschildMetric(Body& MassiveBody);
    void SchwarzschildStep(Body& MassiveBody);

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
    
    float dlambda;
    float t;
    float E;
    float L;
    float M_geo;
    float r;
    float phi;
    float p;
    float f;
    float G;
    float c;
};