#include "Ray.h"
#include <cmath>


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

void Ray::InitSchwarzschildMetricState
                                    ( Body& MassiveBody
                                    , float InitialE
                                    , float InitialL
                                    , float Initialt
                                    , float Initialc
                                    , float InitialG
                                    , float Initialdlambda
                                    )
{
    E = InitialE;
    L = InitialL;
    t = Initialt;

    c = Initialc;
    G = InitialG;

    dlambda = Initialdlambda;

    M_geo = G * MassiveBody.GetMass() / (c * c);
    r = this->GetPosition().Distance(MassiveBody.GetPosition());
    f = 1.0f - 2.0f * M_geo / (r);
    
    Vec3 rel = this->GetPosition() - MassiveBody.GetPosition();
    phi = std::atan2(rel.y, rel.x);
    
    float inside = E * E - f * L * L / (r * r);
    p = -std::sqrt(inside);
}

void Ray::SchwarzschildMetric(Body& MassiveBody)
{
    f = 1.0f - 2.0f * M_geo / (r);

    float t_dot = E / f;
    float phi_dot = L / (r * r); 
    float r_dot = p;
    float p_dot =  (L * L) / (r * r * r) - (3 * M_geo * L * L)/(r * r * r * r);


    t += t_dot * dlambda;
    phi += phi_dot * dlambda;
    p += p_dot * dlambda;
    r += r_dot * dlambda;


    Vec3 NewRel(
        r * std::cos(phi),
        r * std::sin(phi),
        0.0f
    );

    this->SetPosition(MassiveBody.GetPosition() + NewRel);
    
}

void Ray::SchwarzschildStep(Body& MassiveBody)
{
    Vec3 OldPos = this->GetPosition();

    float Rs = 2.0 * M_geo;
    if (Rs >= r){
        Active = false;
        return;
    }

    f = 1.0f - 2.0f * M_geo / (r);

    float t_dot = E / f;
    float phi_dot = L / (r * r); 
    float r_dot = p;
    float p_dot =  (L * L) / (r * r * r) - (3 * M_geo * L * L)/(r * r * r * r);


    t += t_dot * dlambda;
    phi += phi_dot * dlambda;
    p += p_dot * dlambda;
    r += r_dot * dlambda;


    Vec3 NewRel(
        r * std::cos(phi),
        r * std::sin(phi),
        0.0f
    );

    
    Vec3 NewPos = MassiveBody.GetPosition() + NewRel;
    this->SetPosition(NewPos);

    float DeltaDistance = GetPosition().Distance(OldPos);
    TravelTime += t_dot * dlambda;
    TravelDistance += DeltaDistance;

    if (DeltaDistance > 1e-6f){
        Direction = (NewPos - OldPos);
        Direction.Normalize();
    }
}
