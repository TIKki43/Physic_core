#pragma once

#include <Vec3.h>
#include <string>

class Body{
public:
    Body(Vec3 Velocity, Vec3 Position, float Mass, Vec3 Force = Vec3(), std::string BodyType = "") 
        : Velocity(Velocity), Position(Position), Mass(Mass), Force(Force), BodyType(BodyType) {}

    Vec3 GetVelocity() const;
    void SetVelocity(const Vec3& NewVelocity);

    float GetMass() const;
    void SetMass(float NewMass); 

    Vec3 GetPosition() const;
    void SetPosition(const Vec3& NewPosition);
    
    Vec3 GetForce() const;
    void SetForce(const Vec3& NewForce);

    std::string GetBodyType() const;
    void SetBodyType(std::string NewBodyType);

private:
    Vec3 Velocity;
    Vec3 Position;
    float Mass;
    Vec3 Force;
    std::string BodyType; // ToDo ENUM
};