#pragma once

#include "Vec3.h"
#include <string>
#include <Constants.h>

class Body{
public:
    Body(Vec3 Velocity, Vec3 Position, double Mass, Vec3 Force = Vec3(), std::string BodyType = "") 
        : Velocity(Velocity), Position(Position), Mass(Mass), MU(Mass*Constants::G), Force(Force), BodyType(BodyType) {}

    [[nodiscard]] Vec3 GetVelocity() const noexcept;
    void SetVelocity(const Vec3& NewVelocity) noexcept;

    [[nodiscard]] double GetMass() const noexcept;
    void SetMass(double NewMass) noexcept; 

    [[nodiscard]] double GetMU() const noexcept;
    void SetMu(double NewMU) noexcept;

    [[nodiscard]] Vec3 GetPosition() const noexcept;
    void SetPosition(const Vec3& NewPosition) noexcept;
    
    [[nodiscard]] Vec3 GetForce() const noexcept;
    void SetForce(const Vec3& NewForce) noexcept;

    [[nodiscard]] const std::string& GetBodyType() const;
    void SetBodyType(const std::string& NewBodyType);

private:
    Vec3 Velocity;
    Vec3 Position;
    double Mass;
    double MU;
    Vec3 Force;
    std::string BodyType; // ToDo ENUM
};