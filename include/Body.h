#pragma once

#include <Constants.h>

#include <string>

#include "Vec3.h"

class Body
{
public:
  Body (Vec3 Velocity, Vec3 Position, double Mass, Vec3 Force = Vec3 (),
        double J2 = 0.0, double RefRadius = 0.0, Vec3 SpinAxis = Vec3 (),
        std::string BodyType = "")
      : Velocity (Velocity), Position (Position), Mass (Mass),
        MU (Mass * Constants::G), Force (Force), J2 (J2),
        RefRadius (RefRadius), SpinAxis (SpinAxis), BodyType (BodyType)
  {
  }

  [[nodiscard]] Vec3 GetVelocity () const noexcept;
  void SetVelocity (const Vec3 &NewVelocity) noexcept;

  [[nodiscard]] double GetMass () const noexcept;
  void SetMass (double NewMass) noexcept;

  [[nodiscard]] double GetMU () const noexcept;
  void SetMu (double NewMU) noexcept;

  [[nodiscard]] double GetJ2 () const noexcept;
  void SetJ2 (double NewJ2) noexcept;

  [[nodiscard]] double GetRefRadius () const noexcept;
  void SetRefRadius (double NewRefRadius) noexcept;

  [[nodiscard]] Vec3 GetPosition () const noexcept;
  void SetPosition (const Vec3 &NewPosition) noexcept;

  [[nodiscard]] Vec3 GetSpinAxis () const noexcept;
  void SetSpinAxis (const Vec3 &NewSpinAxis) noexcept;

  [[nodiscard]] Vec3 GetForce () const noexcept;
  void SetForce (const Vec3 &NewForce) noexcept;

  [[nodiscard]] const std::string &GetBodyType () const noexcept;
  void SetBodyType (const std::string &NewBodyType);

private:
  Vec3 Velocity{};
  Vec3 Position{};
  double Mass{};
  double MU{};
  Vec3 Force{};
  double J2{};
  double RefRadius{};
  Vec3 SpinAxis{};
  std::string BodyType{}; // ToDo ENUM
};
