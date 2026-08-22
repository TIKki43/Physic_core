#include "Integrator.h"
#include "Body.h"
#include "Vec3.h"
#include <array>
#include <cmath>
#include <span>
#include <vector>

using std::fma;
// a * b + c
[[nodiscard]] constexpr Vec3
fma (const Vec3 &a, double b, const Vec3 &c) noexcept
{
  return Vec3{ std::fma (a.x, b, c.x), std::fma (a.y, b, c.y),
               std::fma (a.z, b, c.z) };
}

void
Integrator::EulerIntegration (Body &BodyA, const Vec3 &Force, double DeltaTime)
{
  const Vec3 Acceleration = Force / BodyA.GetMass ();
  BodyA.SetVelocity (
      fma (Acceleration, DeltaTime, BodyA.GetVelocity ())); // v = v + a * dt
  BodyA.SetPosition (fma (BodyA.GetVelocity (), DeltaTime,
                          BodyA.GetPosition ())); // x = x + v * dt
}

void
Integrator::RungeKutta4Integration1PN (std::vector<Body> &Bodies,
                                       Gravity &GravityModel, double DeltaTime)
{
  (void)GravityModel;

  const std::size_t size{ Bodies.size () };
  if (size == 0) [[unlikely]]
    {
      return;
    }

  std::vector<Body> StageBodies (size * 4, Bodies[0]);
  std::vector<Vec3> PositionDerivatives (size * 4);
  std::vector<Vec3> VelocityDerivatives (size * 4);
  std::vector<Vec3> Accelerations (size);

  auto ComputeAccelerations = [&] (std::span<const Body> Bodies)
    {
      Accelerations.assign (size, Vec3{});

      std::vector<Vec3> newtonAcc (size, Vec3{});
      std::vector<double> potential (size, 0.0);

      for (std::size_t a{}; a < size; ++a)
        {
          const Vec3 xA = Bodies[a].GetPosition ();

          for (std::size_t b{}; b < size; ++b)
            {
              if (a == b)
                {
                  continue;
                }

              const Vec3 xB = Bodies[b].GetPosition ();
              const Vec3 rBA = xB - xA;
              const double r2 = rBA.Dot (rBA);
              const double r = std::sqrt (r2);
              const double r3 = r2 * r;

              const double muB = Constants::G * Bodies[b].GetMass ();

              newtonAcc[a] += rBA * (muB / r3);
              potential[a] += muB / r;
            }
        }

      Accelerations = newtonAcc;

      const double c2 = Constants::c * Constants::c;

      for (std::size_t a{}; a < size; ++a)
        {
          const Vec3 xA = Bodies[a].GetPosition ();
          const Vec3 vA = Bodies[a].GetVelocity ();
          const double vA2 = vA.Dot (vA);

          for (std::size_t b{}; b < size; ++b)
            {
              if (a == b)
                {
                  continue;
                }

              const Vec3 xB = Bodies[b].GetPosition ();
              const Vec3 vB = Bodies[b].GetVelocity ();

              const Vec3 rAB = xA - xB;
              const Vec3 rBA = xB - xA;

              const double r2 = rAB.Dot (rAB);
              const double r = std::sqrt (r2);

              const Vec3 nAB = rAB / r;
              const Vec3 nBA = rBA / r;

              const double muB = Constants::G * Bodies[b].GetMass ();

              const double vB2 = vB.Dot (vB);
              const double vAvB = vA.Dot (vB);
              const double nABvB = nAB.Dot (vB);

              const double sourceAccelTerm = 0.5 * rBA.Dot (newtonAcc[b]);

              const double scalar = vA2 + 2.0 * vB2 - 4.0 * vAvB
                                    - 1.5 * nABvB * nABvB - 4.0 * potential[a]
                                    - potential[b] + sourceAccelTerm;

              const Vec3 term1 = nBA * ((muB / r2) * scalar);

              const double velocityProjection
                  = nAB.Dot ((vA * 4.0) - (vB * 3.0));

              const Vec3 term2 = (vA - vB) * ((muB / r2) * velocityProjection);

              const Vec3 term3 = newtonAcc[b] * (3.5 * muB / r);

              Accelerations[a] += (term1 + term2 + term3) / c2;
            }
        }
    };

  ComputeAccelerations (std::span<const Body> (Bodies.data (), size));

  for (std::size_t i{}; i < size; ++i)
    {
      PositionDerivatives[i] = Bodies[i].GetVelocity ();
      VelocityDerivatives[i] = Accelerations[i];
    }

  constexpr std::array stageWeights{ 0.0, 0.5, 0.5, 1.0 };

  for (int stage{ 1 }; stage < 4; ++stage)
    {
      const double stepScale
          = stageWeights[stage] * static_cast<double> (DeltaTime);

      const std::size_t offset = stage * size;
      const std::size_t previousOffset = (stage - 1) * size;

      for (std::size_t i{}; i < size; ++i)
        {
          Body intermediateBody{ Bodies[i] };

          intermediateBody.SetPosition (
              fma (PositionDerivatives[previousOffset + i], stepScale,
                   intermediateBody.GetPosition ()));

          intermediateBody.SetVelocity (
              fma (VelocityDerivatives[previousOffset + i], stepScale,
                   intermediateBody.GetVelocity ()));

          StageBodies[offset + i] = intermediateBody;
        }

      const std::span<const Body> StageSlice (&StageBodies[offset], size);
      ComputeAccelerations (StageSlice);

      for (std::size_t i{}; i < size; ++i)
        {
          PositionDerivatives[offset + i]
              = StageBodies[offset + i].GetVelocity ();

          VelocityDerivatives[offset + i] = Accelerations[i];
        }
    }

  const double dt6 = static_cast<double> (DeltaTime) / 6.0;

  for (std::size_t i{}; i < size; ++i)
    {
      const Vec3 PositionDelta
          = (PositionDerivatives[i] + PositionDerivatives[size + i] * 2.0
             + PositionDerivatives[2 * size + i] * 2.0
             + PositionDerivatives[3 * size + i])
            * dt6;

      const Vec3 VelocityDelta
          = (VelocityDerivatives[i] + VelocityDerivatives[size + i] * 2.0
             + VelocityDerivatives[2 * size + i] * 2.0
             + VelocityDerivatives[3 * size + i])
            * dt6;

      Bodies[i].SetPosition (Bodies[i].GetPosition () + PositionDelta);
      Bodies[i].SetVelocity (Bodies[i].GetVelocity () + VelocityDelta);
    }
}

void
Integrator::RungeKutta4Integration (std::vector<Body> &Bodies,
                                    Gravity &GravityModel, double DeltaTime)
{
  const std::size_t size{ Bodies.size () };
  if (size == 0) [[unlikely]]
    {
      return;
    }

  std::vector<Body> stageBodies (size * 4, Bodies[0]);
  std::vector<Vec3> positionDerivatives (size * 4);
  std::vector<Vec3> velocityDerivatives (size * 4);
  std::vector<Vec3> forces (size);

  for (std::size_t i{ 0 }; i < size; ++i)
    {
      const Body &bodyI{ Bodies[i] };
      for (std::size_t j{ i + 1 }; j < size; ++j)
        {
          const Vec3 force{ GravityModel.ComputeGravityForce (bodyI,
                                                              Bodies[j]) };
          forces[i] += force;
          forces[j] -= force;
        }
    }

  for (std::size_t i{}; i < size; ++i)
    {
      positionDerivatives[i] = Bodies[i].GetVelocity ();
      velocityDerivatives[i] = forces[i] / Bodies[i].GetMass ();
    }

  constexpr std::array stageWeights{ 0.0, 0.5, 0.5, 1.0 };
  for (int stage{ 1 }; stage < 4; ++stage)
    {
      const double stepScale{ stageWeights[stage]
                              * static_cast<double> (DeltaTime) };
      const std::size_t offset{ stage * size };
      const std::size_t previousOffset{ (stage - 1) * size };

      const std::span<const Vec3> previousPositionDerivatives (
          &positionDerivatives[previousOffset], size);
      const std::span<const Vec3> previousVelocityDerivatives (
          &velocityDerivatives[previousOffset], size);

      for (std::size_t i{}; i < size; ++i)
        {
          Body intermediateBody{ Bodies[i] };
          intermediateBody.SetPosition (fma (previousPositionDerivatives[i],
                                             stepScale,
                                             intermediateBody.GetPosition ()));
          intermediateBody.SetVelocity (fma (previousVelocityDerivatives[i],
                                             stepScale,
                                             intermediateBody.GetVelocity ()));
          stageBodies[offset + i] = intermediateBody;
        }

      forces.assign (size, Vec3{});
      const std::span<const Body> StageSlice (&stageBodies[offset], size);
      for (std::size_t i{}; i < size; ++i)
        {
          const Body &bodyI{ StageSlice[i] };
          for (std::size_t j{ i + 1 }; j < size; ++j)
            {
              const Vec3 force{ GravityModel.ComputeGravityForce (
                  bodyI, StageSlice[j]) };
              forces[i] += force;
              forces[j] -= force;
            }
        }

      for (std::size_t i{}; i < size; ++i)
        {
          positionDerivatives[offset + i]
              = stageBodies[offset + i].GetVelocity ();
          velocityDerivatives[offset + i] = forces[i] / Bodies[i].GetMass ();
        }
    }

  constexpr double div = 1.0 / 6.0;
  const double timeStep{ static_cast<double> (DeltaTime) * div };
  const double twoTimeStep{ 2.0 * timeStep };
  for (std::size_t i{}; i < size; ++i)
    {
      const Vec3 &k0p{ positionDerivatives[i] };
      const Vec3 &k1p{ positionDerivatives[size + i] };
      const Vec3 &k2p{ positionDerivatives[std::fma (2, size, i)] };
      const Vec3 &k3p{ positionDerivatives[std::fma (3, size, i)] };

      const Vec3 &k0v{ velocityDerivatives[i] };
      const Vec3 &k1v{ velocityDerivatives[size + i] };
      const Vec3 &k2v{ velocityDerivatives[std::fma (2, size, i)] };
      const Vec3 &k3v{ velocityDerivatives[std::fma (3, size, i)] };

      Vec3 positionDelta{ k3p * timeStep };
      positionDelta = fma (k2p, twoTimeStep, positionDelta);
      positionDelta = fma (k1p, twoTimeStep, positionDelta);
      positionDelta = fma (k0p, timeStep, positionDelta);

      Vec3 velocityDelta{ k3v * timeStep };
      velocityDelta = fma (k2v, twoTimeStep, velocityDelta);
      velocityDelta = fma (k1v, twoTimeStep, velocityDelta);
      velocityDelta = fma (k0v, timeStep, velocityDelta);

      Bodies[i].SetPosition (
          fma (positionDelta, 1.0, Bodies[i].GetPosition ()));
      Bodies[i].SetVelocity (
          fma (velocityDelta, 1.0, Bodies[i].GetVelocity ()));
    }
}

void
Integrator::RungeKutta4Integration1PNJ2 (std::vector<Body> &Bodies,
                                         const Gravity &GravityModel,
                                         double DeltaTime)
{
  const std::size_t size{ Bodies.size () };
  if (size == 0) [[unlikely]]
    {
      return;
    }

  std::vector<Body> StageBodies (size * 4, Bodies[0]);
  std::vector<Vec3> PositionDerivatives (size * 4);
  std::vector<Vec3> VelocityDerivatives (size * 4);
  std::vector<Vec3> Accelerations (size);

  auto ComputeAccelerations = [&] (std::span<const Body> Bodies)
    {
      Accelerations.assign (size, Vec3{});

      std::vector<Vec3> newtonAcc (size, Vec3{});
      std::vector<double> potential (size, 0.0);
      std::vector<Vec3> accelerationj2 (size, Vec3{});

      for (std::size_t a{}; a < size; ++a)
        {
          const Vec3 xA = Bodies[a].GetPosition ();

          for (std::size_t b{}; b < size; ++b)
            {
              if (a == b)
                {
                  continue;
                }

              const Vec3 xB = Bodies[b].GetPosition ();
              const Vec3 DistanceDif = xB - xA;
              const double r2 = DistanceDif.Dot (DistanceDif);
              const double r = std::sqrt (r2);
              const double r3 = r2 * r;

              const double muB = Constants::G * Bodies[b].GetMass ();

              newtonAcc[a] += DistanceDif * (muB / r3);
              potential[a] += muB / r;
              if (Bodies[b].GetJ2 () != 0.0 && Bodies[b].GetRefRadius () > 0.0
                  && Bodies[a].GetJ2 () != 0.0
                  && Bodies[a].GetRefRadius () > 0.0)
                {
                  Vec3 reversedDistance = xA - xB;
                  accelerationj2[a] += GravityModel.ComputeJ2Acceleration (
                      reversedDistance, muB, Bodies[b].GetJ2 (),
                      Bodies[b].GetRefRadius (), Bodies[b].GetSpinAxis ());
                }
            }
        }

      Accelerations = newtonAcc;
      for (std::size_t a{}; a < size; ++a)
        {
          Accelerations[a] += accelerationj2[a];
        }

      const double c2 = Constants::c * Constants::c;

      for (std::size_t a{}; a < size; ++a)
        {
          const Vec3 xA = Bodies[a].GetPosition ();
          const Vec3 vA = Bodies[a].GetVelocity ();
          const double vA2 = vA.Dot (vA);

          for (std::size_t b{}; b < size; ++b)
            {
              if (a == b)
                {
                  continue;
                }

              const Vec3 xB = Bodies[b].GetPosition ();
              const Vec3 vB = Bodies[b].GetVelocity ();

              const Vec3 rAB = xA - xB;
              const Vec3 rBA = xB - xA;

              const double r2 = rAB.Dot (rAB);
              const double r = std::sqrt (r2);

              const Vec3 nAB = rAB / r;
              const Vec3 nBA = rBA / r;

              const double muB = Constants::G * Bodies[b].GetMass ();

              const double vB2 = vB.Dot (vB);
              const double vAvB = vA.Dot (vB);
              const double nABvB = nAB.Dot (vB);

              const double sourceAccelTerm = 0.5 * rBA.Dot (newtonAcc[b]);

              const double scalar = vA2 + 2.0 * vB2 - 4.0 * vAvB
                                    - 1.5 * nABvB * nABvB - 4.0 * potential[a]
                                    - potential[b] + sourceAccelTerm;

              const Vec3 term1 = nBA * ((muB / r2) * scalar);

              const double velocityProjection
                  = nAB.Dot ((vA * 4.0) - (vB * 3.0));

              const Vec3 term2 = (vA - vB) * ((muB / r2) * velocityProjection);

              const Vec3 term3 = newtonAcc[b] * (3.5 * muB / r);

              Accelerations[a] += (term1 + term2 + term3) / c2;
            }
        }
    };

  ComputeAccelerations (std::span<const Body> (Bodies.data (), size));

  for (std::size_t i{}; i < size; ++i)
    {
      PositionDerivatives[i] = Bodies[i].GetVelocity ();
      VelocityDerivatives[i] = Accelerations[i];
    }

  constexpr std::array stageWeights{ 0.0, 0.5, 0.5, 1.0 };

  for (int stage{ 1 }; stage < 4; ++stage)
    {
      const double stepScale
          = stageWeights[stage] * static_cast<double> (DeltaTime);

      const std::size_t offset = stage * size;
      const std::size_t previousOffset = (stage - 1) * size;

      for (std::size_t i{}; i < size; ++i)
        {
          Body intermediateBody{ Bodies[i] };

          intermediateBody.SetPosition (
              fma (PositionDerivatives[previousOffset + i], stepScale,
                   intermediateBody.GetPosition ()));

          intermediateBody.SetVelocity (
              fma (VelocityDerivatives[previousOffset + i], stepScale,
                   intermediateBody.GetVelocity ()));

          StageBodies[offset + i] = intermediateBody;
        }

      const std::span<const Body> StageSlice (&StageBodies[offset], size);
      ComputeAccelerations (StageSlice);

      for (std::size_t i{}; i < size; ++i)
        {
          PositionDerivatives[offset + i]
              = StageBodies[offset + i].GetVelocity ();

          VelocityDerivatives[offset + i] = Accelerations[i];
        }
    }

  const double dt6 = static_cast<double> (DeltaTime) / 6.0;

  for (std::size_t i{}; i < size; ++i)
    {
      const Vec3 PositionDelta
          = (PositionDerivatives[i] + PositionDerivatives[size + i] * 2.0
             + PositionDerivatives[2 * size + i] * 2.0
             + PositionDerivatives[3 * size + i])
            * dt6;

      const Vec3 VelocityDelta
          = (VelocityDerivatives[i] + VelocityDerivatives[size + i] * 2.0
             + VelocityDerivatives[2 * size + i] * 2.0
             + VelocityDerivatives[3 * size + i])
            * dt6;

      Bodies[i].SetPosition (Bodies[i].GetPosition () + PositionDelta);
      Bodies[i].SetVelocity (Bodies[i].GetVelocity () + VelocityDelta);
    }
}