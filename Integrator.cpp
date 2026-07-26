#include "Integrator.h"
#include "Body.h"
#include "Vec3.h"
#include <array>
#include <cmath>
#include <span>
#include <vector>


// a * b + c
[[nodiscard]] constexpr Vec3 fma(const Vec3& a, double b, const Vec3& c) noexcept {
    return Vec3{std::fma(a.x, b, c.x), std::fma(a.y, b, c.y), std::fma(a.z, b, c.z)};
}


void Integrator::EulerIntegration(Body& BodyA, const Vec3& Force, float DeltaTime){
    const Vec3 Acceleration = Force / BodyA.GetMass();
    BodyA.SetVelocity(fma(Acceleration, DeltaTime, BodyA.GetVelocity())); // v = v + a * dt
    BodyA.SetPosition(fma(BodyA.GetVelocity(), DeltaTime, BodyA.GetPosition())); // x = x + v * dt
}


void Integrator::RungeKutta4Integration(std::vector<Body>& Bodies,
                                        Gravity& GravityModel,
                                        float DeltaTime) {
    const std::size_t size{Bodies.size()};
    if (size == 0) [[unlikely]] { return; }

    std::vector<Body> stageBodies(size * 4, Bodies[0]);
    std::vector<Vec3> positionDerivatives(size * 4);
    std::vector<Vec3> velocityDerivatives(size * 4);
    std::vector<Vec3> forces(size);

    for (std::size_t i{0}; i < size; ++i) {
        const Body& bodyI{Bodies[i]};
        for (std::size_t j{i + 1}; j < size; ++j) {
            const Vec3 force{GravityModel.ComputeGravityForce(bodyI, Bodies[j])};
            forces[i] += force;
            forces[j] -= force;
        }
    }

    for (std::size_t i{}; i < size; ++i) {
        positionDerivatives[i] = Bodies[i].GetVelocity();
        velocityDerivatives[i] = forces[i] / Bodies[i].GetMass();
    }

    constexpr std::array stageWeights{0.0, 0.5, 0.5, 1.0};
    for (int stage{1}; stage < 4; ++stage) {
        const double stepScale{stageWeights[stage] * static_cast<double>(DeltaTime)};
        const std::size_t offset{stage * size};
        const std::size_t previousOffset{(stage - 1) * size};

        const std::span<const Vec3> previousPositionDerivatives(&positionDerivatives[previousOffset], size);
        const std::span<const Vec3> previousVelocityDerivatives(&velocityDerivatives[previousOffset], size);

        for (std::size_t i{}; i < size; ++i) {
            Body intermediateBody{Bodies[i]};
            intermediateBody.SetPosition(fma(previousPositionDerivatives[i], stepScale, intermediateBody.GetPosition()));
            intermediateBody.SetVelocity(fma(previousVelocityDerivatives[i], stepScale, intermediateBody.GetVelocity()));
            stageBodies[offset + i] = intermediateBody;
        }

        forces.assign(size, Vec3{});
        const std::span<const Body> stageSlice(&stageBodies[offset], size);
        for (std::size_t i{}; i < size; ++i) {
            const Body& bodyI{stageSlice[i]};
            for (std::size_t j{i + 1}; j < size; ++j) {
                const Vec3 force{GravityModel.ComputeGravityForce(bodyI, stageSlice[j])};
                forces[i] += force;
                forces[j] -= force;
            }
        }

        for (std::size_t i{}; i < size; ++i) {
            positionDerivatives[offset + i] = stageBodies[offset + i].GetVelocity();
            velocityDerivatives[offset + i] = forces[i] / Bodies[i].GetMass();
        }
    }

    constexpr double div = 1.0/6.0;
    const double timeStep{static_cast<double>(DeltaTime) * div};
    const double twoTimeStep{2.0 * timeStep};
    for (std::size_t i{}; i < size; ++i) {
        const Vec3& k0p{positionDerivatives[i]};
        const Vec3& k1p{positionDerivatives[size + i]};
        const Vec3& k2p{positionDerivatives[std::fma(2, size, i)]};
        const Vec3& k3p{positionDerivatives[std::fma(3, size, i)]};

        const Vec3& k0v{velocityDerivatives[i]};
        const Vec3& k1v{velocityDerivatives[size + i]};
        const Vec3& k2v{velocityDerivatives[std::fma(2, size, i)]};
        const Vec3& k3v{velocityDerivatives[std::fma(3, size, i)]};

        Vec3 positionDelta{k3p * timeStep};
        positionDelta = fma(k2p, twoTimeStep, positionDelta);
        positionDelta = fma(k1p, twoTimeStep, positionDelta);
        positionDelta = fma(k0p, timeStep,    positionDelta);

        Vec3 velocityDelta{k3v * timeStep};
        velocityDelta = fma(k2v, twoTimeStep, velocityDelta);
        velocityDelta = fma(k1v, twoTimeStep, velocityDelta);
        velocityDelta = fma(k0v, timeStep,    velocityDelta);

        Bodies[i].SetPosition(fma(positionDelta, 1.0, Bodies[i].GetPosition()));
        Bodies[i].SetVelocity(fma(velocityDelta, 1.0, Bodies[i].GetVelocity()));
    }
}