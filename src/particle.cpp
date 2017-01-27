#include "Pegas/include/particle.hpp"

#include <stdexcept>
#include <cmath>

void pegas::Particle::integrate(const pegas::real duration)
{
    if (duration <= 0.0f)
    {
        throw std::invalid_argument("Particle::integrate duration <= 0");
    }

    position.addScaledVector(velocity, duration);

    Vector3 resultingAcc(acceleration);
    resultingAcc.addScaledVector(forceAccum, inverseMass);

    velocity.addScaledVector(resultingAcc, duration);

    velocity *= std::pow(damping, duration);
}
