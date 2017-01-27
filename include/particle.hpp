#ifndef PEGAS_PARTICLE_HPP
#define PEGAS_PARTICLE_HPP

#include "Pegas/include/core.hpp"
#include "Pegas/include/math.hpp"

namespace pegas {
class Particle {
public:
    Vector3 position;

    Vector3 velocity;

    Vector3 acceleration;

    real damping;

    real mass;

    real inverseMass;

    void integrate(const real duration);

private:
    Vector3 forceAccum;
};
}

#endif //PEGAS_PARTICLE_HPP
