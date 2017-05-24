#ifndef PEGASUS_PARTICLE_FORCE_GENERATOR_HPP
#define PEGASUS_PARTICLE_FORCE_GENERATOR_HPP

#include "Pegasus/include/particle.hpp"

#include <map>
#include <set>

namespace pegasus {

class ParticleForceGenerator {
public:
    virtual ~ParticleForceGenerator() {}
    virtual void updateForce(Particle & p) = 0;
};

class ParticleForceRegistry {
public:
    void add(Particle & p, ParticleForceGenerator & pfg);
    void remove(Particle & p);
    void remove(Particle & p, ParticleForceGenerator & pfg);
    void clear();
    void updateForces();

private:
    std::map<Particle*, std::set<ParticleForceGenerator*>> mRegistrations;
};

class ParticleGravity : public ParticleForceGenerator {
public:
    explicit ParticleGravity(Vector3 const& g);

    void updateForce(Particle & p) override;

private:
    Vector3 const mGravity;
};

class ParticleDrag : public ParticleForceGenerator {
public:
    ParticleDrag(double k1, double k2);

    void updateForce(Particle & p) override;

private:
    double const mK1;
    double const mK2;
};

class ParticleSpring : public ParticleForceGenerator {
public:
    ParticleSpring(Particle & other, double springConstant, double restLength);

    void updateForce(Particle & p) override;

private:
    Particle & mOther;
    double const mSpringConstant;
    double const mRestLength;
};

class ParticleAnchoredSpring : public ParticleForceGenerator {
public:
    ParticleAnchoredSpring(Vector3 const& anchor, double springConstant, double restLength);

    void updateForce(Particle & p) override;

private:
    Vector3 const mAnchor;
    double const mSpringConstant;
    double const mRestLength;
};

class ParticleBungee : public ParticleForceGenerator {
public:
    ParticleBungee(Particle & other, double springConstant, double restLength);

    void updateForce(Particle & p) override;

private:
    Particle & mOther;
    double const mSpringConstant;
    double const mRestLength;
};

class ParticleBuoyancy : public ParticleForceGenerator {
public:
    ParticleBuoyancy(double maxDepth,   double volume,
                     double waterWight, double liquidDensity);

    void updateForce(Particle & p) override;

private:
    double const mMaxDepth;
    double const mVolume;
    double const mWaterHeight;
    double const mLiquidDensity;
};

class ParticleFakeSpring : public ParticleForceGenerator {
public:
    ParticleFakeSpring(Vector3 const& anchor, double springConstant, double damping);

    void updateForce(Particle & p, double duration) const;
    void updateForce(Particle & p) override;

private:
    Vector3 const mAnchor;
    double const mSpringConstant;
    double const mDamping;
    double mDuration;
};

template < typename Particles >
class BlobForceGenerator : public ParticleForceGenerator {
public:
    explicit BlobForceGenerator(
        Particles & particles, 
        double maxRepulsion = 0,
        double maxAttraction = 0,
        double minNaturalDistance = 0, 
        double maxNaturalDistance = 0,
        double floatHead = 0,
        unsigned int maxFloat = 0,
        double maxDistance = 0
    )
    : particles(particles)
    , maxRepulsion(maxRepulsion)
    , maxAttraction(maxAttraction)
    , minNaturalDistance(minNaturalDistance)
    , maxNaturalDistance(maxNaturalDistance)
    , floatHead(floatHead)
    , maxFloat(maxFloat)
    , maxDistance(maxDistance)
    {
    }

    void updateForce(Particle & particle) override
    {
        unsigned int joinCount = 0;

        for(auto const & currentParticle : particles) {
            if (&currentParticle == &particle)
                continue;

            // Work out the separation distance
            auto separation = currentParticle.getPosition() - particle.getPosition();
            separation.z = 0.0f;
            auto distance = separation.magnitude();

            if (distance < minNaturalDistance) {
                // Use a repulsion force.
                distance = 1.0f - distance / minNaturalDistance;
                particle.addForce(separation.unit() * (1.0f - distance) * maxRepulsion * -1.0f);
                ++joinCount;
            } else if (distance > maxNaturalDistance && distance < maxDistance) {
                // Use an attraction force.
                distance = (distance - maxNaturalDistance) / (maxDistance - maxNaturalDistance);
                particle.addForce(separation.unit() * distance * maxAttraction);
                ++joinCount;
            }
        }

        // If the particle is the head, and we've got a join count, then float it.
        if (   &particle == &(*particles.begin())
            && joinCount > 0 && maxFloat > 0)
        {
            auto force = (static_cast<double>(joinCount) / static_cast<double>(maxFloat)) * floatHead;
            if (force > floatHead) {
                force = floatHead;
            }

            particle.addForce(Vector3(0, force, 0));
        }
    }

private:
    Particles & particles;
    double const maxRepulsion;
    double const maxAttraction;
    double const minNaturalDistance, maxNaturalDistance;
    double const floatHead;
    uint32_t const maxFloat;
    double maxDistance;
};

} // namespace pegasus

#endif // PEGASUS_PARTICLE_FORCE_GENERATOR_HPP
