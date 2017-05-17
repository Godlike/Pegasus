#ifndef PEGASUS_PARTICLE_FORCE_GENERATOR_HPP
#define PEGASUS_PARTICLE_FORCE_GENERATOR_HPP

#include "Pegasus/include/particle.hpp"
#include <map>
#include <memory>
#include <set>
#include <vector>

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
    ParticleSpring(Particle & other, double springConstant, double restLenght);

    void updateForce(Particle & p) override;

private:
    Particle & mOther;
    double const mSpringConstant;
    double const mRestLenght;
};

class ParticleAnchoredSpring : public ParticleForceGenerator {
public:
    ParticleAnchoredSpring(Vector3 const& anchor, double springConstant, double restLenght);

    void updateForce(Particle & p) override;

private:
    Vector3 const mAnchor;
    double const mSpringConstant;
    double const mRestLenght;
};

class ParticleBungee : public ParticleForceGenerator {
public:
    ParticleBungee(Particle & other, double springConstant, double restLenght);

    void updateForce(Particle & p) override;

private:
    Particle & mOther;
    double const mSpringConstant;
    double const mRestLenght;
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
    Particles & particles;
    double maxReplusion;
    double maxAttraction;
    double minNaturalDistance, maxNaturalDistance;
    double floatHead;
    unsigned int maxFloat;
    double maxDistance;

public:
    explicit BlobForceGenerator(Particles & particles)
    : particles(particles)
    , maxReplusion(0)
    , maxAttraction(0)
    , minNaturalDistance(0)
    , maxNaturalDistance(0)
    , floatHead(0)
    , maxFloat(0)
    , maxDistance(0)
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
                particle.addForce(separation.unit() * (1.0f - distance) * maxReplusion * -1.0f);
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
};

} // namespace pegasus

#endif // PEGASUS_PARTICLE_FORCE_GENERATOR_HPP
