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
    using Ptr = std::shared_ptr<ParticleForceGenerator>;

public:
    virtual ~ParticleForceGenerator() {}

    virtual void updateForce(Particle::Ptr const p) = 0;
};

class ParticleForceRegistry {
public:
    using Ptr = std::shared_ptr<ParticleForceRegistry>;
    using Registry = std::map<Particle::Ptr, std::set<ParticleForceGenerator::Ptr> >;

public:
    void add(Particle::Ptr p, ParticleForceGenerator::Ptr pfg);

    void remove(Particle::Ptr const p);

    void remove(Particle::Ptr const p, ParticleForceGenerator::Ptr const pfg);

    void clear();

    void updateForces();

private:
    Registry mRegistrations;
};

class ParticleGravity : public ParticleForceGenerator {
public:
    explicit ParticleGravity(Vector3 const& g);

    void updateForce(Particle::Ptr const p) override;

private:
    Vector3 const mGravity;
};

class ParticleDrag : public ParticleForceGenerator {
public:
    ParticleDrag(double const k1, double const k2);

    void updateForce(Particle::Ptr const p) override;

private:
    double const mK1;
    double const mK2;
};

class ParticleSpring : public ParticleForceGenerator {
public:
    ParticleSpring(Particle::Ptr const other, double const springConstant, double const restLenght);

    void updateForce(Particle::Ptr const p) override;

private:
    Particle::Ptr const mOther;
    double const mSpringConstant;
    double const mRestLenght;
};

class ParticleAnchoredSpring : public ParticleForceGenerator {
public:
    ParticleAnchoredSpring(Vector3 const& anchor, double const springConstant, double const restLenght);

    void updateForce(Particle::Ptr const p) override;

private:
    Vector3 const mAnchor;
    double const mSpringConstant;
    double const mRestLenght;
};

class ParticleBungee : public ParticleForceGenerator {
public:
    ParticleBungee(Particle::Ptr const other, double const springConstant, double const restLenght);

    void updateForce(Particle::Ptr const p) override;

private:
    Particle::Ptr const mOther;
    double const mSpringConstant;
    double const mRestLenght;
};

class ParticleBuoyancy : public ParticleForceGenerator {
public:
    ParticleBuoyancy(double const maxDepth,  double const volume,
                     double const waterWight, double const liquidDensity);

    void updateForce(Particle::Ptr const p) override;

private:
    double const mMaxDepth;
    double const mVolume;
    double const mWaterHeight;
    double const mLiquidDensity;
};

class ParticleFakeSpring : public ParticleForceGenerator {
public:
    ParticleFakeSpring(Vector3 const& anchor, double const springConstant, double const damping);

    void updateForce(Particle::Ptr const p, double const duration) const;

    void updateForce(Particle::Ptr const p) override;

private:
    Vector3 const mAnchor;
    double const mSpringConstant;
    double const mDamping;
    double mDuration;
};

class BlobForceGenerator : public ParticleForceGenerator {
public:
    Particles& particles;
    double maxReplusion;
    double maxAttraction;
    double minNaturalDistance, maxNaturalDistance;
    double floatHead;
    unsigned int maxFloat;
    double maxDistance;

    explicit BlobForceGenerator(Particles& particles);

    void updateForce(Particle::Ptr const particle) override;
};

} // namespace pegasus

#endif // PEGASUS_PARTICLE_FORCE_GENERATOR_HPP
