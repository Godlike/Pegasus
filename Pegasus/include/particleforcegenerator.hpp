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
    ParticleDrag(real const mK1, real const mK2);

    void updateForce(Particle::Ptr const p) override;

private:
    real const mK1;
    real const mK2;
};

class ParticleSpring : public ParticleForceGenerator {
public:
    ParticleSpring(Particle::Ptr const mOther, real const mSpringConstant, real const mRestLenght);

    void updateForce(Particle::Ptr const p) override;

private:
    Particle::Ptr const mOther;
    real const mSpringConstant;
    real const mRestLenght;
};

class ParticleAnchoredSpring : public ParticleForceGenerator {
public:
    ParticleAnchoredSpring(Vector3 const& mAnchor, real const mSpringConstant, real const mRestLenght);

    void updateForce(Particle::Ptr const p) override;

private:
    Vector3 const mAnchor;
    real const mSpringConstant;
    real const mRestLenght;
};

class ParticleBungee : public ParticleForceGenerator {
public:
    ParticleBungee(Particle::Ptr const mOther, real const mSpringConstant, real const mRestLenght);

    void updateForce(Particle::Ptr const p) override;

private:
    Particle::Ptr const mOther;
    real const mSpringConstant;
    real const mRestLenght;
};

class ParticleBuoyancy : public ParticleForceGenerator {
public:
    ParticleBuoyancy(real const mMaxDepth,  real const mVolume,
                     real const waterWight, real const mLiquidDensity);

    void updateForce(Particle::Ptr const p) override;

private:
    real const mMaxDepth;
    real const mVolume;
    real const mWaterHeight;
    real const mLiquidDensity;
};

class ParticleFakeSpring : public ParticleForceGenerator {
public:
    ParticleFakeSpring(Vector3 const& mAnchor, real const mSpringConstant, real const mDamping);

    void updateForce(Particle::Ptr const& p, real const mDuration) const;

    void updateForce(Particle::Ptr const p) override;

private:
    Vector3 const mAnchor;
    real const mSpringConstant;
    real const mDamping;
    real mDuration;
};

class BlobForceGenerator : public ParticleForceGenerator {
public:
    Particles& particles;
    real maxReplusion;
    real maxAttraction;
    real minNaturalDistance, maxNaturalDistance;
    real floatHead;
    unsigned int maxFloat;
    real maxDistance;

    explicit BlobForceGenerator(Particles& particles);

    void updateForce(Particle::Ptr const particle) override;
};

} // namespace pegasus

#endif // PEGASUS_PARTICLE_FORCE_GENERATOR_HPP
