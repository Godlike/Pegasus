#ifndef PEGAS_PARTICLE_FORCE_GENERATOR_HPP
#define PEGAS_PARTICLE_FORCE_GENERATOR_HPP

#include "Pegas/include/particle.hpp"
#include <map>
#include <memory>
#include <vector>
#include <set>

namespace pegas {
class ParticleForceGenerator {
public:
    using Ptr = std::shared_ptr<ParticleForceGenerator>;
    using ConstPtr = std::shared_ptr<ParticleForceGenerator const>;

    virtual ~ParticleForceGenerator() {}

    virtual void updateForce(Particle::Ptr const& p) = 0;
};

class ParticleForceRegistry {
public:
    using Ptr = std::shared_ptr<ParticleForceRegistry>;

    void add(Particle::Ptr& p, ParticleForceGenerator::Ptr& pfg);

    void remove(Particle::Ptr const& p);

    void remove(Particle::Ptr const& p, ParticleForceGenerator::Ptr const& pfg);

    void clear();

    void updateForces();

private:
    using Registry = std::map<Particle::Ptr, std::set<ParticleForceGenerator::Ptr> >;
    Registry mRegistrations;
};

class ParticleGravity : public ParticleForceGenerator {
public:
    ParticleGravity(Vector3 const& g);

    virtual void updateForce(Particle::Ptr const& p) override;

private:
    Vector3 const mGravity;
};

class ParticleDrag : public ParticleForceGenerator {
public:
    ParticleDrag(real const mK1, real const mK2);

    virtual void updateForce(Particle::Ptr const& p) override;

private:
    real const mK1;
    real const mK2;
};

class ParticleSpring : public ParticleForceGenerator {
public:
    ParticleSpring(Particle::Ptr const& mOther, real const mSpringConstant,
        real const mRestLenght);

    virtual void updateForce(Particle::Ptr const& p) override;

private:
    Particle::Ptr const mOther;
    real const mSpringConstant;
    real const mRestLenght;
};

class ParticleAnchoredSpring : public ParticleForceGenerator {
public:
    ParticleAnchoredSpring(Vector3 const& mAnchor, real const mSpringConstant,
        real const mRestLenght);

    virtual void updateForce(Particle::Ptr const& p) override;

private:
    Vector3 const mAnchor;
    real const mSpringConstant;
    real const mRestLenght;
};

class ParticleBungee : public ParticleForceGenerator {
public:
    ParticleBungee(Particle::Ptr const& mOther, real const mSpringConstant,
        real const mRestLenght);

    virtual void updateForce(Particle::Ptr const& p) override;

private:
    Particle::Ptr const mOther;
    real const mSpringConstant;
    real const mRestLenght;
};

class ParticleBuoyancy : public ParticleForceGenerator {
public:
    ParticleBuoyancy(real const mMaxDepth, real const mVolume,
        real const waterWight, real const mLiquidDensity);

    virtual void updateForce(Particle::Ptr const& p) override;

private:
    real const mMaxDepth;
    real const mVolume;
    real const mWaterHeight;
    real const mLiquidDensity;
};

class ParticleFakeSpring : public ParticleForceGenerator {
public:
    ParticleFakeSpring(Vector3 const& mAnchor, real const mSpringConstant,
        real const mDamping);

    void updateForce(Particle::Ptr const& p, real const mDuration);

    virtual void updateForce(Particle::Ptr const& p) override;

private:
    Vector3 const mAnchor;
    real const mSpringConstant;
    real const mDamping;
    real mDuration;
};

class BlobForceGenerator : public ParticleForceGenerator {
public:
    std::vector<Particle::Ptr>& particles;
    real maxReplusion;
    real maxAttraction;
    real minNaturalDistance, maxNaturalDistance;
    real floatHead;
    unsigned int maxFloat;
    real maxDistance;

    BlobForceGenerator(std::vector<Particle::Ptr>& particles);

    virtual void updateForce(Particle::Ptr const& particle);
};

} // namespace pegas

#endif // PEGAS_PARTICLE_FORCE_GENERATOR_HPP
