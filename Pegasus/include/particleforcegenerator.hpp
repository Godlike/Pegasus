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

    virtual void updateForce(Particle & p) = 0;
};

class ParticleForceRegistry {
public:
    using Ptr = std::shared_ptr<ParticleForceRegistry>;
    using Registry = std::map<Particle*, std::set<ParticleForceGenerator*> >;

public:
    void add(Particle & p, ParticleForceGenerator & pfg);

    void remove(Particle & p);

    void remove(Particle & p, ParticleForceGenerator & pfg);

    void clear();

    void updateForces();

private:
    Registry mRegistrations;
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

class BlobForceGenerator : public ParticleForceGenerator {
public:
    Particles & particles;
    double maxReplusion;
    double maxAttraction;
    double minNaturalDistance, maxNaturalDistance;
    double floatHead;
    unsigned int maxFloat;
    double maxDistance;

    explicit BlobForceGenerator(Particles& particles);

    void updateForce(Particle & particle) override;
};

} // namespace pegasus

#endif // PEGASUS_PARTICLE_FORCE_GENERATOR_HPP
