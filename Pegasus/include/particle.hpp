#ifndef PEGASUS_PARTICLE_HPP
#define PEGASUS_PARTICLE_HPP

#include "Pegasus/include/math.hpp"
#include <memory>
#include <vector>

namespace pegasus {

class Particle {
public:
    using Ptr = std::shared_ptr<Particle>;
    using ConstPtr = std::shared_ptr<Particle const>;

public:
    Particle();

    void integrate(double const duration);

    Vector3 getPosition() const;

    void setPosition(Vector3 const& position);

    void setPosition(double const x, double const y, double const z);

    Vector3 getVelocity() const;

    void setVelocity(Vector3 const& velocity);

    void setVelocity(double const x, double const y, double const z);

    Vector3 getAcceleration() const;

    void setAcceleration(Vector3 const& acceleration);

    void setAcceleration(double const x, double const y, double const z);

    double getDamping() const;

    void setDamping(double const damping);

    double getMass() const;

    void setMass(double const mass);

    bool hasFiniteMass() const;

    double getInverseMass() const;

    void setInverseMass(double const inverseMass);

    void addForce(Vector3 const& force);

    void clearForceAccum();

private:
    Vector3 mPosition;

    Vector3 mVelocity;

    Vector3 mAcceleration;

    double mDamping;

    double mMass;

    double mInverseMass;

    Vector3 mForceAccum;
};

using Particles = std::vector<Particle::Ptr>;

} // namespace pegasus

#endif // PEGASUS_PARTICLE_HPP
