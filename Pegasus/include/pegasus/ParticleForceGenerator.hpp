/*
* Copyright (c) Icosagon 2003. All Rights Reserved.
*
* This software is distributed under licence. Use of this software
* implies agreement with all terms and conditions of the accompanying
* software licence.
*/
#ifndef PEGASUS_PARTICLE_FORCE_GENERATOR_HPP
#define PEGASUS_PARTICLE_FORCE_GENERATOR_HPP

#include <pegasus/Object.hpp>

#include <glm/glm.hpp>

#include <map>
#include <set>

namespace pegasus
{
class ParticleForceGenerator
{
public:
    PEGASUS_EXPORT virtual ~ParticleForceGenerator()
    {
    }

    PEGASUS_EXPORT virtual void ApplyForce(mechanics::Body& p) = 0;
};

class ParticleForceRegistry
{
public:
    PEGASUS_EXPORT void Add(mechanics::Body& p, ParticleForceGenerator& pfg);
    PEGASUS_EXPORT void Remove(mechanics::Body& p);
    PEGASUS_EXPORT void Remove(mechanics::Body& p, ParticleForceGenerator& pfg);
    PEGASUS_EXPORT void Clear();
    PEGASUS_EXPORT void ApplyForces();

private:
    std::map<mechanics::Body*, std::set<ParticleForceGenerator*>> mRegistrations;
};

class ParticleGravity : public ParticleForceGenerator
{
public:
    PEGASUS_EXPORT explicit ParticleGravity(glm::dvec3 const& g);
    PEGASUS_EXPORT void ApplyForce(mechanics::Body& p) override;

private:
    glm::dvec3 const m_gravity;
};

class ParticleDrag : public ParticleForceGenerator
{
public:
    PEGASUS_EXPORT ParticleDrag(double k1, double k2);
    PEGASUS_EXPORT void ApplyForce(mechanics::Body& p) override;

private:
    double const m_k1;
    double const m_k2;
};

class ParticleSpring : public ParticleForceGenerator
{
public:
    PEGASUS_EXPORT ParticleSpring(mechanics::Body& other, double springConstant, double restLength);
    PEGASUS_EXPORT void ApplyForce(mechanics::Body& p) override;

private:
    mechanics::Body& m_other;
    double const m_springConstant;
    double const m_restLength;
};

class ParticleAnchoredSpring : public ParticleForceGenerator
{
public:
    PEGASUS_EXPORT ParticleAnchoredSpring(glm::dvec3 const& anchor, double springConstant, double restLength);
    PEGASUS_EXPORT void ApplyForce(mechanics::Body& p) override;

private:
    glm::dvec3 const m_anchor;
    double const m_springConstant;
    double const m_restLength;
};

class ParticleBungee : public ParticleForceGenerator
{
public:
    PEGASUS_EXPORT ParticleBungee(mechanics::Body& other, double springConstant, double restLength);
    PEGASUS_EXPORT void ApplyForce(mechanics::Body& p) override;

private:
    mechanics::Body& m_other;
    double const m_springConstant;
    double const m_restLength;
};

class ParticleBuoyancy : public ParticleForceGenerator
{
public:
    PEGASUS_EXPORT ParticleBuoyancy(double maxDepth, double volume,
        double waterWight, double liquidDensity);
    PEGASUS_EXPORT void ApplyForce(mechanics::Body& p) override;

private:
    double const m_maxDepth;
    double const m_volume;
    double const m_waterHeight;
    double const m_liquidDensity;
};

class ParticleFakeSpring : public ParticleForceGenerator
{
public:
    PEGASUS_EXPORT ParticleFakeSpring(glm::dvec3 const& anchor, double springConstant, double damping);
    PEGASUS_EXPORT void UpdateForce(mechanics::Body& p, double duration) const;
    PEGASUS_EXPORT void ApplyForce(mechanics::Body& p) override;

private:
    glm::dvec3 const m_anchor;
    double const m_springConstant;
    double const m_damping;
    double m_duration;
};
} // namespace pegasus

#endif // PEGASUS_PARTICLE_FORCE_GENERATOR_HPP
