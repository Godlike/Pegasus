/*
* Copyright (c) Icosagon 2003. All Rights Reserved.
*
* This software is distributed under licence. Use of this software
* implies agreement with all terms and conditions of the accompanying
* software licence.
*/
#ifndef PEGASUS_FORCE_HPP
#define PEGASUS_FORCE_HPP

#include <pegasus/Object.hpp>

namespace pegasus
{
namespace force
{
class StaticField
{
public:
    PEGASUS_EXPORT explicit StaticField(glm::dvec3 const& g);
    PEGASUS_EXPORT glm::dvec3 CalculateForce(mechanics::Body& body) const;

private:
    glm::dvec3 m_force;
};

class Drag
{
public:
    PEGASUS_EXPORT Drag(double k1, double k2);
    PEGASUS_EXPORT glm::dvec3 CalculateForce(mechanics::Body& body) const;

private:
    double m_k1;
    double m_k2;
};

class Spring
{
public:
    PEGASUS_EXPORT Spring(mechanics::Body& other, double springConstant, double restLength);
    PEGASUS_EXPORT glm::dvec3 CalculateForce(mechanics::Body& body) const;

private:
    mechanics::Body& m_other;
    double m_springConstant;
    double m_restLength;
};

class AnchoredSpring
{
public:
    PEGASUS_EXPORT AnchoredSpring(glm::dvec3 const& anchor, double springConstant, double restLength);
    PEGASUS_EXPORT glm::dvec3 CalculateForce(mechanics::Body& body) const;

private:
    glm::dvec3 m_anchor;
    double m_springConstant;
    double m_restLength;
};

class Bungee
{
public:
    PEGASUS_EXPORT Bungee(mechanics::Body& other, double springConstant, double restLength);
    PEGASUS_EXPORT glm::dvec3 CalculateForce(mechanics::Body& body) const;

private:
    mechanics::Body& m_other;
    double m_springConstant;
    double m_restLength;
};

class Buoyancy
{
public:
    PEGASUS_EXPORT Buoyancy(double maxDepth, double volume, double waterWight, double liquidDensity);
    PEGASUS_EXPORT glm::dvec3 CalculateForce(mechanics::Body& body) const;

private:
    double m_maxDepth;
    double m_volume;
    double m_waterHeight;
    double m_liquidDensity;
};
} // namespace force
} // namespace pegasus

#endif // PEGASUS_FORCE_HPP
