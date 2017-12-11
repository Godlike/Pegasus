/*
 * Copyright (C) 2017 by Godlike
 * This code is licensed under the MIT license (MIT)
 * (http://opensource.org/licenses/MIT)
 *
 * Copyright (c) Icosagon 2003. All Rights Reserved.
 * This software is distributed under licence. Use of this software
 * implies agreement with all terms and conditions of the accompanying
 * software licence.
 */
#ifndef PEGASUS_FORCE_HPP
#define PEGASUS_FORCE_HPP

#include <pegasus/SharedMacros.hpp>
#include <pegasus/Body.hpp>

namespace pegasus
{
namespace force
{
/**
 * @brief Static field force calculation algorithm
 */
class StaticField
{
public:
    PEGASUS_EXPORT StaticField() = default;

    /**
     * @brief Constructs static field force instance
     * @param force direction and magnitude of the field force
     */
    PEGASUS_EXPORT explicit StaticField(glm::dvec3 force);

    /**
     * @brief Calculates force applied to the body
     * @param body body of interest
     * @return applied force
     */
    PEGASUS_EXPORT glm::dvec3 CalculateForce(mechanics::Body const& body) const;

private:
    glm::dvec3 m_force;
};

/**
 * @brief Drag force calculation algorithm
 */
class Drag
{
public:
    PEGASUS_EXPORT Drag() = default;

    /**
     * @brief Construct drag force calculation instance
     * @param k1 first factor of drag
     * @param k2 second factor of drag
     */
    PEGASUS_EXPORT Drag(double k1, double k2);

    /**
     * @brief Calculates drag force acting on the body
     *
     * The force is calculated by the following equation:
     * F = -normalize(v) * (k1*s + k2*s**2)
     * Where F is a force of drag, v and s are velocity and speed of the body.
     * @param body body data
     * @return drag force
     */
    PEGASUS_EXPORT glm::dvec3 CalculateForce(mechanics::Body const& body) const;

private:
    double m_k1;
    double m_k2;
};

/**
 * @brief Anchored spring force calculation algorithm
 */
class Spring
{
public:
    PEGASUS_EXPORT Spring() = default;

    /**
     * @brief Construct anchored spring calculation instance
     *
     * Simple linear spring model with one end fixed in the anchor position.
     * @param anchor anchor point of the spring
     * @param springConstant spring constant
     * @param restLength rest length of the spring
     */
    PEGASUS_EXPORT Spring(glm::dvec3 anchor, double springConstant, double restLength);

    /**
     * @brief Calculates current spring force acting on the body
     * @param body body attached to the other end of the spring
     * @return force acting on the body
     */
    PEGASUS_EXPORT glm::dvec3 CalculateForce(mechanics::Body const& body) const;

private:
    glm::dvec3 m_anchor;
    double m_springConstant;
    double m_restLength;
};

/**
 * @brief Bungee force calculation algorithm
 */
class Bungee
{
public:
    PEGASUS_EXPORT Bungee() = default;

    /**
     * @brief Constructs instance of the bungee force
     *
     * Acts as a linear spring when body-anchor distance is greater than rest length.
     * Does not apply any force otherwise.
     * @param anchor anchor point of the bungee
     * @param springConstant spring constant
     * @param restLength spring rest length
     */
    PEGASUS_EXPORT Bungee(glm::dvec3 anchor, double springConstant, double restLength);

    /**
     * @brief Calculates bungee force
     * @param body attached to the bungee
     * @return force vector
     */
    PEGASUS_EXPORT glm::dvec3 CalculateForce(mechanics::Body const& body) const;

private:
    glm::dvec3 m_anchor;
    double m_springConstant;
    double m_restLength;
};

/**
 * @brief Buoyancy force calculation algorithm
 */
class Buoyancy
{
public:
    PEGASUS_EXPORT Buoyancy() = default;

    /**
     * @brief Constructs instance of the buoyancy force
     * @param maxDepth depth of the reservoir
     * @param volume volume of the reservoir
     * @param waterWight reservoir water weight
     * @param liquidDensity density of the liquid in the reservoir
     */
    PEGASUS_EXPORT Buoyancy(double maxDepth, double volume, double waterWight, double liquidDensity);

    /**
     * @brief Calculates current buoyancy force
     * @param body body in the reservoir
     * @return buoyancy force
     */
    PEGASUS_EXPORT glm::dvec3 CalculateForce(mechanics::Body const& body) const;

private:
    double m_maxDepth;
    double m_volume;
    double m_waterHeight;
    double m_liquidDensity;
};
} // namespace force
} // namespace pegasus

#endif // PEGASUS_FORCE_HPP