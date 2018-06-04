/*
 * Copyright (C) 2018 by Godlike
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
    StaticField() = default;

    /**
     * @brief Constructs static field force instance
     * @param force direction and magnitude of the field force
     */
    explicit StaticField(glm::dvec3 force);

    /**
     * @brief Calculates force applied to the body
     * @param body body of interest
     * @return applied force
     */
    glm::dvec3 CalculateForce(mechanics::Body const& body) const;

private:
    glm::dvec3 m_force;
};

/**
 * @brief Square distance force calculation algorithm
 */
class SquareDistanceSource
{
public:
    SquareDistanceSource() = default;

    /**
     * @brief Constructs square distance source instance
     * @param magnitude    the magnitude of the force
     * @param centerOfMass the source of the force
     */
    explicit SquareDistanceSource(double magnitude, glm::dvec3 centerOfMass);

    /**
     * @brief Calculates force that is acting on the given body
     *
     * The strength of the force is in inverse ratio with the squared distance
     * from the force's source to the body's center of mass.
     *
     * @attention The distance beetween force and body must not be equal to zero
     * with respect to the system floating point threshold.
     * Otherwise the behavior is undefined.
     *
     * @param body target body
     * @return force acting on the targe body
     */
    glm::dvec3 CalculateForce(mechanics::Body const& body) const;

    //!Center of mass of the force
    glm::dvec3 centerOfMass;

private:
    double m_magnitude;
};

/**
 * @brief Drag force calculation algorithm
 */
class Drag
{
public:
    Drag() = default;

    /**
     * @brief Construct drag force calculation instance
     * @param k1 first factor of drag
     * @param k2 second factor of drag
     */
    Drag(double k1, double k2);

    /**
     * @brief Calculates drag force acting on the body
     *
     * The force is calculated by the following equation:
     * F = -normalize(v) * (k1*s + k2*s**2)
     * Where F is a force of drag, v and s are velocity and speed of the body.
     * @param body body data
     * @return drag force
     */
    glm::dvec3 CalculateForce(mechanics::Body const& body) const;

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
    Spring() = default;

    /**
     * @brief Construct anchored spring calculation instance
     *
     * Simple linear spring model with one end fixed in the anchor position.
     * @param anchor anchor point of the spring
     * @param springConstant spring constant
     * @param restLength rest length of the spring
     */
    Spring(glm::dvec3 anchor, double springConstant, double restLength);

    /**
     * @brief Calculates current spring force acting on the body
     * @param body body attached to the other end of the spring
     * @return force acting on the body
     */
    glm::dvec3 CalculateForce(mechanics::Body const& body) const;

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
    Bungee() = default;

    /**
     * @brief Constructs instance of the bungee force
     *
     * Acts as a linear spring when body-anchor distance is greater than rest length.
     * Does not apply any force otherwise.
     * @param anchor anchor point of the bungee
     * @param springConstant spring constant
     * @param restLength spring rest length
     */
    Bungee(glm::dvec3 anchor, double springConstant, double restLength);

    /**
     * @brief Calculates bungee force
     * @param body attached to the bungee
     * @return force vector
     */
    glm::dvec3 CalculateForce(mechanics::Body const& body) const;

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
    Buoyancy() = default;

    /**
     * @brief Constructs instance of the buoyancy force
     * @param maxDepth depth of the reservoir
     * @param volume volume of the reservoir
     * @param waterWight reservoir water weight
     * @param liquidDensity density of the liquid in the reservoir
     */
    Buoyancy(double maxDepth, double volume, double waterWight, double liquidDensity);

    /**
     * @brief Calculates current buoyancy force
     * @param body body in the reservoir
     * @return buoyancy force
     */
    glm::dvec3 CalculateForce(mechanics::Body const& body) const;

private:
    double m_maxDepth;
    double m_volume;
    double m_waterHeight;
    double m_liquidDensity;
};
} // namespace force
} // namespace pegasus

#endif // PEGASUS_FORCE_HPP
