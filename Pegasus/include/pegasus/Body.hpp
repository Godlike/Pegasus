/*
* Copyright (C) 2017 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#ifndef PEGASUS_OBJECT_HPP
#define PEGASUS_OBJECT_HPP

#include <pegasus/SharedMacros.hpp>
#include <geometry/Geometry.hpp>

namespace pegasus
{
namespace mechanics
{
struct Body
{
    struct Material
    {
        PEGASUS_EXPORT Material();

        PEGASUS_EXPORT void SetMass(double mass);

        PEGASUS_EXPORT void SetInverseMass(double inverseMass);

        PEGASUS_EXPORT double GetMass() const;

        PEGASUS_EXPORT double GetInverseMass() const;

        double damping;

    private:
        double m_mass;
        double m_inverseMass;
    };

    struct LinearMotion
    {
        PEGASUS_EXPORT LinearMotion();

        glm::dvec3 position;
        glm::dvec3 velocity;
        glm::dvec3 acceleration;
        glm::dvec3 force;
    };

    PEGASUS_EXPORT Body();

    Material material;
    LinearMotion linearMotion;
};
} // namespace mechanics
} // namespace pegasus
#endif // PEGASUS_OBJECT_HPP
