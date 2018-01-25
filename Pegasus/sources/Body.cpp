/*
* Copyright (C) 2017 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#include <pegasus/Body.hpp>

namespace pegasus
{
namespace mechanics
{

Body::Material::Material()
    : damping(1)
    , m_mass(1)
    , m_inverseMass(1)
{
}

void Body::Material::SetMass(double mass)
{
    if (mass > 0)
    {
        m_mass = mass;
        m_inverseMass = 1 / mass;
    }
}

void Body::Material::SetInfiniteMass()
{
    m_mass = 0;
    m_inverseMass = 0;
}

bool Body::Material::HasInfiniteMass() const
{
    return m_inverseMass == 0;
}

double Body::Material::GetMass() const
{
    return m_mass;
}

double Body::Material::GetInverseMass() const
{
    return m_inverseMass;
}

Body::LinearMotion::LinearMotion()
    : position(0, 0, 0)
    , velocity(0, 0, 0)
    , acceleration(0, 0, 0)
    , force(0, 0, 0)
{
}

Body::Body()
    : material()
    , linearMotion()
{
}
} // namespace mechanics
} // namespace pegasus
