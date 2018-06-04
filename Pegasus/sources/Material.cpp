/*
* Copyright (C) 2018 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#include <pegasus/Material.hpp>

namespace pegasus
{
namespace mechanics
{

Material::Material()
    : damping(1)
    , m_momentOfInertia(1)
    , m_inverseMomentOfInertia(1)
    , m_mass(1)
    , m_inverseMass(1)
{
}

void Material::SetMass(double mass)
{
    if (mass > 0)
    {
        m_mass = mass;
        m_inverseMass = 1 / mass;
    }
}

void Material::SetInfiniteMass()
{
    m_mass = 0;
    m_inverseMass = 0;
    m_momentOfInertia = glm::dmat3(0);
    m_inverseMomentOfInertia = glm::dmat3(0);
}

void Material::SetMomentOfInertia(glm::mat3 momentOfInertia)
{
    m_momentOfInertia = momentOfInertia;
    m_inverseMomentOfInertia = glm::inverse(m_momentOfInertia);
}

void Material::SetInverseMomentOfInertia(glm::mat3 inverseMomentOfInertia)
{
    m_momentOfInertia = glm::inverse(inverseMomentOfInertia);
    m_inverseMomentOfInertia = inverseMomentOfInertia;
}

bool Material::HasInfiniteMass() const
{
    return m_inverseMass == 0;
}

double Material::GetMass() const
{
    return m_mass;
}

double Material::GetInverseMass() const
{
    return m_inverseMass;
}

glm::dmat3 Material::GetMomentOfInertia() const
{
    return m_momentOfInertia;
}

glm::dmat3 Material::GetInverseMomentOfInertia() const
{
    return m_inverseMomentOfInertia;
}

} // namespace mechanics
} // namespace pegasus
