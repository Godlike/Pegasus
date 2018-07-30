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

void Material::SetMass(float mass)
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
    m_momentOfInertia = glm::mat3(0);
    m_inverseMomentOfInertia = glm::mat3(0);
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

float Material::GetMass() const
{
    return m_mass;
}

float Material::GetInverseMass() const
{
    return m_inverseMass;
}

glm::mat3 Material::GetMomentOfInertia() const
{
    return m_momentOfInertia;
}

glm::mat3 Material::GetInverseMomentOfInertia() const
{
    return m_inverseMomentOfInertia;
}

} // namespace mechanics
} // namespace pegasus
