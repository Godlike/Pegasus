/*
* Copyright (C) 2017 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#include <pegasus/Object.hpp>

using namespace pegasus;

mechanics::Material::Material()
    : damping(1)
    , m_mass(1)
    , m_inverseMass(1)
{
}

void mechanics::Material::SetMass(double mass)
{
    if (mass > 0)
    {
        m_mass = mass;
        m_inverseMass = 1 / mass;
    }
}

void mechanics::Material::SetInverseMass(double inverseMass)
{
    m_inverseMass = inverseMass;
    m_mass = (inverseMass == 0.0) ? 0.0 : (1.0 / inverseMass);
}

double mechanics::Material::GetMass() const
{
    return m_mass;
}

double mechanics::Material::GetInverseMass() const
{
    return m_inverseMass;
}

mechanics::LinearMotion::LinearMotion()
    : position(0, 0, 0)
    , velocity(0, 0, 0)
    , acceleration(0, 0, 0)
    , force(0, 0, 0)
{
}

mechanics::Body::Body()
    : material()
    , linearMotion()
{
}
