/*
* Copyright (C) 2017 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#include <pegasus/Body.hpp>

using namespace pegasus;

mechanics::Body::Material::Material()
    : damping(1)
    , m_mass(1)
    , m_inverseMass(1)
{
}

void mechanics::Body::Material::SetMass(double mass)
{
    if (mass > 0)
    {
        m_mass = mass;
        m_inverseMass = 1 / mass;
    }
}

void mechanics::Body::Material::SetInverseMass(double inverseMass)
{
    m_inverseMass = inverseMass;
    m_mass = (inverseMass == 0.0) ? 0.0 : (1.0 / inverseMass);
}

double mechanics::Body::Material::GetMass() const
{
    return m_mass;
}

double mechanics::Body::Material::GetInverseMass() const
{
    return m_inverseMass;
}

mechanics::Body::LinearMotion::LinearMotion()
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
