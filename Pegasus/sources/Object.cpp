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
    m_mass = (inverseMass == 0.0) ? std::numeric_limits<double>::max() : (1.0 / inverseMass);
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

mechanics::Object::Object(Body& body, std::unique_ptr<geometry::SimpleShape>&& shape)
	: body(&body)
	, shape(std::move(shape))
{
}

mechanics::StaticObject::StaticObject(Body& body, std::unique_ptr<geometry::SimpleShape>&& shape)
    : Object(body, std::move(shape))
{
    body.material.SetInverseMass(0);
}

mechanics::DynamicObject::DynamicObject(Body& body, std::unique_ptr<geometry::SimpleShape>&& shape)
    : Object(body, std::move(shape))
{
}
