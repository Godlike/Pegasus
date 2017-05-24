#include "Pegasus/include/mechanics.hpp"

using namespace pegasus;

pegasus::RigidBody::RigidBody(Particle & p, std::unique_ptr<geometry::SimpleShape> && s)
    : p(&p)
    , s(std::move(s))
{
}
