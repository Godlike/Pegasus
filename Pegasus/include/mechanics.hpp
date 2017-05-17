#ifndef PEGASUS_MECHANICS_HPP
#define PEGASUS_MECHANICS_HPP

#include "Pegasus/include/geometry.hpp"
#include "Pegasus/include/particle.hpp"

#include <memory>
#include <vector>

namespace pegasus {

class RigidBody {
public:
    Particle * p;
    std::unique_ptr<geometry::SimpleShape> s;

public:
    RigidBody(Particle & p, std::unique_ptr<geometry::SimpleShape> && s)
        : p(&p)
        , s(std::move(s))
    {
    }
};

} // namespace pegasus
#endif // PEGASUS_MECHANICS_HPP
