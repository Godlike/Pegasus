#ifndef PEGASUS_MECHANICS_HPP
#define PEGASUS_MECHANICS_HPP

#include "Pegasus/include/geometry.hpp"
#include "Pegasus/include/particle.hpp"

#include <memory>
#include <vector>

namespace pegasus {

class RigidBody {
public:
    using Ptr = std::shared_ptr<RigidBody>;
    std::shared_ptr<Particle> const p;
    std::shared_ptr<geometry::SimpleShape> const s;

public:
    RigidBody(std::shared_ptr<Particle> const p, std::shared_ptr<geometry::SimpleShape> const s)
        : p(p)
        , s(s)
    {
    }
};

using RigidBodies = std::vector<RigidBody::Ptr>;

} // namespace pegasus
#endif // PEGASUS_MECHANICS_HPP
