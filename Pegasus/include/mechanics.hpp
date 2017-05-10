#ifndef PEGAS_MECHANICS_HPP
#define PEGAS_MECHANICS_HPP

#include "Pegasus/include/geometry.hpp"
#include "Pegasus/include/particle.hpp"

#include <memory>
#include <vector>

namespace pegasus {

class Body {
public:
    virtual ~Body() {}
};

class RigidBody : public Body {
public:
    using Ptr = std::shared_ptr<RigidBody>;

    RigidBody(std::shared_ptr<Particle> const p, std::shared_ptr<geometry::SimpleShape> const s)
        : p(p)
        , s(s)
    {
    }

    std::shared_ptr<Particle> const p;
    std::shared_ptr<geometry::SimpleShape> const s;
};

using RigidBodies = std::vector<RigidBody::Ptr>;

} // namespace pegasus
#endif // PEGAS_MECHANICS_HPP
