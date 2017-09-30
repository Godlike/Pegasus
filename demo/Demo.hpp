/*
* Copyright (C) 2017 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#ifndef PEGASUS_DEMO_HPP
#define PEGASUS_DEMO_HPP

#include <pegasus/ParticleWorld.hpp>
#include <demo/Render.hpp>

namespace pegasus
{
/**
 * @brief Represents a singletone instance of the Demo
 */
class Demo
{
public:
    struct Object;

    /**
     * @brief Returns a reference to the singletone Demo instance
     * @return Demo object reference
     */
    static Demo& GetInstance();

    /**
     * @brief Checks if the window is not closed and render is initialized
     * @return @c true if initialize, @c false otherwise
     */
    bool IsValid() const;

    /**
     * @brief Computes and renders frame
     */
    void RunFrame();

    /**
     * @brief Makes physical and render instance of the line and returns it
     * @param[in] particle physical data
     * @param[in] start first point of the line
     * @param[in] end last point of the line
     * @return created object reference
     */
    Object& MakeLine(Particle particle, glm::vec3 start, glm::vec3 end);

    /**
     * @brief Makes physical and render instance of the plane and returns it
     * @param[in] particle physical data
     * @param[in] normal normal of the vector
     * @return created object references
     */
    Object& MakePlane(Particle particle, glm::dvec3 normal);

    /**
     * @brief Makes physical and render instance of the sphere and returns it
     * @param[in] particle physical data
     * @param[in] radius radius of the sphere
     * @return created object reference
     */
    Object& MakeSphere(Particle particle, double radius);

    /**
     * @brief Makes physical and render instance of the bo and returns it
     * @param[in] particle physical data
     * @param[in] i orthogonal basis vector of the box base
     * @param[in] j orthogonal basis vector of the box base
     * @param[in] k orthogonal basis vector of the box base
     * @return created object refernce
     */
    Object& MakeBox(Particle particle, glm::vec3 i, glm::vec3 j, glm::vec3 k);

    /**
     * @brief Removes object from the demo
     * @param[in] object reference to the object to be deleted
     */
    void Remove(Object& object);

    //! Maximum number of particles in the demo
    uint32_t const maxParticles = 200;

    /**
     * @brief Represents an instance of the render and physical objects
     */
    struct Object
    {
        Object(RigidBody* body, std::unique_ptr<render::primitive::Primitive>&& renderShape);

        //! Physical body
        RigidBody* body;

        //! Render data
        std::unique_ptr<render::primitive::Primitive> shape;
    };

private:
    std::list<Object> m_objects;
    render::Renderer& m_pRenderer;
    ParticleWorld m_particleWorld;
    Particles m_particles;
    RigidBodies m_rigidBodies;
    ParticleContactGenerators m_particleContactGenerators;
    ParticleForceRegistry m_particleForceRegistry;
    ParticleGravity m_gravityForce;

    Demo();

    /**
     * @brief Runs physics calculations with given duration and updates the internal states
     * @param duration duration of the physical frame
     */
    void ComputeFrame(double duration);

    /**
     * @brief Renders frame and updates frame buffer
     */
    void RenderFrame() const;

    /**
     * @brief Makes rigid body updates internal structures and returns reference to it
     * @param[in] particle physical data
     * @param[in] shape a pointer to the collision geometry shape
     * @return reference to the created rigid body
     */
    RigidBody& MakeRigidBody(Particle particle, std::unique_ptr<geometry::SimpleShape>&& shape);
};
} // namespace pegasus

#endif // PEGASUS_DEMO_HPP
