/*
* Copyright (C) 2017 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#ifndef PEGASUS_DEMO_HPP
#define PEGASUS_DEMO_HPP

#include "demo/Render.hpp"
#include <pegasus/Scene.hpp>
#include <list>
#include <memory>

namespace pegasus
{
/** Represents a singleton instance of the Demo */
class Demo
{
public:
    struct Primitive;

    /**
     * @brief Returns a reference to the singleton Demo instance
     * @return Demo object reference
     */
    static Demo& GetInstance();

    /**
     * @brief Checks if the window is not closed and render is initialized
     * @return @c true if initialized, @c false otherwise
     */
    bool IsValid() const;

    /**
     * @brief Updates frame
     *
     * Runs physics calculations, renders new frame and swaps buffers
     */
    void RunFrame();

    /**
     * @brief Creates an object describing a line
     *
     * The object contains only render-related part and is not registered in the physics world
     *
     * @param[in] body physical data
     * @param[in] start line start
     * @param[in] end line end
     * @return a newly created Object
     */
    Primitive& MakeLine(mechanics::Body body, glm::vec3 start, glm::vec3 end);

    /**
     * @brief Creates an object describing a plane
     *
     * Body::linearMotion::position is used as the position of the object,
     * both physical and graphical
     *
     * @param[in] body physical data
     * @param[in] normal normal of the vector
     * @return a newly created Object
     */
    Primitive& MakePlane(mechanics::Body body, glm::dvec3 normal, scene::Primitive::Type type);

    /**
     * @brief Creates an object describing a sphere
     *
     * Body::linearMotion::position is used as the position of the object,
     * both physical and graphical
     *
     * @param[in] body physical data
     * @param[in] radius radius of the sphere
     * @return a newly created Object
     */
    Primitive& MakeSphere(mechanics::Body body, double radius, scene::Primitive::Type type);

    /**
     * @brief Creates an object describing a box
     *
     * Body::linearMotion::position is used as the position of the object,
     * both physical and graphical
     *
     * @param[in] body physical data
     * @param[in] i orthogonal basis vector of the box base
     * @param[in] j orthogonal basis vector of the box base
     * @param[in] k orthogonal basis vector of the box base
     * @return a newly created Object
     */
    Primitive& MakeBox(mechanics::Body body, glm::vec3 i, glm::vec3 j, glm::vec3 k, scene::Primitive::Type type);

    /**
     * @brief Removes object from the demo
     * @param[in] object reference to the object to be deleted
     */
    void Remove(Primitive& object);

    //! Maximum number of particles in the demo
    uint32_t const maxObjects = 20;

    /**
     * @brief Represents an instance of the render and physical objects
     */
    struct Primitive
    {
        /**
         * @brief Constructs Primitive instance
         *
         * @note Assumes ownership of @p shape and @p body
         * @param[in] body physical scene object instance
         * @param[in] shape render scene object instance
         */
        Primitive(scene::Primitive* body, render::Primitive* shape);

        //! Physics data
        std::unique_ptr<scene::Primitive> physicalPrimitive;

        //! Render data
        std::unique_ptr<render::Primitive> renderPrimitive;
    };

private:
    scene::Scene& m_scene;
    render::Renderer& m_renderer;
    std::list<Primitive> m_primitives;
    std::unique_ptr<scene::Force<force::StaticField>> m_pGravityForce;

    Demo();

    /**
     * @brief Runs physics calculations with given duration
     * @param duration duration of the physical frame
     */
    void ComputeFrame(double duration);

    /**
     * @brief Redraws the scene
     *
     * Rerenders the frame and updates current frame buffer
     */
    void RenderFrame() const;
};
} // namespace pegasus

#endif // PEGASUS_DEMO_HPP
