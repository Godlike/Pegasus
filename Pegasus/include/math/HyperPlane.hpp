/*
* Copyright (C) 2017 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#ifndef PEGASUS_MATH_HYPER_PLANE_HPP
#define PEGASUS_MATH_HYPER_PLANE_HPP

#include <pegasus/SharedMacros.hpp>
#include <glm/glm.hpp>

namespace pegasus
{
namespace math
{
/**
 * @brief HyperPlane calculation algorithm
 */
class HyperPlane
{
public:
    PEGASUS_EXPORT HyperPlane() = default;

    /**
     * @brief Constructs a plane in Hessian Normal Form
     *
     * Allows for the normal direction correction
     * using below the hyperplane point if one is specified.
     * @param[in] normal plane's normal vector of unit length
     * @param[in] point point on the plane
     * @param[in] below point below the plane, allows for the normal direction correction
     */
    PEGASUS_EXPORT HyperPlane(glm::dvec3 const& normal,
        glm::dvec3 const& point,
        glm::dvec3 const* below = nullptr
    );

    /**
     * @brief Constructs a plane in Hessian Normal Form
     *
     * Constructs a hyperplane from the given vertices
     * and allows for the normal direction correction
     * using below the hyperplane point if one is specified.
     * @param[in] a point on the plane
     * @param[in] b point on the plane
     * @param[in] c point on the plane
     * @param[in] below point below the plane, allows for the normal direction correction
     */
    PEGASUS_EXPORT HyperPlane(glm::dvec3 const& a,
        glm::dvec3 const& b,
        glm::dvec3 const& c,
        glm::dvec3 const* below = nullptr
    );

    /**
     * @brief Constructs a plane in Hessian Normal Form
     *
     * Constructs a hyperplane from the given vertices
     * and allows for the normal direction correction
     * using below the hyperplane point if one is specified.
     * @param[in] vertices points on the plane
     * @param[in] below point below the plane
     */
    PEGASUS_EXPORT HyperPlane(
        glm::dmat3 const& vertices,
        glm::dvec3 const* below = nullptr
    );

    /** brief Returns point on the plane */
    PEGASUS_EXPORT glm::dvec3 const& GetPoint() const;

    /** Returns plane normal vector */
    PEGASUS_EXPORT glm::dvec3 const& GetNormal() const;

    /** Returns a distance from the plane to the origin */
    PEGASUS_EXPORT double GetDistance() const;

    /** Sets the plane normal vector */
    PEGASUS_EXPORT void SetNormal(glm::dvec3 const& normal);

    /** Sets a point on the plane */
    PEGASUS_EXPORT void SetPoint(glm::dvec3 const& point);

    /**
     * @brief Calculates absolute distance from the plane to a point
     * @param[in] point the point of interest
     * @return absolute distance from the point to the plane
     */
    PEGASUS_EXPORT double Distance(glm::dvec3 const& point) const;

    /**
     * @brief Calculates signed distance from the plane to a point
     * @param[in] point the point of interest
     * @return signed distance from the plane to the point
     */
    PEGASUS_EXPORT double SignedDistance(glm::dvec3 const& point) const;

    /**
     * @brief Calculates whether a ray and the plane are intersecting
     * @param[in] rayNormal ray direction vector
     * @param[in] rayPoint point on the ray
     * @param[out] resultPoint intersection point
     * @return @c true if there is an intersection point, @c false otherwise
     */
    PEGASUS_EXPORT bool RayIntersection(
        glm::dvec3 const& rayNormal, glm::dvec3 const& rayPoint, glm::dvec3& resultPoint
    ) const;

    /**
     * @brief Calculates whether a line segment and the plane are intersecting
     * @param[in] lineStart start of the line segment
     * @param[in] lineEnd end of the line segment
     * @param[out] resultPoint intersection point
     * @return @c true if there is intersection point, @c false otherwise
     */
    PEGASUS_EXPORT bool LineSegmentIntersection(
        glm::dvec3 const& lineStart, glm::dvec3 const& lineEnd, glm::dvec3& resultPoint
    ) const;

    /**
     *  @brief  Calculates closest point on the plane to a given point
     *
     *  @param  point point of interest
     *
     *  @return closest point on the plane
     */
    PEGASUS_EXPORT glm::dvec3 ClosestPoint(glm::dvec3 const& point) const;

private:
    glm::dvec3 m_normal;
    glm::dvec3 m_point;
    double m_distance;
};
} // namespace math
} // namespace pegasus
#endif // PEGASUS_MATH_HYPER_PLANE_HPP
