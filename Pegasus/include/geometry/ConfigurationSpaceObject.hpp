/*
* Copyright (C) 2017 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#ifndef PEGASUS_CSO_HPP
#define PEGASUS_CSO_HPP

#include <geometry/Shape.hpp>

namespace pegasus
{
namespace geometry
{
namespace intersection
{
namespace cso
{
/**
*  @brief  Calculates farthest vertex on the surface of the sphere in given direction
*
*  @param  sphere      shape object
*  @param  direction   normalized search vector
*
*  @return point on the surface
*/
glm::dvec3 Support(Sphere const& sphere, glm::dvec3 direction);

/**
*  @brief  Calculates farthest vertex on the surface of the box in given direction
*
*  @param  box         shape object
*  @param  direction   normalized search vector
*
*  @return point on the surface
*/
glm::dvec3 Support(Box const& box, glm::dvec3 direction);

/**
*  @brief  Calculates farthest vertex on the surface of the Configuration Space
*          Object in given direction
*
*  Configuration Space Object (aka Minkowski Difference and Minkowski
*  Configuration Object) is a Cartesian product of two sets of points, where
*  each element in one of the sets is multiplied by -1.
*
*  @tparam ShapeA      any shape type for which gjk::Support is overloaded
*  @tparam ShapeB      any shape type for which gjk::Support is overloaded
*
*  @param  aShape      shape object
*  @param  bShape      shape object
*  @param  direction   normalized search vector
*
*  @return point on the surface
*/
template < typename ShapeA, typename ShapeB >
glm::dvec3 Support(ShapeA const& aShape, ShapeB const& bShape, glm::dvec3 direction)
{
    glm::dvec3 const aShapeSupportVertex = Support(aShape, direction);
    glm::dvec3 const bShapeSupportVertex = Support(bShape, -direction);
    glm::dvec3 const vertex = aShapeSupportVertex - bShapeSupportVertex;
    return vertex;
}
} // namespace cso
} // namespace intersection
} // namespace geometry
} // namespace pegasus
#endif // PEGASUS_CSO_HPP
