/*
* Copyright (C) 2017 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#ifndef PEGASUS_GJK_HPP
#define PEGASUS_GJK_HPP

#include <geometry/ConfigurationSpaceObject.hpp>
#include <math/FloatingPoint.hpp>
#include <glm/glm.hpp>
#include <algorithm>
#include <array>
#include <vector>

namespace pegasus
{
namespace geometry
{
namespace intersection
{
namespace gjk
{
/** Simplex data container */
struct Simplex
{
    //! Simplex vertices. Note that some vertices may be unused
    std::array<glm::dvec3, 4> vertices;

    //! Indicated number of used vertices
    uint8_t size;
};

/**
*  @brief  Checks if simplex contains origin
*
*  @attention  Must only be called on a simplex of size in range [2; 4]
*
*  @param  simplex simplex data
*
*  @return @c true if simplex contains origin @c false otherwise
*/
bool SimplexContainsOrigin(Simplex const& simplex);

/**
*  @brief  Calculates nearest simplex to the origin
*
*  Presumes that simplex vertices are stored in a way such that the latest
*  added vertex has index @c simplexSize - 1
*
*  Given simplex may be reduced down to size 1 as a result of this method.
*
*  @param[in,out]  simplex simplex data
*
*  @return new search direction
*/
glm::dvec3 NearestSimplex(Simplex& simplex);

/**
*  @brief Checks if simplex contains origin
*
*  If simplex does not contain origin it is replaced by a new sub simplex
*  that is closest to the origin
*
*  @param[in,out]  simplex     current simplex
*  @param[in,out]  direction   current search direction
*
*  @return @c true if simplex contains origin, @c false otherwise
*/
bool DoSimplex(gjk::Simplex& simplex, glm::dvec3& direction);

/**
*  @brief  Calculates a tetrahedron from the CSO such that it contains the origin
*
*  If simplex contains origin then there is intersection between given shapes
*
*  @tparam ShapeA  any shape type for which gjk::Support is overloaded
*  @tparam ShapeB  any shape type for which gjk::Support is overloaded
*
*  @param[in,out]  simplex     initial simplex
*  @param[in]      aShape      reference to the shape object
*  @param[in]      bShape      reference to the shape object
*  @param[in]      direction   initial search direction vector of unit length
*
*  @return @c true if simplex contains origin, @c false otherwise
*/
template <typename ShapeA, typename ShapeB>
bool CalculateSimplex(Simplex& simplex, ShapeA const& aShape, ShapeB const& bShape, glm::dvec3 direction)
{
    std::vector<double> distances;

    do
    {
        //Add new vertex to the simplex
        simplex.vertices[simplex.size++] = cso::Support(aShape, bShape, direction);

        //Calculate if the new vertex is past the origin
        double const scalarDirectionProjection = glm::dot(simplex.vertices[simplex.size - 1], direction);
        if (math::fp::IsLess(scalarDirectionProjection, 0.0))
        {
            return false;
        }

        //Endless loop
        if (std::find(distances.begin(), distances.end(), scalarDirectionProjection) != distances.end())
        {
            return false;
        }
        distances.push_back(scalarDirectionProjection);

    } while (!DoSimplex(simplex, direction));

    return true;
}

/**
*  @brief  Checks if two shapes are intersecting using GJK algorithm
*
*  @tparam ShapeA  any shape type for which gjk::Support is overloaded
*  @tparam ShapeB  any shape type for which gjk::Support is overloaded
*
*  @param  aShape  reference to the shape object
*  @param  bShape  reference to the shape object
*
*  @return @c true if there is intersection, @c false otherwise
*
*  @sa CalculateSimplex, CalculateIntersection(Simplex& simplex, ShapeA const& aShape, ShapeB const& bShape)
*/
template <typename ShapeA, typename ShapeB>
bool CalculateIntersection(ShapeA const& aShape, ShapeB const& bShape)
{
    return CalculateIntersection(Simplex(), aShape, bShape);
}

/**
*  @brief  Checks if two shapes are intersecting using GJK algorithm
*
*  @tparam ShapeA  any shape type for which gjk::Support is overloaded
*  @tparam ShapeB  any shape type for which gjk::Support is overloaded
*
*  @param[out] simplex tetrahedron from CSO points containing the origin if one exists
*  @param[in]  aShape  reference to the shape object
*  @param[in]  bShape  reference to the shape object
*
*  @return @c true if there is intersection, @c false otherwise
*
*  @sa CalculateSimplex, CalculateIntersection(ShapeA const& aShape, ShapeB const& bShape)
*/
template <typename ShapeA, typename ShapeB>
bool CalculateIntersection(Simplex& simplex, ShapeA const& aShape, ShapeB const& bShape)
{
    simplex = {
        {{ cso::Support(aShape, bShape, glm::normalize(glm::dvec3{1,1,1})) }},
        1
    };

    return CalculateSimplex(simplex, aShape, bShape, glm::normalize(-simplex.vertices[0]));
}
} // namespace gjk
} // namespace intersection
} // namespace geometry
} // namespace pegasus
#endif // PEGASUS_GJK_HPP
