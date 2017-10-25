/*
* Copyright (C) 2017 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#ifndef PEGASUS_BOUNDING_VOLUMES_HPP
#define PEGASUS_BOUNDING_VOLUMES_HPP

#include <pegasus/SharedMacros.hpp>
#include <geometry/Shape.hpp>
#include <geometry/Geometry.hpp>

#include <vector>
#include <set>
#include <array>

namespace pegasus
{
namespace geometry
{
namespace volumes
{
using Vertices = std::vector<glm::dvec3>;
using Indices  = std::set<std::size_t>;
using Face     = std::array<std::size_t, 3>;
using Faces    = std::vector<Face>;

struct Shape
{
    PEGASUS_EXPORT Shape(volumes::Vertices const& vertices, volumes::Faces const& indices);

    Vertices const& vertices;
    Faces const& indices;
};

PEGASUS_EXPORT glm::dvec3 CalculateMeanVertex(
    volumes::Shape const& shape, volumes::Indices const& indices
);
PEGASUS_EXPORT glm::dmat3 CalculateCovarianceMatrix(
    volumes::Shape const& shape, volumes::Indices const& indices, glm::dvec3 const& mean
);
PEGASUS_EXPORT glm::dmat3 CalculateExtremalVertices(
    glm::dmat3 const& eigenVectors, volumes::Shape const& shape, volumes::Indices const& indices
);

namespace obb
{
class OrientedBoundingBox
{
public:
    struct Box
    {
        glm::dvec3 mean;
        glm::dmat3 covariance;
        glm::dmat3 eigenVectors;
        glm::dmat3 eigenVectorsNormalized;
        glm::dmat3 extremalVertices;
        glm::dmat3 boxAxes;
    };

    PEGASUS_EXPORT OrientedBoundingBox(Shape const& shape, Indices const& indices);
    PEGASUS_EXPORT geometry::Box GetBox() const;

private:
    geometry::Box m_boxShape;
    Box m_box;
    Shape const& m_shape;
    Indices const& m_indices;
};
} // namespace obb

namespace aabb
{
class AxisAlignedBoundingBox
{
public:
    struct Box
    {
        glm::dvec3 xMin;
        glm::dvec3 xMax;
        glm::dvec3 yMin;
        glm::dvec3 yMax;
        glm::dvec3 zMin;
        glm::dvec3 zMax;
        glm::dvec3 extremalMean;
        glm::dvec3 xAxis;
        glm::dvec3 yAxis;
        glm::dvec3 zAxis;
    };

    PEGASUS_EXPORT AxisAlignedBoundingBox(Shape const& shape, Indices const& indices);
    PEGASUS_EXPORT geometry::Box GetBox() const;

private:
    geometry::Box m_boxShape;
    Box m_box;
    Shape const& m_shape;
    Indices const& m_indices;

    static void CalculateExtremalVetices(Shape const& shape, Indices const& indices, Box& box);
    static void CalculateMean(Box& box);
    void CreateBox(Box& box);
};
} // namespace aabb

namespace sphere
{
class BoundingSphere
{
public:
    struct Sphere
    {
        glm::dvec3 mean;
        glm::dmat3 covariance;
        glm::dvec3 eigenValues;
        glm::dmat3 eigenVectors;
        glm::dmat3 eigenVectorsNormalized;
    };

    PEGASUS_EXPORT BoundingSphere(Shape const& shape, Indices const& indices);
    PEGASUS_EXPORT geometry::Sphere GetSphere() const;

private:
    geometry::Sphere m_sphereShape;
    Sphere m_sphere;
    Shape const& m_shape;
    Indices const& m_indices;

    static geometry::Sphere CalculateBoundingSphere(
        glm::dmat3 const& eigenVectors, glm::dvec3 const& eigenValues,
        Shape const& shape, Indices const& indices
    );
    static geometry::Sphere RefineSphere(
        geometry::Sphere const& sphere, Shape const& shape, Indices const& indices
    );
};
} // namespace sphere
} // namespace volumes
} // namespace geometry
} // namespace pegasus
#endif // PEGASUS_BOUNDING_VOLUMES_HPP
