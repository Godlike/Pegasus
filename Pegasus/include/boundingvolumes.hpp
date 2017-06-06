/*
* Copyright (C) 2017 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#ifndef PEGASUS_BOUNDING_VOLUMES_HPP
#define PEGASUS_BOUNDING_VOLUMES_HPP

#include "Pegasus/include/Geometry.hpp"

#include <functional>
#include <vector>
#include <set>
#include <array>
#include <Eigen/Eigen>
#include <Eigen/StdVector>

namespace pegasus {
namespace geometry {
namespace volumes {

using Vertices = std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>;
using Indices  = std::set<std::size_t>;
using Face     = std::array<std::size_t, 3>;
using Faces    = std::vector<Face>;
using Plane    = Eigen::Hyperplane<float, 3>;

struct Shape
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Shape(volumes::Vertices const & vertices, volumes::Faces const & indices);

    Vertices const & vertices;
    Faces    const & indices;
};

Eigen::Vector3f CalculateMeanVertex(
    volumes::Shape const & shape, volumes::Indices const & indices
);
Eigen::Matrix3f CalculateCovarianceMatrix(
    volumes::Shape const & shape, volumes::Indices const & indices, Eigen::Vector3f const & mean
);
Eigen::Vector3f CalculateEigenValues(
    Eigen::Matrix3f const & covariance
);
Eigen::Matrix3f CalculateEigenVectors(
    Eigen::Matrix3f const & covariance
);
Eigen::Matrix3f CalculateExtremalVertices(
    Eigen::Matrix3f const & eigenVectors, volumes::Shape const & shape, volumes::Indices const & indices
);

namespace obb {
class OrientedBoundingBox
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    struct Box
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        Eigen::Vector3f mean;
        Eigen::Matrix3f covariance;
        Eigen::Matrix3f eigenVectors;
        Eigen::Matrix3f eigenVectorsNormalized;
        Eigen::Matrix3f extremalVertices;
        Vertices cubeVertices;
    };

    OrientedBoundingBox(Shape const & shape, Indices const & indices);
    geometry::Box GetBox() const;

private:
    geometry::Box m_boxShape;
    Box m_box;
    Shape const & m_shape;
    Indices const & m_indices;

    Vertices CalculateBoxVertices(
        Eigen::Matrix3f const & extremalPoints, Eigen::Matrix3f const & eigenVectors
    ) const;
};
} // namespace obb

namespace aabb {
class AxisAlignedBoundingBox
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    struct Box
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        Eigen::Vector3f xMin;
        Eigen::Vector3f xMax;
        Eigen::Vector3f yMin;
        Eigen::Vector3f yMax;
        Eigen::Vector3f zMin;
        Eigen::Vector3f zMax;
        Eigen::Vector3f extremalMean;
        Eigen::Vector3f xAxis;
        Eigen::Vector3f yAxis;
        Eigen::Vector3f zAxis;
    };

    AxisAlignedBoundingBox(Shape const & shape, Indices const & indices);
    geometry::Box GetBox() const;

private:
    geometry::Box m_boxShape;
    Box m_box;
    Shape const & m_shape;
    Indices const & m_indices;

    static void CalculateExtremalVetices(Shape const & shape, Indices const & indices, Box & box);
    static void CalculateMean(Box & box);
    void CreateBox(Box & box);

};
} // namespace aabb

namespace sphere {
class BoundingSphere
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    struct Sphere
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        Eigen::Vector3f mean;
        Eigen::Matrix3f covariance;
        Eigen::Vector3f eigenValues;
        Eigen::Matrix3f eigenVectors;
        Eigen::Matrix3f eigenVectorsNormalized;
    };

    BoundingSphere(Shape const & shape, Indices const & indices);
    geometry::Sphere GetSphere() const;

private:
    geometry::Sphere m_sphereShape;
    Sphere m_sphere;
    Shape   const & m_shape;
    Indices const & m_indices;

    static geometry::Sphere CalculateBoundingSphere(
        Eigen::Matrix3f const & eigenVectors, Eigen::Vector3f const & eigenValues,
        Shape const & shape, Indices const & indices
    );
    static geometry::Sphere RefineSphere(
        geometry::Sphere const & sphere, Shape const & shape, Indices const & indices
    );
};
} // namespace sphere

} // namespace volumes
} // namespace geometry
} // namespace pegasus
#endif // PEGASUS_BOUNDING_VOLUMES_HPP
