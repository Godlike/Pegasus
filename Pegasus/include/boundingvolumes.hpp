/*
* Copyright (C) 2017 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#ifndef PEGASUS_BOUNDING_VOLUMES_HPP
#define PEGASUS_BOUNDING_VOLUMES_HPP

#include "Pegasus/include/geometry.hpp"

#include <functional>
#include <vector>
#include <set>
#include <array>
#include <memory>
#include <cmath>
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

Eigen::Vector3f calculateMeanVertex(
    volumes::Shape const & shape, volumes::Indices const & indices
);
Eigen::Matrix3f calculateCovarianceMatrix(
    volumes::Shape const & shape, volumes::Indices const & indices, Eigen::Vector3f const & mean
);
Eigen::Vector3f calculateEigenValues(
    Eigen::Matrix3f const & covariance
);
Eigen::Matrix3f calculateEigenVectors(
    Eigen::Matrix3f const & covariance
);
Eigen::Matrix3f calculateExtremalVertices(
    Eigen::Matrix3f const & eigen_vectors, volumes::Shape const & shape, volumes::Indices const & indices
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
        Eigen::Matrix3f eigen_vectors;
        Eigen::Matrix3f eigen_vectors_normalized;
        Eigen::Matrix3f extremal_vertices;
        Vertices cube_vertices;
    };

    OrientedBoundingBox(Shape const & shape, Indices const & indices);
    geometry::Box getBox() const;

private:
    geometry::Box m_box_shape;
    Box m_box;
    Shape const & m_shape;
    Indices const & m_indices;

    Vertices calculateBoxVertices(
            Eigen::Matrix3f const & extremal_points, Eigen::Matrix3f const & eigen_vectors
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
    geometry::Box getBox() const;

private:
    geometry::Box m_box_shape;
    Box m_box;
    Shape const & m_shape;
    Indices const & m_indices;

    static void calculateExtremalVetices(Shape const & shape, Indices const & indices, Box & box);
    static void calculateMean(Box & box);
    void createBox(Box & box);

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
        Eigen::Vector3f eigen_values;
        Eigen::Matrix3f eigen_vectors;
        Eigen::Matrix3f eigen_vectors_normalized;
    };

    BoundingSphere(Shape const & shape, Indices const & indices);
    geometry::Sphere getSphere() const;

private:
    geometry::Sphere m_sphere_shape;
    Sphere m_sphere;
    Shape   const & m_shape;
    Indices const & m_indices;

    static geometry::Sphere calculateBoundingSphere(
        Eigen::Matrix3f const & eigen_vectors, Eigen::Vector3f const & eigen_values,
        Shape const & shape, Indices const & indices
    );
    static geometry::Sphere refineSphere(
        geometry::Sphere const & sphere, Shape const & shape, Indices const & indices
    );
};
} // namespace sphere

} // namespace volumes
} // namespace geometry
} // namespace pegasus
#endif // PEGASUS_BOUNDING_VOLUMES_HPP
