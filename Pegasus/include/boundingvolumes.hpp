/*
* Copyright (C) 2017 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#ifndef PEGASUS_OBB_HPP
#define PEGASUS_OBB_HPP

#include <vector>
#include <set>
#include <array>
#include <memory>
#include <Eigen/Eigen>
#include <Eigen/StdVector>

#include "Pegasus/include/geometry.hpp"

namespace pegasus {
namespace geometry {
namespace obb {
    class OrientedBoundingBox
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        using Vectors = std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>;
        using Indices = std::set<std::size_t>;
        using Face = std::array<std::size_t, 3>;
        using Faces = std::vector<Face>;
        using Plane = Eigen::Hyperplane<float, 3>;

        struct Shape
        {
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW

            Vectors const vertices;
            Faces   const indices;
        };

        struct Box
        {
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW

            Eigen::Vector3f mean;
            Eigen::Matrix3f covariance;
            Eigen::Matrix3f eigen_vectors;
            Eigen::Matrix3f eigen_vectors_normalized;
            Eigen::Matrix3f extremal_vertices;
            Vectors cube_vertices;
        };

    private:
        geometry::Box m_box_shape;
        Box m_box;
        Shape const & m_shape;
        Indices const & m_indices;

    public:
        OrientedBoundingBox(Shape const & shape, Indices const & indices);
        geometry::Box getBox() const;

        static Eigen::Vector3f calculateMeanVertex(
            Shape const & shape, Indices const & indices);
        static Eigen::Matrix3f calculateCovarianceMatrix(
            Shape const & shape, Indices const & indices, Eigen::Vector3f const & mean);
        static Eigen::Matrix3f calculateEigenVectors(
            Eigen::Matrix3f const & covariance);
        static Eigen::Matrix3f calculateExtremalVertices(
            Eigen::Matrix3f const & eigen_vectors, Shape const & shape, Indices const & indices);
        static Vectors calculateBoxVertices(
            Eigen::Matrix3f const & extremal_points, Eigen::Matrix3f const & eigen_vectors);
    };

} // namespace obb
} // namespace geometry
} // namespace pegasus
#endif // PEGASUS_OBB_HPP
