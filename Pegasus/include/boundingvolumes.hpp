#ifndef PEGASUS_OBB_HPP
#define PEGASUS_OBB_HPP

#include "Eigen/Eigen"
#include "Eigen/StdVector"
#include <vector>
#include <set>
#include <array>
#include <memory>

#include "Pegasus/include/geometry.hpp"

namespace pegasus {
namespace geometry {
namespace obb {
    class OrientedBoundingBox
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        using Matrix = Eigen::Matrix3f;
        using Vector = Eigen::Vector3f;
        using Vectors = std::vector<Vector, Eigen::aligned_allocator<Vector>>;
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

            Vector mean;
            Matrix covariance;
            Matrix eigen_vectors;
            Matrix eigen_vectors_normalized;
            Matrix extremal_vertices;
            Vectors cube_vertices;
        };

    public:
        OrientedBoundingBox(Shape const & shape, Indices const & indices);
        geometry::Box getBox() const;

    private:
        geometry::Box m_box_shape;
        Box m_box;
        Shape const & m_shape;
        Indices const & m_indices;

        static Vector calculateMeanVertex(
            Shape const & shape, Indices const & indices);
        static Matrix calculateCovarianceMatrix(
            Shape const & shape, Indices const & indices, Vector const & mean);
        static Matrix calculateEigenVectors(
            Matrix const & covariance);
        static Matrix calculateExtremalVertices(
            Matrix const & eigen_vectors, Shape const & shape, Indices const & indices);
        static Vectors calculateBoxVertices(
            Matrix const & extremal_points, Matrix const & eigen_vectors);
    };

} // namespace obb
} // namespace geometry
} // namespace pegasus
#endif // PEGASUS_OBB_HPP
