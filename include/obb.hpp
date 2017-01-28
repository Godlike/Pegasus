#ifndef PEGAS_OBB_HPP
#define PEGAS_OBB_HPP

#include "Eigen/Eigen"
#include "Eigen/StdVector"
#include <vector>
#include <set>
#include <array>
#include <memory>


namespace obb {

    using Matrix = Eigen::Matrix3f;
    using Vector = Eigen::Vector3f;
    using Vectors = std::vector<Vector, Eigen::aligned_allocator<Vector>>;
    using Indices = std::set<std::size_t>;
    using Face = std::array<std::size_t, 3>;
    using Faces = std::vector<Face>;
    using Plane = Eigen::Hyperplane<float, 3>;

    class OrientedBoundingBox
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        using ConstPtr = std::shared_ptr<OrientedBoundingBox const>;
        using Ptr = std::shared_ptr<OrientedBoundingBox>;

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
            Matrix extremal_vertices;
            Vectors cube_vertices;
        };

        OrientedBoundingBox(Shape const & shape, Indices const & indices);

        Box const & getBox() const;

        Shape const & getShape() const;

        Vectors const & getVertices() const;

        Faces const & getIndices() const;

    private:
        Box m_box;
        Shape const m_shape;
        Indices const m_indices;

        static Vector calculateMeanVertex(Shape const & shape, Indices const & indices);

        static Matrix calculateCovarianceMatrix(Shape const & shape, Indices const & indices, Vector const & mean);

        static Matrix calculateEigenVectors(Matrix const & covariance);

        static Matrix calculateExtremalVertices(Matrix const & eigen_vectors, Shape const & shape, Indices const & indices);

        static Vectors calculateBoxVertices(Matrix const & extremal_points, Matrix const & eigen_vectors);
    };
} //namespace obb

#endif //PEGAS_OBB_HPP
