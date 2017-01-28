#include "Pegas/include/obb.hpp"

#include <numeric>


obb::OrientedBoundingBox::OrientedBoundingBox(const obb::OrientedBoundingBox::Shape & shape, const obb::Indices & indices) :
    m_shape(shape), m_indices(indices)
{
    //Todo: implement convex hull step
    m_box.mean = calculateMeanVertex(m_shape, m_indices);
    m_box.covariance = calculateCovarianceMatrix(m_shape, m_indices, m_box.mean);
    m_box.eigen_vectors = calculateEigenVectors(m_box.covariance);
    m_box.extremal_vertices = calculateExtremalVertices(m_box.covariance, m_shape, m_indices);
    m_box.cube_vertices = calculateBoxVertices(m_box.extremal_vertices, m_box.eigen_vectors);
}

const obb::OrientedBoundingBox::Box &obb::OrientedBoundingBox::getBox() const
{
    return m_box;
}

const obb::OrientedBoundingBox::Shape &obb::OrientedBoundingBox::getShape() const
{
    return m_shape;
}

const obb::Vectors &obb::OrientedBoundingBox::getVertices() const
{
    return m_shape.vertices;
}

const obb::Faces &obb::OrientedBoundingBox::getIndices() const
{
    return m_shape.indices;
}

obb::Vector obb::OrientedBoundingBox::calculateMeanVertex(const obb::OrientedBoundingBox::Shape & shape, const obb::Indices & indices)
{
    Vector const mean = (1.0f / (3.0f * indices.size())
                         * std::accumulate(indices.begin(), indices.end(), Vector{ 0, 0, 0 },
                                           [&shape, &indices](Vector & s, std::size_t const & index)
    {
        return (s += shape.vertices[index]);
    }));

    return mean;
}

obb::Matrix obb::OrientedBoundingBox::calculateCovarianceMatrix(const obb::OrientedBoundingBox::Shape & shape, const obb::Indices & indices, const obb::Vector & mean)
{
    Matrix C = Matrix::Zero();

    for (auto & face : shape.indices)
    {
        Vector const p = shape.vertices[face[0]] - mean;
        Vector const q = shape.vertices[face[1]] - mean;
        Vector const r = shape.vertices[face[2]] - mean;

        for (size_t j = 0; j < 3; ++j)
        {
            for (size_t k = 0; k < 3; ++k)
            {
                float const v = p[j] * p[k] + q[j] * q[k] + r[j] * r[k];
                C(j, k) += v;
            }
        }
    }

    float const a = 1.0f / (3.0f * indices.size());
    for (size_t j = 0; j < 3; ++j)
    {
        for (size_t k = 0; k < 3; ++k)
        {
            C(j, k) *= a;
        }
    }

    return C;
}

obb::Matrix obb::OrientedBoundingBox::calculateEigenVectors(const obb::Matrix & covariance)
{
    Eigen::EigenSolver<Matrix> es(covariance);
    auto eigen = es.eigenvectors().real();
    eigen.normalize();
    return eigen;
}

obb::Matrix obb::OrientedBoundingBox::calculateExtremalVertices(
        const obb::Matrix & eigen_vectors, const obb::OrientedBoundingBox::Shape & shape, const obb::Indices & indices
    )
{
    Matrix extremal = Matrix::Zero();

    for (size_t i = 0; i < 3; ++i)
    {
        Vector const vec = eigen_vectors.col(i);
        auto const max_index = *std::max_element(indices.begin(), indices.end(),
                                                 [&vec, &shape](std::size_t const & i, std::size_t const & j) {
            return shape.vertices[i].dot(vec) < shape.vertices[j].dot(vec);
        });
        extremal.col(i) = shape.vertices[max_index];
    }

    return extremal;
}

obb::Vectors obb::OrientedBoundingBox::calculateBoxVertices(const obb::Matrix & extremal_points, const obb::Matrix & eigen_vectors)
{
    Vectors box(8, Vector{ 0, 0, 0 });

    std::array<Plane, 6> planes;
    for (size_t i = 0; i < 6; ++i)
    {
        unsigned int const index = (i < 3) ? i : i - 3;
        planes[i] = Plane(Vector(eigen_vectors.col(index)) * (i < 3 ? 1 : -1),
                          Vector(extremal_points.col(index)).dot(Vector(eigen_vectors.col(index))));
    }

    Faces faces = { {0, 1, 2}, {0, 1, 5}, {0, 2, 4}, {0, 4, 5}, {2, 3, 4}, {1, 2, 3}, {1, 3, 5}, {3, 4, 5} };
    for (size_t i = 0; i < faces.size(); ++i)
    {
        Eigen::Vector4f const v0 = planes[faces[i][0]].coeffs();
        Eigen::Vector4f const v1 = planes[faces[i][1]].coeffs();
        Eigen::Vector4f const v2 = planes[faces[i][2]].coeffs();

        Matrix A = Matrix::Identity();
        A.row(0) = v0.head<3>();
        A.row(1) = v1.head<3>();
        A.row(2) = v2.head<3>();

        Vector b;
        b << -v0[3], -v1[3], -v2[3];

        box[i] = A.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV).solve(b);
    }

    return box;
}
