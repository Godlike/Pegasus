/*
* Copyright (C) 2017 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#include "Pegasus/include/boundingvolumes.hpp"

using namespace pegasus;
using namespace geometry;
using namespace volumes;

obb::OrientedBoundingBox::OrientedBoundingBox(Shape const & shape, Indices const & indices)
    : m_box_shape({}, {}, {}, {})
    , m_shape(shape)
    , m_indices(indices)
{
    //Todo: implement convex hull step
    m_box.mean = calculateMeanVertex(m_shape, m_indices);
    m_box.covariance = calculateCovarianceMatrix(m_shape, m_indices, m_box.mean);
    m_box.eigen_vectors = calculateEigenVectors(m_box.covariance);
    m_box.eigen_vectors_normalized = m_box.eigen_vectors.normalized();
    m_box.extremal_vertices = calculateExtremalVertices(m_box.covariance, m_shape, m_indices);
    m_box.cube_vertices = calculateBoxVertices(m_box.extremal_vertices, m_box.eigen_vectors_normalized);

    m_box_shape = geometry::Box(
        { m_box.mean[0], m_box.mean[1], m_box.mean[2] }, 
        { m_box.eigen_vectors.col(0)[0], m_box.eigen_vectors.col(0)[1], m_box.eigen_vectors.col(0)[2] },
        { m_box.eigen_vectors.col(1)[0], m_box.eigen_vectors.col(1)[1], m_box.eigen_vectors.col(1)[2] },
        { m_box.eigen_vectors.col(2)[0], m_box.eigen_vectors.col(2)[1], m_box.eigen_vectors.col(2)[2] }
    );
}

geometry::Box obb::OrientedBoundingBox::getBox() const
{
    return m_box_shape;
}

Eigen::Vector3f
obb::OrientedBoundingBox::calculateMeanVertex(
    Shape const & shape, Indices const & indices)
{
    Eigen::Vector3f sum{ 0, 0, 0 };
    for (auto index : indices)
    {
        auto const & face = shape.indices[index];
        
        sum += shape.vertices[face[0]];
        sum += shape.vertices[face[1]];
        sum += shape.vertices[face[2]];
    }

    Eigen::Vector3f const mean = 1.0f / (3.0f * indices.size()) * sum;
    return mean;
}

Eigen::Matrix3f
obb::OrientedBoundingBox::calculateCovarianceMatrix(
    Shape const & shape, Indices const & indices, Eigen::Vector3f const & mean)
{
    Eigen::Matrix3f C = Eigen::Matrix3f::Zero();

    for (auto const & index : indices) 
    {
        auto const & face = shape.indices[index];
        
        Eigen::Vector3f const p = shape.vertices[face[0]] - mean;
        Eigen::Vector3f const q = shape.vertices[face[1]] - mean;
        Eigen::Vector3f const r = shape.vertices[face[2]] - mean;

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

Eigen::Matrix3f
obb::OrientedBoundingBox::calculateEigenVectors(Eigen::Matrix3f const & covariance)
{
    Eigen::EigenSolver<Eigen::Matrix3f> es(covariance);
    return es.eigenvectors().real();;
}

Eigen::Matrix3f
obb::OrientedBoundingBox::calculateExtremalVertices(
    Eigen::Matrix3f const & eigen_vectors, Shape const & shape, Indices const & indices)
{
    std::vector<Eigen::Vector3f const *, Eigen::aligned_allocator<Eigen::Vector3f const*>> vertices;
    for (auto index : indices)
    {
        auto const & face = shape.indices[index];
        vertices.insert(vertices.end(), { &shape.vertices[face[0]], &shape.vertices[face[1]], &shape.vertices[face[2]] });
    }

    Eigen::Matrix3f extremal = Eigen::Matrix3f::Zero();
    for (size_t i = 0; i < 3; ++i)
    {
        Eigen::Vector3f const vec = eigen_vectors.col(i);
        auto max_vertex = *std::max_element(vertices.begin(), vertices.end(),
            [&vec](auto a, auto b) { return a->dot(vec) < b->dot(vec); });
        extremal.col(i) = *max_vertex;
    }

    return extremal;
}

volumes::Vectors
obb::OrientedBoundingBox::calculateBoxVertices(
    Eigen::Matrix3f const & extremal_points, Eigen::Matrix3f const & eigen_vectors)
{
    Vectors box(8, Eigen::Vector3f{ 0, 0, 0 });

    std::array<Plane, 6> planes;
    for (size_t i = 0; i < 6; ++i)
    {
        unsigned int const index = (i < 3) ? i : i - 3;
        planes[i] = Plane(Eigen::Vector3f(eigen_vectors.col(index)) * (i < 3 ? 1 : -1),
            Eigen::Vector3f(extremal_points.col(index)).dot(Eigen::Vector3f(eigen_vectors.col(index))));
    }

    Faces faces = { { 0, 1, 2 },{ 0, 1, 5 },{ 0, 2, 4 },{ 0, 4, 5 },{ 2, 3, 4 },{ 1, 2, 3 },{ 1, 3, 5 },{ 3, 4, 5 } };
    for (size_t i = 0; i < faces.size(); ++i)
    {
        Eigen::Vector4f const v0 = planes[faces[i][0]].coeffs();
        Eigen::Vector4f const v1 = planes[faces[i][1]].coeffs();
        Eigen::Vector4f const v2 = planes[faces[i][2]].coeffs();

        Eigen::Matrix3f A = Eigen::Matrix3f::Identity();
        A.row(0) = v0.head<3>();
        A.row(1) = v1.head<3>();
        A.row(2) = v2.head<3>();

        Eigen::Vector3f b;
        b << -v0[3], -v1[3], -v2[3];

        box[i] = A.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV).solve(b);
    }

    return box;
}

aabb::AxisAlignedBoundingBox::AxisAlignedBoundingBox(const volumes::Shape & shape, Indices const & indices)
        : m_box_shape({}, {}, {}, {})
        , m_shape(shape)
        , m_indices(indices)
{
    calculateExtremalVetices(m_shape, indices, m_box);
    calculateMean(m_box);
    createBox(m_box);
}

geometry::Box aabb::AxisAlignedBoundingBox::getBox() const {
    return m_box_shape;
}

void aabb::AxisAlignedBoundingBox::calculateExtremalVetices(
        volumes::Shape const & shape, Indices const & indices, aabb::AxisAlignedBoundingBox::Box & box)
{
    static auto compareFaces = [&](auto axis_index, auto face_index_a, auto face_index_b) -> bool {
        static auto compareFaceIndices = [&](auto index_a, auto index_b) -> bool {
            return shape.vertices[index_a][axis_index] < shape.vertices[index_b][axis_index];
        };

        std::size_t const max_index_a = *std::max_element(
                shape.indices[face_index_a].begin(), shape.indices[face_index_a].end(), compareFaceIndices
        );

        std::size_t const max_index_b = *std::max_element(
                shape.indices[face_index_b].begin(), shape.indices[face_index_b].end(), compareFaceIndices
        );

        return     shape.vertices[shape.indices[face_index_a][max_index_a]][axis_index]
                 < shape.vertices[shape.indices[face_index_b][max_index_b]][axis_index];
    };

    static auto xCompareFaces = std::bind(compareFaces, 0, std::placeholders::_1, std::placeholders::_2);
    static auto yCompareFaces = std::bind(compareFaces, 1, std::placeholders::_1, std::placeholders::_2);
    static auto zCompareFaces = std::bind(compareFaces, 2, std::placeholders::_1, std::placeholders::_2);

    std::size_t const x_min_index = *std::min_element(indices.begin(), indices.end(), xCompareFaces);
    std::size_t const x_max_index = *std::max_element(indices.begin(), indices.end(), xCompareFaces);
    std::size_t const y_min_index = *std::min_element(indices.begin(), indices.end(), yCompareFaces);
    std::size_t const y_max_index = *std::max_element(indices.begin(), indices.end(), yCompareFaces);
    std::size_t const z_min_index = *std::min_element(indices.begin(), indices.end(), zCompareFaces);
    std::size_t const z_max_index = *std::max_element(indices.begin(), indices.end(), zCompareFaces);

    box.xMin = shape.vertices[shape.indices[x_min_index][0]];
    box.xMax = shape.vertices[shape.indices[x_max_index][0]];
    box.yMin = shape.vertices[shape.indices[y_min_index][1]];
    box.yMax = shape.vertices[shape.indices[y_max_index][1]];
    box.zMin = shape.vertices[shape.indices[z_min_index][2]];
    box.zMax = shape.vertices[shape.indices[z_max_index][2]];
}

void aabb::AxisAlignedBoundingBox::calculateMean(aabb::AxisAlignedBoundingBox::Box & box)
{
    box.extremalMean << (box.xMax + box.xMin) / 2.0f,
            (box.yMax + box.yMin) / 2.0f,
            (box.zMax + box.zMin) / 2.0f;
    box.xAxis = box.extremalMean - box.xMax;
    box.yAxis = box.extremalMean - box.yMax;
    box.zAxis = box.extremalMean - box.zMax;
}

void aabb::AxisAlignedBoundingBox::createBox(aabb::AxisAlignedBoundingBox::Box & box)
{
    m_box_shape = geometry::Box(
            {box.extremalMean[0], box.extremalMean[1], box.extremalMean[2]},
            {box.xAxis[0], box.xAxis[1], box.xAxis[2]},
            {box.yAxis[0], box.yAxis[1], box.yAxis[2]},
            {box.zAxis[0], box.zAxis[1], box.zAxis[2]}
    );
}
