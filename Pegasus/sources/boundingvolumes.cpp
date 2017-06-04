/*
* Copyright (C) 2017 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#include "Pegasus/include/boundingvolumes.hpp"

#include <functional>

using namespace pegasus;
using namespace geometry;
using namespace volumes;

volumes::Shape::Shape(volumes::Vertices const & vertices, volumes::Faces const & indices)
        : vertices(vertices), indices(indices)
{
}

Eigen::Vector3f volumes::calculateMeanVertex(volumes::Shape const & shape, volumes::Indices const & indices) {
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

Eigen::Matrix3f volumes::calculateCovarianceMatrix(
        volumes::Shape const & shape, volumes::Indices const & indices, Eigen::Vector3f const & mean)
{
    Eigen::Matrix3f C = Eigen::Matrix3f::Zero();

    for (auto index : indices)
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

Eigen::Vector3f volumes::calculateEigenValues(Eigen::Matrix3f const & covariance)
{
    Eigen::EigenSolver<Eigen::Matrix3f> es(covariance);
    return es.eigenvalues().real();
}

Eigen::Matrix3f volumes::calculateEigenVectors(Eigen::Matrix3f const & covariance)
{
    Eigen::EigenSolver<Eigen::Matrix3f> es(covariance);
    return es.eigenvectors().real();;
}

Eigen::Matrix3f volumes::calculateExtremalVertices(
        Eigen::Matrix3f const & eigen_vectors, volumes::Shape const & shape, volumes::Indices const & indices)
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

obb::OrientedBoundingBox::OrientedBoundingBox(volumes::Shape const & shape, volumes::Indices const & indices)
    : m_box_shape({}, {}, {}, {})
    , m_shape(shape)
    , m_indices(indices)
{
    //Todo: implement convex hull step
    m_box.mean = volumes::calculateMeanVertex(m_shape, m_indices);
    m_box.covariance = volumes::calculateCovarianceMatrix(m_shape, m_indices, m_box.mean);
    m_box.eigen_vectors = volumes::calculateEigenVectors(m_box.covariance);
    m_box.extremal_vertices = volumes::calculateExtremalVertices(m_box.covariance, m_shape, m_indices);
    m_box.eigen_vectors_normalized = m_box.eigen_vectors.normalized();
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

volumes::Vertices
obb::OrientedBoundingBox::calculateBoxVertices(
    Eigen::Matrix3f const & extremal_points, Eigen::Matrix3f const & eigen_vectors) const
{
    Vertices box(8, Eigen::Vector3f{ 0, 0, 0 });

    std::array<Plane, 6> planes;
    for (size_t i = 0; i < 6; ++i)
    {
        size_t const index = (i < 3) ? i : i - 3;
        planes[i] = Plane(Eigen::Vector3f(eigen_vectors.col(index)) * (i < 3 ? 1 : -1),
            Eigen::Vector3f(extremal_points.col(index)).dot(Eigen::Vector3f(eigen_vectors.col(index))));
    }

    Faces faces = { { 0, 1, 2 }, { 0, 1, 5 }, { 0, 2, 4 }, { 0, 4, 5 },
                    { 2, 3, 4 }, { 1, 2, 3 }, { 1, 3, 5 }, { 3, 4, 5 } };
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

aabb::AxisAlignedBoundingBox::AxisAlignedBoundingBox(
    const volumes::Shape & shape, volumes::Indices const & indices)
    : m_box_shape({}, {}, {}, {})
    , m_shape(shape)
    , m_indices(indices)
{
    //ToDo: Calculate extremal vertices from a convex hull
    calculateExtremalVetices(m_shape, indices, m_box);
    calculateMean(m_box);
    createBox(m_box);
}

geometry::Box aabb::AxisAlignedBoundingBox::getBox() const {
    return m_box_shape;
}

void aabb::AxisAlignedBoundingBox::calculateExtremalVetices(
        volumes::Shape const & shape, volumes::Indices const & indices, aabb::AxisAlignedBoundingBox::Box & box)
{
    using namespace std::placeholders;

    //ToDo: optimize it, currently O(n), should be possible to do O(log(n))
    std::vector<Eigen::Vector3f const *> valid_vertices;
    valid_vertices.reserve(indices.size() * 3);
    for(auto face_index : indices)
    {
        for (auto vertex_index : shape.indices[face_index])
        {
            valid_vertices.push_back(&shape.vertices[vertex_index]);
        }
    }

    static auto axisCompareVertices = [](
        uint32_t axis, Eigen::Vector3f const * a, Eigen::Vector3f const * b
    ) -> bool
    {
        return (*a)[axis] < (*b)[axis];
    };
    static auto xCompareVertices = std::bind(axisCompareVertices, 0, _1, _2);
    static auto yCompareVertices = std::bind(axisCompareVertices, 1, _1, _2);
    static auto zCompareVertices = std::bind(axisCompareVertices, 2, _1, _2);

    std::sort(valid_vertices.begin(), valid_vertices.end(), xCompareVertices);
    box.xMin = *valid_vertices.front();
    box.xMax = *valid_vertices.back();

    std::sort(valid_vertices.begin(), valid_vertices.end(), yCompareVertices);
    box.yMin = *valid_vertices.front();
    box.yMax = *valid_vertices.back();

    std::sort(valid_vertices.begin(), valid_vertices.end(), zCompareVertices);
    box.zMin = *valid_vertices.front();
    box.zMax = *valid_vertices.back();
}

void aabb::AxisAlignedBoundingBox::calculateMean(aabb::AxisAlignedBoundingBox::Box & box)
{
    box.extremalMean[0] = (box.xMax[0] + box.xMin[0]) / 2.0f;
    box.extremalMean[1] = (box.yMax[1] + box.yMin[1]) / 2.0f;
    box.extremalMean[2] = (box.zMax[2] + box.zMin[2]) / 2.0f;
    box.xAxis = box.xMax - box.extremalMean;
    box.yAxis = box.yMax - box.extremalMean;
    box.zAxis = box.zMax - box.extremalMean;
}

void aabb::AxisAlignedBoundingBox::createBox(aabb::AxisAlignedBoundingBox::Box & box)
{
    m_box_shape = geometry::Box(
            {box.extremalMean[0], box.extremalMean[1], box.extremalMean[2]},
            {box.xAxis[0], 0, 0},
            {0, box.yAxis[1], 0},
            {0, 0, box.zAxis[2]}
    );
}

sphere::BoundingSphere::BoundingSphere(volumes::Shape const & shape, volumes::Indices const & indices)
        : m_sphere_shape({}, 0)
        , m_shape(shape)
        , m_indices(indices)
{
    m_sphere.mean = volumes::calculateMeanVertex(shape, indices);
    m_sphere.covariance = volumes::calculateCovarianceMatrix(shape, indices, m_sphere.mean);
    m_sphere.eigen_values = volumes::calculateEigenValues(m_sphere.covariance);
    m_sphere.eigen_vectors = volumes::calculateEigenVectors(m_sphere.covariance);
    m_sphere.eigen_vectors_normalized = m_sphere.eigen_vectors.normalized();
    m_sphere_shape = calculateBoundingSphere(
            m_sphere.eigen_vectors_normalized, m_sphere.eigen_values, m_shape, m_indices);
    m_sphere_shape = refineSphere(m_sphere_shape, shape, indices);
}

geometry::Sphere sphere::BoundingSphere::getSphere() const
{
    return m_sphere_shape;
}

geometry::Sphere sphere::BoundingSphere::calculateBoundingSphere(
        Eigen::Matrix3f const & eigen_vectors, Eigen::Vector3f const & eigen_values,
        const volumes::Shape & shape, volumes::Indices const & indices
)
{
    //Find max dispersion axis
    uint32_t max_eigen_value_index = (eigen_values[0] < eigen_values[1] ? 1 : 0);
    if (eigen_values[max_eigen_value_index] < eigen_values[2]) {
        max_eigen_value_index = 2;
    }
    Eigen::Vector3f max_dispersion_axis = eigen_vectors.col(max_eigen_value_index).normalized();

    //Find extremal points on it
    size_t min_vertex_index = 0;
    size_t max_vertex_index = 0;
    for (auto face_index : indices) {
        for (auto vertex_index : shape.indices[face_index]) {
            auto const currentVertexProjection = shape.vertices[vertex_index].dot(max_dispersion_axis);
            if (currentVertexProjection > shape.vertices[max_vertex_index].dot(max_dispersion_axis))
            {
                max_vertex_index = vertex_index;
            }
            if (currentVertexProjection < shape.vertices[min_vertex_index].dot(max_dispersion_axis))
            {
                min_vertex_index = vertex_index;
            }
        }
    }

    //Calculate sphere
    auto const diameter = (shape.vertices[max_vertex_index] - shape.vertices[min_vertex_index]).norm();
    auto const radius = diameter / 2.0f;
    auto const center = (shape.vertices[max_vertex_index] - shape.vertices[min_vertex_index]) / 2.0f
                        + shape.vertices[min_vertex_index];

    return geometry::Sphere(
            {center[0], center[1], center[2]}, static_cast<double>(radius)
    );
}

geometry::Sphere sphere::BoundingSphere::refineSphere(
        geometry::Sphere const & sphere, volumes::Shape const & shape, volumes::Indices const & indices
)
{
    Vector3 const sphereMassCenter = sphere.getCenterOfMass();
    float sphereRadius = static_cast<float>(sphere.getRadius());
    Eigen::Vector3f sphereCenter(static_cast<float>(sphereMassCenter.x), 
                                 static_cast<float>(sphereMassCenter.y), 
                                 static_cast<float>(sphereMassCenter.z));

    //Find point outside of the sphere and resize sphere
    for (auto face_index : indices) {
        for (auto vertex_index : shape.indices[face_index]) {
            if ((shape.vertices[vertex_index] - sphereCenter).squaredNorm() > std::pow(sphereRadius, 2))
            {
                Eigen::Vector3f oppositeSphereVertex =
                        (shape.vertices[vertex_index] * -1.0f).normalized() * sphereRadius;
                sphereCenter =
                        (oppositeSphereVertex - shape.vertices[vertex_index]) / 2.0f
                        + shape.vertices[vertex_index];
                sphereRadius = (sphereCenter - oppositeSphereVertex).norm();
            }
        }
    }

    return geometry::Sphere({sphereCenter[0], sphereCenter[1], sphereCenter[2]}, sphereRadius);
}
