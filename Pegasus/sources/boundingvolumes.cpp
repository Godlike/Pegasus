/*
* Copyright (C) 2017 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#include "Pegasus/include/BoundingVolumes.hpp"
#include "Pegasus/include/Math.hpp"

#include <cmath>

using namespace pegasus;
using namespace geometry;
using namespace volumes;

volumes::Shape::Shape(volumes::Vertices const& vertices, volumes::Faces const& indices)
    : vertices(vertices)
    , indices(indices)
{
}

Eigen::Vector3f volumes::CalculateMeanVertex(volumes::Shape const& shape, volumes::Indices const& indices)
{
    Eigen::Vector3f sum{0, 0, 0};
    for (auto index : indices)
    {
        auto const& face = shape.indices[index];

        sum += shape.vertices[face[0]];
        sum += shape.vertices[face[1]];
        sum += shape.vertices[face[2]];
    }

    Eigen::Vector3f const mean = 1.0f / (3.0f * indices.size()) * sum;
    return mean;
}

Eigen::Matrix3f volumes::CalculateCovarianceMatrix(
    volumes::Shape const& shape, volumes::Indices const& indices, Eigen::Vector3f const& mean)
{
    Eigen::Matrix3f c = Eigen::Matrix3f::Zero();

    for (auto index : indices)
    {
        auto const& face = shape.indices[index];

        Eigen::Vector3f const p = shape.vertices[face[0]] - mean;
        Eigen::Vector3f const q = shape.vertices[face[1]] - mean;
        Eigen::Vector3f const r = shape.vertices[face[2]] - mean;

        for (uint8_t j = 0; j < 3; ++j)
        {
            for (uint8_t k = 0; k < 3; ++k)
            {
                float const v = p[j] * p[k] + q[j] * q[k] + r[j] * r[k];
                c(j, k) += v;
            }
        }
    }

    float const a = 1.0f / (3.0f * indices.size());
    for (uint8_t j = 0; j < 3; ++j)
    {
        for (uint8_t k = 0; k < 3; ++k)
        {
            c(j, k) *= a;
        }
    }

    return c;
}

Eigen::Vector3f volumes::CalculateEigenValues(Eigen::Matrix3f const& covariance)
{
    Eigen::EigenSolver<Eigen::Matrix3f> es(covariance);
    return es.eigenvalues().real();
}

Eigen::Matrix3f volumes::CalculateEigenVectors(Eigen::Matrix3f const& covariance)
{
    Eigen::EigenSolver<Eigen::Matrix3f> es(covariance);
    return es.eigenvectors().real();;
}

Eigen::Matrix3f volumes::CalculateExtremalVertices(
    Eigen::Matrix3f const& eigenVectors, volumes::Shape const& shape, volumes::Indices const& indices)
{
    std::vector<Eigen::Vector3f const *, Eigen::aligned_allocator<Eigen::Vector3f const*>> vertices;
    for (auto index : indices)
    {
        auto const& face = shape.indices[index];
        vertices.insert(vertices.end(), {&shape.vertices[face[0]], &shape.vertices[face[1]], &shape.vertices[face[2]]});
    }

    Eigen::Matrix3f extremal = Eigen::Matrix3f::Zero();
    for (uint8_t i = 0; i < 3; ++i)
    {
        Eigen::Vector3f const vec = eigenVectors.col(i);
        auto maxVertex = *std::max_element(vertices.begin(), vertices.end(),
                                           [&vec](auto a, auto b) { return a->dot(vec) < b->dot(vec); });
        extremal.col(i) = *maxVertex;
    }

    return extremal;
}

obb::OrientedBoundingBox::OrientedBoundingBox(volumes::Shape const& shape, volumes::Indices const& indices)
    : m_boxShape({}, {}, {}, {})
    , m_shape(shape)
    , m_indices(indices)
{
    //Todo: implement convex hull step
    m_box.mean = volumes::CalculateMeanVertex(m_shape, m_indices);
    m_box.covariance = volumes::CalculateCovarianceMatrix(m_shape, m_indices, m_box.mean);
    m_box.eigenVectors = volumes::CalculateEigenVectors(m_box.covariance);
    m_box.extremalVertices = volumes::CalculateExtremalVertices(m_box.covariance, m_shape, m_indices);
    m_box.eigenVectorsNormalized = m_box.eigenVectors.normalized();
    m_box.cubeVertices = CalculateBoxVertices(m_box.extremalVertices, m_box.eigenVectorsNormalized);

    m_boxShape = geometry::Box(
        {m_box.mean[0], m_box.mean[1], m_box.mean[2]},
        {m_box.eigenVectors.col(0)[0], m_box.eigenVectors.col(0)[1], m_box.eigenVectors.col(0)[2]},
        {m_box.eigenVectors.col(1)[0], m_box.eigenVectors.col(1)[1], m_box.eigenVectors.col(1)[2]},
        {m_box.eigenVectors.col(2)[0], m_box.eigenVectors.col(2)[1], m_box.eigenVectors.col(2)[2]}
    );
}

geometry::Box obb::OrientedBoundingBox::GetBox() const
{
    return m_boxShape;
}

volumes::Vertices
obb::OrientedBoundingBox::CalculateBoxVertices(
    Eigen::Matrix3f const& extremalPoints, Eigen::Matrix3f const& eigenVectors) const
{
    Vertices box(8, Eigen::Vector3f{0, 0, 0});

    std::array<Plane, 6> planes;
    for (uint8_t i = 0; i < 6; ++i)
    {
        uint8_t const index = (i < 3) ? i : i - 3;
        planes[i] = Plane(Eigen::Vector3f(eigenVectors.col(index)) * (i < 3 ? 1 : -1),
                          Eigen::Vector3f(extremalPoints.col(index)).dot(Eigen::Vector3f(eigenVectors.col(index))));
    }

    Faces faces = {{0, 1, 2}, {0, 1, 5}, {0, 2, 4}, {0, 4, 5},
        {2, 3, 4}, {1, 2, 3}, {1, 3, 5}, {3, 4, 5}};
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
        b << -v0[3] , -v1[3] , -v2[3];

        box[i] = A.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV).solve(b);
    }

    return box;
}

aabb::AxisAlignedBoundingBox::AxisAlignedBoundingBox(
    const volumes::Shape& shape, volumes::Indices const& indices)
    : m_boxShape({}, {}, {}, {})
    , m_shape(shape)
    , m_indices(indices)
{
    //ToDo: Calculate extremal vertices from a convex hull
    CalculateExtremalVetices(m_shape, indices, m_box);
    CalculateMean(m_box);
    CreateBox(m_box);
}

geometry::Box aabb::AxisAlignedBoundingBox::GetBox() const
{
    return m_boxShape;
}

void aabb::AxisAlignedBoundingBox::CalculateExtremalVetices(
    volumes::Shape const& shape, volumes::Indices const& indices, aabb::AxisAlignedBoundingBox::Box& box)
{
    using namespace std::placeholders;

    //ToDo: optimize it, currently O(n), should be possible to do O(log(n))
    std::vector<Eigen::Vector3f const *> validVertices;
    validVertices.reserve(indices.size() * 3);
    for (auto faceIndex : indices)
    {
        for (auto vertexIndex : shape.indices[faceIndex])
        {
            validVertices.push_back(&shape.vertices[vertexIndex]);
        }
    }

    static auto axisCompareVertices = []
            (uint32_t axis, Eigen::Vector3f const* a, Eigen::Vector3f const* b)
            {
                return (*a)[axis] < (*b)[axis];
            };
    static auto xCompareVertices = std::bind(axisCompareVertices, 0, _1, _2);
    static auto yCompareVertices = std::bind(axisCompareVertices, 1, _1, _2);
    static auto zCompareVertices = std::bind(axisCompareVertices, 2, _1, _2);

    std::sort(validVertices.begin(), validVertices.end(), xCompareVertices);
    box.xMin = *validVertices.front();
    box.xMax = *validVertices.back();

    std::sort(validVertices.begin(), validVertices.end(), yCompareVertices);
    box.yMin = *validVertices.front();
    box.yMax = *validVertices.back();

    std::sort(validVertices.begin(), validVertices.end(), zCompareVertices);
    box.zMin = *validVertices.front();
    box.zMax = *validVertices.back();
}

void aabb::AxisAlignedBoundingBox::CalculateMean(aabb::AxisAlignedBoundingBox::Box& box)
{
    box.extremalMean[0] = (box.xMax[0] + box.xMin[0]) / 2.0f;
    box.extremalMean[1] = (box.yMax[1] + box.yMin[1]) / 2.0f;
    box.extremalMean[2] = (box.zMax[2] + box.zMin[2]) / 2.0f;
    box.xAxis = box.xMax - box.extremalMean;
    box.yAxis = box.yMax - box.extremalMean;
    box.zAxis = box.zMax - box.extremalMean;
}

void aabb::AxisAlignedBoundingBox::CreateBox(aabb::AxisAlignedBoundingBox::Box& box)
{
    m_boxShape = geometry::Box(
        {box.extremalMean[0], box.extremalMean[1], box.extremalMean[2]},
        {box.xAxis[0], 0, 0},
        {0, box.yAxis[1], 0},
        {0, 0, box.zAxis[2]}
    );
}

sphere::BoundingSphere::BoundingSphere(volumes::Shape const& shape, volumes::Indices const& indices)
    : m_sphereShape({}, 0)
    , m_shape(shape)
    , m_indices(indices)
{
    m_sphere.mean = volumes::CalculateMeanVertex(shape, indices);
    m_sphere.covariance = volumes::CalculateCovarianceMatrix(shape, indices, m_sphere.mean);
    m_sphere.eigenValues = volumes::CalculateEigenValues(m_sphere.covariance);
    m_sphere.eigenVectors = volumes::CalculateEigenVectors(m_sphere.covariance);
    m_sphere.eigenVectorsNormalized = m_sphere.eigenVectors.normalized();
    m_sphereShape = CalculateBoundingSphere(
        m_sphere.eigenVectorsNormalized, m_sphere.eigenValues, m_shape, m_indices);
    m_sphereShape = RefineSphere(m_sphereShape, shape, indices);
}

geometry::Sphere sphere::BoundingSphere::GetSphere() const
{
    return m_sphereShape;
}

geometry::Sphere sphere::BoundingSphere::CalculateBoundingSphere(
    Eigen::Matrix3f const& eigenVectors, Eigen::Vector3f const& eigenValues,
    volumes::Shape const& shape, volumes::Indices const& indices
)
{
    //Find max dispersion axis
    uint32_t maxEigenValueIndex = (eigenValues[0] < eigenValues[1] ? 1 : 0);
    if (eigenValues[maxEigenValueIndex] < eigenValues[2])
    {
        maxEigenValueIndex = 2;
    }
    Eigen::Vector3f maxDispersionAxis = eigenVectors.col(maxEigenValueIndex).normalized();

    //Find extremal points on it
    size_t minVertexIndex = 0;
    size_t maxVertexIndex = 0;
    for (auto face_index : indices)
    {
        for (auto vertex_index : shape.indices[face_index])
        {
            auto const currentVertexProjection = shape.vertices[vertex_index].dot(maxDispersionAxis);
            if (currentVertexProjection > shape.vertices[maxVertexIndex].dot(maxDispersionAxis))
            {
                maxVertexIndex = vertex_index;
            }
            if (currentVertexProjection < shape.vertices[minVertexIndex].dot(maxDispersionAxis))
            {
                minVertexIndex = vertex_index;
            }
        }
    }

    //Calculate sphere
    auto const diameter = (shape.vertices[maxVertexIndex] - shape.vertices[minVertexIndex]).norm();
    auto const radius = diameter / 2.0f;
    auto const center = (shape.vertices[maxVertexIndex] - shape.vertices[minVertexIndex]) / 2.0f
            + shape.vertices[minVertexIndex];

    return geometry::Sphere(
        {center[0], center[1], center[2]}, static_cast<double>(radius)
    );
}

geometry::Sphere sphere::BoundingSphere::RefineSphere(
    geometry::Sphere const& sphere, volumes::Shape const& shape, volumes::Indices const& indices
)
{
    auto tempCentroid = sphere.getCenterOfMass();
    Vector3 const sphereMassCenter(tempCentroid.x, tempCentroid.y, tempCentroid.z);
    float sphereRadius = static_cast<float>(sphere.GetRadius());
    Eigen::Vector3f sphereCenter(static_cast<float>(sphereMassCenter.x),
                                 static_cast<float>(sphereMassCenter.y),
                                 static_cast<float>(sphereMassCenter.z));

    //Find point outside of the sphere and resize sphere
    for (auto faceIndex : indices)
    {
        for (auto vertexIndex : shape.indices[faceIndex])
        {
            if ((shape.vertices[vertexIndex] - sphereCenter).squaredNorm() > std::pow(sphereRadius, 2))
            {
                Eigen::Vector3f oppositeSphereVertex =
                        (shape.vertices[vertexIndex] * -1.0f).normalized() * sphereRadius;
                sphereCenter =
                        (oppositeSphereVertex - shape.vertices[vertexIndex]) / 2.0f
                        + shape.vertices[vertexIndex];
                sphereRadius = (sphereCenter - oppositeSphereVertex).norm();
            }
        }
    }

    return geometry::Sphere({sphereCenter[0], sphereCenter[1], sphereCenter[2]}, sphereRadius);
}
