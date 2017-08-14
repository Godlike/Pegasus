/*
* Copyright (C) 2017 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#include "Pegasus/include/BoundingVolumes.hpp"
#include "Pegasus/include/Math.hpp"

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/norm.hpp>
#include <cmath>

using namespace pegasus;
using namespace geometry;
using namespace volumes;

volumes::Shape::Shape(volumes::Vertices const& vertices, volumes::Faces const& faces)
    : vertices(vertices)
    , faces(faces)
{
}

glm::dvec3 volumes::CalculateMeanVertex(volumes::Shape const& shape, volumes::Indices const& indices)
{
    glm::dvec3 sum{0, 0, 0};
    for (auto index : indices)
    {
        auto const& face = shape.faces[index];

        sum += shape.vertices[face[0]];
        sum += shape.vertices[face[1]];
        sum += shape.vertices[face[2]];
    }

    glm::dvec3 const mean = 1.0 / (3.0 * indices.size()) * sum;
    return mean;
}

glm::dmat3 volumes::CalculateCovarianceMatrix(
    volumes::Shape const& shape, volumes::Indices const& indices, glm::dvec3 const& mean)
{
    glm::dmat3 c(0.0);

    for (auto index : indices)
    {
        auto const& face = shape.faces[index];

        glm::dvec3 const p = shape.vertices[face[0]] - mean;
        glm::dvec3 const q = shape.vertices[face[1]] - mean;
        glm::dvec3 const r = shape.vertices[face[2]] - mean;

        for (uint8_t j = 0; j < 3; ++j)
        {
            for (uint8_t k = 0; k < 3; ++k)
            {
                double const v = p[j] * p[k] + q[j] * q[k] + r[j] * r[k];
                c[j][k] += v;
            }
        }
    }

    float const a = 1.0f / (3.0f * indices.size());
    for (uint8_t j = 0; j < 3; ++j)
    {
        for (uint8_t k = 0; k < 3; ++k)
        {
            c[j][k] *= a;
        }
    }

    return c;
}

glm::dmat3 volumes::CalculateExtremalVertices(
    glm::dmat3 const& eigenVectors, volumes::Shape const& shape, volumes::Indices const& indices)
{
    std::vector<glm::dvec3 const *> vertices;
    for (auto index : indices)
    {
        auto const& face = shape.faces[index];
        vertices.insert(vertices.end(), {&shape.vertices[face[0]], &shape.vertices[face[1]], &shape.vertices[face[2]]});
    }

    glm::dmat3 extremal(0);
    for (uint8_t i = 0; i < 3; ++i)
    {
        auto maxVertex = *std::max_element(vertices.begin(), vertices.end(), [&eigenVectors, i](auto a, auto b) -> bool {
           return glm::abs(glm::dot(*a, eigenVectors[i])) < glm::abs(glm::dot(*b, eigenVectors[i]));
        });
        extremal[i] = *maxVertex;
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

    pegasus::math::JacobiEigenvalue jacobiEigen(m_box.covariance);
    m_box.eigenVectors = jacobiEigen.GetEigenvectors();
    m_box.extremalVertices = volumes::CalculateExtremalVertices(m_box.eigenVectors, m_shape, m_indices);
    
    for (uint8_t i = 0; i < 3; ++i) {
        m_box.eigenVectorsNormalized[i] = glm::normalize(m_box.eigenVectors[i]);
    }

    m_box.boxAxes[0] = m_box.eigenVectorsNormalized[0] * glm::dot(m_box.extremalVertices[0] - m_box.mean, m_box.eigenVectorsNormalized[0]);
    m_box.boxAxes[1] = m_box.eigenVectorsNormalized[1] * glm::dot(m_box.extremalVertices[1] - m_box.mean, m_box.eigenVectorsNormalized[1]);
    m_box.boxAxes[2] = m_box.eigenVectorsNormalized[2] * glm::dot(m_box.extremalVertices[2] - m_box.mean, m_box.eigenVectorsNormalized[2]);
    
    m_boxShape = geometry::Box(m_box.mean, m_box.boxAxes[0], m_box.boxAxes[1], m_box.boxAxes[2]);
}

geometry::Box obb::OrientedBoundingBox::GetBox() const
{
    return m_boxShape;
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
    std::vector<glm::dvec3 const *> validVertices;
    validVertices.reserve(indices.size() * 3);
    for (auto faceIndex : indices)
    {
        for (auto vertexIndex : shape.faces[faceIndex])
        {
            validVertices.push_back(&shape.vertices[vertexIndex]);
        }
    }

    static auto axisCompareVertices = [] (uint32_t axis, glm::dvec3 const* a, glm::dvec3 const* b) -> bool {
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
        box.extremalMean,
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

    pegasus::math::JacobiEigenvalue jacobiEigen(m_sphere.covariance);
    m_sphere.eigenValues = jacobiEigen.GetEigenvalues();
    m_sphere.eigenVectors = jacobiEigen.GetEigenvectors();

    for (uint8_t i = 0; i < 3; ++i) {
        m_sphere.eigenVectorsNormalized[i] = glm::normalize(m_sphere.eigenVectors[i]);
    }   
    m_sphereShape = CalculateBoundingSphere(
        m_sphere.eigenVectorsNormalized, m_sphere.eigenValues, m_shape, m_indices);
    m_sphereShape = RefineSphere(m_sphereShape, shape, indices);
}

geometry::Sphere sphere::BoundingSphere::GetSphere() const
{
    return m_sphereShape;
}

geometry::Sphere sphere::BoundingSphere::CalculateBoundingSphere(
    glm::dmat3 const& eigenVectors, glm::dvec3 const& eigenValues,
    volumes::Shape const& shape, volumes::Indices const& indices
)
{
    //Find max dispersion axis
    uint32_t maxEigenValueIndex = (eigenValues[0] < eigenValues[1] ? 1 : 0);
    if (eigenValues[maxEigenValueIndex] < eigenValues[2])
    {
        maxEigenValueIndex = 2;
    }
    glm::dvec3 maxDispersionAxis = glm::normalize(eigenVectors[maxEigenValueIndex]);

    //Find extremal points on it
    size_t minVertexIndex = 0;
    size_t maxVertexIndex = 0;
    for (auto face_index : indices)
    {
        for (auto vertex_index : shape.faces[face_index])
        {
            auto const currentVertexProjection = glm::dot(shape.vertices[vertex_index], maxDispersionAxis);
            if (currentVertexProjection > glm::dot(shape.vertices[maxVertexIndex], maxDispersionAxis))
            {
                maxVertexIndex = vertex_index;
            }
            if (currentVertexProjection < glm::dot(shape.vertices[minVertexIndex], maxDispersionAxis))
            {
                minVertexIndex = vertex_index;
            }
        }
    }

    //Calculate sphere
    auto const diameter = glm::length(shape.vertices[maxVertexIndex] - shape.vertices[minVertexIndex]);
    auto const radius = diameter / 2.0;
    auto const center = (shape.vertices[maxVertexIndex] - shape.vertices[minVertexIndex]) / 2.0
            + shape.vertices[minVertexIndex];

    return geometry::Sphere(center, radius);
}

geometry::Sphere sphere::BoundingSphere::RefineSphere(
    geometry::Sphere const& sphere, volumes::Shape const& shape, volumes::Indices const& indices
)
{
    double sphereRadius = sphere.GetRadius();
    glm::dvec3 sphereCenter = sphere.GetCenterOfMass();

    //Find point outside of the sphere and resize sphere
    for (auto faceIndex : indices)
    {
        for (auto vertexIndex : shape.faces[faceIndex])
        {
            glm::dvec3 const & vertex = shape.vertices[vertexIndex];
            glm::dvec3 const sphereVertexVec = vertex - sphereCenter;
            double const sphereVertexNorm = glm::length(sphereVertexVec);

            if (sphereVertexNorm > sphereRadius)
            {
                glm::dvec3 oppositeSphereVertex = glm::normalize(sphereVertexVec) * -1.0 * sphereRadius + sphereCenter;
                sphereCenter = (oppositeSphereVertex - vertex) / 2.0 + vertex;
                sphereRadius = glm::length(sphereCenter - oppositeSphereVertex);
            }
        }
    }

    return geometry::Sphere(sphereCenter, sphereRadius);
}
