/*
* Copyright (C) 2017 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#ifndef PEGASUS_BOUNDING_VOLUMES_HPP
#define PEGASUS_BOUNDING_VOLUMES_HPP

#include "Pegasus/include/Geometry.hpp"
#include "Pegasus/include/Math.hpp"

#include <vector>
#include <set>
#include <array>
#include <algorithm>
#include <random>
#include <unordered_map>
#include <iostream>
#include <Pegasus/third_party/glm/glm/ext.hpp>

namespace pegasus
{
namespace geometry
{
namespace volumes
{
using Vertices = std::vector<glm::dvec3>;
using Indices  = std::set<std::size_t>;
using Face     = std::array<std::size_t, 3>;
using Faces    = std::vector<Face>;

struct Shape
{
    Shape(volumes::Vertices const& vertices, volumes::Faces const& faces);

    Vertices const& vertices;
    Faces const& faces;
};

glm::dvec3 CalculateMeanVertex(
    volumes::Shape const& shape, volumes::Indices const& indices
);
glm::dmat3 CalculateCovarianceMatrix(
    volumes::Shape const& shape, volumes::Indices const& indices, glm::dvec3 const& mean
);
glm::dmat3 CalculateExtremalVertices(
    glm::dmat3 const& eigenVectors, volumes::Shape const& shape, volumes::Indices const& indices
);

namespace obb
{
class OrientedBoundingBox
{
public:
    struct Box
    {
        glm::dvec3 mean;
        glm::dmat3 covariance;
        glm::dmat3 eigenVectors;
        glm::dmat3 eigenVectorsNormalized;
        glm::dmat3 extremalVertices;
        glm::dmat3 boxAxes;
    };

    OrientedBoundingBox(Shape const& shape, Indices const& indices);
    geometry::Box GetBox() const;

private:
    geometry::Box m_boxShape;
    Box m_box;
    Shape const& m_shape;
    Indices const& m_indices;
};
} // namespace obb

namespace aabb
{
class AxisAlignedBoundingBox
{
public:
    struct Box
    {
        glm::dvec3 xMin;
        glm::dvec3 xMax;
        glm::dvec3 yMin;
        glm::dvec3 yMax;
        glm::dvec3 zMin;
        glm::dvec3 zMax;
        glm::dvec3 extremalMean;
        glm::dvec3 xAxis;
        glm::dvec3 yAxis;
        glm::dvec3 zAxis;
    };

    AxisAlignedBoundingBox(Shape const& shape, Indices const& indices);
    geometry::Box GetBox() const;

private:
    geometry::Box m_boxShape;
    Box m_box;
    Shape const& m_shape;
    Indices const& m_indices;

    static void CalculateExtremalVetices(Shape const& shape, Indices const& indices, Box& box);
    static void CalculateMean(Box& box);
    void CreateBox(Box& box);
};
} // namespace aabb

namespace sphere
{
class BoundingSphere
{
public:
    struct Sphere
    {
        glm::dvec3 mean;
        glm::dmat3 covariance;
        glm::dvec3 eigenValues;
        glm::dmat3 eigenVectors;
        glm::dmat3 eigenVectorsNormalized;
    };

    BoundingSphere(Shape const& shape, Indices const& indices);
    geometry::Sphere GetSphere() const;

private:
    geometry::Sphere m_sphereShape;
    Sphere m_sphere;
    Shape const& m_shape;
    Indices const& m_indices;

    static geometry::Sphere CalculateBoundingSphere(
        glm::dmat3 const& eigenVectors, glm::dvec3 const& eigenValues,
        Shape const& shape, Indices const& indices
    );
    static geometry::Sphere RefineSphere(
        geometry::Sphere const& sphere, Shape const& shape, Indices const& indices
    );
};
} // namespace sphere

namespace hierarchy
{
template <typename BoundingVolume>
class BoundingVolumeHierarchy
{
public:
    BoundingVolumeHierarchy(Shape const& shape, Indices const& indices)
        : m_shape(shape)
        , m_indices(indices)
    {
    }

public:
    Shape const& m_shape;
    Indices const& m_indices;

    void MaximumDistanceVectorApprox(glm::dvec3& min, glm::dvec3& max)
    {
        std::size_t minx = 0, maxx = 0, miny = 0, maxy = 0, minz = 0, maxz = 0;
        for (std::size_t faceIndex : m_indices)
        {
            for (std::size_t vertIndex : m_shape.faces[faceIndex])
            {
                if (m_shape.vertices[vertIndex][0] < m_shape.vertices[minx][0]) minx = vertIndex;
                if (m_shape.vertices[vertIndex][0] > m_shape.vertices[maxx][0]) maxx = vertIndex;
                if (m_shape.vertices[vertIndex][1] < m_shape.vertices[miny][1]) miny = vertIndex;
                if (m_shape.vertices[vertIndex][1] > m_shape.vertices[maxy][1]) maxy = vertIndex;
                if (m_shape.vertices[vertIndex][2] < m_shape.vertices[minz][2]) minz = vertIndex;
                if (m_shape.vertices[vertIndex][2] > m_shape.vertices[maxz][2]) maxz = vertIndex;
            }
        }

        min = m_shape.vertices[minz];
        max = m_shape.vertices[maxz];

        double dist2x = glm::distance2(m_shape.vertices[minx], m_shape.vertices[maxx]);
        double dist2y = glm::distance2(m_shape.vertices[miny], m_shape.vertices[maxy]);
        double dist2z = glm::distance2(m_shape.vertices[minz], m_shape.vertices[maxz]);

        if (dist2y > dist2x && dist2y > dist2z)
        {
            min = m_shape.vertices[miny];
            max = m_shape.vertices[maxy];
        }
        if (dist2z > dist2y && dist2z > dist2x)
        {
            min = m_shape.vertices[minz];
            max = m_shape.vertices[maxz];
        }
    }

    glm::dvec3 getMedianCut(pegasus::math::Plane const& minPlane)
    {
        std::vector<std::size_t> indicesArr(m_indices.begin(), m_indices.end());
        std::unordered_map<std::size_t, double> distances;

        for (std::size_t index : indicesArr)
        {
            Face const& currFace = m_shape.faces[index];
            glm::dvec3 centroid = pegasus::math::calculateCentroid(
                { m_shape.vertices[currFace[0]],
                  m_shape.vertices[currFace[1]],
                  m_shape.vertices[currFace[2]] });
            distances[index] = minPlane.distanceToPoint(centroid);
        }

        std::size_t middleIndex = indicesArr.size() / 2;
        std::size_t left = 0, right = indicesArr.size();
        std::random_device(rd);
        std::mt19937 rng(rd());
        while (true)
        {
            if (right - left <= 1)
            {
                break;
            }

            std::uniform_int_distribution<std::size_t> distr(left, right);
            std::size_t pivot = distr(rng);
            std::size_t pivotValue = indicesArr[pivot];

            // partition the vector
            std::iter_swap(indicesArr.begin() + pivot, indicesArr.begin() + right - 1);
            std::size_t pivotIndex = left;
            for (std::size_t i = left + 1; i < right - 1; ++i)
            {
                if (distances[indicesArr[i]] < distances[pivotValue])
                {
                    std::iter_swap(indicesArr.begin() + i, indicesArr.begin() + pivotIndex);
                    ++pivotIndex;
                }
            }
            std::iter_swap(indicesArr.begin() + right - 1, indicesArr.begin() + pivotIndex);

            if (middleIndex == pivotIndex)
            {
                break;
            }
            if (middleIndex < pivotIndex)
            {
                right = pivotIndex;
            }
            else
            {
                left = pivotIndex + 1;
            }
        }

        Face const& midFace = m_shape.faces[indicesArr[middleIndex]];
        return pegasus::math::calculateCentroid(
            { m_shape.vertices[midFace[0]],
              m_shape.vertices[midFace[1]],
              m_shape.vertices[midFace[2]] });
    }
};
} // namespace hierarchy
} // namespace volumes
} // namespace geometry
} // namespace pegasus
#endif // PEGASUS_BOUNDING_VOLUMES_HPP
