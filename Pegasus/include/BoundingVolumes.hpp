/*
* Copyright (C) 2017 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#ifndef PEGASUS_BOUNDING_VOLUMES_HPP
#define PEGASUS_BOUNDING_VOLUMES_HPP

#include "Pegasus/include/Geometry.hpp"

#include <vector>
#include <set>
#include <array>
#include <algorithm>
#include <random>
#include <unordered_map>
#include <iostream>
#include <glm/ext.hpp>

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
        , m_volume(BoundingVolume(shape, indices))
    {
        if (indices.size() <= MAX_NODE_SIZE)
        {
            return;
        }

        Vertices shapeVertices = GetShapeVertices();
        glm::dvec3 maxDistanceVector = GetMaximumDistanceDirectionApprox(shapeVertices);
        glm::dvec3 min = GetMinVectorAlongDirection(maxDistanceVector, shapeVertices);
        math::Plane minPlane(min, maxDistanceVector);

        CentroidsMap centroids = GetFacesCentroids();
        Indices lowerIndices, upperIndices;
        std::tie(lowerIndices, upperIndices) = GetPartitionIndices(minPlane, centroids);

        std::cout << lowerIndices.size() << " " << upperIndices.size() << "\n";

        lowerChild = std::make_unique<BoundingVolumeHierarchy<BoundingVolume>>(m_shape, lowerIndices);
        upperChild = std::make_unique<BoundingVolumeHierarchy<BoundingVolume>>(m_shape, upperIndices);
    }

    std::unique_ptr<BoundingVolumeHierarchy> const& getLowerChild() const
    {
        return lowerChild;
    }

    std::unique_ptr<BoundingVolumeHierarchy> const& getUpperChild() const
    {
        return upperChild;
    }

    BoundingVolume const getVolume() const
    {
        return m_volume;
    }

private:
    using CentroidsMap = std::unordered_map<std::size_t, glm::dvec3>;

    std::size_t static const MAX_NODE_SIZE = 5;

    Shape const& m_shape;
    Indices const& m_indices;
    BoundingVolume const m_volume;

    std::unique_ptr<BoundingVolumeHierarchy<BoundingVolume>> lowerChild;
    std::unique_ptr<BoundingVolumeHierarchy<BoundingVolume>> upperChild;

    Vertices GetShapeVertices()
    {
        Vertices result;
        result.reserve(3 * m_indices.size());
        for (std::size_t i : m_indices)
        {
            result.insert(result.end(), {
                m_shape.vertices[m_shape.faces[i][0]],
                m_shape.vertices[m_shape.faces[i][1]],
                m_shape.vertices[m_shape.faces[i][2]]
            });
        }
        return result;
    }

    glm::dvec3 GetMaximumDistanceDirectionApprox(Vertices const& vertices)
    {
        glm::dvec3
            xMin(vertices[0]), yMin(vertices[0]), zMin(vertices[0]),
            xMax(vertices[0]), yMax(vertices[0]), zMax(vertices[0]);

        for (glm::dvec3 const& vertice : vertices)
        {
            if (vertice.x < xMin.x) xMin = vertice;
            if (vertice.y < yMin.y) yMin = vertice;
            if (vertice.z < zMin.z) zMin = vertice;
            if (vertice.x > xMax.x) xMax = vertice;
            if (vertice.y > yMax.y) yMax = vertice;
            if (vertice.z > zMax.z) zMax = vertice;
        }

        glm::dvec3 min = xMin, max = xMax;

        double dist2x = glm::distance2(xMin, xMax);
        double dist2y = glm::distance2(yMin, yMax);
        double dist2z = glm::distance2(zMin, zMax);

        if (dist2y > dist2x && dist2y > dist2z)
        {
            min = yMin;
            max = yMax;
        }
        if (dist2z > dist2y && dist2z > dist2x)
        {
            min = zMin;
            max = zMax;
        };

        return glm::normalize(max - min);
    }

    glm::dvec3 GetMinVectorAlongDirection(glm::dvec3 maxDistanceVector, Vertices vertices)
    {
        return *std::min_element(
            vertices.begin(),
            vertices.end(),
            [&maxDistanceVector](glm::dvec3 const& a, glm::dvec3 const& b) -> bool
            {
                return glm::dot(a, maxDistanceVector) < glm::dot(b, maxDistanceVector);
            });
    }

    CentroidsMap GetFacesCentroids()
    {
        CentroidsMap centroids;
        for (std::size_t index : m_indices)
        {
            Face const& currFace = m_shape.faces[index];
            centroids[index] = pegasus::math::CalculateCentroid(
                {
                    m_shape.vertices[currFace[0]],
                    m_shape.vertices[currFace[1]],
                    m_shape.vertices[currFace[2]]
                }
            );
        }
        return centroids;
    }

    std::pair<Indices, Indices> GetPartitionIndices(math::Plane const& minPlane, CentroidsMap const& centroids)
    {
        std::vector<std::size_t> indicesArr;
        std::unordered_map<std::size_t, double> distances;
        indicesArr.reserve(centroids.size());
        for (auto const& keyValuePair : centroids)
        {
            std::size_t index;
            glm::dvec3 centroid;
            std::tie(index, centroid) = keyValuePair;

            indicesArr.push_back(index);
            distances[index] = minPlane.DistanceToPoint(centroid);
        }

        std::size_t middleIndex = indicesArr.size() / 2;
        std::size_t left = 0, right = indicesArr.size();
        std::random_device(rd);
        std::mt19937 rng(rd());
        while (true)
        {
            std::uniform_int_distribution<std::size_t> distr(left, right - 1);
            std::size_t pivot = distr(rng);
            std::size_t pivotValue = indicesArr[pivot];

            std::iter_swap(indicesArr.begin() + pivot, indicesArr.begin() + right - 1);
            std::size_t pivotIndex = left;
            for (std::size_t i = left; i < right - 1; ++i)
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

        Indices lowerIndices(indicesArr.begin(), indicesArr.begin() + middleIndex);
        Indices upperIndices(indicesArr.begin() + middleIndex, indicesArr.end());
        return std::make_pair(std::move(lowerIndices), std::move(upperIndices));
    }
};
} // namespace hierarchy
} // namespace volumes
} // namespace geometry
} // namespace pegasus
#endif // PEGASUS_BOUNDING_VOLUMES_HPP
