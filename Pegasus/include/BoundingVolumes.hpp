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
#include <glm/ext.hpp>
#include <queue>
#include <stack>

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

struct SplitIndices
{
    Indices lowerIndices;
    Indices upperIndices;
};

template <typename BoundingVolume>
struct SimpleShapeExtractor
{
};

template <>
struct SimpleShapeExtractor<aabb::AxisAlignedBoundingBox>
{
    Box GetSimpleShape(aabb::AxisAlignedBoundingBox const& aabb)
    {
        return aabb.GetBox();
    }
};

template <>
struct SimpleShapeExtractor<obb::OrientedBoundingBox>
{
    Box GetSimpleShape(obb::OrientedBoundingBox const& obb)
    {
        return obb.GetBox();
    }
};

template <>
struct SimpleShapeExtractor<sphere::BoundingSphere>
{
    Sphere GetSimpleShape(sphere::BoundingSphere const& sphere)
    {
        return sphere.GetSphere();
    }
};

template <typename BoundingVolume>
class BoundingVolumeHierarchy
{
public:
    class Node
    {
    public:
        BoundingVolume const volume;

        Node(Node const&) = delete;
        Node(Node &&) = delete;
        Node & operator=(Node const&) = delete;
        Node & operator=(Node &&) = delete;

        std::unique_ptr<Node> const& GetLowerChild() const
        {
            return lowerChild;
        }

        std::unique_ptr<Node> const& GetUpperChild() const
        {
            return upperChild;
        }

        bool IsLeaf() const
        {
            return isLeaf;
        }

        ~Node() = default;

    private:
        mutable std::unique_ptr<Node> lowerChild;
        mutable std::unique_ptr<Node> upperChild;
        bool isLeaf;

        explicit Node(BoundingVolume && volume)
            : volume(volume)
            , lowerChild(nullptr)
            , upperChild(nullptr)
            , isLeaf(true)
        {}

        friend class BoundingVolumeHierarchy;
    };

private:
    using CentroidsMap = std::unordered_map<std::size_t, glm::dvec3>;
    using NodePtr = std::unique_ptr<Node>;

    std::size_t static const MAX_NODE_SIZE = 5;

    Shape const& shape;
    NodePtr root;
    mutable SimpleShapeIntersectionDetector detector;
    mutable SimpleShapeExtractor<BoundingVolume> extractor;

    Vertices GetShapeVertices(Indices const& indices) const
    {
        Vertices result;
        result.reserve(3 * indices.size());
        for (std::size_t i : indices)
        {
            result.insert(result.end(), {
                shape.vertices[shape.faces[i][0]],
                shape.vertices[shape.faces[i][1]],
                shape.vertices[shape.faces[i][2]]
            });
        }
        return result;
    }

    glm::dvec3 GetMaximumDistanceDirectionApprox(Vertices const& vertices) const
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

    glm::dvec3 GetMinVectorAlongDirection(glm::dvec3 maxDistanceVector, Vertices const& vertices) const
    {
        return *std::min_element(
            vertices.begin(),
            vertices.end(),
            [&maxDistanceVector](glm::dvec3 const& a, glm::dvec3 const& b) -> bool
            {
                return glm::dot(a, maxDistanceVector) < glm::dot(b, maxDistanceVector);
            });
    }

    CentroidsMap GetFacesCentroids(Indices const& indices) const
    {
        CentroidsMap centroids;
        for (std::size_t index : indices)
        {
            Face const& currFace = shape.faces[index];
            centroids[index] = pegasus::math::CalculateCentroid(
                {
                    shape.vertices[currFace[0]],
                    shape.vertices[currFace[1]],
                    shape.vertices[currFace[2]]
                }
            );
        }
        return centroids;
    }

    SplitIndices GetPartitionIndices(
        math::HyperPlane const& minPlane,
        CentroidsMap const& centroids) const
    {
        std::vector<std::size_t> indicesArr;
        std::unordered_map<std::size_t, double> distances;
        indicesArr.reserve(centroids.size());
        for (auto const& keyValuePair : centroids)
        {
            std::size_t index = keyValuePair.first;
            glm::dvec3 const& centroid = keyValuePair.second;

            indicesArr.push_back(index);
            distances[index] = minPlane.Distance(centroid);
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

        return {
            Indices(indicesArr.begin(), indicesArr.begin() + middleIndex),
            Indices(indicesArr.begin() + middleIndex, indicesArr.end())
        };
    }

    bool CollideNode(Node* node, SimpleShape const* shape) const
    {
        auto nodeShape = extractor.GetSimpleShape(node->volume);
        return detector.CalculateIntersection(&nodeShape, shape);
    }

    bool GenericCollide(SimpleShape const* shape) const
    {
        std::stack<Node*> nodeStack;
        nodeStack.push(root.get());

        while (!nodeStack.empty())
        {
            Node* currNode = nodeStack.top();
            nodeStack.pop();
            if (!CollideNode(currNode, shape))
            {
                continue;
            }
            if (currNode->isLeaf)
            {
                return true;
            }
            nodeStack.push(currNode->lowerChild.get());
            nodeStack.push(currNode->upperChild.get());
        }
        return false;
    }

public:

    BoundingVolumeHierarchy(Shape const& shape, Indices const& indices)
        : shape(shape)
        , root(new Node(BoundingVolume(shape, indices)))
    {
        std::stack<Node*> nodeStack;
        std::stack<Indices> indicesStack;
        nodeStack.push(root.get());
        indicesStack.push(indices);

        while (!nodeStack.empty())
        {
            Node* currNode = nodeStack.top();
            Indices currIndices = std::move(indicesStack.top());
            nodeStack.pop();
            indicesStack.pop();

            if (currIndices.size() < MAX_NODE_SIZE)
            {
                currNode->lowerChild.reset();
                currNode->upperChild.reset();
                continue;
            }

            currNode->isLeaf = false;

            Vertices currVertices = GetShapeVertices(currIndices);
            glm::dvec3 maxDistanceVector = GetMaximumDistanceDirectionApprox(currVertices);
            glm::dvec3 min = GetMinVectorAlongDirection(maxDistanceVector, currVertices);
            math::HyperPlane minPlane(maxDistanceVector, min);

            CentroidsMap centroids = GetFacesCentroids(currIndices);
            SplitIndices splitIndices = GetPartitionIndices(minPlane, centroids);

            BoundingVolume lowerVolume(shape, splitIndices.lowerIndices);
            BoundingVolume upperVolume(shape, splitIndices.upperIndices);

            currNode->lowerChild = NodePtr(new Node(std::move(lowerVolume)));
            currNode->upperChild = NodePtr(new Node(std::move(upperVolume)));

            nodeStack.push(currNode->lowerChild.get());
            indicesStack.push(std::move(splitIndices.lowerIndices));
            nodeStack.push(currNode->upperChild.get());
            indicesStack.push(std::move(splitIndices.upperIndices));
        }
    }

    ~BoundingVolumeHierarchy()
    {
        std::stack<NodePtr> nodeStack;
        nodeStack.push(NodePtr(root.release()));

        while (!nodeStack.empty())
        {
            NodePtr & currNode = nodeStack.top();
            if (currNode->lowerChild.get() != nullptr)
            {
                nodeStack.push(NodePtr(currNode->lowerChild.release()));
                continue;
            }
            if (currNode->upperChild.get() != nullptr)
            {
                nodeStack.push(NodePtr(currNode->upperChild.release()));
                continue;
            }
            nodeStack.pop();
        }
    }

    NodePtr const& GetRoot() const
    {
        return root;
    }

    bool Collide(Plane const* plane) const
    {
        return GenericCollide(plane);
    }

    bool Collide(Sphere const* sphere) const
    {
        return GenericCollide(sphere);
    }

    bool Collide(Box const* box) const
    {
        return GenericCollide(box);
    }
};
} // namespace hierarchy
} // namespace volumes
} // namespace geometry
} // namespace pegasus
#endif // PEGASUS_BOUNDING_VOLUMES_HPP
