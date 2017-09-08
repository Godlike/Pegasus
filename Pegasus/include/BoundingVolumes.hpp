/*
* Copyright (C) 2017 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#ifndef PEGASUS_BOUNDING_VOLUMES_HPP
#define PEGASUS_BOUNDING_VOLUMES_HPP

#include <Pegasus/include/Geometry.hpp>
#include <Pegasus/include/Math.hpp>

#include <glm/ext.hpp>

#include <algorithm>
#include <array>
#include <random>
#include <queue>
#include <set>
#include <stack>
#include <unordered_map>
#include <vector>

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

/**
 * Template struct for extracting SimpleShape instances out of bounding volumes,
 * specialized for different bounding volumes.
 *
 * @tparam BoundingVolume bounding volume type
 */
template <typename BoundingVolume>
struct SimpleShapeExtractor
{
};

/**
 * AxisAlignedBoundingBox SimpleShapeExtractor specialization
 */
template <>
struct SimpleShapeExtractor<aabb::AxisAlignedBoundingBox>
{
    /**
     * Extracts Box from AxisAlignedBoundingBox instance.
     *
     * @param aabb AxisAlignedBoundingBox instance to extract volume from
     * @return extracted Box
     */
    Box GetSimpleShape(aabb::AxisAlignedBoundingBox const& aabb)
    {
        return aabb.GetBox();
    }
};

/**
 * OrientedBoundingBox SimpleShapeExtractor specialization
 */
template <>
struct SimpleShapeExtractor<obb::OrientedBoundingBox>
{
    /**
     * Extracts Box from OrientedBoundingBox instance.
     *
     * @param obb OrientedBoundingBox instance to extract volume from
     * @return extracted Box
     */
    Box GetSimpleShape(obb::OrientedBoundingBox const& obb)
    {
        return obb.GetBox();
    }
};

/**
 * BoundingSphere SimpleShapeExtractor specialization
 */
template <>
struct SimpleShapeExtractor<sphere::BoundingSphere>
{
    /**
     * Extracts Sphere from BoundingSphere instance.
     *
     * @param sphere BoundingSphere instance to extract volume from
     * @return extracted Sphere
     */
    Sphere GetSimpleShape(sphere::BoundingSphere const& sphere)
    {
        return sphere.GetSphere();
    }
};

/**
 * General bounding volume hierarchy calculation class, builds a binary tree of bounding volumes.
 *
 * @tparam BoundingVolume bounding volume type to be used in hierarchy
 */
template <typename BoundingVolume>
class BoundingVolumeHierarchy
{
public:
    std::size_t static const MAX_NODE_SIZE = 5;

    /**
     * BoundingVolumeHierarchy internal node data structure.
     *
     * Cannot be copied or moved. Can only be constructed inside BoundingVolumeHierarchy class.
     */
    class Node
    {
    public:
        BoundingVolume const volume;

        Node(Node const&) = delete;
        Node(Node &&) = delete;
        Node & operator=(Node const&) = delete;
        Node & operator=(Node &&) = delete;

        /**
         * Gets a lower child of this node.
         *
         * @return lower child
         */
        std::unique_ptr<Node> const& GetLowerChild() const
        {
            return m_lowerChild;
        }

        /**
         * Gets an upper child of this node.
         *
         * @return upper child
         */
        std::unique_ptr<Node> const& GetUpperChild() const
        {
            return m_upperChild;
        }

        /**
         * Checks if this node is a leaf, i.e. doesn't have any children.
         *
         * @return true if this node is a leaf, false otherwise
         */
        bool IsLeaf() const
        {
            return m_isLeaf;
        }

        ~Node() = default;

    private:
        mutable std::unique_ptr<Node> m_lowerChild;
        mutable std::unique_ptr<Node> m_upperChild;
        bool m_isLeaf;

        /**
         * Constructs a node out of volume. Children are initialized
         * to nullptr, and the node is assumed to be a leaf.
         *
         * @param volume
         */
        explicit Node(BoundingVolume && volume)
            : volume(volume)
            , m_lowerChild(nullptr)
            , m_upperChild(nullptr)
            , m_isLeaf(true)
        {}

        friend class BoundingVolumeHierarchy;
    };

    using NodePtr = std::unique_ptr<Node>;

    /**
     * Constructs BoundingVolumeHierarchy, using iterative depth-first traversal.
     *
     * @param shape shape to construct volume hierarchy around
     * @param indices indices of faces of shape to use in algorithm
     */
    BoundingVolumeHierarchy(Shape const& shape, Indices const& indices)
        : m_shape(shape)
        , m_root(new Node(BoundingVolume(shape, indices)))
    {
        std::stack<Node*> nodeStack;
        std::stack<Indices> indicesStack;
        nodeStack.push(m_root.get());
        indicesStack.push(indices);

        while (!nodeStack.empty())
        {
            Node* currNode = nodeStack.top();
            Indices currIndices = std::move(indicesStack.top());
            nodeStack.pop();
            indicesStack.pop();

            if (currIndices.size() < MAX_NODE_SIZE)
            {
                currNode->m_lowerChild.reset();
                currNode->m_upperChild.reset();
                continue;
            }

            currNode->m_isLeaf = false;

            Vertices currVertices = GetShapeVertices(currIndices);
            glm::dvec3 maxDistanceVector = GetMaximumDistanceDirectionApprox(currVertices);
            glm::dvec3 min = GetMinVectorAlongDirection(maxDistanceVector, currVertices);
            math::HyperPlane minPlane(maxDistanceVector, min);

            CentroidsMap centroids = GetFacesCentroids(currIndices);
            SplitIndices splitIndices = GetPartitionIndices(minPlane, centroids);

            BoundingVolume lowerVolume(m_shape, splitIndices.lowerIndices);
            BoundingVolume upperVolume(m_shape, splitIndices.upperIndices);

            currNode->m_lowerChild = NodePtr(new Node(std::move(lowerVolume)));
            currNode->m_upperChild = NodePtr(new Node(std::move(upperVolume)));

            nodeStack.push(currNode->m_lowerChild.get());
            indicesStack.push(std::move(splitIndices.lowerIndices));
            nodeStack.push(currNode->m_upperChild.get());
            indicesStack.push(std::move(splitIndices.upperIndices));
        }
    }

    ~BoundingVolumeHierarchy()
    {
        std::stack<NodePtr> nodeStack;
        nodeStack.push(NodePtr(m_root.release()));

        while (!nodeStack.empty())
        {
            NodePtr & currNode = nodeStack.top();
            if (currNode->m_lowerChild.get() != nullptr)
            {
                nodeStack.push(NodePtr(currNode->m_lowerChild.release()));
                continue;
            }
            if (currNode->m_upperChild.get() != nullptr)
            {
                nodeStack.push(NodePtr(currNode->m_upperChild.release()));
                continue;
            }
            nodeStack.pop();
        }
    }

    /**
     * Gets hierarchy's root.
     *
     * @return unique_ptr to root
     */
    NodePtr const& GetRoot() const
    {
        return m_root;
    }

    /**
     * Tests volume hierarchy collision with a plane.
     *
     * @param plane plane to test collision with
     * @return true if there exists a leaf node, whose volume intersects with a plane; false otherwise
     */
    bool Collide(Plane const* plane) const
    {
        return GenericCollide(plane);
    }

    /**
     * Tests volume hierarchy collision with a sphere.
     *
     * @param sphere sphere to test collision with
     * @return true if there exists a leaf node, whose volume intersects with a sphere; false otherwise
     */
    bool Collide(Sphere const* sphere) const
    {
        return GenericCollide(sphere);
    }

    /**
     * Tests volume hierarchy collision with a box.
     *
     * @param box plane to test collision with
     * @return true if there exists a leaf node, whose volume intersects with a box; false otherwise
     */
    bool Collide(Box const* box) const
    {
        return GenericCollide(box);
    }

private:

    struct SplitIndices
    {
        Indices lowerIndices;
        Indices upperIndices;
    };

    using CentroidsMap = std::unordered_map<std::size_t, glm::dvec3>;

    Shape const& m_shape;
    NodePtr m_root;
    mutable SimpleShapeIntersectionDetector m_detector;
    mutable SimpleShapeExtractor<BoundingVolume> m_extractor;

    /**
     * Gets vertices, belonging to m_shape with given indices.
     *
     * @param indices indices of faces to extract vertices from
     * @return vertices of a shape
     */
    Vertices GetShapeVertices(Indices const& indices) const
    {
        Vertices result;
        result.reserve(3 * indices.size());
        for (std::size_t i : indices)
        {
            result.insert(result.end(), {
                m_shape.vertices[m_shape.faces[i][0]],
                m_shape.vertices[m_shape.faces[i][1]],
                m_shape.vertices[m_shape.faces[i][2]]
            });
        }
        return result;
    }

    /**
     * Calculates an approximation of a vector, alongside which the shape has maximum length.
     *
     * @param vertices vertices of a shape
     * @return approximate maximum distance vector
     */
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

    /**
     * Calculates a vertice, which has minimal radius when projected onto
     * a given maximum distance vector.
     *
     * @param maxDistanceVector approximate maximum distance vector
     * @param vertices vertices of a shape
     * @return calculated vertice
     */
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

    /**
     * Gets centroids of faces of the shape.
     *
     * @param indices indices of faces to use
     * @return map of indices to centroids
     */
    CentroidsMap GetFacesCentroids(Indices const& indices) const
    {
        CentroidsMap centroids;
        for (std::size_t index : indices)
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

    /**
     * Calculates indices of shapes for a lower child and an upper child.
     * Finds a median centroid, using distance to minPlane,
     * then returning the closest half of vertices as lower child indices
     * and furthest half of vertices as upper child indices.
     *
     * @param minPlane a plane to measure distance of centroids from
     * @param centroids centroids of faces
     * @return indices for children
     */
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

    /**
     * Checks if shape in a node intersects with a given shape.
     *
     * @param node node to extract the shape from
     * @param shape shape to intersect node's shape with
     * @return true if node's shape intersects with a given shape; false otherwise
     */
    bool CollideNode(Node* node, SimpleShape const* shape) const
    {
        auto nodeShape = m_extractor.GetSimpleShape(node->volume);
        return m_detector.CalculateIntersection(&nodeShape, shape);
    }

    /**
     * Checks if a given shape intersects with any of the hierarchy's leafs.
     *
     * @param shape
     * @return
     */
    bool GenericCollide(SimpleShape const* shape) const
    {
        std::stack<Node*> nodeStack;
        nodeStack.push(m_root.get());

        while (!nodeStack.empty())
        {
            Node* currNode = nodeStack.top();
            nodeStack.pop();
            if (!CollideNode(currNode, shape))
            {
                continue;
            }
            if (currNode->m_isLeaf)
            {
                return true;
            }
            nodeStack.push(currNode->m_lowerChild.get());
            nodeStack.push(currNode->m_upperChild.get());
        }
        return false;
    }
};
} // namespace hierarchy
} // namespace volumes
} // namespace geometry
} // namespace pegasus
#endif // PEGASUS_BOUNDING_VOLUMES_HPP
