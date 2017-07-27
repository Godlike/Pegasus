/*
* Copyright (C) 2017 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#ifndef PEGASUS_MATH_HPP
#define PEGASUS_MATH_HPP

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/norm.hpp>
#include <glm/glm.hpp>

#include <iterator>
#include <vector>
#include <array>
#include <list>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include <utility>

namespace pegasus
{
namespace math
{
/**
 * @brief HyperPlane calculation algorithm.
 */
class HyperPlane
{
public:
    HyperPlane() = default;

    /**
     * @brief Constructs a plane in Hessian Normal Form.
     * @param normal normal plane vector
     * @param point point on the plane
     * @param below point below the plane
     */
    HyperPlane(glm::dvec3 const& normal,
        glm::dvec3 const& point,
        glm::dvec3 const* below = nullptr
    );

    /**
     * @brief Constructs a plane in Hessian Normal Form.
     * @param a point on a plane
     * @param b point on a plane
     * @param c point on a plane
     * @param below point below plane
     */
    HyperPlane(glm::dvec3 const& a,
        glm::dvec3 const& b,
        glm::dvec3 const& c,
        glm::dvec3 const* below = nullptr
    );

    /**
     * @brief Constructs a plane in Hessian Normal Form.
     * @param vertices points on a plane
     * @param below point below plane
     */
    HyperPlane(
        glm::dmat3 const& vertices,
        glm::dvec3 const* below
    );

    /**
     * @brief Plane point getter.
     * @return point on a plane
     */
    glm::dvec3 const& GetPoint() const;

    /**
     * @brief Plane normal getter.
     * @return plane normal
     */
    glm::dvec3 const& GetNormal() const;

    /**
     * @brief Distance from a plane to the origin getter.
     * @return distance from a plane to the origin
     */
    double GetDistance() const;

    /**
     * @brief Plane normal setter.
     * @param normal plane normal vector
     */
    void SetNormal(glm::dvec3 const& normal);

    /**
     * @brief Point on a plane setter.
     * @param point point on a plane
     */
    void SetPoint(glm::dvec3 const& point);

    /**
     * @brief Calculates absolute distance from the plane to a point.
     * @param point point of interest
     * @return distance
     */
    double Distance(glm::dvec3 const& point) const;

    /**
     * @brief Calculates signed distance from the plane to a point.
     * @param point point of interest
     * @return distance
     */
    double SignedDistance(glm::dvec3 const& point) const;

    /**
     * @brief Calculater wether a line segment and the plane are intersecting.
     * @param lineStart start of the line segment
     * @param lineEnd end of the lilne segment
     * @param resultPoint intersetion point
     * @return intersection state
     */
    bool Intersection(
        glm::dvec3 const& lineStart, glm::dvec3 const& lineEnd, glm::dvec3& resultPoint
    ) const;

private:
    glm::dvec3 m_normal;
    glm::dvec3 m_point;
    double m_distance;
    glm::dvec3 const* m_below;
};

/**
 * @brief Data structure dedicated to store the mesh with most of the information contained in half-edges.
 */
class HalfEdgeDataStructure
{
public:
    struct HalfEdge;
    class Face;

    using Faces = std::list<Face>;
    using HalfEdges = std::list<HalfEdge>;
    using face_iterator = Faces::iterator;
    using const_face_iterator = Faces::const_iterator;

    /**
     * @brief HEDS Face data container.
     */
    class Face
    {
    public:
        template <typename HalfEdgeType>
        class EdgeIterator;
        template <typename FaceType>
        class FaceIterator;

        using edge_iterator = EdgeIterator<HalfEdge>;
        using const_edge_iterator = EdgeIterator<HalfEdge const>;
        using face_iterator = FaceIterator<Face>;
        using const_face_iterator = FaceIterator<Face const>;

        /**
         * @brief Face half-edges iterator
         * @tparam HalfEdgeType half-edge type
         */
        template <typename HalfEdgeType>
        class EdgeIterator : public std::iterator<std::bidirectional_iterator_tag, HalfEdgeType>
        {
        public:
            explicit EdgeIterator(HalfEdgeType* halfEdge)
                : m_current(halfEdge)
            {
            }

            HalfEdgeType& operator*() const
            {
                return *m_current;
            }

            HalfEdgeType* operator->() const
            {
                return m_current;
            }

            EdgeIterator& operator++()
            {
                m_current = m_current->next;
                return *this;
            }

            EdgeIterator operator++(int)
            {
                EdgeIterator result(*this);
                operator++();
                return result;
            }

            EdgeIterator& operator--()
            {
                m_current = m_current->prev;
                return *this;
            }

            EdgeIterator operator--(int)
            {
                EdgeIterator result(*this);
                operator--();
                return result;
            }

            bool operator==(EdgeIterator const& other) const
            {
                return m_current == other.m_current;
            }

            bool operator!=(EdgeIterator const& other) const
            {
                return !operator==(other);
            }

            friend void swap(EdgeIterator& lhs, EdgeIterator& rhs) noexcept
            {
                std::swap(lhs.m_current, rhs.m_current);
            }

            operator EdgeIterator<HalfEdgeType const>() const
            {
                return EdgeIterator(*this);
            }

        private:
            HalfEdgeType* m_current;
        };

        /**
         * @brief Adjacent faces iterator
         * @tparam FaceType face type
         */
        template <typename FaceType>
        class FaceIterator : public std::iterator<std::bidirectional_iterator_tag, FaceType>
        {
        public:
            explicit FaceIterator(HalfEdge* halfEdge)
                : m_current(halfEdge)
            {
            }

            FaceType& operator*() const
            {
                return *m_current->twin->face;
            }

            FaceType* operator->() const
            {
                return m_current->twin->face;
            }

            FaceIterator& operator++()
            {
                m_current = m_current->next;
                return *this;
            }

            FaceIterator operator++(int)
            {
                FaceIterator result(*this);
                operator++();
                return result;
            }

            FaceIterator& operator--()
            {
                m_current = m_current->prev;
                return *this;
            }

            FaceIterator operator--(int)
            {
                FaceIterator result(*this);
                operator++();
                return result;
            }

            bool operator==(FaceIterator const& other) const
            {
                return m_current == other.m_current;
            }

            bool operator!=(FaceIterator const& other) const
            {
                return !operator==(other);
            }

            friend void swap(FaceIterator& lhs, FaceIterator& rhs) noexcept
            {
                std::swap(lhs.m_current, lhs.m_current);
            }

            operator FaceIterator<FaceType const>() const
            {
                return FaceIterator(*this);
            }

        private:
            HalfEdge* m_current;
        };

        /**
         * @brief Constructs a face from a given half-edge
         * @param halfEdge face hald-edge pointer
         */
        explicit Face(HalfEdge* halfEdge = nullptr);

        /**
         * @brief Faces half-edge iterator getter
         * @return half-edge iterator
         */
        edge_iterator GetHalfEdgeIterator();

        /**
        * @brief Faces half-edge const iterator getter
        * @return half-edge const iterator
        */
        const_edge_iterator GetHalfEdgeIterator() const;

        /**
         * @brief Faces adjacent face iteratir getter
         * @return adjacent face iterator
         */
        face_iterator GetAdjacentFaceIterator();

        /**
        * @brief Faces adjacent face const iteratir getter
        * @return adjacent face const iterator
        */
        const_face_iterator GetAdjacentFaceIterator() const;

    private:
        HalfEdge* m_halfEdge;
    };

    /**
     * @brief Face key for associative containers.
     */
    struct FaceVertices
    {
        uint64_t a;
        uint64_t b;
        uint64_t c;

        bool operator==(FaceVertices const& other) const;
    };

    /**
     * @brief FaceVertices hasher.
     */
    struct FaceVerticesHash
    {
        size_t operator()(FaceVertices const& face) const;
    };

    /**
     * @brief Data structure that containes a half of the edge.
     */
    struct HalfEdge
    {
        HalfEdge* next;
        HalfEdge* prev;
        HalfEdge* twin;
        Face* face;
        uint64_t vertexIndex;
    };

    /**
     * @brief Inserts face into current data structure.
     * @param a new face vertex index
     * @param b new face vertex index
     * @param c new face vertex index
     */
    void MakeFace(uint64_t a, uint64_t b, uint64_t c);

    /**
     * @brief Face iterator getter.
     * @param a face vertex index
     * @param b face vertex index
     * @param c face vertex index
     * @return face iterator
     */
    face_iterator GetFace(uint64_t a, uint64_t b, uint64_t c);

    /**
    * @brief Face const iterator getter.
    * @param a face vertex index
    * @param b face vertex index
    * @param c face vertex index
    * @return face const iterator
    */
    const_face_iterator GetFace(uint64_t a, uint64_t b, uint64_t c) const;

    /**
     * @brief Faces end iterator getter.
     * @return end face const iterator
     */
    const_face_iterator GetFaceEnd() const;

    /**
     * @brief Removes face from a current data structure.
     * @param a face vertex index
     * @param b face vertex index
     * @param c face vertex index
     */
    void RemoveFace(uint64_t a, uint64_t b, uint64_t c);

    /**
     * @brief Removes face from a current data structure.
     * @param faceIterator face to be removed iterator
     */
    void RemoveFace(face_iterator faceIterator);

private:
    /**
     * @brief HalfEdge key for an associative containers.
     */
    struct HalfEdgeVertices
    {
        uint64_t vertexIndexFrom;
        uint64_t vertexIndexTo;

        bool operator==(HalfEdgeVertices const& other) const;
    };

    /**
     * @brief HalfEdgeVertices hasher.
     */
    struct HalfEdgeVerticesHash
    {
        size_t operator()(HalfEdgeVertices const& edge) const;
    };

    HalfEdges m_halfEdgeList;
    std::unordered_map<
        HalfEdge const*, HalfEdges::iterator
    > m_halfEdgePointerIteratorMap;
    std::unordered_map<
        HalfEdgeVertices, HalfEdges::iterator, HalfEdgeVerticesHash
    > m_halfEdgeVerticesIteratorMap;

    Faces m_facesList;
    std::unordered_map<
        Face const*, Faces::iterator
    > m_faceIteratorMap;
    std::unordered_map<
        FaceVertices, Faces::iterator, FaceVerticesHash
    > m_faceVerticesIteratorMap;

    /**
     * @brief HalfEdge initializer.
     * @param he half-edge
     * @param next next half-edge
     * @param prev previous half-edge
     * @param face half-edge face
     * @param vertexIndexFrom start edge vertex index
     * @param vertexIndexTo end edge vertex index
     */
    void IntializeHalfEdge(
        HalfEdges::iterator he,
        HalfEdge* next, HalfEdge* prev,
        Face* face,
        uint64_t vertexIndexFrom,
        uint64_t vertexIndexTo
    );
};

/**
 * @brief Calculates mean value for a given range.
 * @tparam Iterator forward iterator
 * @param begin start of the range
 * @param end end of the range
 * @return mean value
 */
template <typename Iterator>
decltype(auto) CalculateExpectedValue(Iterator begin, Iterator end)
{
    auto E = *(begin++);
    uint32_t size = 1;

    for (; begin != end; ++begin)
    {
        E += *begin;
        ++size;
    }

    return E * (1.0 / size);
}

/**
 * @brief Calculates covariance matrix for a given range of values.
 * @tparam Iterator forward iterator
 * @param begin start of the range
 * @param end end of the range
 * @param mean expected value
 * @return covariance matrix
 */
template <typename Iterator>
glm::dmat3 CalculateCovarianceMatrix(Iterator begin, Iterator end, glm::dvec3 const& mean)
{
    glm::dmat3 covariance(0.0);
    uint32_t size = 0;

    for (; begin != end; ++begin)
    {
        for (uint8_t i = 0; i < 3; ++i)
        {
            for (uint8_t j = 0; j < 3; ++j)
            {
                covariance[i][j] += ((*begin)[i] - mean[i]) * ((*begin)[j] - mean[j]);
            }
        }
        ++size;
    }

    return covariance * (1.0 / size);
}

/**
 * @brief Finds farthest above point for a given hyperplane.
 * @tparam Iterator forward iterator
 * @param begin start of the range
 * @param end end of the range
 * @param hyperPlane hyperplane
 * @return farthest vertex iterator
 */
template <typename Iterator>
Iterator FindExtremalVertex(Iterator begin, Iterator end, HyperPlane const& hyperPlane)
{
    Iterator extremalVertexIt = std::max_element(
        begin, end, [&hyperPlane](glm::dvec3 const& a, glm::dvec3 const& b)
    {
        return hyperPlane.SignedDistance(a) < hyperPlane.SignedDistance(b);
    });

    return extremalVertexIt;
}

/**
 * @brief Finds farthest above point for a given hyperplane.
 * @tparam Iterator forwars iterator
 * @param begin start of the range
 * @param end end of the range
 * @param hyperPlane hyperplane
 * @return farthest vertex index
 */
template <typename Iterator>
size_t FindExtremalVertexIndex(Iterator begin, Iterator end, HyperPlane const& hyperPlane)
{
    size_t index = 0;
    double maxDistance = 0.0;
    double distance = 0.0;
    size_t maxIndex = std::numeric_limits<size_t>::max();

    for (; begin != end; ++begin)
    {
        distance = hyperPlane.SignedDistance(*begin);
        if (maxDistance < distance)
        {
            maxDistance = distance;
            maxIndex = index;
        }
        ++index;
    }

    return maxIndex;
}

/**
 * @brief Finds extremal vertices from a given range on a given basis.
 * @tparam Iterator forward iterator
 * @param begin start of the range
 * @param end end of the range
 * @param basis basis vectors
 * @param minimaVertices vertices with a minima projections on a basis
 * @param maximaVertices vertices with a maxima projections on a basis
 */
template <typename Iterator>
void FindExtremalVertices(
    Iterator begin, Iterator end, glm::dmat3 const& basis,
    std::array<Iterator, 3>& minimaVertices, std::array<Iterator, 3>& maximaVertices
)
{
    glm::dvec3 maximaProjections{std::numeric_limits<double>::min()};
    glm::dvec3 minimaProjections{std::numeric_limits<double>::max()};

    for (; begin != end; ++begin)
    {
        glm::dvec3 const projection = {
            glm::dot(*begin, basis[0]), glm::dot(*begin, basis[1]), glm::dot(*begin, basis[2])
        };

        for (uint8_t i = 0; i < 3; ++i)
        {
            if (projection[i] > maximaProjections[i])
            {
                maximaProjections[i] = projection[i];
                maximaVertices[i] = begin;
            }
            if (projection[i] < minimaProjections[i])
            {
                minimaProjections[i] = projection[i];
                minimaVertices[i] = begin;
            }
        }
    }
}

/**
 * @brief Calculates distance from a point to a line segment.
 * @param lineStart start of the line segment
 * @param lineEnd end of the line segment
 * @param point point of interest
 * @return distance
 */
double LineSegmentPointDistance(
    glm::dvec3 const& lineStart, glm::dvec3 const& lineEnd, glm::dvec3 const& point
);

/**
 * @brief Jacobi eigenvalue calculation algorithm.
 */
class JacobiEigenvalue
{
public:
    /**
     * @brief Constructs and calculates eigenvalues and eigenvectors of a given symmetric matrix.
     * @param symmetricMatrix
     * @param coverageThreshold
     * @param maxIterations
     */
    explicit JacobiEigenvalue(
        glm::dmat3 const& symmetricMatrix,
        double coverageThreshold = 1.0e-4,
        uint32_t maxIterations = 100
    );

    /**
     * @brief Eigenvectors getter.
     * @return eigenvectos matrix
     */
    glm::dmat3 const& GetEigenvectors() const;

    /**
     * @brief Eigenvalue getter.
     * @return eigenvalue vector
     */
    glm::dvec3 const& GetEigenvalues() const;

private:
    glm::dmat3 const& m_symmetricMatrix;
    double const m_coverageThreshold;
    uint32_t const m_maxIterations;
    glm::dmat3 m_eigenvectors;
    glm::dvec3 m_eigenvalues;

    /**
     * @brief Finds maximal off diagonal matrix element.
     * @param mat matrix to search
     * @param i max element row index
     * @param j max element columnt index
     */
    static void FindMaxNormOffDiagonal(glm::dmat3 const& mat, uint8_t& i, uint8_t& j);

    /**
     * @brief Calculates rotation angle for a given matrix and it's element.
     * @param mat matrix to rotate
     * @param i element row index
     * @param j element column index
     * @return angle in radians
     */
    double CalculateRotationAngle(glm::dmat3 const& mat, uint8_t i, uint8_t j) const;

    /**
     * @brief Makes Givens rotation matrix from the angle and indices.
     * @param theta rotation angle in radians
     * @param i row index
     * @param j column index
     * @return Givens rotation matrix
     */
    static glm::dmat3 MakeGivensRotationMatrix(double theta, uint8_t i, uint8_t j);

    /**
     * @brief Calculates eigenvalues and eigenvectors.
     */
    void Calculate();
};

/**
 * @brief Quickhull convex hull calculation algorithm.
 * @tparam Vertices STL compatible random access glm::dvec3 container
 */
template <typename Vertices>
class QuickhullConvexHull
{
public:
    class Face;
    using Faces = std::list<Face>;

    /**
     * @brief Convex hull face container.
     */
    class Face
    {
    public:
        /**
         * @brief Constructs a face without initializing above vertices.
         * @param hedsFaceIterator HEDS face iterator
         * @param hyperPlane HyperPlane
         * @param indices face indices
         */
        Face(HalfEdgeDataStructure::face_iterator hedsFaceIterator,
            HyperPlane const& hyperPlane,
            std::array<size_t, 3> const& indices
        )
            : m_extremalVertexIndex()
            , m_hedsFaceIterator(hedsFaceIterator)
            , m_hyperPlane(hyperPlane)
            , m_indices(indices)
        {
            std::sort(m_indices.begin(), m_indices.end());
        }

        /**
         * @brief Constructs face and initializes it's abobe vertices.
         * @param vertexBuffer common vertex buffer
         * @param partitionMarkedVertices vertices marked for partitioning
         * @param hedsFaceIterator HEDS face iterator
         * @param hyperPlane HypePlane
         * @param indices face indices
         */
        Face(Vertices& vertexBuffer,
            std::list<typename Vertices::iterator>& partitionMarkedVertices,
            HalfEdgeDataStructure::face_iterator hedsFaceIterator,
            HyperPlane const& hyperPlane,
            std::array<size_t, 3> const& indices
        )
            : Face(hedsFaceIterator, hyperPlane, indices)
        {
            SetVertices(vertexBuffer, partitionMarkedVertices);
        }

        /**
         * @brief Partitions vertices that are above the face.
         * @param vertexBuffer common vertex buffer
         * @param partitionMarkedVertices vertices marked for partitioning
         */
        void SetVertices(Vertices& vertexBuffer, std::list<typename Vertices::iterator>& partitionMarkedVertices)
        {
            auto findAboveVertices = [this](typename Vertices::iterator& vertexIt)
            {
                return m_hyperPlane.SignedDistance(*vertexIt) > 0;
            };

            std::copy_if(partitionMarkedVertices.begin(), partitionMarkedVertices.end(),
                std::back_inserter(m_vertices), findAboveVertices
            );
            partitionMarkedVertices.erase(
                std::remove_if(partitionMarkedVertices.begin(), partitionMarkedVertices.end(), findAboveVertices),
                partitionMarkedVertices.end()
            );

            if (!m_vertices.empty())
            {
                m_extremalVertex = *std::max_element(m_vertices.begin(), m_vertices.end(),
                    [this](typename Vertices::iterator& a, typename Vertices::iterator& b)
                {
                    return m_hyperPlane.SignedDistance(*a) < m_hyperPlane.SignedDistance(*b);
                });

                m_extremalVertexIndex = std::distance(vertexBuffer.begin(), m_extremalVertex);
            }
        }

        /**
         * @brief Face owned vertices getter.
         * @return face vertices container
         */
        std::vector<typename Vertices::iterator>& GetVertices()
        {
            return m_vertices;
        }

        /**
         * @brief Extremal face vertex getter.
         * @return extremal vertex iterator
         */
        typename Vertices::iterator GetExtremalVertex() const
        {
            return m_extremalVertex;
        }

        /**
         * @brief Extremal faces vertex index getter.
         * @return extremal vertex index
         */
        size_t GetExtremalVertexIndex() const
        {
            return m_extremalVertexIndex;
        }

        /**
         * @brief Face HEDS iterator getter.
         * @return HEDS face iterator
         */
        HalfEdgeDataStructure::face_iterator GetHedsFaceIterator() const
        {
            return m_hedsFaceIterator;
        }

        /**
         * @brief HyperPlane getter.
         * @return hyperplane
         */
        HyperPlane const& GetHyperPlane() const
        {
            return m_hyperPlane;
        }

        /**
         * @brief Face indices getter.
         * @return face indices array
         */
        std::array<size_t, 3> const& GetIndices() const
        {
            return m_indices;
        }

    private:
        std::vector<typename Vertices::iterator> m_vertices;
        typename Vertices::iterator m_extremalVertex;
        size_t m_extremalVertexIndex;
        HalfEdgeDataStructure::face_iterator m_hedsFaceIterator;
        HyperPlane m_hyperPlane;
        std::array<size_t, 3> m_indices;
    };

    /**
     * @brief Constructs a quickhull convex hull calculation algorithm object.
     * @param vertices vertex buffer object
     */
    explicit QuickhullConvexHull(Vertices& vertices)
        : m_vertices(vertices)
    {
    }

    /**
     * @brief Calculates a convex hull from a given vertex buffer.
     */
    void Calculate()
    {
        CalculateInitialGuess();
        Refine();
    }

    /**
     * @brief Returns convex hull faces.
     * @return convex hull faces.
     */
    Faces const& GetFaces() const
    {
        return m_faces;
    }

    /**
     * @brief Returns convex hull vertices.
     * @return convex hull vertices.
     */
    std::list<typename Vertices::iterator> const& GetVertices() const
    {
        return m_convexHullVertices;
    }

private:
    Faces m_faces;
    Vertices& m_vertices;
    std::list<typename Vertices::iterator> m_convexHullVertices;
    glm::dvec3 m_mean;
    HalfEdgeDataStructure m_heds;
    std::unordered_map<
        HalfEdgeDataStructure::Face*, typename Faces::iterator
    > m_hedsFaceIteratorMap;

    /**
     * @brief Calculates initial guess tetrahedron and performs initalization.
     */
    void CalculateInitialGuess()
    {
        //Calculate covariance matrix and it's eigenvectors
        m_mean = CalculateExpectedValue(m_vertices.begin(), m_vertices.end());
        JacobiEigenvalue const eigenvalue(CalculateCovarianceMatrix(m_vertices.begin(), m_vertices.end(), m_mean));
        glm::dmat3 eigenvectorsNormalized = eigenvalue.GetEigenvectors();
        eigenvectorsNormalized = {
            glm::normalize(eigenvectorsNormalized[0]),
            glm::normalize(eigenvectorsNormalized[1]),
            glm::normalize(eigenvectorsNormalized[2]),
        };

        //Calculate extremal vertices
        std::array<typename Vertices::iterator, 3> maximaVertices, minimaVertices;
        FindExtremalVertices(m_vertices.begin(), m_vertices.end(), eigenvectorsNormalized, maximaVertices, minimaVertices);
        std::set<typename Vertices::iterator> extremalPoints(maximaVertices.begin(), maximaVertices.end());
        extremalPoints.insert(minimaVertices.begin(), minimaVertices.end());

        //Calculate base line points
        auto mostDistantPair = std::make_pair(*maximaVertices.begin(), *minimaVertices.begin());
        double maxDistanceSq = 0.0;
        for (auto a : extremalPoints)
        {
            for (auto b : extremalPoints)
            {
                double const distSq = glm::length2(*a - *b);
                if (distSq > maxDistanceSq)
                {
                    maxDistanceSq = distSq;
                    mostDistantPair = std::make_pair(a, b);
                }
            }
        }

        //Calculate triangle base point
        auto triangleBasePoint = *std::max_element(
            extremalPoints.begin(), extremalPoints.end(),
            [&mostDistantPair](typename Vertices::iterator v1, typename Vertices::iterator v2)
        {
            return LineSegmentPointDistance(*mostDistantPair.first, *mostDistantPair.second, *v1)
                < LineSegmentPointDistance(*mostDistantPair.first, *mostDistantPair.second, *v2);
        });

        //Calculate tetrahedron apex point
        HyperPlane baseFacePlane(*mostDistantPair.first, *mostDistantPair.second, *triangleBasePoint);
        auto tetrahedronApexPoint = std::max_element(
            m_vertices.begin(), m_vertices.end(),
            [&baseFacePlane](glm::dvec3& a, glm::dvec3& b)
        {
            return baseFacePlane.Distance(a) < baseFacePlane.Distance(b);
        });

        //Initialize base hull tetrahedron
        m_convexHullVertices = {
            mostDistantPair.first, mostDistantPair.second, triangleBasePoint, tetrahedronApexPoint
        };
        std::array<size_t, 4> indices = {
            static_cast<size_t>(std::distance(m_vertices.begin(), mostDistantPair.first)),
            static_cast<size_t>(std::distance(m_vertices.begin(), mostDistantPair.second)),
            static_cast<size_t>(std::distance(m_vertices.begin(), triangleBasePoint)),
            static_cast<size_t>(std::distance(m_vertices.begin(), tetrahedronApexPoint))
        };

        std::list<typename Vertices::iterator> markedVertexIterators;
        for (auto vertexIt = m_vertices.begin(); vertexIt != m_vertices.end(); ++vertexIt)
        {
            markedVertexIterators.push_back(vertexIt);
        }
        MakeFace(indices[0], indices[1], indices[2], markedVertexIterators);
        MakeFace(indices[0], indices[1], indices[3], markedVertexIterators);
        MakeFace(indices[0], indices[2], indices[3], markedVertexIterators);
        MakeFace(indices[1], indices[2], indices[3], markedVertexIterators);
    }

    /**
     * @brief Refines initial guess tetrahedron by calculating final convex hull.
     */
    void Refine()
    {
        std::list<typename Faces::iterator> faceStack;
        for (auto faceIt = m_faces.begin(); faceIt != m_faces.end(); ++faceIt)
        {
            faceStack.push_back(faceIt);
        }

        while (!faceStack.empty())
        {
            typename Faces::iterator faceIt = faceStack.front();
            faceStack.pop_front();

            //If face does not has vertices above it, remove it
            if (!faceIt->GetVertices().empty())
            {
                std::list<typename Vertices::iterator> markedVertexIterators;
                std::list<std::pair<size_t, size_t>> horizonRidges;
                std::list<typename Faces::iterator> visibleFaces(1, faceIt);
                std::unordered_set<Face*> visibleFacesSet{&*faceIt};
                std::unordered_set<Face*> visitedFaces;

                auto extremalVertex = faceIt->GetExtremalVertex();
                auto extremalVertexIndex = faceIt->GetExtremalVertexIndex();

                {
                    std::vector<typename Vertices::iterator>& faceVertices = faceIt->GetVertices();
                    markedVertexIterators.insert(markedVertexIterators.end(), faceVertices.begin(), faceVertices.end());
                    std::vector<typename Vertices::iterator>().swap(faceVertices);
                }

                //For each visible face
                for (auto visibleFaceIt = visibleFaces.begin(); visibleFaceIt != visibleFaces.end(); ++visibleFaceIt)
                {
                    auto adjHedsFaceIt = (*visibleFaceIt)->GetHedsFaceIterator()->GetAdjacentFaceIterator();
                    auto adjHedsFaceBegin = adjHedsFaceIt;
                    visitedFaces.insert(&(**visibleFaceIt));

                    //For all adjacent faces
                    do
                    {
                        typename Faces::iterator adjFace = m_hedsFaceIteratorMap[&*adjHedsFaceIt];

                        //If face is unvisited
                        if (visitedFaces.find(&*adjFace) == visitedFaces.end()
                            && visibleFacesSet.find(&*adjFace) == visibleFacesSet.end())
                        {
                            //If face is visible add it to the visible faces set
                            if (adjFace->GetHyperPlane().SignedDistance(*extremalVertex) > 0)
                            {
                                visibleFaces.push_back(adjFace);
                                visibleFacesSet.insert(&*adjFace);

                                std::vector<typename Vertices::iterator>& faceVertices = adjFace->GetVertices();
                                markedVertexIterators.insert(markedVertexIterators.end(), faceVertices.begin(), faceVertices.end());
                                std::vector<typename Vertices::iterator>().swap(faceVertices);
                            }
                            //If face is not visible then find a ridge and save it
                            else
                            {
                                std::vector<size_t> ridgeIndices;
                                ridgeIndices.reserve(2);

                                auto adjFaceIndices = adjFace->GetIndices();
                                auto faceIndices = (*visibleFaceIt)->GetIndices();
                                std::set_intersection(adjFaceIndices.begin(), adjFaceIndices.end(),
                                    faceIndices.begin(), faceIndices.end(), std::back_inserter(ridgeIndices));

                                horizonRidges.emplace_back(ridgeIndices[0], ridgeIndices[1]);
                            }
                        }
                    }
                    while (++adjHedsFaceIt != adjHedsFaceBegin);
                }

                m_convexHullVertices.push_back(extremalVertex);

                //Remove visible faces
                for (typename Faces::iterator visibleFaceIt : visibleFaces)
                {
                    faceStack.erase(std::remove_if(faceStack.begin(), faceStack.end(),
                        [&visibleFaceIt](typename Faces::iterator& it)
                    {
                        return &*it == &*visibleFaceIt;
                    }), faceStack.end());
                    RemoveFace(visibleFaceIt);
                }

                //Make new faces from horizon ridges and an extremal point
                for (auto& ridge : horizonRidges)
                {
                    faceStack.push_back(
                        MakeFace(ridge.first, ridge.second, extremalVertexIndex, markedVertexIterators)
                    );
                }
            }
        }
    }

    /**
     * @brief Inserts a new face into current convex hull.
     * @param vertexIndex1 index of a face
     * @param vertexIndex2 index of a face
     * @param vertexIndex3 index of a face
     * @param vertices modifiable vertex buffer
     * @return iterator to a new face
     */
    typename Faces::iterator MakeFace(
        size_t vertexIndex1, size_t vertexIndex2, size_t vertexIndex3,
        std::list<typename Vertices::iterator>& vertices
    )
    {
        m_heds.MakeFace(vertexIndex1, vertexIndex2, vertexIndex3);
        m_faces.push_front(Face{
            m_vertices,
            vertices,
            m_heds.GetFace(vertexIndex1, vertexIndex2, vertexIndex3),
            HyperPlane{m_vertices[vertexIndex1], m_vertices[vertexIndex2], m_vertices[vertexIndex3], &m_mean},
            std::array<size_t, 3>{vertexIndex1, vertexIndex2, vertexIndex3}
        });
        m_hedsFaceIteratorMap[&*m_faces.front().GetHedsFaceIterator()] = m_faces.begin();

        return m_faces.begin();
    }

    /**
     * @brief Removes face from a current convex hull.
     * @param faceIterator face to be removed iterator
     */
    void RemoveFace(typename Faces::iterator faceIterator)
    {
        m_hedsFaceIteratorMap.erase(&*faceIterator->GetHedsFaceIterator());
        m_heds.RemoveFace(faceIterator->GetHedsFaceIterator());
        m_faces.erase(faceIterator);
    }
};
} // namespace math
} // namespace pegasus

#endif // PEGASUS_MATH_HPP
