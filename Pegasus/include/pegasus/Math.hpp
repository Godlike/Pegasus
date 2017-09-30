/*
* Copyright (C) 2017 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#ifndef PEGASUS_MATH_HPP
#define PEGASUS_MATH_HPP

#include <pegasus/SharedMacros.hpp>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/norm.hpp>
#include <glm/glm.hpp>

#include <algorithm>
#include <iterator>
#include <array>
#include <vector>
#include <list>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <numeric>

namespace pegasus
{
namespace math
{
namespace fp
{
constexpr double g_floatingPointThreshold = 1e-4;

/**
 *  @brief  Checks if given floating point number is equal to null within a set threshold
 *
 *  @param  n   number
 *
 *  @return @c true if a number is equal to null, @c false otherwise
 */
inline bool IsZero(double n)
{
    return glm::abs(n) < g_floatingPointThreshold;
}

/**
 *  @brief  Checks if left-hand side value is equal to the right-hand side
 *          within a set threshold
 *
 *  @param  lhs left-hand side value
 *  @param  rhs right-hand side value
 *
 *  @return @c true if numbers are equal, @c false otherwise
 *
 *  @sa IsNotEqual
 */
inline bool IsEqual(double lhs, double rhs)
{
    return glm::abs(glm::abs(lhs) - glm::abs(rhs)) < g_floatingPointThreshold;
}

/**
 *  @brief  Checks if left-hand side value is not equal to the right-hand side
 *          within a set threshold
 *
 *  @param  lhs left-hand side value
 *  @param  rhs right-hand side value
 *
 *  @return @c true if numbers are equal, @c false otherwise
 *
 *  @sa IsEqual
 */
inline bool IsNotEqual(double lhs, double rhs)
{
    return !IsEqual(lhs, rhs);
}

/**
 *  @brief  Checks if left-hand side value is less than right-hand side value
 *          within a set threshold
 *
 *  @param  lhs left-hand side value
 *  @param  rhs right-hand side value
 *
 *  @return @c true if lhs is less than rhs, @c false othewise
 */
inline bool IsLess(double lhs, double rhs)
{
    return !IsEqual(lhs, rhs) && (lhs - rhs) < 0.0;
}

/**
 *  @brief  Checks if left-hand side value is greater than right-hand side value
 *          within a set threshold and returns true if so
 *
 *  @param  lhs left-hand side value
 *  @param  rhs right-hand side value
 *
 *  @return @c true if lhs is greater than rhs, @c false otherwise
 */
inline bool IsGreater(double lhs, double rhs)
{
    return !IsEqual(lhs, rhs) && (lhs - rhs) > 0.0;
}

/**
 *  @brief  Checks if left-hand side value is less than or equal to right-hand
 *          side value within a set threshold
 *
 *  @param  lhs left-hand side value
 *  @param  rhs right-hand side value
 *
 *  @return @c true if lhs is less than or equal to rhs, @c false otherwise
 */
inline bool IsLessOrEqual(double lhs, double rhs)
{
    return !IsGreater(lhs, rhs);
}

/**
 *  @brief  Checks if left-hand side value is greater than or equal to a
 *          right-hand side value within a set threshold
 *
 *  @param  lhs left-hand side value
 *  @param  rhs right-hand side value
 *
 *  @return @c true if lhs is greater than or equal to rhs, @c false otherwise
 */
inline bool IsGreaterOrEqual(double lhs, double rhs)
{
    return !IsLess(lhs, rhs);
}
} // namespace fp

/**
 * @brief HyperPlane calculation algorithm
 */
class HyperPlane
{
public:
    HyperPlane() = default;

    /**
     * @brief Constructs a plane in Hessian Normal Form
     *
     * Allows for the normal direction correction
     * using below the hyperplane point if one is specified.
     * @param[in] normal plane's normal vector of unit length
     * @param[in] point point on the plane
     * @param[in] below point below the plane, allows for the normal direction correction
     */
    PEGASUS_EXPORT HyperPlane(glm::dvec3 const& normal,
        glm::dvec3 const& point,
        glm::dvec3 const* below = nullptr
    );

    /**
     * @brief Constructs a plane in Hessian Normal Form
     *
     * Constructs a hyperplane from the given vertices
     * and allows for the normal direction correction
     * using below the hyperplane point if one is specified.
     * @param[in] a point on the plane
     * @param[in] b point on the plane
     * @param[in] c point on the plane
     * @param[in] below point below the plane, allows for the normal direction correction
     */
    PEGASUS_EXPORT HyperPlane(glm::dvec3 const& a,
        glm::dvec3 const& b,
        glm::dvec3 const& c,
        glm::dvec3 const* below = nullptr
    );

    /**
     * @brief Constructs a plane in Hessian Normal Form
     *
     * Constructs a hyperplane from the given vertices
     * and allows for the normal direction correction
     * using below the hyperplane point if one is specified.
     * @param[in] vertices points on the plane
     * @param[in] below point below the plane
     */
    PEGASUS_EXPORT HyperPlane(
        glm::dmat3 const& vertices,
        glm::dvec3 const* below = nullptr
    );

    /** brief Returns point on the plane */
    PEGASUS_EXPORT glm::dvec3 const& GetPoint() const;

    /** Returns plane normal vector */
    PEGASUS_EXPORT glm::dvec3 const& GetNormal() const;

    /** Returns a distance from the plane to the origin */
    PEGASUS_EXPORT double GetDistance() const;

    /** Sets the plane normal vector */
    PEGASUS_EXPORT void SetNormal(glm::dvec3 const& normal);

    /** Sets a point on the plane */
    PEGASUS_EXPORT void SetPoint(glm::dvec3 const& point);

    /**
     * @brief Calculates absolute distance from the plane to a point
     * @param[in] point the point of interest
     * @return absolute distance from the point to the plane
     */
    PEGASUS_EXPORT double Distance(glm::dvec3 const& point) const;

    /**
     * @brief Calculates signed distance from the plane to a point
     * @param[in] point the point of interest
     * @return signed distance from the plane to the point
     */
    PEGASUS_EXPORT double SignedDistance(glm::dvec3 const& point) const;

    /**
     * @brief Calculates whether a ray and the plane are intersecting
     * @param[in] rayNormal ray direction vector
     * @param[in] rayPoint point on the ray
     * @param[out] resultPoint intersection point
     * @return @c true if there is an intersection point, @c false otherwise
     */
    bool RayIntersection(
        glm::dvec3 const& rayNormal, glm::dvec3 const& rayPoint, glm::dvec3& resultPoint
    ) const;

    /**
     * @brief Calculates whether a line segment and the plane are intersecting
     * @param[in] lineStart start of the line segment
     * @param[in] lineEnd end of the line segment
     * @param[out] resultPoint intersection point
     * @return @c true if there is intersection point, @c false otherwise
     */
    PEGASUS_EXPORT bool LineSegmentIntersection(
        glm::dvec3 const& lineStart, glm::dvec3 const& lineEnd, glm::dvec3& resultPoint
    ) const;

    /**
     *  @brief  Calculates closest point on the plane to a given point
     *
     *  @param  point point of interest
     *
     *  @return closest point on the plane
     */
    glm::dvec3 ClosestPoint(glm::dvec3 const& point) const;

private:
    glm::dvec3 m_normal;
    glm::dvec3 m_point;
    double m_distance;
};

/**
 * @brief Data structure dedicated to store the mesh with most of the information contained in half-edges
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
     * @brief HEDS Face data container
     */
    class Face
    {
    public:
        template <typename HalfEdgeType>
        class HalfEdgeCirculator;
        template <typename FaceType>
        class AdjacentFaceCirculator;

        using edge_iterator = HalfEdgeCirculator<HalfEdge>;
        using const_edge_iterator = HalfEdgeCirculator<HalfEdge const>;
        using face_iterator = AdjacentFaceCirculator<Face>;
        using const_face_iterator = AdjacentFaceCirculator<Face const>;

        /**
         * @brief Face half-edges circular iterator
         *
         * Iterator of a circular data structure,
         * does not have an end state. If the data structure is in
         * valid state always dereferencable.
         * Under the hood iterates along the inner half-edges of the face.
         * @tparam HalfEdgeType half-edge type
         */
        template <typename HalfEdgeType>
        class HalfEdgeCirculator : public std::iterator<std::bidirectional_iterator_tag, HalfEdgeType>
        {
        public:
            /**
             * @brief Constructs a circular iterator from a given half-edge
             * @param halfEdge
             */
            explicit HalfEdgeCirculator(HalfEdgeType& halfEdge)
                : m_current(&halfEdge)
            {
            }

            /**
             * @brief Returns a reference to the current half-edge
             * @return reference to the current half-edge
             */
            HalfEdgeType& operator*() const
            {
                return *m_current;
            }

            /**
             * @brief Returns a pointer to the current half-edge
             * @return pointer to the current half-edge
             */
            HalfEdgeType* operator->() const
            {
                return m_current;
            }

            /**
             * @brief Increments an iterator and returns its reference
             * @return reference to the incremented iterator
             */
            HalfEdgeCirculator& operator++()
            {
                m_current = m_current->next;
                return *this;
            }

            /**
             * @brief Increments an iterator and returns the iterator before the increment
             * @return iterator before the increment
             */
            HalfEdgeCirculator operator++(int)
            {
                HalfEdgeCirculator result(*this);
                operator++();
                return result;
            }

            /**
             * @brief Decrements an iterator and returns its reference
             * @return reference to the decremented iterator
             */
            HalfEdgeCirculator& operator--()
            {
                m_current = m_current->prev;
                return *this;
            }

            /**
             * @brief Decrements an iterator and returns the iterator before the decrement
             * @return iterator before the decrement
             */
            HalfEdgeCirculator operator--(int)
            {
                HalfEdgeCirculator result(*this);
                operator--();
                return result;
            }

            /**
             * @brief Performs a memberwise comparison of the iterators and returns true if they are equal
             * @param[in] other right hand side iterator reference
             * @return true if iterators are equal
             */
            bool operator==(HalfEdgeCirculator const& other) const
            {
                return m_current == other.m_current;
            }

            /**
             * @brief Performs a memberwise comparison of the iterators and returns true if they are not equal
             * @param[in] other right hand side iterator reference
             * @return false if iterators are equal
             */
            bool operator!=(HalfEdgeCirculator const& other) const
            {
                return !operator==(other);
            }

            /**
            * @brief Swaps two iterators content
            * @param[in] lhs left hand side iterator
            * @param[in] rhs right hand side iterator
            */
            friend void swap(HalfEdgeCirculator& lhs, HalfEdgeCirculator& rhs) noexcept
            {
                std::swap(lhs.m_current, rhs.m_current);
            }

            /**
            * @brief Constructs const iterator from the current iterator and returns it
            */
            operator HalfEdgeCirculator<HalfEdgeType const>() const
            {
                return HalfEdgeCirculator(*this);
            }

        private:
            HalfEdgeType* m_current;
        };

        /**
         * @brief Adjacent faces iterator
         *
         * Iterator of a circular data structure,
         * does not have an end state. If the data structure is in
         * valid state always dereferencable.
         * Under the hood iterates along the inner half-edges of the face.
         * Access to adjacent faces is performed via twin half-edge.
         * @tparam FaceType face type
         */
        template <typename FaceType>
        class AdjacentFaceCirculator : public std::iterator<std::bidirectional_iterator_tag, FaceType>
        {
        public:
            /**
             * @brief Constructs an adjacent face iterator from the given half-edge
             * @param[in] halfEdge a half-edge object
             */
            explicit AdjacentFaceCirculator(HalfEdge& halfEdge)
                : m_current(&halfEdge)
            {
            }

            /**
             * @brief Returns a reference to the current adjacent face
             * @return a reference to the current adjacent face
             */
            FaceType& operator*() const
            {
                return *m_current->twin->face;
            }

            /**
             * @brief Returns a pointer to the current adjacent face
             * @return a pointer to the current adjacent face
             */
            FaceType* operator->() const
            {
                return m_current->twin->face;
            }

            /**
             * @brief Increments an iterator to the next adjacent face
             * @return incremented iterator reference
             */
            AdjacentFaceCirculator& operator++()
            {
                m_current = m_current->next;
                return *this;
            }

            /**
             * @brief Increments an iterator to the next adjacent face
             * @return iterator before increment
             */
            AdjacentFaceCirculator operator++(int)
            {
                AdjacentFaceCirculator result(*this);
                operator++();
                return result;
            }

            /**
             * @brief Decrements an interator to the previous adjacent face
             * @return decremented iterator
             */
            AdjacentFaceCirculator& operator--()
            {
                m_current = m_current->prev;
                return *this;
            }

            /**
             * @brief Decrements an interator to the previous adjacent face
             * @return iterator before decrement
             */
            AdjacentFaceCirculator operator--(int)
            {
                AdjacentFaceCirculator result(*this);
                operator++();
                return result;
            }

            /**
             * @brief Performs a memberwise comparison and returns true if iterators are equal
             * @param[in] other reference to the right hand side iterator
             * @return result of the comparation
             */
            bool operator==(AdjacentFaceCirculator const& other) const
            {
                return m_current == other.m_current;
            }

            /**
             * @brief Performs a memberwise comparison and returns true if iterators are different
             * @param[in] other reference to the right hand side iterator
             * @return result of the comparation
             */
            bool operator!=(AdjacentFaceCirculator const& other) const
            {
                return !operator==(other);
            }

            /**
             * @brief Swaps two iterators content
             * @param[in] lhs left hand side iterator
             * @param[in] rhs right hand side iterator
             */
            friend void swap(AdjacentFaceCirculator& lhs, AdjacentFaceCirculator& rhs) noexcept
            {
                std::swap(lhs.m_current, lhs.m_current);
            }

            /**
             * @brief Constructs const iterator from the current iterator and returns it
             */
            operator AdjacentFaceCirculator<FaceType const>() const
            {
                return AdjacentFaceCirculator<FaceType const>(*this);
            }

        private:
            HalfEdge* m_current;
        };

        /**
         * @brief Constructs a face from a given half-edge
         * @param[in] halfEdge face hald-edge
         */
        PEGASUS_EXPORT explicit Face(HalfEdge& halfEdge);

        /**
         * @brief Returns half-edge iterator
         * @return half-edge iterator of current face
         */
        PEGASUS_EXPORT edge_iterator GetHalfEdgeIterator();

        /**
        * @brief Returns half-edge const iterator
        * @return half-edge const iterator
        */
        PEGASUS_EXPORT const_edge_iterator GetHalfEdgeIterator() const;

        /**
         * @brief Returns iterator of adjacent faces
         * @return adjacent face iterator
         */
        PEGASUS_EXPORT face_iterator GetAdjacentFaceIterator();

        /**
        * @brief Returns iterator to const adjacent faces
        * @return adjacent face const iterator
        */
        PEGASUS_EXPORT const_face_iterator GetAdjacentFaceIterator() const;

    private:
        HalfEdge* m_halfEdge;
    };

    /**
     * @brief Object that can be used as a key for associative containers
     */
    struct FaceVertices
    {
        uint64_t a;
        uint64_t b;
        uint64_t c;

        /* Hasher struct for the key object */
        struct Hasher
        {
            /**
            * @brief Calculates a hash sum over the given FaceVertices object
            * @param face key for the hash calculation
            * @return hash sum value
            */
            PEGASUS_EXPORT size_t operator()(FaceVertices const& face) const;
        };

        PEGASUS_EXPORT bool operator==(FaceVertices const& other) const;
    };

    /**
     * @brief Data structure that describes one side of the edge
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
     * @brief Inserts face into the current data structure
     * @param[in] a vertex index
     * @param[in] b vertex index
     * @param[in] c vertex index
     */
    PEGASUS_EXPORT void MakeFace(uint64_t a, uint64_t b, uint64_t c);

    /**
     * @brief Returns a face iterator
     * @param[in] a vertex index
     * @param[in] b vertex index
     * @param[in] c vertex index
     * @return face iterator
     */
    PEGASUS_EXPORT face_iterator GetFace(uint64_t a, uint64_t b, uint64_t c);

    /**
    * @brief Returns a const face iterator
    * @param[in] a vertex index
    * @param[in] b vertex index
    * @param[in] c vertex index
    * @return face const iterator
    */
    PEGASUS_EXPORT const_face_iterator GetFace(uint64_t a, uint64_t b, uint64_t c) const;

    /**
     * @brief Returns end face iterator
     * @return end face const iterator
     */
    PEGASUS_EXPORT const_face_iterator GetFaceEnd() const;

    /**
     * @brief Removes face from the current data structure
     * @param[in] a vertex index
     * @param[in] b vertex index
     * @param[in] c vertex index
     */
    PEGASUS_EXPORT void RemoveFace(uint64_t a, uint64_t b, uint64_t c);

    /**
     * @brief Removes a face from the current data structure
     * @param[in] faceIterator iterator pointing to the element to be removed
     */
    PEGASUS_EXPORT void RemoveFace(face_iterator faceIterator);

private:
    /**
     * @brief Object that can be used as a key for associative containers
     */
    struct HalfEdgeVertices
    {
        uint64_t vertexIndexFrom;
        uint64_t vertexIndexTo;

        /* Hasher struct for the key object */
        struct Hasher
        {
            /**
            * @brief Calculates a hash sum over the given HalfEdgeVertices object
            * @param edge key for the hash calculation
            * @return hash sum value
            */
            size_t operator()(HalfEdgeVertices const& edge) const;
        };

        bool operator==(HalfEdgeVertices const& other) const;
    };

    HalfEdges m_halfEdgeList;
    std::unordered_map<
        HalfEdge const*, HalfEdges::iterator
    > m_halfEdgePointerIteratorMap;
    std::unordered_map<
        HalfEdgeVertices, HalfEdges::iterator, HalfEdgeVertices::Hasher
    > m_halfEdgeVerticesIteratorMap;

    Faces m_facesList;
    std::unordered_map<
        Face const*, Faces::iterator
    > m_faceIteratorMap;
    std::unordered_map<
        FaceVertices, Faces::iterator, FaceVertices::Hasher
    > m_faceVerticesIteratorMap;

    /**
     * @brief Initializes given half-edge
     * @param[in] he half-edge iterator
     * @param[in] next next half-edge pointer
     * @param[in] prev previous half-edge pointer
     * @param[in] face half-edge face pointer
     * @param[in] vertexIndexFrom start edge vertex index
     * @param[in] vertexIndexTo end edge vertex index
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
 * @brief Calculates mean value for a given range
 * @tparam Iterator forward iterator
 * @param[in] begin start of the range
 * @param[in] end end of the range
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
 * @brief Calculates covariance matrix for a given range of values
 * @tparam Iterator forward iterator
 * @param[in] begin start of the range
 * @param[in] end end of the range
 * @param[in] mean expected value
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
 * @brief Finds farthest point above given hyperplane
 * @tparam Iterator forward iterator
 * @param[in] begin start of the range
 * @param[in] end end of the range
 * @param[in] hyperPlane hyperplane
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
 * @brief Finds farthest point above given hyperplane and returns its index
 * @tparam Iterator forwars iterator
 * @param[in] begin start of the range
 * @param[in] end end of the range
 * @param[in] hyperPlane hyperplane
 * @return farthest vertex index
 */
template <typename Iterator>
size_t FindExtremalVertexIndex(Iterator begin, Iterator end, HyperPlane const& hyperPlane)
{
    size_t index = 0;
    double maxDistance = 0.0;
    size_t maxIndex = std::numeric_limits<size_t>::max();

    for (; begin != end; ++begin)
    {
        double const distance = hyperPlane.SignedDistance(*begin);
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
 * @brief Finds extremal vertices for a given range on given @p basis and writes them to @p minimaVertices and @p maximaVertices
 * @tparam Iterator forward iterator
 * @param[in] begin start of the range
 * @param[in] end end of the range
 * @param[in] basis basis vectors
 * @param[out] minimaVertices vertices with a minima projections on a basis
 * @param[out] maximaVertices vertices with a maxima projections on a basis
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
 *  @brief  Calculates box vertices in the world coordinate space from a given
 *          orthogonal basis and it's position
 *
 *  Writes output vertices to the container starting with @p verticesBeginIterator
 *
 *  @attention  There must be at least 7 more elements following given iterator
 *
 *  @tparam VerticesContainerIt Random access iterator
 *
 *  @param[in]  i       box axis vector
 *  @param[in]  j       box axis vector
 *  @param[in]  k       box axis vector
 *  @param[in]  center  center of the box
 *  @param[out] verticesBeginIterator   iterator to the container
 */
template <typename VerticesContainerIt>
void CalculateBoxVertices(
    glm::dvec3 const& i, glm::dvec3 const& j, glm::dvec3 const& k, glm::dvec3 const& center,
    VerticesContainerIt verticesBeginIterator
)
{
    verticesBeginIterator[0] = (i + j + k) + center;
    verticesBeginIterator[1] = (i - j + k) + center;
    verticesBeginIterator[2] = (j - i + k) + center;
    verticesBeginIterator[3] = (-i - j + k) + center;
    verticesBeginIterator[4] = (i + j - k) + center;
    verticesBeginIterator[5] = (i - j - k) + center;
    verticesBeginIterator[6] = (j - i - k) + center;
    verticesBeginIterator[7] = (-i - j - k) + center;
}

/**
* @brief Calculates box vertices in the model coordinate space from a given orthogonal basis
*
* Writes output vertices to the container starting with @p verticesBeginIterator. There must
* be at least 7 more elements following given iterator.
* @tparam VerticesContainerIt Random access iterator
* @param[in] i box axis vector
* @param[in] j box axis vector
* @param[in] k box axis vector
* @param[out] verticesBeginIterator iterator to the container that is able to store 8 vertices
*/
template <typename VerticesContainerIt>
void CalculateBoxVertices(
    glm::dvec3 const& i, glm::dvec3 const& j, glm::dvec3 const& k,
    VerticesContainerIt verticesBeginIterator
)
{
    CalculateBoxVertices(i, j, k, glm::dvec3{}, verticesBeginIterator);
}

/**
* @brief Effectively calculate all cross product vectors from two input ranges and writes all valid results to the output container
* @tparam SrcIt1 Forward iterator from the container of GLM vectors of size 3
* @tparam SrcIt2 Forward iterator from the container of GLM vectors of size 3
* @tparam DestIt Forward iterator from the container of GLM vectors of size 3
* @param[in] srcBegin1 iterator pointing to the start of the first input range
* @param[in] srcEnd1 iterator pointing to the end of the first input range
* @param[in] srcBegin2 iterator pointing to the start of the second input range
* @param[in] srcEnd2 iterator pointing to the end of the second input range
* @param[out] destBegin iterator pointing to the output container
*/
template <typename SrcIt1, typename SrcIt2, typename DestIt>
void CalculateCrossProductForeach(SrcIt1 srcBegin1, SrcIt1 srcEnd1, SrcIt2 srcBegin2, SrcIt2 srcEnd2,
    std::back_insert_iterator<DestIt> destBegin)
{
    for (auto it1 = srcBegin1; it1 != srcEnd1; ++it1)
    {
        for (auto it2 = srcBegin2; it2 != srcEnd2; ++it2)
        {
            auto const axis = glm::normalize(glm::cross(*it1, *it2));
            if (glm::length2(axis) != 0.0)
            {
                destBegin++ = axis;
            }
        }
    }
}

/**
* @brief Calculates a dot product for every element in the input range with given vector and writes it to the output container
* @tparam VectorType GLM vector type
* @tparam InIterator forward iterator from the container of VectorType objects
* @tparam OutIterator forward iterator from the container of double or float
* @param[in] axis vector along which to calculate dot products
* @param[in] srcBegin iterator pointing to the start of the input range
* @param[in] srcEnd iterator pointing to the end of the input range
* @param[out] destBegin iterator pointing to the output container
*/
template <typename VectorType, typename InIterator, typename OutIterator>
void CalculateDotProductForeach(
    VectorType const& axis, InIterator srcBegin, InIterator srcEnd, OutIterator destBegin)
{
    while (srcBegin != srcEnd)
    {
        *destBegin++ = glm::dot(axis, (*srcBegin++));
    }
}

/**
 * @brief Calculates arbitrary orthonormal vector to the given one
 * @tparam VectorType glm vector of size 3
 * @param vector of interest
 * @return orthonormal vector to the vector of interest
 */
template < typename VectorType >
VectorType CalculateOrthogonalVector(VectorType vector)
{
    VectorType result;

    for (uint8_t i = 0; i < result.length(); ++i)
    {
        if (vector[i] != 0.0)
        {
            result[(1 + i) % 3] = vector[i];
            result[i] = -vector[(1 + i) % 3];
        }
    }
    result = glm::normalize(result);

    return result;
}

/**
 * @brief Calculates distance between a point and line segment
 * @param[in] lineStart start of the line segment
 * @param[in] lineEnd end of the line segment
 * @param[in] point point of interest
 * @return distance between a point and line segment
 */
PEGASUS_EXPORT double LineSegmentPointDistance(
    glm::dvec3 const& lineStart, glm::dvec3 const& lineEnd, glm::dvec3 point
);

/**
 * @brief Jacobi eigenvalue calculation algorithm
 */
class JacobiEigenvalue
{
public:
    /**
     * @brief Constructs and calculates eigenvalues and eigenvectors of a given symmetric matrix
     * @param[in] symmetricMatrix
     * @param[in] coverageThreshold
     * @param[in] maxIterations
     */
    PEGASUS_EXPORT explicit JacobiEigenvalue(
        glm::dmat3 const& symmetricMatrix,
        double coverageThreshold = 1.0e-4,
        uint32_t maxIterations = 100
    );

    /**
     * @brief Returns eigenvectors
     * @return eigenvectos matrix
     */
    PEGASUS_EXPORT glm::dmat3 const& GetEigenvectors() const;

    /**
     * @brief Returns eigenvalues
     * @return eigenvalue vector
     */
    PEGASUS_EXPORT glm::dvec3 const& GetEigenvalues() const;

private:
    glm::dmat3 const& m_symmetricMatrix;
    double const m_coverageThreshold;
    uint32_t const m_maxIterations;
    glm::dmat3 m_eigenvectors;
    glm::dvec3 m_eigenvalues;

    /**
     * @brief Finds maximal absolute value off diagonal matrix element and sets its indices
     * @param[in] mat matrix to search
     * @param[out] i max element row index
     * @param[out] j max element columnt index
     */
    static void FindMaxNormOffDiagonal(glm::dmat3 const& mat, uint8_t& i, uint8_t& j);

    /**
     * @brief Calculates rotation angle for a given matrix and its element
     * @param[in] mat matrix to rotate
     * @param[in] i element row index
     * @param[in] j element column index
     * @return angle in radians
     */
    double CalculateRotationAngle(glm::dmat3 const& mat, uint8_t i, uint8_t j) const;

    /**
     * @brief Makes Givens rotation matrix from the angle and indices
     * @param[in] theta rotation angle in radians
     * @param[in] i row index
     * @param[in] j column index
     * @return Givens rotation matrix
     */
    static glm::dmat3 MakeGivensRotationMatrix(double theta, uint8_t i, uint8_t j);

    /**
     * @brief Calculates eigenvalues and eigenvectors
     */
    void Calculate();
};

/**
 * @brief Quickhull convex hull calculation algorithm
 * @tparam VerticesType STL compatible random access glm::dvec3 container
 */
template <typename VerticesType>
class QuickhullConvexHull
{
public:
    class Face;
    using Faces = std::list<Face>;
    using Vertices = VerticesType;

    /**
     * @brief Convex hull face container
     */
    class Face
    {
    public:
        /**
         * @brief Constructs a face without initializing above vertices
         * @param[in] hedsFaceIterator HEDS face iterator
         * @param[in] hyperPlane hyperplane that given face lies on
         * @param[in] indices indices of the face's vertices
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
         * @brief Constructs face and initializes vertices above it
         * @param[in] vertexBuffer common vertex buffer
         * @param[in] partitionMarkedVertices vertices marked for partitioning
         * @param[in] hedsFaceIterator HEDS face iterator
         * @param[in] hyperPlane hyperplane that given face lies on
         * @param[in] indices indices of the face's vertices
         */
        Face(Vertices& vertexBuffer,
            std::list<size_t>& partitionMarkedVertices,
            HalfEdgeDataStructure::face_iterator hedsFaceIterator,
            HyperPlane const& hyperPlane,
            std::array<size_t, 3> const& indices
        )
            : Face(hedsFaceIterator, hyperPlane, indices)
        {
            AddVertices(vertexBuffer, partitionMarkedVertices);
        }

        /**
         * @brief Moves vertices that are above the face from the input container to the local container
         *
         * Uses vertexBuffer as a reference point for the index calculation without changing it,
         * removes vertices from partitionMarkedVertices list that are above the current face's hyperplane
         * effectively moving them to the current face.
         * @param[in] vertexBuffer common vertex buffer
         * @param[in, out] partitionMarkedVertices vertices marked for partitioning
         */
        void AddVertices(Vertices const& vertexBuffer, std::list<size_t>& partitionMarkedVertices)
        {
            auto findAboveVertices = [this, &vertexBuffer](size_t index)
            {
                return m_hyperPlane.SignedDistance(vertexBuffer[index]) > 0;
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
                m_extremalVertexIndex = *std::max_element(m_vertices.begin(), m_vertices.end(),
                    [this, &vertexBuffer](size_t a, size_t b)
                {
                    return m_hyperPlane.SignedDistance(vertexBuffer[a]) < m_hyperPlane.SignedDistance(vertexBuffer[b]);
                });

                m_extremalVertex = vertexBuffer[m_extremalVertexIndex];
            }
        }

        /**
         * @brief Returns vertices that are above current face's hyperplane
         * @return face vertices container
         */
        std::vector<size_t>& GetVertices()
        {
            return m_vertices;
        }

        /**
         * @brief Returns farthest vertex above face's hyperplane
         * @return extremal vertex iterator
         */
        glm::dvec3 GetExtremalVertex() const
        {
            return m_extremalVertex;
        }

        /**
         * @brief Returns farthest vertex index above face's hyperplane
         * @return extremal vertex index
         */
        size_t GetExtremalVertexIndex() const
        {
            return m_extremalVertexIndex;
        }

        /**
         * @brief Returns a HEDS face iterator for the current face
         * @return HEDS face iterator
         */
        HalfEdgeDataStructure::face_iterator GetHedsFaceIterator() const
        {
            return m_hedsFaceIterator;
        }

        /**
         * @brief Returns current face's hyperplane
         * @return current face's hyperplane
         */
        HyperPlane const& GetHyperPlane() const
        {
            return m_hyperPlane;
        }

        /**
         * @brief Returns indices that form current face
         * @return face indices array
         */
        std::array<size_t, 3> const& GetIndices() const
        {
            return m_indices;
        }

    private:
        std::vector<size_t> m_vertices;
        glm::dvec3 m_extremalVertex;
        size_t m_extremalVertexIndex;
        HalfEdgeDataStructure::face_iterator m_hedsFaceIterator;
        HyperPlane m_hyperPlane;
        std::array<size_t, 3> m_indices;
    };

    /**
     * @brief Constructs a quickhull convex hull calculation algorithm object
     * @param vertices vertex buffer object
     */
    explicit QuickhullConvexHull(Vertices& vertices)
        : m_vertexBuffer(vertices)
    {
    }

    /**
     *  @brief  Adds vertex to the current convex hull if it's outside
     *
     *  Does nothing if vertex is positioned inside conver hull
     *
     *  @param  index   index of a point to be added to the convex hull
     *
     *  @return @c true if vertex was added, @c false otherwise
     */
    bool AddVertex(size_t index)
    {
        //Create stack of convex hull faces
        std::list<typename Faces::iterator> faceIteratorStack;
        for (auto faceIt = m_faces.begin(); faceIt != m_faces.end(); ++faceIt)
        {
            faceIteratorStack.push_back(faceIt);
        }

        //Find initial visible face
        typename Faces::iterator visibleFaceIterator = m_faces.end();
        for (typename Faces::iterator faceIt : faceIteratorStack)
        {
            if (faceIt->GetHyperPlane().SignedDistance(m_vertexBuffer[index]) > 0)
            {
                visibleFaceIterator = faceIt;
                break;
            }
        }

        //If the point is outside of the convex hull, add it
        if (visibleFaceIterator != m_faces.end())
        {
            AddVertex(faceIteratorStack, index, visibleFaceIterator);
            return true;
        }

        return false;
    }

    /**
     * @brief Calculates a convex hull from a given vertex buffer
     */
    void Calculate()
    {
        CalculateInitialGuess();
        Refine();
    }

    /**
     * @brief Returns convex hull faces
     * @return convex hull faces
     */
    Faces const& GetFaces() const
    {
        return m_faces;
    }

    /**
     * @brief Returns convex hull vertices
     * @return convex hull vertices
     */
    std::list<size_t> const& GetVertices() const
    {
        return m_convexHullVertices;
    }

private:
    Faces m_faces;
    Vertices& m_vertexBuffer;
    std::list<size_t> m_convexHullVertices;
    glm::dvec3 m_mean;
    HalfEdgeDataStructure m_heds;
    std::unordered_map<
        HalfEdgeDataStructure::Face*, typename Faces::iterator
    > m_hedsFaceIteratorMap;

    /**
     *  @brief Adds vertex to the convex hull
     *
     *  @param[in,out]  faceStack           container of the convex hull's faces
     *  @param[in]      extremalVertexIndex index of extremal vertex on the visible face
     *  @param[in]      visibleFaceIterator initial visible face of the convex hull
     */
    void AddVertex(
        std::list<typename Faces::iterator>& faceStack,
        size_t extremalVertexIndex,
        typename Faces::iterator visibleFaceIterator
    )
    {
        std::list<size_t> partitionMarkedVertexIndices;
        std::list<std::pair<size_t, size_t>> horizonRidges;
        std::list<typename Faces::iterator> visibleFaces;
        std::unordered_set<Face*> visibleFacesSet;
        std::unordered_set<Face*> visitedFaces;

        //Initialize containers with the first visible face
        {
            visibleFaces.push_back(visibleFaceIterator);
            visibleFacesSet.insert(&*visibleFaceIterator);
            std::vector<size_t>& faceVertices = visibleFaceIterator->GetVertices();
            partitionMarkedVertexIndices.insert(partitionMarkedVertexIndices.end(), faceVertices.begin(), faceVertices.end());
        }

        //Find all visible faces and horizon ridges
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
                    if (adjFace->GetHyperPlane().SignedDistance(m_vertexBuffer[extremalVertexIndex]) > 0)
                    {
                        visibleFaces.push_back(adjFace);
                        visibleFacesSet.insert(&*adjFace);

                        std::vector<size_t>& faceVertices = adjFace->GetVertices();
                        partitionMarkedVertexIndices.insert(partitionMarkedVertexIndices.end(), faceVertices.begin(), faceVertices.end());
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

        //Add vertex to the convex hull set
        m_convexHullVertices.push_back(extremalVertexIndex);

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

        //Make new faces from horizon ridges and the convex hull point
        for (auto& ridge : horizonRidges)
        {
            faceStack.push_back(
                MakeFace(ridge.first, ridge.second, extremalVertexIndex, partitionMarkedVertexIndices)
            );
        }
    }

    /**
     *  @brief  Inserts a new face into current convex hull
     *
     *  Removes vertices from @p partitionMarkedVertices list if they are above
     *  the current face's hyperplane
     *
     *  @param[in]      vertexIndex1            index of a face
     *  @param[in]      vertexIndex2            index of a face
     *  @param[in]      vertexIndex3            index of a face
     *  @param[in,out]  partitionMarkedVertices modifiable vertex buffer
     *
     *  @return iterator to a new face
     */
    typename Faces::iterator MakeFace(
        size_t vertexIndex1, size_t vertexIndex2, size_t vertexIndex3,
        std::list<size_t>& partitionMarkedVertices
    )
    {
        m_heds.MakeFace(vertexIndex1, vertexIndex2, vertexIndex3);
        m_faces.push_front(Face{
            m_vertexBuffer,
            partitionMarkedVertices,
            m_heds.GetFace(vertexIndex1, vertexIndex2, vertexIndex3),
            HyperPlane{m_vertexBuffer[vertexIndex1], m_vertexBuffer[vertexIndex2], m_vertexBuffer[vertexIndex3], &m_mean},
            std::array<size_t, 3>{{vertexIndex1, vertexIndex2, vertexIndex3}}
        });
        m_hedsFaceIteratorMap[&*m_faces.front().GetHedsFaceIterator()] = m_faces.begin();

        return m_faces.begin();
    }

    /**
     *  @brief  Removes face from a current convex hull and corresponding HEDS object
     *
     *  @param  faceIterator    face to be removed
     */
    void RemoveFace(typename Faces::iterator faceIterator)
    {
        m_hedsFaceIteratorMap.erase(&*faceIterator->GetHedsFaceIterator());
        m_heds.RemoveFace(faceIterator->GetHedsFaceIterator());
        m_faces.erase(faceIterator);
    }

    /**
     *  @brief  Calculates initial tetrahedron from the vertex buffer
     */
    void CalculateInitialTetrahedron()
    {
        //Calculate covariance matrix and its eigenvectors
        m_mean = CalculateExpectedValue(m_vertexBuffer.begin(), m_vertexBuffer.end());
        JacobiEigenvalue const eigenvalue(CalculateCovarianceMatrix(m_vertexBuffer.begin(), m_vertexBuffer.end(), m_mean));
        glm::dmat3 eigenvectorsNormalized = eigenvalue.GetEigenvectors();
        eigenvectorsNormalized = {
            glm::normalize(eigenvectorsNormalized[0]),
            glm::normalize(eigenvectorsNormalized[1]),
            glm::normalize(eigenvectorsNormalized[2]),
        };

        //Calculate extremal vertices
        std::array<typename Vertices::iterator, 3> maximaVertices, minimaVertices;
        FindExtremalVertices(m_vertexBuffer.begin(), m_vertexBuffer.end(), eigenvectorsNormalized, minimaVertices, maximaVertices);
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
            m_vertexBuffer.begin(), m_vertexBuffer.end(),
            [&baseFacePlane](glm::dvec3& a, glm::dvec3& b)
        {
            return baseFacePlane.Distance(a) < baseFacePlane.Distance(b);
        });

        //Initialize base hull tetrahedron
        m_convexHullVertices = {
            static_cast<size_t>(std::distance(m_vertexBuffer.begin(), mostDistantPair.first)),
            static_cast<size_t>(std::distance(m_vertexBuffer.begin(), mostDistantPair.second)),
            static_cast<size_t>(std::distance(m_vertexBuffer.begin(), triangleBasePoint)),
            static_cast<size_t>(std::distance(m_vertexBuffer.begin(), tetrahedronApexPoint))
        };

        std::list<size_t> markedVertexIndices(m_vertexBuffer.size());
        std::iota(markedVertexIndices.begin(), markedVertexIndices.end(), 0);
        std::array<size_t, 4> const indices = {{
            static_cast<size_t>(std::distance(m_vertexBuffer.begin(), mostDistantPair.first)),
            static_cast<size_t>(std::distance(m_vertexBuffer.begin(), mostDistantPair.second)),
            static_cast<size_t>(std::distance(m_vertexBuffer.begin(), triangleBasePoint)),
            static_cast<size_t>(std::distance(m_vertexBuffer.begin(), tetrahedronApexPoint))
        }};

        MakeFace(indices[0], indices[1], indices[2], markedVertexIndices);
        MakeFace(indices[0], indices[1], indices[3], markedVertexIndices);
        MakeFace(indices[0], indices[2], indices[3], markedVertexIndices);
        MakeFace(indices[1], indices[2], indices[3], markedVertexIndices);
    }

    /**
     *  @brief  Calculates initial guess tetrahedron
     */
    void CalculateInitialGuess()
    {
        if (m_vertexBuffer.size() == 4)
        {
            m_mean = CalculateExpectedValue(m_vertexBuffer.begin(), m_vertexBuffer.end());
            m_convexHullVertices.resize(m_vertexBuffer.size());
            std::iota(m_convexHullVertices.begin(), m_convexHullVertices.end(), 0);

            std::list<size_t> partitionMarkedVertices;
            MakeFace(0, 1, 2, partitionMarkedVertices);
            MakeFace(0, 1, 3, partitionMarkedVertices);
            MakeFace(0, 2, 3, partitionMarkedVertices);
            MakeFace(1, 2, 3, partitionMarkedVertices);
        }
        else
        {
            CalculateInitialTetrahedron();
        }
    }

    /**
     *  @brief  Refines initial guess tetrahedron by calculating final convex hull
     */
    void Refine()
    {
        //Create stack of convex hull faces
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
                AddVertex(
                    faceStack, faceIt->GetExtremalVertexIndex(), faceIt
                );
            }
        }
    }
};

} // namespace math
} // namespace pegasus

#endif // PEGASUS_MATH_HPP
