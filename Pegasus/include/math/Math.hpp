/*
* Copyright (C) 2017 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#ifndef PEGASUS_MATH_HPP
#define PEGASUS_MATH_HPP

#include <pegasus/SharedMacros.hpp>
#include <math/HyperPlane.hpp>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/norm.hpp>
#include <glm/glm.hpp>

#include <algorithm>
#include <array>

namespace pegasus
{
namespace math
{
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
 * @tparam Iterator forward iterator
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
    glm::dvec3 maximaProjections{std::numeric_limits<double>::lowest()};
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

    return glm::normalize(result);;
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
} // namespace math
} // namespace pegasus

#endif // PEGASUS_MATH_HPP
