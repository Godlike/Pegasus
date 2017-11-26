/*
* Copyright (C) 2017 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#ifndef PEGASUS_MATH_CONVEX_HULL_HPP
#define PEGASUS_MATH_CONVEX_HULL_HPP

#include <math/HyperPlane.hpp>
#include <math/HalfEdgeDataStructure.hpp>
#include <math/JacobiEigenvalue.hpp>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/norm.hpp>
#include <glm/glm.hpp>

#include <set>
#include <unordered_set>
#include <array>
#include <list>
#include <algorithm>
#include <numeric>

namespace pegasus
{
namespace math
{
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
     *  Does nothing if vertex is positioned inside convex hull
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
#endif // PEGASUS_MATH_CONVEX_HULL_HPP
