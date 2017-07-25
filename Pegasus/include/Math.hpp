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
#include <map>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include <utility>


namespace pegasus
{
namespace math
{

class HyperPlane
{
public:
	HyperPlane() = default;

    HyperPlane(glm::dvec3 const& normal,
               glm::dvec3 const& point,
               glm::dvec3 const* below = nullptr);

    HyperPlane(glm::dvec3 const& a,
               glm::dvec3 const& b,
               glm::dvec3 const& c,
               glm::dvec3 const* below = nullptr);

    HyperPlane(glm::dmat3 const& vertices, glm::dvec3 const* below);

    glm::dvec3 const& GetPoint() const;

    glm::dvec3 const& GetNormal() const;

    double GetDistance() const;

    void SetNormal(glm::dvec3 const& normal);

    void SetPoint(glm::dvec3 const& point);

    double Distance(glm::dvec3 const& point) const;

    double SignedDistance(glm::dvec3 const& point) const;

    bool Intersection(glm::dvec3 const& lineStart, glm::dvec3 const& lineEnd, glm::dvec3 & resultPoint) const;

private:
    glm::dvec3 m_normal;
    glm::dvec3 m_point;
    double m_distance;
    glm::dvec3 const* m_below;
};

class HalfEdgeDataStructure
{
public:
    struct HalfEdge;
	class Face;

	using Faces = std::list<Face>;
	using face_iterator = Faces::iterator;
	using const_face_iterator = Faces::const_iterator;

    class Face
    {
    public:
        template < typename HalfEdgeType >
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

		template < typename FaceType >
		class FaceIterator : public std::iterator<std::bidirectional_iterator_tag, FaceType >
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

        using edge_iterator = EdgeIterator<HalfEdge>;
        using const_edge_iterator = EdgeIterator<HalfEdge const>;
		using face_iterator = FaceIterator<Face>;
		using const_face_iterator = FaceIterator<Face const>;


        Face(HalfEdge* halfEdge = nullptr)
            : m_halfEdge(halfEdge)
        {
        }

        edge_iterator GetHalfEdgeIterator()
        {
            return edge_iterator{ m_halfEdge };
        }

        const_edge_iterator GetHalfEdgeIterator() const
        {
            return const_edge_iterator{ m_halfEdge };
        }

		face_iterator GetAdjacentFaceIterator()
        {
			return face_iterator{ m_halfEdge };
        }

		const_face_iterator GetAdjacentFaceIterator() const
        {
			return const_face_iterator{ m_halfEdge };
        }

    private:
        HalfEdge* m_halfEdge;
    };

	struct FaceVertices
	{
		uint64_t a;
		uint64_t b;
		uint64_t c;

		bool operator==(FaceVertices const& other) const
		{
			std::array<uint64_t, 3> pointers = { a, b, c };
			std::sort(pointers.begin(), pointers.end());
			std::array<uint64_t, 3> otherPointers = { other.a, other.b, other.c };
			std::sort(otherPointers.begin(), otherPointers.end());

			return pointers[0] == otherPointers[0]
				&& pointers[1] == otherPointers[1]
				&& pointers[2] == otherPointers[2];
		}
	};

	struct FaceVerticesHash
	{
		size_t operator()(FaceVertices const& face) const
		{
			return std::hash<uint64_t>{}(face.a)
				^ std::hash<uint64_t>{}(face.b)
				^ std::hash<uint64_t>{}(face.c);
		}
	};

    struct HalfEdge
    {
        HalfEdge* next;
        HalfEdge* prev;
        HalfEdge* twin;
        Face* face;
        uint64_t vertexIndex;
    };
    using HalfEdges = std::list<HalfEdge>;

    void MakeFace(uint64_t a, uint64_t b, uint64_t c)
    {
        FaceVertices const faceVerticesKey{a, b, c};
        if (m_faceVerticesIteratorMap.find(faceVerticesKey) == m_faceVerticesIteratorMap.end())
        {
			//Allocate half edges
            std::array<HalfEdges::iterator, 3> newHalfEdges = {
                m_halfEdgeList.emplace(m_halfEdgeList.end()),
                m_halfEdgeList.emplace(m_halfEdgeList.end()),
                m_halfEdgeList.emplace(m_halfEdgeList.end())
            };

			//Allocate face
            auto backFaceIterator = m_facesList.emplace(m_facesList.end(), &*newHalfEdges.back());
            m_faceIteratorMap[&*backFaceIterator] = backFaceIterator;
            m_faceVerticesIteratorMap[faceVerticesKey] = backFaceIterator;

			//Init half edges
            IntializeHalfEdge(newHalfEdges[0], &*newHalfEdges[1], &*newHalfEdges[2], &*backFaceIterator, a, b);
            IntializeHalfEdge(newHalfEdges[1], &*newHalfEdges[2], &*newHalfEdges[0], &*backFaceIterator, b, c);
            IntializeHalfEdge(newHalfEdges[2], &*newHalfEdges[0], &*newHalfEdges[1], &*backFaceIterator, c, a);
        }
    }

    face_iterator GetFace(uint64_t a, uint64_t b, uint64_t c)
    {
        FaceVertices faceVerticesKey{ a, b, c };
        auto faceIterator = m_faceVerticesIteratorMap.find(faceVerticesKey);
		if (faceIterator == m_faceVerticesIteratorMap.end()) {
			return m_facesList.end();
		}

        return faceIterator->second;
    }

    const_face_iterator GetFace(uint64_t a, uint64_t b, uint64_t c) const
    {
        FaceVertices const faceVerticesKey{ a, b, c };
        auto faceIterator = m_faceVerticesIteratorMap.find(faceVerticesKey);

    	if (faceIterator == m_faceVerticesIteratorMap.end()) {
			return m_facesList.end();
		}

        return faceIterator->second;
    }

	const_face_iterator GetFaceEnd() const
    {
		return m_facesList.end();
    }

    void RemoveFace(uint64_t a, uint64_t b, uint64_t c)
    {
		RemoveFace(GetFace(a, b, c));
    }

	void RemoveFace(face_iterator faceIterator)
    {
		auto heIterator = faceIterator->GetHalfEdgeIterator();
		std::array<HalfEdges::iterator, 3> markedHalfEdgeIterators;
		markedHalfEdgeIterators[0] = m_halfEdgePointerIteratorMap[&*heIterator++];
		markedHalfEdgeIterators[1] = m_halfEdgePointerIteratorMap[&*heIterator++];
		markedHalfEdgeIterators[2] = m_halfEdgePointerIteratorMap[&*heIterator];
		std::array<HalfEdge*, 3> twinMarkedHalfEdgeIterators = {
			markedHalfEdgeIterators[0]->twin,
			markedHalfEdgeIterators[1]->twin,
			markedHalfEdgeIterators[2]->twin,
		};

		for (auto markedHalfEdgeIterator : markedHalfEdgeIterators)
		{
			m_halfEdgePointerIteratorMap.erase(&*markedHalfEdgeIterator);
		}

		auto removeTwin = [this](uint64_t vertexFrom, uint64_t vertexTo, HalfEdge* twin)
		{
			HalfEdgeVertices halfEdgeVerticesKey{ vertexFrom, vertexTo };
			m_halfEdgeVerticesIteratorMap.erase(halfEdgeVerticesKey);

			if (twin != nullptr)
			{
				m_halfEdgeVerticesIteratorMap[halfEdgeVerticesKey] = m_halfEdgePointerIteratorMap[twin];
				twin->twin = nullptr;
			}
		};

		uint64_t const a = markedHalfEdgeIterators[2]->vertexIndex;
		uint64_t const b = markedHalfEdgeIterators[0]->vertexIndex;
		uint64_t const c = markedHalfEdgeIterators[1]->vertexIndex;
		removeTwin(a, b, twinMarkedHalfEdgeIterators[0]);
		removeTwin(b, c, twinMarkedHalfEdgeIterators[1]);
		removeTwin(c, a, twinMarkedHalfEdgeIterators[2]);

		for (auto markedHalfEdgeIterator : markedHalfEdgeIterators)
		{
			m_halfEdgeList.erase(markedHalfEdgeIterator);
		}

		m_faceIteratorMap.erase(&*faceIterator);
		m_facesList.erase(m_faceVerticesIteratorMap.find({ a, b, c })->second);
		m_faceVerticesIteratorMap.erase({ a, b, c });
    }

private:
    struct HalfEdgeVertices
    {
		uint64_t vertexIndexFrom;
		uint64_t vertexIndexTo;

		bool operator==(HalfEdgeVertices const& other) const
		{
			std::array<uint64_t, 2> indices = { vertexIndexFrom, vertexIndexTo };
			std::sort(indices.begin(), indices.end());
			std::array<uint64_t, 2> otherIndices = { other.vertexIndexFrom, other.vertexIndexTo };
			std::sort(otherIndices.begin(), otherIndices.end());

			return indices[0] == otherIndices[0] && indices[1] == otherIndices[1];
		}
    };

    struct HalfEdgeVerticesHash
    {
        size_t operator()(HalfEdgeVertices const& edge) const
        {
			return std::hash<uint64_t>{}(edge.vertexIndexFrom)
				^ std::hash<uint64_t>{}(edge.vertexIndexTo);
        }
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

    void IntializeHalfEdge (
            HalfEdges::iterator he,
            HalfEdge* next, HalfEdge* prev,
            Face* face,
            uint64_t vertexIndexFrom,
			uint64_t vertexIndexTo
        )
    {
		m_halfEdgePointerIteratorMap[&*he] = he;

		auto twinIterator = m_halfEdgeVerticesIteratorMap.find({ vertexIndexFrom, vertexIndexTo });
		HalfEdge* twin = nullptr;
        if (twinIterator == m_halfEdgeVerticesIteratorMap.end())
		{
			m_halfEdgeVerticesIteratorMap[{vertexIndexFrom, vertexIndexTo}] = he;
        }
    	else
		{
            twin = &*(twinIterator->second);
			twinIterator->second->twin = &*he;
        }

		*he = { next, prev, twin, face, vertexIndexTo };
    }
};

template < typename Iterator >
decltype(auto) CalculateExpectedValue(Iterator begin, Iterator end)
{
    auto E = *(begin++);
    uint32_t size = 1;

    for (; begin != end; ++begin) {
        E += *begin;
        ++size;
    }

    return E * (1.0 / size);
}

template < typename Iterator >
glm::dmat3 CalculateCovarianceMatrix(Iterator begin, Iterator end, glm::dvec3 const& mean)
{
    glm::dmat3 covariance(0.0);
    uint32_t size = 0;

    for (; begin != end; ++begin) {
        for (uint8_t i = 0; i < 3; ++i) {
            for (uint8_t j = 0; j < 3; ++j) {
                covariance[i][j] += ((*begin)[i] - mean[i]) * ((*begin)[j] - mean[j]);
            }
        }
        ++size;
    }

    return covariance * (1.0 / size);
}

template < typename Iterator >
Iterator FindExtremalVertex(Iterator begin, Iterator end, HyperPlane const& hyperPlane)
{
    Iterator extremalVertexIt = std::max_element(
        begin, end, [&hyperPlane](glm::dvec3 const& a, glm::dvec3 const& b) {
        return hyperPlane.SignedDistance(a) < hyperPlane.SignedDistance(b);
    });

    return extremalVertexIt;
}

template < typename Iterator >
uint64_t FindExtremalVertexIndex(Iterator begin, Iterator end, HyperPlane const& hyperPlane)
{
	uint64_t index = 0;
	double maxDistance = 0.0;
	double distance = 0.0;
	uint64_t maxIndex = std::numeric_limits<uint64_t>::max();

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

template < typename Iterator >
void FindExtremalVertices(
    Iterator begin, Iterator end, glm::dmat3 const& basis,
    std::array<Iterator, 3>& minimaVertices,
    std::array<Iterator, 3>& maximaVertices
)
{
    glm::dvec3 maximaProjections{ std::numeric_limits<double>::min() };
    glm::dvec3 minimaProjections{ std::numeric_limits<double>::max() };

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

double LineSegmentPointDistance(
    glm::dvec3 const& lineStart, glm::dvec3 const& lineEnd, glm::dvec3 const& point
);

class JacobiEigenvalue
{
public:
    explicit JacobiEigenvalue(
        glm::dmat3 const& symmetricMatrix, double coverageThreshold = 1.0e-4, uint32_t maxIterations = 100
    );

    glm::dmat3 const& GetEigenvectors() const;
    glm::dvec3 const& GetEigenvalues() const;

private:
    glm::dmat3 const& m_symmetricMatrix;
    double const m_coverageThreshold;
    uint32_t const m_maxIterations;
    glm::dmat3 m_eigenvectors;
    glm::dvec3 m_eigenvalues;

    static void FindMaxNormOffDiagonal(glm::dmat3 const& mat, uint8_t& i, uint8_t& j);
    double CalculateRotationAngle(glm::dmat3 const& mat, uint8_t i, uint8_t j) const;
    static glm::dmat3 MakeGivensRotationMatrix(double theta, uint8_t i, uint8_t j);
    void Calculate();
};

class QuickhullConvexHull
{
public:
	class Face;
	using Vertices = std::vector<glm::dvec3>;
	using Faces = std::list<Face>;

	class Face
	{
	public:
		std::vector<Vertices::iterator> m_vertices;
		Vertices::iterator m_extremalVertex;
		size_t m_extremalVertexIndex;
		HalfEdgeDataStructure::face_iterator m_hedsFaceIter;
		HyperPlane m_hyperPlane;
		std::array<size_t, 3> m_indices;

		Face(HalfEdgeDataStructure::face_iterator hedsFaceIter, HyperPlane const& hyperPlane, std::array<size_t, 3> const& indices)
			: m_extremalVertexIndex()
			, m_hedsFaceIter(hedsFaceIter)
			, m_hyperPlane(hyperPlane)
			, m_indices(indices)
		{
			std::sort(m_indices.begin(), m_indices.end());
		}

		Face(Vertices& vertices,
			std::list<Vertices::iterator>& markedVertexIterators,
			HalfEdgeDataStructure::face_iterator hedsFaceIter,
			HyperPlane const& hyperPlane,
			std::array<size_t, 3> const& indices)
			: Face(hedsFaceIter, hyperPlane, indices)
		{
			SetVertices(vertices, markedVertexIterators);
		}

		void SetVertices(Vertices& vertices, std::list<Vertices::iterator>& markedVertexIterators)
		{
			auto findAboveVertices = [this](Vertices::iterator& vertexIt) {
				return m_hyperPlane.SignedDistance(*vertexIt) > 0;
			};

			std::copy_if(markedVertexIterators.begin(), markedVertexIterators.end(),
				std::back_inserter(m_vertices), findAboveVertices
			);
			markedVertexIterators.erase(
				std::remove_if(markedVertexIterators.begin(), markedVertexIterators.end(), findAboveVertices),
				markedVertexIterators.end()
			);

			if (!m_vertices.empty())
			{
				m_extremalVertex = *std::max_element(m_vertices.begin(), m_vertices.end(),
					[this](Vertices::iterator& a, Vertices::iterator& b) {
						return m_hyperPlane.SignedDistance(*a) < m_hyperPlane.SignedDistance(*b);
				});

				m_extremalVertexIndex = std::distance(vertices.begin(), m_extremalVertex);
			}
		}

		std::vector<Vertices::iterator>& GetVertices()
		{
			return m_vertices;
		}

		Vertices::iterator GetExtremalVertex() const
		{
			return m_extremalVertex;
		}

		size_t GetExtremalVertexIndex() const
		{
			return m_extremalVertexIndex;
		}

		HalfEdgeDataStructure::face_iterator GetHedsFaceIterator() const
		{
			return m_hedsFaceIter;
		}

		HyperPlane const& GetHyperPlane() const
		{
			return m_hyperPlane;
		}

		std::array<size_t, 3> const& GetIndices() const
		{
			return m_indices;
		}
	};

	QuickhullConvexHull(Vertices& vertices)
		: m_vertices(vertices)
	{
	}

	Faces const& GetFaces() const
	{
		return m_faces;
	}

	std::list<Vertices::iterator> const& GetVertices()
	{
		return m_convexHullVertices;
	}

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
		std::array<Vertices::iterator, 3> maximaVertices, minimaVertices;
		FindExtremalVertices(m_vertices.begin(), m_vertices.end(), eigenvectorsNormalized, maximaVertices, minimaVertices);
		std::set<Vertices::iterator> extremalPoints(maximaVertices.begin(), maximaVertices.end());
		extremalPoints.insert(minimaVertices.begin(), minimaVertices.end());

		//Calculate base line points
		auto mostDistantPair = std::make_pair(*maximaVertices.begin(), *minimaVertices.begin());
		double maxDistanceSq = 0.0;
		for (auto a : extremalPoints) {
			for (auto b : extremalPoints) {
				double const distSq = glm::length2(*a - *b);
				if (distSq > maxDistanceSq) {
					maxDistanceSq = distSq;
					mostDistantPair = std::make_pair(a, b);
				}
			}
		}

		//Calculate triangle base point
		auto triangleBasePoint = *std::max_element(
			extremalPoints.begin(), extremalPoints.end(),
			[&mostDistantPair](Vertices::iterator v1, Vertices::iterator v2) {
				return LineSegmentPointDistance(*mostDistantPair.first, *mostDistantPair.second, *v1)
					< LineSegmentPointDistance(*mostDistantPair.first, *mostDistantPair.second, *v2);
		});

		//Calculate tetrahedron apex point
		HyperPlane baseFacePlane(*mostDistantPair.first, *mostDistantPair.second, *triangleBasePoint);
		auto tetrahedronApexPoint = std::max_element(
			m_vertices.begin(), m_vertices.end(),
			[&baseFacePlane](glm::dvec3& a, glm::dvec3& b) {
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

		std::list<Vertices::iterator> markedVertexIterators;
		for (auto vertexIt = m_vertices.begin(); vertexIt != m_vertices.end(); ++vertexIt) {
			markedVertexIterators.push_back(vertexIt);
		}
		MakeFace(indices[0], indices[1], indices[2], markedVertexIterators);
		MakeFace(indices[0], indices[1], indices[3], markedVertexIterators);
		MakeFace(indices[0], indices[2], indices[3], markedVertexIterators);
		MakeFace(indices[1], indices[2], indices[3], markedVertexIterators);
	}

	void Refine()
	{
		std::list<Faces::iterator> faceStack;
		for (auto faceIt = m_faces.begin(); faceIt != m_faces.end(); ++faceIt)
		{
			faceStack.push_back(faceIt);
		}

		while(!faceStack.empty())
		{
			Faces::iterator faceIt = faceStack.front();
			faceStack.pop_front();

			//If face does not has vertices above it, remove it
			if (!faceIt->GetVertices().empty())
			{
				std::list<Vertices::iterator> markedVertexIterators;
				std::list<std::pair<size_t, size_t>> horizonRidges;
				std::list<Faces::iterator> visibleFaces(1, faceIt);
				std::unordered_set<Face*> visibleFacesSet{&*faceIt};
				std::unordered_set<Face*> visitedFaces;

				auto extremalVertex = faceIt->GetExtremalVertex();
				auto extremalVertexIndex = faceIt->GetExtremalVertexIndex();

				{
					std::vector<Vertices::iterator>& faceVertices = faceIt->GetVertices();
					markedVertexIterators.insert(markedVertexIterators.end(), faceVertices.begin(), faceVertices.end());
					std::vector<Vertices::iterator>().swap(faceVertices);
				}

				//For each visible face
				for (auto visibleFaceIt = visibleFaces.begin(); visibleFaceIt != visibleFaces.end(); ++visibleFaceIt)
				{
					auto adjHedsFaceIt = (*visibleFaceIt)->GetHedsFaceIterator()->GetAdjacentFaceIterator();
					auto adjHedsFaceBegin = adjHedsFaceIt;
					visitedFaces.insert(&(**visibleFaceIt));

					//For all adjacent faces
					do {
						Faces::iterator adjFace = m_hedsFaceIteratorMap[&*adjHedsFaceIt];

						//If face is unvisited
						if (visitedFaces.find(&*adjFace) == visitedFaces.end()
							&& visibleFacesSet.find(&*adjFace) == visibleFacesSet.end())
						{
							//If face is visible add it to the visible faces set
							if (adjFace->GetHyperPlane().SignedDistance(*extremalVertex) > 0)
							{
								visibleFaces.push_back(adjFace);
								visibleFacesSet.insert(&*adjFace);

								std::vector<Vertices::iterator>& faceVertices = adjFace->GetVertices();
								markedVertexIterators.insert(markedVertexIterators.end(), faceVertices.begin(), faceVertices.end());
								std::vector<Vertices::iterator>().swap(faceVertices);
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
								assert(ridgeIndices.size() == 2);

								horizonRidges.emplace_back(ridgeIndices[0], ridgeIndices[1]);
							}
						}
					} while (++adjHedsFaceIt != adjHedsFaceBegin);
				}

				m_convexHullVertices.push_back(extremalVertex);

				//Remove visible faces
				for (Faces::iterator visibleFaceIt : visibleFaces)
				{
					faceStack.erase(std::remove_if(faceStack.begin(), faceStack.end(),
						[&visibleFaceIt](Faces::iterator& it){ return &*it == &*visibleFaceIt; }), faceStack.end());
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

private:
	Faces		m_faces;
	Vertices&	m_vertices;
	std::list<Vertices::iterator> m_convexHullVertices;
	glm::dvec3	m_mean;
	HalfEdgeDataStructure m_heds;
	std::unordered_map<
			HalfEdgeDataStructure::Face*, Faces::iterator
		> m_hedsFaceIteratorMap;

	Faces::iterator MakeFace(size_t vertexIndex1, size_t vertexIndex2, size_t vertexIndex3,
		std::list<Vertices::iterator>& markedVertices)
	{
		m_heds.MakeFace(vertexIndex1, vertexIndex2, vertexIndex3);
		m_faces.push_front(Face{
			m_vertices,
			markedVertices,
			m_heds.GetFace(vertexIndex1, vertexIndex2, vertexIndex3),
			HyperPlane{ m_vertices[vertexIndex1], m_vertices[vertexIndex2], m_vertices[vertexIndex3], &m_mean },
			std::array<size_t, 3>{ vertexIndex1, vertexIndex2, vertexIndex3 }
		});
		m_hedsFaceIteratorMap[&*m_faces.front().GetHedsFaceIterator()] = m_faces.begin();

		return m_faces.begin();
	}

	void RemoveFace(Faces::iterator faceIterator)
	{
		m_hedsFaceIteratorMap.erase(&*faceIterator->GetHedsFaceIterator());
		m_heds.RemoveFace(faceIterator->GetHedsFaceIterator());
		m_faces.erase(faceIterator);
	}
};

} // namespace math
} // namespace pegasus

#endif // PEGASUS_MATH_HPP
