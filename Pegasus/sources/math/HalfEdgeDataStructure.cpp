/*
* Copyright (C) 2017 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#include <math/HalfEdgeDataStructure.hpp>
#include <array>
#include <algorithm>

using namespace pegasus;
using namespace math;

HalfEdgeDataStructure::Face::Face(HalfEdge& halfEdge)
    : m_halfEdge(&halfEdge)
{
}

HalfEdgeDataStructure::Face::edge_iterator HalfEdgeDataStructure::Face::GetHalfEdgeIterator()
{
    return edge_iterator{*m_halfEdge};
}

HalfEdgeDataStructure::Face::const_edge_iterator HalfEdgeDataStructure::Face::GetHalfEdgeIterator() const
{
    return const_edge_iterator{*m_halfEdge};
}

HalfEdgeDataStructure::Face::face_iterator HalfEdgeDataStructure::Face::GetAdjacentFaceIterator()
{
    return face_iterator{*m_halfEdge};
}

HalfEdgeDataStructure::Face::const_face_iterator HalfEdgeDataStructure::Face::GetAdjacentFaceIterator() const
{
    return const_face_iterator{*m_halfEdge};
}

bool HalfEdgeDataStructure::FaceVertices::operator==(FaceVertices const& other) const
{
    std::array<uint64_t, 3> pointers = {{a, b, c}};
    std::sort(pointers.begin(), pointers.end());
    std::array<uint64_t, 3> otherPointers = {{other.a, other.b, other.c}};
    std::sort(otherPointers.begin(), otherPointers.end());

    return pointers[0] == otherPointers[0]
        && pointers[1] == otherPointers[1]
        && pointers[2] == otherPointers[2];
}

size_t HalfEdgeDataStructure::FaceVertices::Hasher::operator()(FaceVertices const& face) const
{
    return std::hash<uint64_t>{}(face.a)
        ^ std::hash<uint64_t>{}(face.b)
        ^ std::hash<uint64_t>{}(face.c);
}

void HalfEdgeDataStructure::MakeFace(uint64_t a, uint64_t b, uint64_t c)
{
    FaceVertices const faceVerticesKey{a, b, c};
    if (m_faceVerticesIteratorMap.find(faceVerticesKey) == m_faceVerticesIteratorMap.end())
    {
        //Allocate half edges
        std::array<HalfEdges::iterator, 3> newHalfEdges = {{
            m_halfEdgeList.emplace(m_halfEdgeList.end()),
            m_halfEdgeList.emplace(m_halfEdgeList.end()),
            m_halfEdgeList.emplace(m_halfEdgeList.end())
        }};

        //Allocate face
        auto const backFaceIterator = m_facesList.emplace(m_facesList.end(), *newHalfEdges.back());
        m_faceIteratorMap[&*backFaceIterator] = backFaceIterator;
        m_faceVerticesIteratorMap[faceVerticesKey] = backFaceIterator;

        //Initialize half edges
        IntializeHalfEdge(newHalfEdges[0], &*newHalfEdges[1], &*newHalfEdges[2], &*backFaceIterator, a, b);
        IntializeHalfEdge(newHalfEdges[1], &*newHalfEdges[2], &*newHalfEdges[0], &*backFaceIterator, b, c);
        IntializeHalfEdge(newHalfEdges[2], &*newHalfEdges[0], &*newHalfEdges[1], &*backFaceIterator, c, a);
    }
}

HalfEdgeDataStructure::face_iterator HalfEdgeDataStructure::GetFace(uint64_t a, uint64_t b, uint64_t c)
{
    FaceVertices faceVerticesKey{a, b, c};
    auto faceIterator = m_faceVerticesIteratorMap.find(faceVerticesKey);
    if (faceIterator == m_faceVerticesIteratorMap.end())
    {
        return m_facesList.end();
    }

    return faceIterator->second;
}

HalfEdgeDataStructure::const_face_iterator HalfEdgeDataStructure::GetFace(uint64_t a, uint64_t b, uint64_t c) const
{
    FaceVertices const faceVerticesKey{a, b, c};
    auto faceIterator = m_faceVerticesIteratorMap.find(faceVerticesKey);

    if (faceIterator == m_faceVerticesIteratorMap.end())
    {
        return m_facesList.end();
    }

    return faceIterator->second;
}

HalfEdgeDataStructure::const_face_iterator HalfEdgeDataStructure::GetFaceEnd() const
{
    return m_facesList.end();
}

void HalfEdgeDataStructure::RemoveFace(uint64_t a, uint64_t b, uint64_t c)
{
    RemoveFace(GetFace(a, b, c));
}

void HalfEdgeDataStructure::RemoveFace(face_iterator faceIterator)
{
    auto heIterator = faceIterator->GetHalfEdgeIterator();
    std::array<HalfEdges::iterator, 3> markedHalfEdgeIterators;
    markedHalfEdgeIterators[0] = m_halfEdgePointerIteratorMap[&*heIterator++];
    markedHalfEdgeIterators[1] = m_halfEdgePointerIteratorMap[&*heIterator++];
    markedHalfEdgeIterators[2] = m_halfEdgePointerIteratorMap[&*heIterator];
    std::array<HalfEdge*, 3> twinMarkedHalfEdgeIterators = {{
        markedHalfEdgeIterators[0]->twin,
        markedHalfEdgeIterators[1]->twin,
        markedHalfEdgeIterators[2]->twin,
    }};

    for (auto markedHalfEdgeIterator : markedHalfEdgeIterators)
    {
        m_halfEdgePointerIteratorMap.erase(&*markedHalfEdgeIterator);
    }

    auto removeTwin = [this](uint64_t vertexFrom, uint64_t vertexTo, HalfEdge* twin)
    {
        HalfEdgeVertices halfEdgeVerticesKey{vertexFrom, vertexTo};
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
    m_facesList.erase(m_faceVerticesIteratorMap.find({a, b, c})->second);
    m_faceVerticesIteratorMap.erase({a, b, c});
}

bool HalfEdgeDataStructure::HalfEdgeVertices::operator==(HalfEdgeVertices const& other) const
{
    std::array<uint64_t, 2> indices = {{vertexIndexFrom, vertexIndexTo}};
    std::sort(indices.begin(), indices.end());
    std::array<uint64_t, 2> otherIndices = {{other.vertexIndexFrom, other.vertexIndexTo}};
    std::sort(otherIndices.begin(), otherIndices.end());

    return indices[0] == otherIndices[0] && indices[1] == otherIndices[1];
}

size_t HalfEdgeDataStructure::HalfEdgeVertices::Hasher::operator()(HalfEdgeVertices const& edge) const
{
    return std::hash<uint64_t>{}(edge.vertexIndexFrom)
        ^ std::hash<uint64_t>{}(edge.vertexIndexTo);
}

void HalfEdgeDataStructure::IntializeHalfEdge(
    HalfEdges::iterator he, HalfEdge* next, HalfEdge* prev, Face* face,
    uint64_t vertexIndexFrom, uint64_t vertexIndexTo
)
{
    m_halfEdgePointerIteratorMap[&*he] = he;

    auto const twinIterator = m_halfEdgeVerticesIteratorMap.find({vertexIndexFrom, vertexIndexTo});
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

    *he = {next, prev, twin, face, vertexIndexTo};
}
