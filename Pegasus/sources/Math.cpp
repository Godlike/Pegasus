/*
* Copyright (C) 2017 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#include <Pegasus/include/Math.hpp>

#include <glm/ext.hpp>
#include <array>

using namespace pegasus;
using namespace math;

HyperPlane::HyperPlane(glm::dvec3 const& normal, glm::dvec3 const& point, glm::dvec3 const* below)
    : m_normal(normal)
    , m_point(point)
    , m_distance(glm::dot(m_normal, m_point))
{
    if (below != nullptr)
    {
        glm::dvec3 const outward = point - *below;
        if (glm::dot(outward, m_normal) < 0.0)
        {
            SetNormal(m_normal * -1.0);
        }
    }
}

HyperPlane::HyperPlane(
    glm::dvec3 const& a, glm::dvec3 const& b, glm::dvec3 const& c, glm::dvec3 const* below
)
    : HyperPlane(glm::normalize(glm::cross(a - c, b - c)), c, below)
{
}

HyperPlane::HyperPlane(glm::dmat3 const& vertices, glm::dvec3 const* below)
    : HyperPlane(vertices[0], vertices[1], vertices[2], below)
{
}

glm::dvec3 const& HyperPlane::GetPoint() const
{
    return m_point;
}

glm::dvec3 const& HyperPlane::GetNormal() const
{
    return m_normal;
}

double HyperPlane::GetDistance() const
{
    return m_distance;
}

void HyperPlane::SetNormal(glm::dvec3 const& normal)
{
    m_normal = normal;
    m_distance = glm::dot(m_normal, m_point);
}

void HyperPlane::SetPoint(glm::dvec3 const& point)
{
    m_point = point;
    m_distance = glm::dot(m_normal, m_point);
}

double HyperPlane::Distance(glm::dvec3 const& point) const
{
    return glm::abs(SignedDistance(point));
}

double HyperPlane::SignedDistance(glm::dvec3 const& point) const
{
    return glm::dot(m_normal, point) - m_distance;
}

bool HyperPlane::RayIntersection(
    glm::dvec3 const& rayNormal, glm::dvec3 const& rayPoint, glm::dvec3& resultPoint
) const
{
    double const rayPlaneProjection = glm::dot(m_normal, rayNormal);

    if (rayPlaneProjection != 0.0)
    {
        double const t = (glm::dot(m_normal, m_point - rayPoint)) / rayPlaneProjection;
        resultPoint = rayPoint + rayNormal * t;
        return true;
    }

    return false;
}

bool HyperPlane::LineSegmentIntersection(
    glm::dvec3 const& lineStart, glm::dvec3 const& lineEnd, glm::dvec3& resultPoint
) const
{
    if ((glm::dot(lineStart, m_normal) - m_distance)
        * (glm::dot(lineEnd, m_normal) - m_distance) >= 0)
    {
        return false;
    }

    glm::dvec3 const lineNormal = glm::normalize(lineEnd - lineStart);

    return RayIntersection(lineNormal, lineStart, resultPoint);
}

glm::dvec3 HyperPlane::ClosestPoint(const glm::dvec3& point) const
{
    glm::dvec3 const closestPoint = point - (glm::dot(point, m_normal) - m_distance) * m_normal;

    return closestPoint;
}

JacobiEigenvalue::JacobiEigenvalue(glm::dmat3 const& symmetricMatrix, double coverageThreshold, uint32_t maxIterations)
    : m_symmetricMatrix{symmetricMatrix}
    , m_coverageThreshold{glm::abs(coverageThreshold)}
    , m_maxIterations{maxIterations}
{
    Calculate();
}

glm::dmat3 const& JacobiEigenvalue::GetEigenvectors() const
{
    return m_eigenvectors;
}

glm::dvec3 const& JacobiEigenvalue::GetEigenvalues() const
{
    return m_eigenvalues;
}

void JacobiEigenvalue::FindMaxNormOffDiagonal(glm::dmat3 const& mat, uint8_t& i, uint8_t& j)
{
    i = 0;
    j = 1;

    for (uint8_t p = 0; p < 3; ++p)
    {
        for (uint8_t q = 0; q < 3; ++q)
        {
            if (p != q)
            {
                if (glm::abs(mat[i][j]) < glm::abs(mat[p][q]))
                {
                    i = p;
                    j = q;
                }
            }
        }
    }
}

double JacobiEigenvalue::CalculateRotationAngle(glm::dmat3 const& mat, uint8_t i, uint8_t j) const
{
    if (glm::abs(mat[i][i] - mat[j][j]) < m_coverageThreshold)
    {
        return (glm::pi<double>() / 4.0) * (mat[i][j] > 0 ? 1.0 : -1.0);
    }

    return 0.5 * glm::atan(2.0 * mat[i][j] / (mat[i][i] - mat[j][j]));
}

glm::dmat3 JacobiEigenvalue::MakeGivensRotationMatrix(double theta, uint8_t i, uint8_t j)
{
    glm::dmat3 g;
    g[i][i] = glm::cos(theta);
    g[i][j] = glm::sin(theta);
    g[j][i] = -glm::sin(theta);
    g[j][j] = glm::cos(theta);

    return g;
}

void JacobiEigenvalue::Calculate()
{
    glm::dmat3 D(m_symmetricMatrix);
    glm::dmat3 S;

    uint16_t iterations = 0;
    bool iterate = true;
    while (iterate)
    {
        uint8_t i, j;
        FindMaxNormOffDiagonal(D, i, j);

        glm::dmat3 S1 = MakeGivensRotationMatrix(CalculateRotationAngle(D, i, j), i, j);
        S = S * S1;
        D = (glm::transpose(S1) * D) * S1;

        FindMaxNormOffDiagonal(D, i, j);
        iterate = (++iterations < m_maxIterations) && (glm::abs(D[i][j]) > m_coverageThreshold);
    }

    m_eigenvalues = {D[0][0], D[1][1], D[2][2]};
    m_eigenvectors = S;
}

glm::dvec3 pegasus::math::CalculateCentroid(std::initializer_list<glm::dvec3> vectors)
{
    glm::dvec3 result(0, 0, 0);
    for (glm::dvec3 vector : vectors)
    {
        result += vector;
    }
    result /= vectors.size();
    return result;
}

glm::dvec3 pegasus::math::GetScalarProjection(glm::dvec3 const& toProject, glm::dvec3 const& projectOnto)
{
    return glm::dot(toProject, projectOnto) / glm::pow(glm::length(projectOnto), 2) * projectOnto;
}

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
        auto backFaceIterator = m_facesList.emplace(m_facesList.end(), *newHalfEdges.back());
        m_faceIteratorMap[&*backFaceIterator] = backFaceIterator;
        m_faceVerticesIteratorMap[faceVerticesKey] = backFaceIterator;

        //Init half edges
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

    auto twinIterator = m_halfEdgeVerticesIteratorMap.find({vertexIndexFrom, vertexIndexTo});
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

double pegasus::math::LineSegmentPointDistance(
    glm::dvec3 const& lineStart, glm::dvec3 const& lineEnd, glm::dvec3 point
)
{
    point = point - lineStart;
    glm::dvec3 const lineDirection = glm::normalize(lineEnd - lineStart);
    glm::dvec3 const pointLineProjection = glm::dot(lineDirection, point) * lineDirection;
    return glm::distance(point, pointLineProjection);
}
