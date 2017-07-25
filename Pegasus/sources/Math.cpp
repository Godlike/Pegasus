/*
* Copyright (c) Icosagon 2003. All Rights Reserved.
*
* This software is distributed under licence. Use of this software
* implies agreement with all terms and conditions of the accompanying
* software licence.
*/
#include "Pegasus/include/Math.hpp"

double pegasus::math::LineSegmentPointDistance(
        glm::dvec3 const& lineStart, glm::dvec3 const& lineEnd, glm::dvec3 const& point
    )
{
    return   glm::length(glm::cross(lineEnd - lineStart, lineStart - point))
           / glm::length(lineEnd - lineStart);
}

pegasus::math::JacobiEigenvalue::JacobiEigenvalue(glm::dmat3 const& symmetricMatrix, double coverageThreshold, uint32_t maxIterations)
    : m_symmetricMatrix{symmetricMatrix}
    , m_coverageThreshold{glm::abs(coverageThreshold)}
    , m_maxIterations{maxIterations}
{
    Calculate();
}

glm::dmat3 const& pegasus::math::JacobiEigenvalue::GetEigenvectors() const
{
    return m_eigenvectors;
}

glm::dvec3 const& pegasus::math::JacobiEigenvalue::GetEigenvalues() const
{
    return m_eigenvalues;
}

void pegasus::math::JacobiEigenvalue::FindMaxNormOffDiagonal(glm::dmat3 const& mat, uint8_t& i, uint8_t& j)
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

double pegasus::math::JacobiEigenvalue::CalculateRotationAngle(glm::dmat3 const& mat, uint8_t i, uint8_t j) const
{
    if (glm::abs(mat[i][i] - mat[j][j]) < m_coverageThreshold)
    {
        return (glm::pi<double>() / 4.0) * (mat[i][j] > 0 ? 1.0 : -1.0);
    }

    return 0.5 * glm::atan(2.0 * mat[i][j] / (mat[i][i] - mat[j][j]));
}

glm::dmat3 pegasus::math::JacobiEigenvalue::MakeGivensRotationMatrix(double theta, uint8_t i, uint8_t j)
{
    glm::dmat3 g;
    g[i][i] = glm::cos(theta);
    g[i][j] = glm::sin(theta);
    g[j][i] = -glm::sin(theta);
    g[j][j] = glm::cos(theta);

    return g;
}

void pegasus::math::JacobiEigenvalue::Calculate()
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

    m_eigenvalues = { D[0][0], D[1][1], D[2][2] };
    m_eigenvectors = S;
}

pegasus::math::HyperPlane::HyperPlane(glm::dvec3 const & normal, glm::dvec3 const & point, glm::dvec3 const* below)
    : m_normal(normal), m_point(point), m_distance(glm::dot(m_normal, m_point)), m_below(below)
{
    if (m_below != nullptr)
    {
        glm::dvec3 const outward = point - *m_below;
        if (glm::dot(outward, m_normal) < 0.0) {
			SetNormal(m_normal * -1.0);
        }
    }
}

pegasus::math::HyperPlane::HyperPlane(glm::dvec3 const & a, glm::dvec3 const & b, glm::dvec3 const & c, glm::dvec3 const* below)
    : HyperPlane(glm::normalize(glm::cross(a - c, b - c)), c, below)
{
}

pegasus::math::HyperPlane::HyperPlane(glm::dmat3 const & vertices, glm::dvec3 const* below)
    : HyperPlane(vertices[0], vertices[1], vertices[2], below)
{
}

glm::dvec3 const& pegasus::math::HyperPlane::GetPoint() const
{
    return m_point;
}

glm::dvec3 const& pegasus::math::HyperPlane::GetNormal() const
{
    return m_normal;
}

double pegasus::math::HyperPlane::GetDistance() const
{
    return m_distance;
}

void pegasus::math::HyperPlane::SetNormal(glm::dvec3 const & normal) {
    m_normal = normal;
    m_distance = glm::dot(m_normal, m_point);
}

void pegasus::math::HyperPlane::SetPoint(glm::dvec3 const & point) {
    m_point = point;
    m_distance = glm::dot(m_normal, m_point);
}

double pegasus::math::HyperPlane::Distance(glm::dvec3 const & point) const {
    return glm::abs(SignedDistance(point));
}

double pegasus::math::HyperPlane::SignedDistance(glm::dvec3 const & point) const {
    return glm::dot(m_normal, point) - m_distance;
}

bool pegasus::math::HyperPlane::Intersection(glm::dvec3 const & lineStart, glm::dvec3 const & lineEnd,
                                             glm::dvec3 & resultPoint) const
{
    if (  (glm::dot(lineStart, m_normal) - m_distance)
        * (glm::dot(lineEnd,   m_normal) - m_distance) >= 0)
    {
        return false;
    }

    glm::dvec3 const line = lineEnd - lineStart;
    glm::dvec3 const lineNormal = glm::normalize(line);
    double const linePlaneProjection = glm::dot(m_normal, lineNormal);

    if (linePlaneProjection != 0.0)
    {
        double const t = (glm::dot(m_normal, m_point - lineStart)) / linePlaneProjection;
        resultPoint = lineStart + lineNormal * t;
        return true;
    }

    return false;
}
