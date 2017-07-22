/*
* Copyright (c) Icosagon 2003. All Rights Reserved.
*
* This software is distributed under licence. Use of this software
* implies agreement with all terms and conditions of the accompanying
* software licence.
*/
#include "Pegasus/include/Math.hpp"

#include <glm/ext.hpp>
#include <array>

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
