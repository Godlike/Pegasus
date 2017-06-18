/*
* Copyright (c) Icosagon 2003. All Rights Reserved.
*
* This software is distributed under licence. Use of this software
* implies agreement with all terms and conditions of the accompanying
* software licence.
*/
#ifndef PEGASUS_MATH_HPP
#define PEGASUS_MATH_HPP

#include <glm/glm.hpp>

namespace pegasus
{
namespace math
{

class JacobiEigenvalue
{
public:
    explicit JacobiEigenvalue(glm::dmat3 const& symmetricMatrix, double coverageThreshold = 1.0e-4, uint32_t maxIterations = 100);

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

} // namespace math
} // namespace pegasus 

#endif // PEGASUS_MATH_HPP
