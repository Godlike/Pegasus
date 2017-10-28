/*
* Copyright (C) 2017 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#ifndef PEGASUS_MATH_JACOBI_EIGENVALUE_HPP
#define PEGASUS_MATH_JACOBI_EIGENVALUE_HPP

#include <pegasus/SharedMacros.hpp>
#include <glm/glm.hpp>

namespace pegasus
{
namespace math
{
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
     * @param[out] j max element column index
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
} // namespace math
} // namespace pegasus
#endif // PEGASUS_MATH_JACOBI_EIGENVALUE_HPP
