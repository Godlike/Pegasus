/*
* Copyright (C) 2017 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#ifndef PEGASUS_BOUNDING_VOLUMES_HPP
#define PEGASUS_BOUNDING_VOLUMES_HPP

#include <pegasus/SharedMacros.hpp>
#include <geometry/Shape.hpp>
#include <geometry/Geometry.hpp>

#include <vector>
#include <set>

namespace pegasus
{
namespace geometry
{
namespace volume
{
/**
 * @brief Represents an instance of mesh data
 */
struct Mesh
{
    /**
     * @brief Constructs mesh data object
     * @param vertices vertex buffer data
     * @param indices index buffer data
     */
    PEGASUS_EXPORT Mesh(std::vector<glm::dvec3> const& vertices, std::vector<glm::u64vec3> const& indices);

    std::vector<glm::dvec3> const& vertices;
    std::vector<glm::u64vec3> const& indices;
};

/**
 * @brief Calculates average value of a vertex for the given set of indices
 * @param mesh vertex and index data
 * @param indices set of indices for calculation
 * @return average vertex value
 */
PEGASUS_EXPORT glm::dvec3 CalculateMeanVertex(
    volume::Mesh const& mesh, std::set<std::size_t> const& indices
);

/**
 * @brief Calculates covariance matrix for the given set of vertices
 * @param mesh vertex and index data
 * @param indices set of indices for calculation
 * @param mean average vertex value for the given set
 * @return covariance matrix
 */
PEGASUS_EXPORT glm::dmat3 CalculateCovarianceMatrix(
    volume::Mesh const& mesh, std::set<std::size_t> const& indices, glm::dvec3 const& mean
);

/**
 * @brief Calculates extremal vertices for the given set in the given directions
 * @param basis direction vectors
 * @param mesh vertex and index data
 * @param indices set of indices for calculation
 * @return extremal vertices
 */
PEGASUS_EXPORT glm::dmat3 CalculateExtremalVertices(
    glm::dmat3 const& basis, volume::Mesh const& mesh, std::set<std::size_t> const& indices
);

namespace obb
{
/**
 * @brief Oriented bounding box calculation algorithm
 */
class OrientedBoundingBox
{
public:
    /**
     * @brief Stores calculation data
     */
    struct Box
    {
        glm::dvec3 mean;
        glm::dmat3 covariance;
        glm::dmat3 eigenVectors;
        glm::dmat3 eigenVectorsNormalized;
        glm::dmat3 extremalVertices;
        glm::dmat3 boxAxes;
    };

    /**
     * @brief Constructs OBB for the given set of vertices in the given mesh
     * @param mesh vertex and index data
     * @param indices set of indices for calculation
     */
    PEGASUS_EXPORT OrientedBoundingBox(Mesh const& mesh, std::set<std::size_t> const& indices);

    /**
     * @brief Returns pre-calculated OBB shape
     * @return obb shape
     */
    PEGASUS_EXPORT geometry::Box GetVolume() const;

private:
    geometry::Box m_boxShape;
    Box m_box;
    Mesh const& m_shape;
    std::set<std::size_t> const& m_indices;
};
} // namespace obb

namespace aabb
{
/**
 * @brief Axis aligned bounding box calculation algorithm
 */
class AxisAlignedBoundingBox
{
public:
    /**
     * @brief Stores representations of AABB
     */
    struct Box
    {
        glm::dvec3 xMin;
        glm::dvec3 xMax;
        glm::dvec3 yMin;
        glm::dvec3 yMax;
        glm::dvec3 zMin;
        glm::dvec3 zMax;
        glm::dvec3 extremalMean;
        glm::dvec3 xAxis;
        glm::dvec3 yAxis;
        glm::dvec3 zAxis;
    };

    /**
     * @brief Constructs AABB for the given set of vertices in the given mesh
     * @param mesh vertex and index data
     * @param indices set of indices for calculation
     */
    PEGASUS_EXPORT AxisAlignedBoundingBox(Mesh const& mesh, std::set<std::size_t> const& indices);

    /**
     * @brief Returns pre-calculated AABB shape
     * @return aabb shape
     */
    PEGASUS_EXPORT geometry::Box GetVolume() const;

private:
    geometry::Box m_boxShape;
    Box m_box;
    Mesh const& m_shape;
    std::set<std::size_t> const& m_indices;

    /**
     * @brief Calculates extremal vertices from the given set along the x,y and z axes
     * @param[in] mesh vertex and index data
     * @param[in] indices set of indices for calculation
     * @param[out] box aabb data
     */
    static void CalculateExtremalVetices(Mesh const& mesh, std::set<std::size_t> const& indices, Box& box);

    /**
     * @brief Calculates average box vertex
     * @param[out] box aabb data
     */
    static void CalculateMean(Box& box);

    /**
     * @brief Creates shape representation of AABB
     * @param[in,out] box aabb data
     */
    void CreateBox(Box& box);
};
} // namespace aabb

namespace sphere
{
/**
 * @brief Bounding sphere calculation algorithm
 */
class BoundingSphere
{
public:
    /**
     * @brief Stores representation of Bounding sphere
     */
    struct Sphere
    {
        glm::dvec3 mean;
        glm::dmat3 covariance;
        glm::dvec3 eigenValues;
        glm::dmat3 eigenVectors;
        glm::dmat3 eigenVectorsNormalized;
    };

    /**
     * @brief Calculates bounding sphere for the given set of vertices in the given mesh
     *
     * The implementation of the algorithm accounts for the dispersion and density of the
     * vertex data by calculating covariance matrix and refining initial sphere in the direction
     * of the maximum spread.
     * @param mesh vertex and index data
     * @param indices set of indices for calculation
     */
    PEGASUS_EXPORT BoundingSphere(Mesh const& mesh, std::set<std::size_t> const& indices);

    /**
    * @brief Returns pre-calculated Bounding sphere shape
    * @return bounding sphere shape
    */
    PEGASUS_EXPORT geometry::Sphere GetVolume() const;

private:
    geometry::Sphere m_sphereShape;
    Sphere m_sphere;
    Mesh const& m_shape;
    std::set<std::size_t> const& m_indices;

    /**
     * @brief Calculates initial bounding sphere
     *
     * @note: This method requires calculation of the covariance matrix
     * for the given set of vertices in order to construct initial guess more efficiently.
     * @param[in] eigenVectors eigenvetors of the covariance matrix
     * @param[in] eigenValues eigenvalues of the covariance matrix
     * @param[in] mesh vertex and index data
     * @param[in] indices set of indices for calculation
     * @return bounding sphere shape
     */
    static geometry::Sphere CalculateInitialBoundingSphere(
        glm::dmat3 const& eigenVectors, glm::dvec3 const& eigenValues,
        Mesh const& mesh, std::set<std::size_t> const& indices
    );

    /**
     * @brief Iteratively refines sphere
     *
     * Refining happens by accounting for the points that are outside of the current bounding sphere
     * @param[in] sphere initial sphere shape
     * @param[in] mesh vertex and index data
     * @param[in] indices set of indices for calculation
     * @return bounding sphere shape
     */
    static geometry::Sphere RefineSphere(
        geometry::Sphere const& sphere, Mesh const& mesh, std::set<std::size_t> const& indices
    );
};
} // namespace sphere
} // namespace volumes
} // namespace geometry
} // namespace pegasus
#endif // PEGASUS_BOUNDING_VOLUMES_HPP
