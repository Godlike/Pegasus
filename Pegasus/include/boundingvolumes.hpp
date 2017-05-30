/*
* Copyright (C) 2017 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#ifndef PEGASUS_OBB_HPP
#define PEGASUS_OBB_HPP

#include <functional>
#include <vector>
#include <set>
#include <array>
#include <memory>
#include <Eigen/Eigen>
#include <Eigen/StdVector>

#include "Pegasus/include/geometry.hpp"

namespace pegasus {
namespace geometry {
namespace volumes {

using Vectors = std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>;
using Indices = std::set<std::size_t>;
using Face    = std::array<std::size_t, 3>;
using Faces   = std::vector<Face>;
using Plane   = Eigen::Hyperplane<float, 3>;

struct Shape
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Vectors const vertices;
    Faces   const indices;
};

namespace obb {
class OrientedBoundingBox
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    struct Box
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        Eigen::Vector3f mean;
        Eigen::Matrix3f covariance;
        Eigen::Matrix3f eigen_vectors;
        Eigen::Matrix3f eigen_vectors_normalized;
        Eigen::Matrix3f extremal_vertices;
        Vectors cube_vertices;
    };

    OrientedBoundingBox(Shape const & shape, Indices const & indices);
    geometry::Box getBox() const;

private:
    geometry::Box m_box_shape;
    Box m_box;
    Shape const & m_shape;
    Indices const & m_indices;

    static Eigen::Vector3f calculateMeanVertex(
        Shape const & shape, Indices const & indices
    );
    static Eigen::Matrix3f calculateCovarianceMatrix(
        Shape const & shape, Indices const & indices, Eigen::Vector3f const & mean
    );
    static Eigen::Matrix3f calculateEigenVectors(
        Eigen::Matrix3f const & covariance
    );
    static Eigen::Matrix3f calculateExtremalVertices(
        Eigen::Matrix3f const & eigen_vectors, Shape const & shape, Indices const & indices
    );
    static Vectors calculateBoxVertices(
        Eigen::Matrix3f const & extremal_points, Eigen::Matrix3f const & eigen_vectors
    );
};
} // namespace obb

namespace aabb {
    class AxisAlignedBoundingBox
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        struct Box
        {
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW

            Eigen::Vector3f xMin;
            Eigen::Vector3f xMax;
            Eigen::Vector3f yMin;
            Eigen::Vector3f yMax;
            Eigen::Vector3f zMin;
            Eigen::Vector3f zMax;
            Eigen::Vector3f extremalMean;
            Eigen::Vector3f xAxis;
            Eigen::Vector3f yAxis;
            Eigen::Vector3f zAxis;
        };

        AxisAlignedBoundingBox(Shape const & shape, Indices const & indices)
            : m_box_shape({}, {}, {}, {})
            , m_shape(shape)
            , m_indices(indices)
        {
            calculateExtremalVetices(m_shape, indices, m_box);
            calculateMean(m_box);
            createBox(m_box);

        }

        geometry::Box getBox() const
        {
            return m_box_shape;
        }

    private:
        geometry::Box m_box_shape;
        Box m_box;
        Shape const & m_shape;
        Indices const & m_indices;

        static void calculateExtremalVetices(Shape const & shape, Indices const & indices, Box & box)
        {
            static auto compareFaces = [&](auto axis_index, auto face_index_a, auto face_index_b) -> bool {
                static auto compareFaceIndices = [&](auto index_a, auto index_b) -> bool {
                    return shape.vertices[index_a][axis_index] < shape.vertices[index_b][axis_index];
                };

                std::size_t const max_index_a = *std::max_element(
                        shape.indices[face_index_a].begin(), shape.indices[face_index_a].end(), compareFaceIndices
                );

                std::size_t const max_index_b = *std::max_element(
                        shape.indices[face_index_b].begin(), shape.indices[face_index_b].end(), compareFaceIndices
                );

                return   shape.vertices[shape.indices[face_index_a][max_index_a]][axis_index]
                       < shape.vertices[shape.indices[face_index_b][max_index_b]][axis_index];
            };

            static auto xCompareFaces = std::bind(compareFaces, 0, std::placeholders::_1, std::placeholders::_2);
            static auto yCompareFaces = std::bind(compareFaces, 1, std::placeholders::_1, std::placeholders::_2);
            static auto zCompareFaces = std::bind(compareFaces, 2, std::placeholders::_1, std::placeholders::_2);

            std::size_t const x_min_index = *std::min_element(indices.begin(), indices.end(), xCompareFaces);
            std::size_t const x_max_index = *std::max_element(indices.begin(), indices.end(), xCompareFaces);
            std::size_t const y_min_index = *std::min_element(indices.begin(), indices.end(), yCompareFaces);
            std::size_t const y_max_index = *std::max_element(indices.begin(), indices.end(), yCompareFaces);
            std::size_t const z_min_index = *std::min_element(indices.begin(), indices.end(), zCompareFaces);
            std::size_t const z_max_index = *std::max_element(indices.begin(), indices.end(), zCompareFaces);

            box.xMin = shape.vertices[shape.indices[x_min_index][0]];
            box.xMax = shape.vertices[shape.indices[x_max_index][0]];
            box.yMin = shape.vertices[shape.indices[y_min_index][1]];
            box.yMax = shape.vertices[shape.indices[y_max_index][1]];
            box.zMin = shape.vertices[shape.indices[z_min_index][2]];
            box.zMax = shape.vertices[shape.indices[z_max_index][2]];
        }

        static void calculateMean(Box & box)
        {
            box.extremalMean << (box.xMax + box.xMin) / 2.0f,
                                (box.yMax + box.yMin) / 2.0f,
                                (box.zMax + box.zMin) / 2.0f;
            box.xAxis = box.extremalMean - box.xMax;
            box.yAxis = box.extremalMean - box.yMax;
            box.zAxis = box.extremalMean - box.zMax;
        }

        void createBox(Box & box)
        {
            m_box_shape = geometry::Box(
                {box.extremalMean[0], box.extremalMean[1], box.extremalMean[2]},
                {box.xAxis[0], box.xAxis[1], box.xAxis[2]},
                {box.yAxis[0], box.yAxis[1], box.yAxis[2]},
                {box.zAxis[0], box.zAxis[1], box.zAxis[2]}
            );
        }

    };
} // namespace aabb

} // namespace volumes
} // namespace geometry
} // namespace pegasus
#endif // PEGASUS_OBB_HPP
