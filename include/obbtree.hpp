#ifndef PEGAS_OBB_TREE_HPP
#define PEGAS_OBB_TREE_HPP

#include "Pegas/include/obb.hpp"

namespace obb {

    class ObbTree
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        struct Leaf
        {
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW

            using Ptr = std::shared_ptr<Leaf>;
            using ConstPtr = std::shared_ptr<Leaf const>;

            Leaf::ConstPtr const parent;
            OrientedBoundingBox::ConstPtr const obb;
            Leaf::Ptr lower_child;
            Leaf::Ptr upper_child;
        };


        ObbTree(OrientedBoundingBox::Shape const & shape, Indices const & indices);

    private:
        Leaf::Ptr const m_root;

        static Plane calculateHyperplane(
                OrientedBoundingBox::ConstPtr const & obb_ptr,
                Vector const & hyperplane_normal);

        static void calculateHalfspaces(
            OrientedBoundingBox::ConstPtr const & obb_ptr, Plane const & hyperplane,
            Faces & lower_faces, Faces & upper_faces,
            Indices & lower_indices, Indices & upper_indices
        );

        static bool splitObb(OrientedBoundingBox::ConstPtr const & obb_ptr,
                             OrientedBoundingBox::Ptr & lower_obb,
                             OrientedBoundingBox::Ptr & upper_obb);

        static void buildTree(Leaf::Ptr const & root_leaf_ptr);
    };

}//obb namespace

#endif //PEGAS_OBB_TREE_HPP
