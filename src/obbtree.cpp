#include "Pegas/include/obbtree.hpp"

#include <algorithm>

obb::ObbTree::ObbTree(const obb::OrientedBoundingBox::Shape & shape, const obb::Indices & indices) :
    m_root(std::make_shared<Leaf>(
               Leaf({ Leaf::Ptr(), std::make_shared<OrientedBoundingBox>(shape, indices) })
            ))
{
    buildTree(m_root);
}

obb::Plane obb::ObbTree::calculateHyperplane(const obb::OrientedBoundingBox::ConstPtr & obb_ptr, const obb::Vector & hyperplane_normal)
{
    auto const box = obb_ptr->getBox();
    Plane const hyperplane(hyperplane_normal, box.mean);

    return hyperplane;
}

void obb::ObbTree::calculateHalfspaces(
        const obb::OrientedBoundingBox::ConstPtr & obb_ptr,
        const obb::Plane & hyperplane, obb::Faces & lower_faces,
        obb::Faces & upper_faces, obb::Indices & lower_indices, obb::Indices & upper_indices
    )
{

    Vectors const & vertices = obb_ptr->getVertices();
    Faces   const & indices  = obb_ptr->getIndices();

    for (auto const & face : indices)
    {
        Vector const center = vertices[face[0]] + vertices[face[1]] + vertices[face[2]];

        if (hyperplane.signedDistance(center) < 0)
        {
            lower_faces.push_back(face);
            lower_indices.insert(face.begin(), face.end());
        }
        else
        {
            upper_faces.push_back(face);
            upper_indices.insert(face.begin(), face.end());
        }
    }
}

bool obb::ObbTree::splitObb(
        const obb::OrientedBoundingBox::ConstPtr & obb_ptr,
        obb::OrientedBoundingBox::Ptr & lower_obb,
        obb::OrientedBoundingBox::Ptr & upper_obb
    )
{
    if (obb_ptr->getIndices().size() > 1)
    {
        auto const box = obb_ptr->getBox();
        std::array<std::pair<float, std::size_t>, 3> extremal_projections({
            std::make_pair(Vector(box.extremal_vertices.col(0)).dot(Vector(box.eigen_vectors.col(0))), 0),
            std::make_pair(Vector(box.extremal_vertices.col(1)).dot(Vector(box.eigen_vectors.col(1))), 1),
            std::make_pair(Vector(box.extremal_vertices.col(2)).dot(Vector(box.eigen_vectors.col(2))), 2)
        });
        std::sort(extremal_projections.begin(), extremal_projections.end(),
                  [](std::pair<float, std::size_t> const & a, std::pair<float, std::size_t> const & b) {
            return a.first < b.first;
        });

        for (auto const & projection : extremal_projections)
        {
            Plane const hyperplane = calculateHyperplane(obb_ptr, box.extremal_vertices.col(projection.second));

            Faces upper_faces, lower_faces;
            Indices upper_indices, lower_indices;

            calculateHalfspaces(obb_ptr, hyperplane, lower_faces, upper_faces, lower_indices, upper_indices);

            if (!upper_faces.empty() && !lower_faces.empty())
            {
                upper_obb = std::make_shared<OrientedBoundingBox>(
                            OrientedBoundingBox::Shape({ obb_ptr->getVertices(), upper_faces }), upper_indices);
                lower_obb = std::make_shared<OrientedBoundingBox>(
                            OrientedBoundingBox::Shape({ obb_ptr->getVertices(), lower_faces }), lower_indices);

                return true;
            }
        }
    }

    return false;
}

void obb::ObbTree::buildTree(const obb::ObbTree::Leaf::Ptr & root_leaf_ptr)
{
    OrientedBoundingBox::Ptr upper_obb_ptr, lower_obb_ptr;
    std::vector<Leaf::Ptr> stack(1, root_leaf_ptr);

    while (!stack.empty())
    {
        Leaf::Ptr const leaf_ptr = stack.back();
        stack.pop_back();

        if (splitObb(leaf_ptr->obb, lower_obb_ptr, upper_obb_ptr))
        {
            if (upper_obb_ptr)
            {
                leaf_ptr->upper_child = std::make_shared<Leaf>(Leaf({ leaf_ptr, upper_obb_ptr }));
                stack.push_back(leaf_ptr->upper_child);
            }
            if (lower_obb_ptr)
            {
                leaf_ptr->lower_child = std::make_shared<Leaf>(Leaf({ leaf_ptr, lower_obb_ptr }));
                stack.push_back(leaf_ptr->lower_child);
            }
        }
    }
}
