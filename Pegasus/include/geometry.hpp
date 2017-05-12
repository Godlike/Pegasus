#ifndef PEGASUS_GEOMETRY_HPP
#define PEGASUS_GEOMETRY_HPP

#include <algorithm>
#include <array>
#include <functional>
#include <limits>
#include <memory>
#include <unordered_map>
#include <vector>

#include "Pegasus/include/math.hpp"

namespace pegasus {
namespace geometry {

    class Shape {
    public:
        explicit Shape(Vector3 const& centerOfMass);

        void setCenterOfMass(Vector3 const& centerOfMass);
        Vector3 getCenterOfMass() const;

    private:
        Vector3 mCenterOfMass;
    };

    class SimpleShape : public Shape {
    public:
        enum Type {
            PLANE,
            TRIANGLE,
            SPHERE,
            CONE,
            CYLINDER,
            CAPSULE,
            BOX,
            NONE
        };
        Type type;

    public:
        SimpleShape(Vector3 const& centerOfMass, Type type);
    };

    class Plane : public SimpleShape {
    public:
        Plane(Vector3 const& centerOfMass, Vector3 const& normal);

        void setNormal(Vector3 const& normal);
        Vector3 getNormal() const;

    private:
        Vector3 mNormal;
    };

    class Triangle : public SimpleShape {
    public:
        Triangle(Vector3 const& centerOfMass, Vector3 const& a, Vector3 const& b, Vector3 const& c);

        void setAxes(Vector3 const& a, Vector3 const& b, Vector3 const& c);
        void getAxes(Vector3& a, Vector3& b, Vector3& c) const;
        Vector3 getNormal() const;

    private:
        Vector3 mA;
        Vector3 mB;
        Vector3 mC;
        Vector3 mNormal;

        void calculateNormal();
    };

    class Sphere : public SimpleShape {
    public:
        using Ptr = std::shared_ptr<Sphere>;

    public:
        Sphere(Vector3 const& centerOfMass, real const r);

        void setRadius(real const r);
        real getRadius() const;

    private:
        real mR;
    };

    class Cone : public SimpleShape {
    public:
        Cone(Vector3 const& centerOfMass, Vector3 const& a, real const r);

        void setAppex(Vector3 const& a);
        Vector3 getAppex() const;

        void setRadius(real const r);
        real getRadius() const;

    private:
        Vector3 mA;
        real mR;
    };

    class Capsule : public SimpleShape {
    public:
        Capsule(Vector3 const& centerOfMass, Vector3 const& halfHeight, real const r);

        void setHalfHeight(Vector3 const& halfHeight);
        Vector3 getHalfHeight() const;

        void setRadius(real const r);
        real getRadius() const;

    private:
        Vector3 mHalfHeight;
        real mR;
    };

    class Cylinder : public Capsule {
    public:
        Cylinder(Vector3 const& centerOfMass, Vector3 const& halfHeight, real const r);
    };

    class Box : public SimpleShape {
    public:
        Box(Vector3 const& centerOfMass, Vector3 const& a, Vector3 const& b, Vector3 const& c);

        void setAxes(Vector3 const& a, Vector3 const& b, Vector3 const& c);
        void getAxes(Vector3& a, Vector3& b, Vector3& c) const;

    private:
        Vector3 mA;
        Vector3 mB;
        Vector3 mC;
    };

    namespace intersection {

    //Utility functions
    template <typename T>
    bool isPointOnSameSide(
        T const& p1, T const& p2, T const& a, T const& b)
    {
        auto const ab = b - a;
        auto const cp1 = ab.vectorProduct(p1 - a);
        auto const cp2 = ab.vectorProduct(p2 - a);
        return cp1.scalarProduct(cp2) >= 0;
    }

    template <typename Vector, typename VerticesContainer>
    void calculateBoxVertices(
        Vector const& i, Vector const& j, Vector const& k, VerticesContainer& vertices)
    {
        vertices[0] = (i + j + k);
        vertices[1] = (i - j + k);
        vertices[2] = (j - i + k);
        vertices[3] = (i * -1 - j + k);

        vertices[4] = (i + j - k);
        vertices[5] = (i - j - k);
        vertices[6] = (j - i - k);
        vertices[7] = (i * -1 - j - k);
    }

    template <typename SrcIt1, typename SrcIt2, typename DestIt>
    void calculateSeparatingAxes(SrcIt1 srcBegin1, SrcIt1 srcEnd1, SrcIt2 srcBegin2, SrcIt2 srcEnd2,
        std::back_insert_iterator<DestIt> destBegin)
    {
        for (auto it1 = srcBegin1; it1 != srcEnd1; ++it1) {
            for (auto it2 = srcBegin2; it2 != srcEnd2; ++it2) {
                auto const axis = it1->vectorProduct(*it2).unit();
                if (axis.squareMagnitude() != 0.0f) {
                    destBegin++ = axis;
                }
            }
        }
    }

    template <typename Vector, typename VertIt, typename ProjIt>
    void projectAllVertices(
        Vector const& axisNormal, VertIt srcBegin, VertIt srcEnd, ProjIt destBegin)
    {
        while (srcBegin != srcEnd) {
            *destBegin++ = axisNormal.scalarProduct(*srcBegin++);
        }
    }

    //Intersection query computation caches
    struct CacheBase {};

    template< typename ShapeA, typename ShapeB >
    struct Cache : CacheBase{};

    template <>
    struct Cache<Plane, Plane> : CacheBase
    {
        Vector3 aNormal;
        Vector3 bNormal;
        Vector3 crossProduct;
    };

    template <>
    struct Cache<Plane, Sphere> : CacheBase
    {
        Vector3 planeMassCenter;
        Vector3 planeNormal;
        Vector3 sphereMassCenter;
        real sphereRadius;
        real penetration;
    };

    template <>
    struct Cache<Plane, Box> : CacheBase
    {
        Vector3 boxMassCenter;
        std::array<Vector3, 3> boxAxes;
        std::array<Vector3, 8> boxVertices;
        std::array<Vector3, 6> boxFaces;
        std::array<real, 6> boxFaceDistances;
        std::array<real, 8> boxPenetrations;
        Vector3 planeNormal;
        real planeDistance;
    };

    template <>
    struct Cache<Sphere, Plane> : CacheBase
    {
        Cache<Plane, Sphere> psCache;
    };

    template <>
    struct Cache<Sphere, Sphere> : CacheBase
    {
        Vector3 bMassCenter;
        Vector3 baVector;
        real bRadius;
        Vector3 aMassCenter;
        real aRadius;
        real radiusSum;
    };

    template <>
    struct Cache<Sphere, Box> : CacheBase
    {
        Vector3 boxMassCenter;
        std::array<Vector3, 3> boxAxes;
        std::array<Vector3, 6> boxNormals;
        std::array<Vector3, 6> boxFaces;
        std::array<real, 6> boxFaceDistances;
        std::array<Vector3, 8> boxVertices;
        std::array<real, 8> boxVerticesProjections;
        std::array<Vector3, 4> separatingAxes;
        Vector3 sphereMassCenter;
        real sphereRadius;
    };

    template <>
    struct Cache<Box, Plane> : CacheBase
    {
        Cache<Plane, Box> pbCache;
        Vector3 planeMassCenter;
        Vector3 boxContactNormal;
    };

    template <>
    struct Cache<Box, Sphere> : CacheBase
    {
        Cache<Sphere, Box> sbCache;
    };

    template <>
    struct Cache<Box, Box> : CacheBase
    {
        Vector3 aMassCenter;
        Vector3 bMassCenter;
        std::array<Vector3, 8> aBoxVertices, bBoxVertices;
        std::array<Vector3, 6> aBoxFaces, bBoxFaces;
        std::vector<Vector3> separatingAxes;
        real penetration = 0;
    };

    template < typename ShapeA, typename ShapeB >
    void initialize(SimpleShape const * a, SimpleShape const * b, CacheBase  * cache);

    template < typename ShapeA, typename ShapeB >
    bool overlap(SimpleShape const * a, SimpleShape const * b, CacheBase  * cache);

    template < typename ShapeA, typename ShapeB >
    Vector3 calculateContactNormal(SimpleShape const * a, SimpleShape const * b, CacheBase  * cache);

    template < typename ShapeA, typename ShapeB >
    real calculatePenetration(SimpleShape const * a, SimpleShape const * b, CacheBase  * cache);

    //Plane, Plane
    template <>
    inline void initialize<Plane, Plane>(const SimpleShape *a, const SimpleShape *b, CacheBase *cache)
    {
        auto const * p1 = static_cast<Plane const *>(a);
        auto const * p2 = static_cast<Plane const *>(b);
        auto * c = static_cast<Cache<Plane, Plane>*>(cache);

        c->aNormal = p1->getNormal();
        c->bNormal = p2->getNormal();
        c->crossProduct = c->aNormal % c->bNormal;
    }

    template <>
    inline bool overlap<Plane, Plane>(const SimpleShape *a, const SimpleShape *b, CacheBase *cache)
    {
        auto * c = static_cast<Cache<Plane, Plane>*>(cache);
        return c->crossProduct.squareMagnitude() != static_cast<real>(0);
    }

    template <>
    inline Vector3 calculateContactNormal<Plane, Plane>(const SimpleShape *a, const SimpleShape *b, CacheBase *cache)
    {
        auto * c = static_cast<Cache<Plane, Plane>*>(cache);
        return c->bNormal;
    }

    template <>
    inline real calculatePenetration<Plane, Plane>(const SimpleShape *a, const SimpleShape *b, CacheBase *cache)
    {
        return std::numeric_limits<real>::max();
    }

    //Plane, Sphere
    template <>
    inline void initialize<Plane, Sphere>(const SimpleShape *a, const SimpleShape *b, CacheBase *cache)
    {
        auto const * p = static_cast<Plane const *>(a);
        auto const * s = static_cast<Sphere const *>(b);
        auto * c = static_cast<Cache<Plane, Sphere>*>(cache);

        c->planeNormal = p->getNormal();
        c->planeMassCenter = p->getCenterOfMass();
        c->sphereMassCenter = s->getCenterOfMass();
        c->sphereRadius = s->getRadius();
        c->penetration = c->sphereRadius - (c->sphereMassCenter * c->planeNormal - c->planeMassCenter * c->planeNormal);
    }

    template <>
    inline bool overlap<Plane, Sphere>(const SimpleShape *a, const SimpleShape *b, CacheBase *cache)
    {
        auto * c = static_cast<Cache<Plane,Sphere>*>(cache);
        return c->penetration >= real(0);
    }

    template <>
    inline Vector3 calculateContactNormal<Plane, Sphere>(const SimpleShape *a, const SimpleShape *b, CacheBase *cache)
    {
        auto * c = static_cast<Cache<Plane,Sphere>*>(cache);
        return c->planeNormal.inverse();
    }

    template <>
    inline real calculatePenetration<Plane, Sphere>(const SimpleShape *a, const SimpleShape *b, CacheBase *cache)
    {
        auto * c = static_cast<Cache<Plane,Sphere>*>(cache);
        return c->penetration;
    }

    //Plane, Box
    template <>
    inline void initialize<Plane, Box>(const SimpleShape *a, const SimpleShape *b, CacheBase *cache)
    {
        auto const * plane = static_cast<Plane const *>(a);
        auto const * box = static_cast<Box const *>(b);
        auto * c = static_cast<Cache<Plane,Box>*>(cache);

        c->planeNormal = plane->getNormal();
        c->planeDistance = plane->getCenterOfMass() * c->planeNormal;

        box->getAxes(c->boxAxes[0], c->boxAxes[1], c->boxAxes[2]);
        c->boxMassCenter = box->getCenterOfMass();
        calculateBoxVertices(c->boxAxes[0], c->boxAxes[1], c->boxAxes[2], c->boxVertices);
        c->boxFaces = { c->boxAxes[0], c->boxAxes[1], c->boxAxes[2], c->boxAxes[0].inverse(), c->boxAxes[1].inverse(), c->boxAxes[2].inverse() };
        std::for_each(c->boxVertices.begin(), c->boxVertices.end(), [c](auto& n) { n += c->boxMassCenter; });
        std::transform(c->boxVertices.begin(), c->boxVertices.end(), c->boxPenetrations.begin(),
            [c](auto const& p) { return c->planeDistance - p * c->planeNormal; });
    }

    template <>
    inline bool overlap<Plane, Box>(const SimpleShape *a, const SimpleShape *b, CacheBase *cache)
    {
        auto * c = static_cast<Cache<Plane,Box>*>(cache);

        return *max_element(c->boxPenetrations.begin(), c->boxPenetrations.end()) >= 0;
    }

    template <>
    inline Vector3 calculateContactNormal<Plane, Box>(const SimpleShape *a, const SimpleShape *b, CacheBase *cache)
    {
        auto * c = static_cast<Cache<Plane,Box>*>(cache);

        std::transform(c->boxFaces.begin(), c->boxFaces.end(), c->boxFaceDistances.begin(), 
            [c](auto const& v) { return v * c->planeNormal; });
        auto const minIndex = distance(c->boxFaceDistances.begin(),
            min_element(c->boxFaceDistances.begin(), c->boxFaceDistances.end()));

        return c->boxFaces[minIndex].unit();
    }

    template <>
    inline real calculatePenetration<Plane, Box>(const SimpleShape *a, const SimpleShape *b, CacheBase *cache)
    {
        auto * c = static_cast<Cache<Plane,Box>*>(cache);

        return (*max_element(c->boxPenetrations.begin(), c->boxPenetrations.end()));
    }

    //Sphere, Plane
    template <>
    inline void initialize<Sphere, Plane>(const SimpleShape *a, const SimpleShape *b, CacheBase *cache)
    {
        auto * c = static_cast<Cache<Sphere, Plane>*>(cache);
        initialize<Plane, Sphere>(b, a, &c->psCache);
    }

    template <>
    inline bool overlap<Sphere, Plane>(const SimpleShape *a, const SimpleShape *b, CacheBase *cache)
    {
        auto * c = static_cast<Cache<Sphere, Plane>*>(cache);
        return overlap<Plane, Sphere>(b, a, &c->psCache);
    }

    template <>
    inline Vector3 calculateContactNormal<Sphere, Plane>(const SimpleShape *a, const SimpleShape *b, CacheBase *cache)
    {
        auto * c = static_cast<Cache<Sphere, Plane>*>(cache);
        return calculateContactNormal<Plane, Sphere>(a, b, &c->psCache).inverse();
    }

    template <>
    inline real calculatePenetration<Sphere, Plane>(const SimpleShape *a, const SimpleShape *b, CacheBase *cache)
    {
        auto * c = static_cast<Cache<Sphere, Plane>*>(cache);
        return calculatePenetration<Plane, Sphere>(b, a, &c->psCache);
    }

    //Sphere, Sphere
    template <>
    inline void initialize<Sphere, Sphere>(const SimpleShape *a, const SimpleShape *b, CacheBase *cache)
    {
        auto const * s1 = static_cast<Sphere const *>(a);
        auto const * s2 = static_cast<Sphere const *>(b);
        auto * c = static_cast<Cache<Sphere, Sphere>*>(cache);
       
        c->aRadius = s1->getRadius();
        c->aMassCenter = s1->getCenterOfMass();
        c->bRadius = s2->getRadius();
        c->bMassCenter = s2->getCenterOfMass();
        c->baVector = c->aMassCenter - c->bMassCenter;
        c->radiusSum = c->aRadius + c->aRadius;
    }

    template <>
    inline bool overlap<Sphere, Sphere>(const SimpleShape *a, const SimpleShape *b, CacheBase *cache)
    {
        auto * c = static_cast<Cache<Sphere, Sphere>*>(cache);
        return pow(c->radiusSum, 2) > c->baVector.squareMagnitude();
    }

    template <>
    inline Vector3 calculateContactNormal<Sphere, Sphere>(const SimpleShape *a, const SimpleShape *b, CacheBase *cache)
    {
        auto * c = static_cast<Cache<Sphere, Sphere>*>(cache);
        return c->baVector.unit();;
    }

    template <>
    inline real calculatePenetration<Sphere, Sphere>(const SimpleShape *a, const SimpleShape *b, CacheBase *cache)
    {
        auto * c = static_cast<Cache<Sphere, Sphere>*>(cache);
        return (c->radiusSum - c->baVector.magnitude());
    }

    //Sphere, Box
    template <>
    inline void initialize<Sphere, Box>(const SimpleShape *a, const SimpleShape *b, CacheBase *cache)
    {
        auto const * sphere = static_cast<Sphere const *>(a);
        auto const * box = static_cast<Box const *>(b);
        auto * c = static_cast<Cache<Sphere, Box>*>(cache);

        c->sphereMassCenter = sphere->getCenterOfMass();
        c->sphereRadius = sphere->getRadius();

        c->boxMassCenter = box->getCenterOfMass();
        box->getAxes(c->boxAxes[0], c->boxAxes[1], c->boxAxes[2]);
        c->separatingAxes = { c->boxAxes[0].unit(), c->boxAxes[1].unit(), c->boxAxes[2].unit(), 
                             (c->sphereMassCenter - c->boxMassCenter).unit() };
        calculateBoxVertices(c->boxAxes[0], c->boxAxes[1], c->boxAxes[2], c->boxVertices);
        std::for_each(c->boxVertices.begin(), c->boxVertices.end(), [c](auto& n) { n += c->boxMassCenter; });
        c->boxNormals = { c->boxAxes[0], c->boxAxes[1], c->boxAxes[2], 
                          c->boxAxes[0].inverse(), c->boxAxes[1].inverse(), c->boxAxes[2].inverse() };
        for (unsigned int i = 0; i < c->boxNormals.size(); ++i) {
            c->boxFaces[i] = c->boxNormals[i] + c->boxMassCenter;
            c->boxNormals[i].normalize();
            c->boxFaceDistances[i] = c->boxFaces[i] * c->boxNormals[i] - c->sphereMassCenter * c->boxNormals[i];
        }
    }

    template <>
    inline bool overlap<Sphere, Box>(const SimpleShape *a, const SimpleShape *b, CacheBase *cache)
    {
        auto * c = static_cast<Cache<Sphere, Box>*>(cache);

        for(auto const & axis : c->separatingAxes)
        {
            projectAllVertices(axis, c->boxVertices.begin(), c->boxVertices.end(), c->boxVerticesProjections.begin());
            sort(c->boxVerticesProjections.begin(), c->boxVerticesProjections.end());
            auto const boxMassCenterProjection = c->boxMassCenter * axis;
            auto const sphereMassCenterProjection = c->sphereMassCenter * axis;

            if (boxMassCenterProjection < sphereMassCenterProjection) {
                if (sphereMassCenterProjection - c->boxVerticesProjections.back() > c->sphereRadius) {
                    return false;
                }
            }
            else if (c->boxVerticesProjections.front() > sphereMassCenterProjection + c->sphereRadius) {
                return false;
            }
        }

        return true;
    }

    template <>
    inline Vector3 calculateContactNormal<Sphere, Box>(const SimpleShape *a, const SimpleShape *b, CacheBase *cache)
    {
        auto * c = static_cast<Cache<Sphere, Box>*>(cache);

        auto minIt = min_element(c->boxFaceDistances.begin(), c->boxFaceDistances.end());
        auto minIndex = distance(c->boxFaceDistances.begin(), minIt);
        return c->boxNormals[minIndex];
    }

    template <>
    inline real calculatePenetration<Sphere, Box>(const SimpleShape *a, const SimpleShape *b, CacheBase *cache)
    {
        auto * c = static_cast<Cache<Sphere, Box>*>(cache);

        auto minIt = min_element(c->boxFaceDistances.begin(), c->boxFaceDistances.end());
        return c->sphereRadius - *minIt;
    }

    //<Box, Plane
    template <>
    inline void initialize<Box, Plane>(const SimpleShape *a, const SimpleShape *b, CacheBase *cache)
    {
        auto * c = static_cast<Cache<Box, Plane>*>(cache);
        initialize<Plane, Box>(b, a, &c->pbCache);
    }

    template <>
    inline bool overlap<Box, Plane>(const SimpleShape *a, const SimpleShape *b, CacheBase *cache)
    {
        auto * c = static_cast<Cache<Box, Plane>*>(cache);
        return overlap<Plane, Box>(b, a, &c->pbCache);
    }

    template <>
    inline Vector3 calculateContactNormal<Box, Plane>(const SimpleShape *a, const SimpleShape *b, CacheBase *cache)
    {
        auto const * plane = static_cast<Plane const *>(b);
        return plane->getNormal();
    }

    template <>
    inline real calculatePenetration<Box, Plane>(const SimpleShape *a, const SimpleShape *b, CacheBase *cache)
    {
        auto * c = static_cast<Cache<Box, Plane>*>(cache);
        return calculatePenetration<Plane, Box>(b, a, &c->pbCache);
    }

    //Box, Sphere
    template <>
    inline void initialize<Box, Sphere>(const SimpleShape *a, const SimpleShape *b, CacheBase *cache)
    {
        auto * c = static_cast<Cache<Box, Sphere>*>(cache);
        initialize<Sphere, Box>(b, a, &c->sbCache);
    }

    template <>
    inline bool overlap<Box, Sphere>(const SimpleShape *a, const SimpleShape *b, CacheBase *cache)
    {
        auto * c = static_cast<Cache<Box, Sphere>*>(cache);
        return overlap<Sphere, Box>(b, a, &c->sbCache);
    }

    template <>
    inline Vector3 calculateContactNormal<Box, Sphere>(const SimpleShape *a, const SimpleShape *b, CacheBase *cache)
    {
        auto * c = static_cast<Cache<Box, Sphere>*>(cache);
        return calculateContactNormal<Sphere, Box>(b, a, &c->sbCache).inverse();
    }

    template <>
    inline real calculatePenetration<Box, Sphere>(const SimpleShape *a, const SimpleShape *b, CacheBase *cache)
    {
        auto * c = static_cast<Cache<Box, Sphere>*>(cache);
        return calculatePenetration<Sphere, Box>(b, a, &c->sbCache);
    }

    //Box, Box
    template <>
    inline void initialize<Box, Box>(const SimpleShape *a, const SimpleShape *b, CacheBase *cache)
    {
        auto const * aBox = static_cast<Box const *>(a);
        auto const * bBox = static_cast<Box const *>(b);
        auto * c = static_cast<Cache<Box, Box>*>(cache);

        c->aMassCenter = aBox->getCenterOfMass();
        aBox->getAxes(c->aBoxFaces[0], c->aBoxFaces[1], c->aBoxFaces[2]);
        c->aBoxFaces = { c->aBoxFaces[0], c->aBoxFaces[1], c->aBoxFaces[2], 
                         c->aBoxFaces[0].inverse(), c->aBoxFaces[1].inverse(), c->aBoxFaces[2].inverse() };
        c->bMassCenter = bBox->getCenterOfMass();
        bBox->getAxes(c->bBoxFaces[0], c->bBoxFaces[1], c->bBoxFaces[2]);
        c->bBoxFaces = { c->bBoxFaces[0], c->bBoxFaces[1], c->bBoxFaces[2], 
                         c->bBoxFaces[0].inverse(), c->bBoxFaces[1].inverse(), c->bBoxFaces[2].inverse() };
        calculateBoxVertices(c->aBoxFaces[0], c->aBoxFaces[1], c->aBoxFaces[2], c->aBoxVertices);
        calculateBoxVertices(c->bBoxFaces[0], c->bBoxFaces[1], c->bBoxFaces[2], c->bBoxVertices);
        std::for_each(c->aBoxVertices.begin(), c->aBoxVertices.end(), [c](auto& v) { v += c->aMassCenter; });
        std::for_each(c->bBoxVertices.begin(), c->bBoxVertices.end(), [c](auto& v) { v += c->bMassCenter; });
        c->separatingAxes = { c->aBoxFaces[0].unit(), c->aBoxFaces[1].unit(), c->aBoxFaces[2].unit(),
                              c->bBoxFaces[0].unit(), c->bBoxFaces[1].unit(), c->bBoxFaces[2].unit() };
        calculateSeparatingAxes(c->aBoxFaces.begin(), c->aBoxFaces.begin() + 3,
                                c->bBoxFaces.begin(), c->bBoxFaces.begin() + 3,
                                back_inserter(c->separatingAxes));
    }

    template <>
    inline bool overlap<Box, Box>(const SimpleShape *a, const SimpleShape *b, CacheBase *cache)
    {
        auto * c = static_cast<Cache<Box, Box>*>(cache);

        std::array<real, 8> aBoxProjections, bBoxProjections;
        for (auto axis : c->separatingAxes) {
            projectAllVertices(axis, c->aBoxVertices.begin(), c->aBoxVertices.end(), aBoxProjections.begin());
            projectAllVertices(axis, c->bBoxVertices.begin(), c->bBoxVertices.end(), bBoxProjections.begin());
            sort(aBoxProjections.begin(), aBoxProjections.end());
            sort(bBoxProjections.begin(), bBoxProjections.end());

            if (aBoxProjections.back() < bBoxProjections.back()) {
                if (aBoxProjections.back() < bBoxProjections.front()) {
                    return false;
                }
                c->penetration = aBoxProjections.back() - bBoxProjections.front();
            }
            else {
                if (bBoxProjections.back() < aBoxProjections.front()) {
                    return false;
                }
                c->penetration = bBoxProjections.back() - aBoxProjections.front();
            }
        }

        return true;
    }

    template <>
    inline Vector3 calculateContactNormal<Box, Box>(const SimpleShape *a, const SimpleShape *b, CacheBase *cache)
    {
        auto * c = static_cast<Cache<Box, Box>*>(cache);

        std::array<real, 6> distances;
        for (unsigned i = 0; i < distances.size(); ++i) {
            distances[i] = (c->aMassCenter - c->bMassCenter - c->bBoxFaces[i]).squareMagnitude();
        }

        auto const minIt = min_element(distances.begin(), distances.end());
        auto const minIndex = distance(distances.begin(), minIt);

        return c->bBoxFaces[minIndex].unit();
    }

    template <>
    inline real calculatePenetration<Box, Box>(const SimpleShape *a, const SimpleShape *b, CacheBase *cache)
    {
        auto * c = static_cast<Cache<Box, Box>*>(cache);
        return c->penetration;
    }

    }// namespace IntersectionQuery


    //General intersection
    using ShapeTypePair = std::pair<SimpleShape::Type, SimpleShape::Type>;

    size_t shapeTypePairHash(ShapeTypePair const & p);

    class IntersectionQuery {
    private:
        std::unordered_map<ShapeTypePair,
            std::unique_ptr<intersection::CacheBase>,
            std::function<size_t(ShapeTypePair const& p)> >
            intersectionCaches;

        std::unordered_map<ShapeTypePair,
            std::function<void(SimpleShape const *, SimpleShape const *, intersection::CacheBase*)>,
            std::function<size_t(ShapeTypePair const& p)> >
            initializeFunctors;

        std::unordered_map<ShapeTypePair,
            std::function<bool(SimpleShape const *, SimpleShape const *, intersection::CacheBase*)>,
            std::function<size_t(ShapeTypePair const& p)> >
            overlapFunctors;

        std::unordered_map<ShapeTypePair,
            std::function<Vector3(SimpleShape const *, SimpleShape const *, intersection::CacheBase*)>,
            std::function<size_t(ShapeTypePair const& p)> >
            calculateContactNormalFunctors;

        std::unordered_map<ShapeTypePair,
            std::function<real(SimpleShape const *, SimpleShape const *, intersection::CacheBase*)>,
            std::function<size_t(ShapeTypePair const& p)> >
            calculatePenetrationFunctors;

    public:
        IntersectionQuery()
            : intersectionCaches(11, &shapeTypePairHash)
            , initializeFunctors(11, &shapeTypePairHash)
            , overlapFunctors(11, &shapeTypePairHash)
            , calculateContactNormalFunctors(11, &shapeTypePairHash)
            , calculatePenetrationFunctors(11, &shapeTypePairHash)
        {            
            intersectionCaches[std::make_pair(SimpleShape::PLANE, SimpleShape::PLANE)] 
                = std::make_unique<intersection::Cache<Plane, Plane>>();
            intersectionCaches[std::make_pair(SimpleShape::PLANE, SimpleShape::SPHERE)] 
                = std::make_unique<intersection::Cache<Plane, Sphere>>();
            intersectionCaches[std::make_pair(SimpleShape::PLANE, SimpleShape::BOX)] 
                = std::make_unique<intersection::Cache<Plane, Box>>();
            intersectionCaches[std::make_pair(SimpleShape::SPHERE, SimpleShape::PLANE)]
                = std::make_unique<intersection::Cache<Sphere, Plane>>();
            intersectionCaches[std::make_pair(SimpleShape::SPHERE, SimpleShape::SPHERE)]
                = std::make_unique<intersection::Cache<Sphere, Sphere>>();
            intersectionCaches[std::make_pair(SimpleShape::SPHERE, SimpleShape::BOX)]
                = std::make_unique<intersection::Cache<Sphere, Box>>();
            intersectionCaches[std::make_pair(SimpleShape::BOX, SimpleShape::PLANE)]
                = std::make_unique<intersection::Cache<Box, Plane>>();
            intersectionCaches[std::make_pair(SimpleShape::BOX, SimpleShape::SPHERE)]
                = std::make_unique<intersection::Cache<Box, Sphere>>();
            intersectionCaches[std::make_pair(SimpleShape::BOX, SimpleShape::BOX)]
                = std::make_unique<intersection::Cache<Box, Box>>();

            initializeFunctors[std::make_pair(SimpleShape::PLANE, SimpleShape::PLANE)]
                = &intersection::initialize<Plane, Plane>;
            initializeFunctors[std::make_pair(SimpleShape::PLANE, SimpleShape::SPHERE)]
                = &intersection::initialize<Plane, Sphere>;
            initializeFunctors[std::make_pair(SimpleShape::PLANE, SimpleShape::BOX)]
                = &intersection::initialize<Plane, Box>;
            initializeFunctors[std::make_pair(SimpleShape::SPHERE, SimpleShape::PLANE)]
                = &intersection::initialize<Sphere, Plane>;
            initializeFunctors[std::make_pair(SimpleShape::SPHERE, SimpleShape::SPHERE)]
                = &intersection::initialize<Sphere, Sphere>;
            initializeFunctors[std::make_pair(SimpleShape::SPHERE, SimpleShape::BOX)]
                = &intersection::initialize<Sphere, Box>;
            initializeFunctors[std::make_pair(SimpleShape::BOX, SimpleShape::PLANE)]
                = &intersection::initialize<Box, Plane>;
            initializeFunctors[std::make_pair(SimpleShape::BOX, SimpleShape::SPHERE)]
                = &intersection::initialize<Box, Sphere>;
            initializeFunctors[std::make_pair(SimpleShape::BOX, SimpleShape::BOX)]
                = &intersection::initialize<Box, Box>;

            overlapFunctors[std::make_pair(SimpleShape::PLANE, SimpleShape::PLANE)]
                = &intersection::overlap<Plane, Plane>;
            overlapFunctors[std::make_pair(SimpleShape::PLANE, SimpleShape::SPHERE)]
                = &intersection::overlap<Plane, Sphere>;
            overlapFunctors[std::make_pair(SimpleShape::PLANE, SimpleShape::BOX)]
                = &intersection::overlap<Plane, Box>;
            overlapFunctors[std::make_pair(SimpleShape::SPHERE, SimpleShape::PLANE)]
                = &intersection::overlap<Sphere, Plane>;
            overlapFunctors[std::make_pair(SimpleShape::SPHERE, SimpleShape::SPHERE)]
                = &intersection::overlap<Sphere, Sphere>;
            overlapFunctors[std::make_pair(SimpleShape::SPHERE, SimpleShape::BOX)]
                = &intersection::overlap<Sphere, Box>;
            overlapFunctors[std::make_pair(SimpleShape::BOX, SimpleShape::PLANE)]
                = &intersection::overlap<Box, Plane>;
            overlapFunctors[std::make_pair(SimpleShape::BOX, SimpleShape::SPHERE)]
                = &intersection::overlap<Box, Sphere>;
            overlapFunctors[std::make_pair(SimpleShape::BOX, SimpleShape::BOX)]
                = &intersection::overlap<Box, Box>;

            calculateContactNormalFunctors[std::make_pair(SimpleShape::PLANE, SimpleShape::PLANE)]
                = &intersection::calculateContactNormal<Plane, Plane>;
            calculateContactNormalFunctors[std::make_pair(SimpleShape::PLANE, SimpleShape::SPHERE)]
                = &intersection::calculateContactNormal<Plane, Sphere>;
            calculateContactNormalFunctors[std::make_pair(SimpleShape::PLANE, SimpleShape::BOX)]
                = &intersection::calculateContactNormal<Plane, Box>;
            calculateContactNormalFunctors[std::make_pair(SimpleShape::SPHERE, SimpleShape::PLANE)]
                = &intersection::calculateContactNormal<Sphere, Plane>;
            calculateContactNormalFunctors[std::make_pair(SimpleShape::SPHERE, SimpleShape::SPHERE)]
                = &intersection::calculateContactNormal<Sphere, Sphere>;
            calculateContactNormalFunctors[std::make_pair(SimpleShape::SPHERE, SimpleShape::BOX)]
                = &intersection::calculateContactNormal<Sphere, Box>;
            calculateContactNormalFunctors[std::make_pair(SimpleShape::BOX, SimpleShape::PLANE)]
                = &intersection::calculateContactNormal<Box, Plane>;
            calculateContactNormalFunctors[std::make_pair(SimpleShape::BOX, SimpleShape::SPHERE)]
                = &intersection::calculateContactNormal<Box, Sphere>;
            calculateContactNormalFunctors[std::make_pair(SimpleShape::BOX, SimpleShape::BOX)]
                = &intersection::calculateContactNormal<Box, Box>;

            calculatePenetrationFunctors[std::make_pair(SimpleShape::PLANE, SimpleShape::PLANE)]
                = &intersection::calculatePenetration<Plane, Plane>;
            calculatePenetrationFunctors[std::make_pair(SimpleShape::PLANE, SimpleShape::SPHERE)]
                = &intersection::calculatePenetration<Plane, Sphere>;
            calculatePenetrationFunctors[std::make_pair(SimpleShape::PLANE, SimpleShape::BOX)]
                = &intersection::calculatePenetration<Plane, Box>;
            calculatePenetrationFunctors[std::make_pair(SimpleShape::SPHERE, SimpleShape::PLANE)]
                = &intersection::calculatePenetration<Sphere, Plane>;
            calculatePenetrationFunctors[std::make_pair(SimpleShape::SPHERE, SimpleShape::SPHERE)]
                = &intersection::calculatePenetration<Sphere, Sphere>;
            calculatePenetrationFunctors[std::make_pair(SimpleShape::SPHERE, SimpleShape::BOX)]
                = &intersection::calculatePenetration<Sphere, Box>;
            calculatePenetrationFunctors[std::make_pair(SimpleShape::BOX, SimpleShape::PLANE)]
                = &intersection::calculatePenetration<Box, Plane>;
            calculatePenetrationFunctors[std::make_pair(SimpleShape::BOX, SimpleShape::SPHERE)]
                = &intersection::calculatePenetration<Box, Sphere>;
            calculatePenetrationFunctors[std::make_pair(SimpleShape::BOX, SimpleShape::BOX)]
                = &intersection::calculatePenetration<Box, Box>;
        }

        void initialize(SimpleShape* const s1, SimpleShape* const s2)
        {
            initializeFunctors[std::make_pair(s1->type, s2->type)](
                s1, s2, intersectionCaches[std::make_pair(s1->type, s2->type)].get());
        }

        bool overlap(SimpleShape* const s1, SimpleShape* const s2)
        {
            return overlapFunctors[std::make_pair(s1->type, s2->type)](
                s1, s2, intersectionCaches[std::make_pair(s1->type, s2->type)].get());
        }

        Vector3 calculateContactNormal(SimpleShape* const s1, SimpleShape* const s2)
        {
            return calculateContactNormalFunctors[std::make_pair(s1->type, s2->type)](
                s1, s2, intersectionCaches[std::make_pair(s1->type, s2->type)].get());
        }

        real calculatePenetration(SimpleShape* const s1, SimpleShape* const s2)
        {
            return calculatePenetrationFunctors[std::make_pair(s1->type, s2->type)](
                s1, s2, intersectionCaches[std::make_pair(s1->type, s2->type)].get());
        }
    };

} // namespace geometry
} // namespace pegasus

#endif // PEGASUS_GEOMETRY_HPP
