#ifndef PEGASUS_GEOMETRY_HPP
#define PEGASUS_GEOMETRY_HPP

#include <algorithm>
#include <array>
#include <functional>
#include <limits>
#include <memory>
#include <cmath>
#include <unordered_map>
#include <vector>
#include <utility>
#include <cstdint>

#include "Pegasus/include/math.hpp"

namespace pegasus {
namespace geometry {

    //Shapes
    class Shape {
    public:
        explicit Shape(Vector3 const & centerOfMass);

        void setCenterOfMass(Vector3 const & centerOfMass);
        Vector3 getCenterOfMass() const;

    private:
        Vector3 mCenterOfMass;
    };

    enum class SimpleShapeType : uint32_t {
        PLANE,
        TRIANGLE,
        SPHERE,
        CONE,
        CYLINDER,
        CAPSULE,
        BOX,
        NONE
    };

    class SimpleShape : public Shape {
    public:
        SimpleShapeType type;
    public:
        SimpleShape(Vector3 const & centerOfMass, SimpleShapeType type);
    };

    class Plane : public SimpleShape {
    public:
        Plane(Vector3 const & centerOfMass, Vector3 const & normal);

        void setNormal(Vector3 const & normal);
        Vector3 getNormal() const;

    private:
        Vector3 mNormal;
    };

    class Triangle : public SimpleShape {
    public:
        Triangle(Vector3 const & centerOfMass, Vector3 const & a, Vector3 const & b, Vector3 const & c);

        void setAxes(Vector3 const & a, Vector3 const & b, Vector3 const & c);
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
        Sphere(Vector3 const & centerOfMass, double r);

        void setRadius(double r);
        double getRadius() const;

    private:
        double mR;
    };

    class Cone : public SimpleShape {
    public:
        Cone(Vector3 const & centerOfMass, Vector3 const & a, double r);

        void setAppex(Vector3 const & a);
        Vector3 getAppex() const;

        void setRadius(double r);
        double getRadius() const;

    private:
        Vector3 mA;
        double mR;
    };

    class Capsule : public SimpleShape {
    public:
        Capsule(Vector3 const & centerOfMass, Vector3 const & halfHeight, double r);

        void setHalfHeight(Vector3 const & halfHeight);
        Vector3 getHalfHeight() const;

        void setRadius(double r);
        double getRadius() const;

    private:
        Vector3 mHalfHeight;
        double mR;
    };

    class Cylinder : public Capsule {
    public:
        Cylinder(Vector3 const & centerOfMass, Vector3 const & halfHeight, double r);
    };

    class Box : public SimpleShape {
    public:
        Box(Vector3 const & centerOfMass, Vector3 const & a, Vector3 const & b, Vector3 const & c);

        void setAxes(Vector3 const & a, Vector3 const & b, Vector3 const & c);
        void getAxes(Vector3 & a, Vector3 & b, Vector3 & c) const;

    private:
        Vector3 mA;
        Vector3 mB;
        Vector3 mC;
    };

namespace intersection {

    // Utility functions
    template <typename T>
    bool isPointOnSameSide(
        T const & p1, T const & p2, T const & a, T const & b)
    {
        auto const ab = b - a;
        auto const cp1 = ab.vectorProduct(p1 - a);
        auto const cp2 = ab.vectorProduct(p2 - a);
        return cp1.scalarProduct(cp2) >= 0;
    }

    template <typename Vector, typename VerticesContainer>
    void calculateBoxVertices(
        Vector const & i, Vector const & j, Vector const & k, VerticesContainer& vertices)
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
        Vector const & axisNormal, VertIt srcBegin, VertIt srcEnd, ProjIt destBegin)
    {
        while (srcBegin != srcEnd) {
            *destBegin++ = axisNormal.scalarProduct(*srcBegin++);
        }
    }

    // Intersection query computation cacheBases
    struct CacheBase {};

    template< typename ShapeA, typename ShapeB >
    struct Cache : CacheBase {};

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
        double sphereRadius;
        double penetration;
    };

    template <>
    struct Cache<Plane, Box> : CacheBase
    {
        Vector3 boxMassCenter;
        std::array<Vector3, 3> boxAxes;
        std::array<Vector3, 8> boxVertices;
        std::array<Vector3, 6> boxFaces;
        std::array<double, 6> boxFaceDistances;
        std::array<double, 8> boxPenetrations;
        Vector3 planeNormal;
        double planeDistance;
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
        double bRadius;
        Vector3 aMassCenter;
        double aRadius;
        double radiusSum;
    };

    template <>
    struct Cache<Sphere, Box> : CacheBase
    {
        Vector3 boxMassCenter;
        std::array<Vector3, 6> boxAxes;
        std::array<Vector3, 6> boxNormals;
        std::array<Vector3, 6> boxFaces;
        std::array<double, 6> boxFaceDistances;
        std::array<Vector3, 8> boxVertices;
        std::array<double, 8> boxVerticesProjections;
        std::array<Vector3, 4> separatingAxes;
        Vector3 boxContactNormal;
        Vector3 boxContactAxis;
        Vector3 sphereContactNormal;
        Vector3 boxSphereVector;
        Vector3 sphereMassCenter;
        double sphereRadius;
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
        std::array<Vector3, 6> aBoxAxes, bBoxAxes;
        std::array<Vector3, 6> aBoxFaces, bBoxFaces;
        std::vector<Vector3> separatingAxes;
        std::array<double, 6> aBoxFaceDistances, bBoxFaceDistances;
        std::array<double, 8> aBoxVerticesDistances, bBoxVerticesDistances;
        Vector3 contactNormal;
        double penetration = 0;
    };

    // General intersection queries
    template < typename ShapeA, typename ShapeB >
    void initialize(SimpleShape const * a, SimpleShape const * b, CacheBase  * cacheBase);

    template < typename ShapeA, typename ShapeB >
    bool overlap(SimpleShape const * a, SimpleShape const * b, CacheBase  * cacheBase);

    template < typename ShapeA, typename ShapeB >
    Vector3 calculateContactNormal(SimpleShape const * a, SimpleShape const * b, CacheBase  * cacheBase);

    template < typename ShapeA, typename ShapeB >
    double calculatePenetration(SimpleShape const * a, SimpleShape const * b, CacheBase  * cacheBase);

    // Plane, Plane
    template <>
    inline void initialize<Plane, Plane>(SimpleShape const *a, SimpleShape const *b, CacheBase *cacheBase)
    {
        auto aPlane = static_cast<Plane const *>(a);
        auto bPlane = static_cast<Plane const *>(b);
        auto cache = static_cast<Cache<Plane, Plane>*>(cacheBase);

        cache->aNormal = aPlane->getNormal();
        cache->bNormal = bPlane->getNormal();
        cache->crossProduct = cache->aNormal % cache->bNormal;
    }

    template <>
    inline bool overlap<Plane, Plane>(SimpleShape const *a, SimpleShape const *b, CacheBase *cacheBase)
    {
        auto cache = static_cast<Cache<Plane, Plane>*>(cacheBase);
        return cache->crossProduct.squareMagnitude() != 0;
    }

    template <>
    inline Vector3 calculateContactNormal<Plane, Plane>(SimpleShape const *a, SimpleShape const *b, CacheBase *cacheBase)
    {
        auto cache = static_cast<Cache<Plane, Plane>*>(cacheBase);
        return cache->bNormal;
    }

    template <>
    inline double calculatePenetration<Plane, Plane>(SimpleShape const *a, SimpleShape const *b, CacheBase *cacheBase)
    {
        return std::numeric_limits<double>::max();
    }

    // Plane, Sphere
    template <>
    inline void initialize<Plane, Sphere>(SimpleShape const *a, SimpleShape const *b, CacheBase *cacheBase)
    {
        auto plane = static_cast<Plane const *>(a);
        auto sphere = static_cast<Sphere const *>(b);
        auto cache = static_cast<Cache<Plane, Sphere>*>(cacheBase);

        cache->planeNormal = plane->getNormal();
        cache->planeMassCenter = plane->getCenterOfMass();
        cache->sphereMassCenter = sphere->getCenterOfMass();
        cache->sphereRadius = sphere->getRadius();
        cache->penetration = cache->sphereRadius - (cache->sphereMassCenter * cache->planeNormal - cache->planeMassCenter * cache->planeNormal);
    }

    template <>
    inline bool overlap<Plane, Sphere>(SimpleShape const *a, SimpleShape const *b, CacheBase *cacheBase)
    {
        auto cache = static_cast<Cache<Plane,Sphere>*>(cacheBase);
        return cache->penetration >= 0.0;
    }

    template <>
    inline Vector3 calculateContactNormal<Plane, Sphere>(SimpleShape const *a, SimpleShape const *b, CacheBase *cacheBase)
    {
        auto cache = static_cast<Cache<Plane,Sphere>*>(cacheBase);
        return cache->planeNormal.inverse();
    }

    template <>
    inline double calculatePenetration<Plane, Sphere>(SimpleShape const *a, SimpleShape const *b, CacheBase *cacheBase)
    {
        auto cache = static_cast<Cache<Plane,Sphere>*>(cacheBase);
        return cache->penetration;
    }

    // Plane, Box
    template <>
    inline void initialize<Plane, Box>(SimpleShape const *a, SimpleShape const *b, CacheBase *cacheBase)
    {
        auto plane = static_cast<Plane const *>(a);
        auto box = static_cast<Box const *>(b);
        auto cache = static_cast<Cache<Plane,Box>*>(cacheBase);

        cache->planeNormal = plane->getNormal();
        cache->planeDistance = plane->getCenterOfMass() * cache->planeNormal;

        box->getAxes(cache->boxAxes[0], cache->boxAxes[1], cache->boxAxes[2]);
        cache->boxMassCenter = box->getCenterOfMass();
        calculateBoxVertices(cache->boxAxes[0], cache->boxAxes[1], cache->boxAxes[2], cache->boxVertices);
        cache->boxFaces = { cache->boxAxes[0], cache->boxAxes[1], cache->boxAxes[2],
                            cache->boxAxes[0].inverse(), cache->boxAxes[1].inverse(), cache->boxAxes[2].inverse() };
        std::for_each(cache->boxVertices.begin(), cache->boxVertices.end(), [cache](auto& n) { n += cache->boxMassCenter; });
        std::transform(cache->boxVertices.begin(), cache->boxVertices.end(), cache->boxPenetrations.begin(),
            [cache](auto const & p) { return cache->planeDistance - p * cache->planeNormal; });
    }

    template <>
    inline bool overlap<Plane, Box>(SimpleShape const *a, SimpleShape const *b, CacheBase *cacheBase)
    {
        auto cache = static_cast<Cache<Plane,Box>*>(cacheBase);
        return *std::max_element(cache->boxPenetrations.begin(), cache->boxPenetrations.end()) >= 0;
    }

    template <>
    inline Vector3 calculateContactNormal<Plane, Box>(SimpleShape const *a, SimpleShape const *b, CacheBase *cacheBase)
    {
        auto cache = static_cast<Cache<Plane,Box>*>(cacheBase);

        std::transform(cache->boxFaces.begin(), cache->boxFaces.end(), cache->boxFaceDistances.begin(), 
            [cache](auto const & v) { return v * cache->planeNormal; });
        auto const minIndex = std::distance(cache->boxFaceDistances.begin(),
            std::min_element(cache->boxFaceDistances.begin(), cache->boxFaceDistances.end()));

        return cache->boxFaces[minIndex].unit();
    }

    template <>
    inline double calculatePenetration<Plane, Box>(SimpleShape const *a, SimpleShape const *b, CacheBase *cacheBase)
    {
        auto cache = static_cast<Cache<Plane,Box>*>(cacheBase);
        return (*std::max_element(cache->boxPenetrations.begin(), cache->boxPenetrations.end()));
    }

    // Sphere, Plane
    template <>
    inline void initialize<Sphere, Plane>(SimpleShape const *a, SimpleShape const *b, CacheBase *cacheBase)
    {
        auto cache = static_cast<Cache<Sphere, Plane>*>(cacheBase);
        initialize<Plane, Sphere>(b, a, &cache->psCache);
    }

    template <>
    inline bool overlap<Sphere, Plane>(SimpleShape const *a, SimpleShape const *b, CacheBase *cacheBase)
    {
        auto cache = static_cast<Cache<Sphere, Plane>*>(cacheBase);
        return overlap<Plane, Sphere>(b, a, &cache->psCache);
    }

    template <>
    inline Vector3 calculateContactNormal<Sphere, Plane>(SimpleShape const *a, SimpleShape const *b, CacheBase *cacheBase)
    {
        auto cache = static_cast<Cache<Sphere, Plane>*>(cacheBase);
        return calculateContactNormal<Plane, Sphere>(a, b, &cache->psCache).inverse();
    }

    template <>
    inline double calculatePenetration<Sphere, Plane>(SimpleShape const *a, SimpleShape const *b, CacheBase *cacheBase)
    {
        auto cache = static_cast<Cache<Sphere, Plane>*>(cacheBase);
        return calculatePenetration<Plane, Sphere>(b, a, &cache->psCache);
    }

    // Sphere, Sphere
    template <>
    inline void initialize<Sphere, Sphere>(SimpleShape const *a, SimpleShape const *b, CacheBase *cacheBase)
    {
        auto aSphere = static_cast<Sphere const *>(a);
        auto bSphere = static_cast<Sphere const *>(b);
        auto cache = static_cast<Cache<Sphere, Sphere>*>(cacheBase);
       
        cache->aRadius = aSphere->getRadius();
        cache->aMassCenter = aSphere->getCenterOfMass();
        cache->bRadius = bSphere->getRadius();
        cache->bMassCenter = bSphere->getCenterOfMass();
        cache->baVector = cache->aMassCenter - cache->bMassCenter;
        cache->radiusSum = cache->aRadius + cache->bRadius;
    }

    template <>
    inline bool overlap<Sphere, Sphere>(SimpleShape const *a, SimpleShape const *b, CacheBase *cacheBase)
    {
        auto cache = static_cast<Cache<Sphere, Sphere>*>(cacheBase);
        return pow(cache->radiusSum, 2) > cache->baVector.squareMagnitude();
    }

    template <>
    inline Vector3 calculateContactNormal<Sphere, Sphere>(SimpleShape const *a, SimpleShape const *b, CacheBase *cacheBase)
    {
        auto cache = static_cast<Cache<Sphere, Sphere>*>(cacheBase);
        return cache->baVector.unit();
    }

    template <>
    inline double calculatePenetration<Sphere, Sphere>(SimpleShape const *a, SimpleShape const *b, CacheBase *cacheBase)
    {
        auto cache = static_cast<Cache<Sphere, Sphere>*>(cacheBase);
        return (cache->radiusSum - cache->baVector.magnitude());
    }

    // Sphere, Box
    template <>
    inline void initialize<Sphere, Box>(SimpleShape const *a, SimpleShape const *b, CacheBase *cacheBase)
    {
        auto sphere = static_cast<Sphere const *>(a);
        auto box = static_cast<Box const *>(b);
        auto cache = static_cast<Cache<Sphere, Box>*>(cacheBase);

        cache->sphereMassCenter = sphere->getCenterOfMass();
        cache->sphereRadius = sphere->getRadius();

        cache->boxMassCenter = box->getCenterOfMass();
        box->getAxes(cache->boxAxes[0], cache->boxAxes[1], cache->boxAxes[2]);
        cache->separatingAxes = { cache->boxAxes[0].unit(), cache->boxAxes[1].unit(), cache->boxAxes[2].unit(), 
                                 (cache->sphereMassCenter - cache->boxMassCenter).unit() };
        calculateBoxVertices(cache->boxAxes[0], cache->boxAxes[1], cache->boxAxes[2], cache->boxVertices);
        std::for_each(cache->boxVertices.begin(), cache->boxVertices.end(), [cache](auto& n) { n += cache->boxMassCenter; });
        cache->boxNormals = { cache->boxAxes[0], cache->boxAxes[1], cache->boxAxes[2], 
                              cache->boxAxes[0].inverse(), cache->boxAxes[1].inverse(), cache->boxAxes[2].inverse() };
        cache->boxAxes = cache->boxNormals;
        for (uint32_t i = 0; i < cache->boxNormals.size(); ++i) {
            cache->boxFaces[i] = cache->boxNormals[i] + cache->boxMassCenter;
            cache->boxNormals[i].normalize();
            cache->boxFaceDistances[i] = (cache->boxFaces[i] - cache->sphereMassCenter).magnitude();
        }
    }

    template <>
    inline bool overlap<Sphere, Box>(SimpleShape const *a, SimpleShape const *b, CacheBase *cacheBase)
    {
        auto cache = static_cast<Cache<Sphere, Box>*>(cacheBase);

        for(auto const & axis : cache->separatingAxes)
        {
            projectAllVertices(axis, cache->boxVertices.begin(), cache->boxVertices.end(), cache->boxVerticesProjections.begin());
            std::sort(cache->boxVerticesProjections.begin(), cache->boxVerticesProjections.end());
            auto const boxMassCenterProjection = cache->boxMassCenter * axis;
            auto const sphereMassCenterProjection = cache->sphereMassCenter * axis;

            if (boxMassCenterProjection < sphereMassCenterProjection) {
                if (sphereMassCenterProjection - cache->boxVerticesProjections.back() > cache->sphereRadius) {
                    return false;
                }
            }
            else if (cache->boxVerticesProjections.front() > sphereMassCenterProjection + cache->sphereRadius) {
                return false;
            }
        }

        return true;
    }

    template <>
    inline Vector3 calculateContactNormal<Sphere, Box>(SimpleShape const *a, SimpleShape const *b, CacheBase *cacheBase)
    {
        auto cache = static_cast<Cache<Sphere, Box>*>(cacheBase);      

        auto minIt = std::min_element(cache->boxFaceDistances.begin(), cache->boxFaceDistances.end());
        auto minIndex = std::distance(cache->boxFaceDistances.begin(), minIt);
        cache->boxContactAxis = cache->boxAxes[minIndex];
        cache->boxContactNormal = cache->boxNormals[minIndex];
        cache->boxSphereVector = cache->boxMassCenter - cache->sphereMassCenter;
        cache->sphereContactNormal = cache->boxSphereVector.unit();

        return cache->boxContactNormal;
    }

    template <>
    inline double calculatePenetration<Sphere, Box>(SimpleShape const *a, SimpleShape const *b, CacheBase *cacheBase)
    {
        auto cache = static_cast<Cache<Sphere, Box>*>(cacheBase);
        double const penetration = std::abs(cache->sphereContactNormal * cache->boxContactAxis) - cache->boxSphereVector.magnitude() - cache->sphereRadius;

        return penetration;
    }

    // Box, Plane
    template <>
    inline void initialize<Box, Plane>(SimpleShape const *a, SimpleShape const *b, CacheBase *cacheBase)
    {
        auto cache = static_cast<Cache<Box, Plane>*>(cacheBase);
        initialize<Plane, Box>(b, a, &cache->pbCache);
    }

    template <>
    inline bool overlap<Box, Plane>(SimpleShape const *a, SimpleShape const *b, CacheBase *cacheBase)
    {
        auto cache = static_cast<Cache<Box, Plane>*>(cacheBase);
        return overlap<Plane, Box>(b, a, &cache->pbCache);
    }

    template <>
    inline Vector3 calculateContactNormal<Box, Plane>(SimpleShape const *a, SimpleShape const *b, CacheBase *cacheBase)
    {
        auto plane = static_cast<Plane const *>(b);
        return plane->getNormal();
    }

    template <>
    inline double calculatePenetration<Box, Plane>(SimpleShape const *a, SimpleShape const *b, CacheBase *cacheBase)
    {
        auto cache = static_cast<Cache<Box, Plane>*>(cacheBase);
        return calculatePenetration<Plane, Box>(b, a, &cache->pbCache);
    }

    // Box, Sphere
    template <>
    inline void initialize<Box, Sphere>(SimpleShape const *a, SimpleShape const *b, CacheBase *cacheBase)
    {
        auto cache = static_cast<Cache<Box, Sphere>*>(cacheBase);
        initialize<Sphere, Box>(b, a, &cache->sbCache);
    }

    template <>
    inline bool overlap<Box, Sphere>(SimpleShape const *a, SimpleShape const *b, CacheBase *cacheBase)
    {
        auto cache = static_cast<Cache<Box, Sphere>*>(cacheBase);
        return overlap<Sphere, Box>(b, a, &cache->sbCache);
    }

    template <>
    inline Vector3 calculateContactNormal<Box, Sphere>(SimpleShape const *a, SimpleShape const *b, CacheBase *cacheBase)
    {
        auto cache = static_cast<Cache<Box, Sphere>*>(cacheBase);
        calculateContactNormal<Sphere, Box>(b, a, &cache->sbCache);

        return cache->sbCache.sphereContactNormal;
    }

    template <>
    inline double calculatePenetration<Box, Sphere>(SimpleShape const *a, SimpleShape const *b, CacheBase *cacheBase)
    {
        auto cache = static_cast<Cache<Box, Sphere>*>(cacheBase);
        return calculatePenetration<Sphere, Box>(b, a, &cache->sbCache);
    }

    // Box, Box
    template <>
    inline void initialize<Box, Box>(SimpleShape const *a, SimpleShape const *b, CacheBase *cacheBase)
    {
        auto aBox = static_cast<Box const *>(a);
        auto bBox = static_cast<Box const *>(b);
        auto cache = static_cast<Cache<Box, Box>*>(cacheBase);

        cache->aMassCenter = aBox->getCenterOfMass();
        aBox->getAxes(cache->aBoxAxes[0], cache->aBoxAxes[1], cache->aBoxAxes[2]);
        cache->aBoxAxes = { cache->aBoxAxes[0], cache->aBoxAxes[1], cache->aBoxAxes[2], 
                            cache->aBoxAxes[0].inverse(), cache->aBoxAxes[1].inverse(), cache->aBoxAxes[2].inverse() };
        calculateBoxVertices(cache->aBoxAxes[0], cache->aBoxAxes[1], cache->aBoxAxes[2], cache->aBoxVertices);
        std::for_each(cache->aBoxVertices.begin(), cache->aBoxVertices.end(), [cache](auto& v) { v += cache->aMassCenter; });
        std::transform(cache->aBoxAxes.begin(), cache->aBoxAxes.end(), cache->aBoxFaces.begin(), 
                      [cache](Vector3 const & v) { return v += cache->aMassCenter; });

        cache->bMassCenter = bBox->getCenterOfMass();
        bBox->getAxes(cache->bBoxAxes[0], cache->bBoxAxes[1], cache->bBoxAxes[2]);
        cache->bBoxAxes = { cache->bBoxAxes[0], cache->bBoxAxes[1], cache->bBoxAxes[2], 
                            cache->bBoxAxes[0].inverse(), cache->bBoxAxes[1].inverse(), cache->bBoxAxes[2].inverse() };
        calculateBoxVertices(cache->bBoxAxes[0], cache->bBoxAxes[1], cache->bBoxAxes[2], cache->bBoxVertices);
        std::for_each(cache->bBoxVertices.begin(), cache->bBoxVertices.end(), [cache](auto& v) { v += cache->bMassCenter; });
        std::transform(cache->bBoxAxes.begin(), cache->bBoxAxes.end(), cache->bBoxFaces.begin(),
            [cache](Vector3 const & v) { return v += cache->bMassCenter; });
        
        cache->separatingAxes = { cache->aBoxAxes[0].unit(), cache->aBoxAxes[1].unit(), cache->aBoxAxes[2].unit(),
                                  cache->bBoxAxes[0].unit(), cache->bBoxAxes[1].unit(), cache->bBoxAxes[2].unit() };
        calculateSeparatingAxes(cache->aBoxAxes.begin(), cache->aBoxAxes.begin() + 3,
                                cache->bBoxAxes.begin(), cache->bBoxAxes.begin() + 3,
                                back_inserter(cache->separatingAxes));

        cache->aBoxVerticesDistances = {};
        cache->bBoxVerticesDistances = {};
    }

    template <>
    inline bool overlap<Box, Box>(SimpleShape const *a, SimpleShape const *b, CacheBase *cacheBase)
    {
        auto cache = static_cast<Cache<Box, Box>*>(cacheBase);

        for (auto axis : cache->separatingAxes) {
            projectAllVertices(axis, cache->aBoxVertices.begin(), cache->aBoxVertices.end(), cache->aBoxVerticesDistances.begin());
            projectAllVertices(axis, cache->bBoxVertices.begin(), cache->bBoxVertices.end(), cache->bBoxVerticesDistances.begin());
            std::sort(cache->aBoxVerticesDistances.begin(), cache->aBoxVerticesDistances.end());
            std::sort(cache->bBoxVerticesDistances.begin(), cache->bBoxVerticesDistances.end());

            if (cache->aBoxVerticesDistances.back() < cache->bBoxVerticesDistances.back()) {
                if (cache->aBoxVerticesDistances.back() < cache->bBoxVerticesDistances.front()) {
                    return false;
                }
            }
            else {
                if (cache->bBoxVerticesDistances.back() < cache->aBoxVerticesDistances.front()) {
                    return false;
                }
            }
        }

        return true;
    }

    template <>
    inline Vector3 calculateContactNormal<Box, Box>(SimpleShape const *a, SimpleShape const *b, CacheBase *cacheBase)
    {
        auto cache = static_cast<Cache<Box, Box>*>(cacheBase);

        std::array<double, 6> distances;
        for (uint32_t i = 0; i < distances.size(); ++i) {
            distances[i] = (cache->aMassCenter - cache->bBoxFaces[i]).magnitude();
        }

        auto const minIt = std::min_element(distances.begin(), distances.end());
        auto const minIndex = std::distance(distances.begin(), minIt);
        cache->contactNormal = cache->bBoxAxes[minIndex].unit();

        return cache->contactNormal;
    }

    template <>
    inline double calculatePenetration<Box, Box>(SimpleShape const *a, SimpleShape const *b, CacheBase *cacheBase)
    {       
        auto cache = static_cast<Cache<Box, Box>*>(cacheBase);      

        projectAllVertices(cache->contactNormal, cache->aBoxVertices.begin(),
                           cache->aBoxVertices.end(), cache->aBoxVerticesDistances.begin());
        projectAllVertices(cache->contactNormal, cache->bBoxVertices.begin(),
                           cache->bBoxVertices.end(), cache->bBoxVerticesDistances.begin());

        double bMaxVertexDistance = *std::max_element(cache->bBoxVerticesDistances.begin(), cache->bBoxVerticesDistances.end());
        double aMinVertexDistance = *std::min_element(cache->aBoxVerticesDistances.begin(), cache->aBoxVerticesDistances.end());
        cache->penetration = bMaxVertexDistance - aMinVertexDistance;

        return cache->penetration;
    }

} // namespace IntersectionQuery

    // General intersection
    using ShapeTypePair = std::pair<SimpleShapeType, SimpleShapeType>;

    size_t shapeTypePairHash(ShapeTypePair const & p);

    class IntersectionQuery {
    private:
        std::unordered_map<ShapeTypePair,
            std::unique_ptr<intersection::CacheBase>,
            std::function<size_t(ShapeTypePair const & p)> >
            intersectionCaches;

        std::unordered_map<ShapeTypePair,
            std::function<void(SimpleShape const *, SimpleShape const *, intersection::CacheBase*)>,
            std::function<size_t(ShapeTypePair const & p)> >
            initializeFunctors;

        std::unordered_map<ShapeTypePair,
            std::function<bool(SimpleShape const *, SimpleShape const *, intersection::CacheBase*)>,
            std::function<size_t(ShapeTypePair const & p)> >
            overlapFunctors;

        std::unordered_map<ShapeTypePair,
            std::function<Vector3(SimpleShape const *, SimpleShape const *, intersection::CacheBase*)>,
            std::function<size_t(ShapeTypePair const & p)> >
            calculateContactNormalFunctors;

        std::unordered_map<ShapeTypePair,
            std::function<double(SimpleShape const *, SimpleShape const *, intersection::CacheBase*)>,
            std::function<size_t(ShapeTypePair const & p)> >
            calculatePenetrationFunctors;

    public:
        IntersectionQuery()
            : intersectionCaches(11, &shapeTypePairHash)
            , initializeFunctors(11, &shapeTypePairHash)
            , overlapFunctors(11, &shapeTypePairHash)
            , calculateContactNormalFunctors(11, &shapeTypePairHash)
            , calculatePenetrationFunctors(11, &shapeTypePairHash)
        {            
            intersectionCaches[std::make_pair(SimpleShapeType::PLANE, SimpleShapeType::PLANE)] 
                = std::make_unique<intersection::Cache<Plane, Plane>>();
            intersectionCaches[std::make_pair(SimpleShapeType::PLANE, SimpleShapeType::SPHERE)] 
                = std::make_unique<intersection::Cache<Plane, Sphere>>();
            intersectionCaches[std::make_pair(SimpleShapeType::PLANE, SimpleShapeType::BOX)] 
                = std::make_unique<intersection::Cache<Plane, Box>>();
            intersectionCaches[std::make_pair(SimpleShapeType::SPHERE, SimpleShapeType::PLANE)]
                = std::make_unique<intersection::Cache<Sphere, Plane>>();
            intersectionCaches[std::make_pair(SimpleShapeType::SPHERE, SimpleShapeType::SPHERE)]
                = std::make_unique<intersection::Cache<Sphere, Sphere>>();
            intersectionCaches[std::make_pair(SimpleShapeType::SPHERE, SimpleShapeType::BOX)]
                = std::make_unique<intersection::Cache<Sphere, Box>>();
            intersectionCaches[std::make_pair(SimpleShapeType::BOX, SimpleShapeType::PLANE)]
                = std::make_unique<intersection::Cache<Box, Plane>>();
            intersectionCaches[std::make_pair(SimpleShapeType::BOX, SimpleShapeType::SPHERE)]
                = std::make_unique<intersection::Cache<Box, Sphere>>();
            intersectionCaches[std::make_pair(SimpleShapeType::BOX, SimpleShapeType::BOX)]
                = std::make_unique<intersection::Cache<Box, Box>>();

            initializeFunctors[std::make_pair(SimpleShapeType::PLANE, SimpleShapeType::PLANE)]
                = &intersection::initialize<Plane, Plane>;
            initializeFunctors[std::make_pair(SimpleShapeType::PLANE, SimpleShapeType::SPHERE)]
                = &intersection::initialize<Plane, Sphere>;
            initializeFunctors[std::make_pair(SimpleShapeType::PLANE, SimpleShapeType::BOX)]
                = &intersection::initialize<Plane, Box>;
            initializeFunctors[std::make_pair(SimpleShapeType::SPHERE, SimpleShapeType::PLANE)]
                = &intersection::initialize<Sphere, Plane>;
            initializeFunctors[std::make_pair(SimpleShapeType::SPHERE, SimpleShapeType::SPHERE)]
                = &intersection::initialize<Sphere, Sphere>;
            initializeFunctors[std::make_pair(SimpleShapeType::SPHERE, SimpleShapeType::BOX)]
                = &intersection::initialize<Sphere, Box>;
            initializeFunctors[std::make_pair(SimpleShapeType::BOX, SimpleShapeType::PLANE)]
                = &intersection::initialize<Box, Plane>;
            initializeFunctors[std::make_pair(SimpleShapeType::BOX, SimpleShapeType::SPHERE)]
                = &intersection::initialize<Box, Sphere>;
            initializeFunctors[std::make_pair(SimpleShapeType::BOX, SimpleShapeType::BOX)]
                = &intersection::initialize<Box, Box>;

            overlapFunctors[std::make_pair(SimpleShapeType::PLANE, SimpleShapeType::PLANE)]
                = &intersection::overlap<Plane, Plane>;
            overlapFunctors[std::make_pair(SimpleShapeType::PLANE, SimpleShapeType::SPHERE)]
                = &intersection::overlap<Plane, Sphere>;
            overlapFunctors[std::make_pair(SimpleShapeType::PLANE, SimpleShapeType::BOX)]
                = &intersection::overlap<Plane, Box>;
            overlapFunctors[std::make_pair(SimpleShapeType::SPHERE, SimpleShapeType::PLANE)]
                = &intersection::overlap<Sphere, Plane>;
            overlapFunctors[std::make_pair(SimpleShapeType::SPHERE, SimpleShapeType::SPHERE)]
                = &intersection::overlap<Sphere, Sphere>;
            overlapFunctors[std::make_pair(SimpleShapeType::SPHERE, SimpleShapeType::BOX)]
                = &intersection::overlap<Sphere, Box>;
            overlapFunctors[std::make_pair(SimpleShapeType::BOX, SimpleShapeType::PLANE)]
                = &intersection::overlap<Box, Plane>;
            overlapFunctors[std::make_pair(SimpleShapeType::BOX, SimpleShapeType::SPHERE)]
                = &intersection::overlap<Box, Sphere>;
            overlapFunctors[std::make_pair(SimpleShapeType::BOX, SimpleShapeType::BOX)]
                = &intersection::overlap<Box, Box>;

            calculateContactNormalFunctors[std::make_pair(SimpleShapeType::PLANE, SimpleShapeType::PLANE)]
                = &intersection::calculateContactNormal<Plane, Plane>;
            calculateContactNormalFunctors[std::make_pair(SimpleShapeType::PLANE, SimpleShapeType::SPHERE)]
                = &intersection::calculateContactNormal<Plane, Sphere>;
            calculateContactNormalFunctors[std::make_pair(SimpleShapeType::PLANE, SimpleShapeType::BOX)]
                = &intersection::calculateContactNormal<Plane, Box>;
            calculateContactNormalFunctors[std::make_pair(SimpleShapeType::SPHERE, SimpleShapeType::PLANE)]
                = &intersection::calculateContactNormal<Sphere, Plane>;
            calculateContactNormalFunctors[std::make_pair(SimpleShapeType::SPHERE, SimpleShapeType::SPHERE)]
                = &intersection::calculateContactNormal<Sphere, Sphere>;
            calculateContactNormalFunctors[std::make_pair(SimpleShapeType::SPHERE, SimpleShapeType::BOX)]
                = &intersection::calculateContactNormal<Sphere, Box>;
            calculateContactNormalFunctors[std::make_pair(SimpleShapeType::BOX, SimpleShapeType::PLANE)]
                = &intersection::calculateContactNormal<Box, Plane>;
            calculateContactNormalFunctors[std::make_pair(SimpleShapeType::BOX, SimpleShapeType::SPHERE)]
                = &intersection::calculateContactNormal<Box, Sphere>;
            calculateContactNormalFunctors[std::make_pair(SimpleShapeType::BOX, SimpleShapeType::BOX)]
                = &intersection::calculateContactNormal<Box, Box>;

            calculatePenetrationFunctors[std::make_pair(SimpleShapeType::PLANE, SimpleShapeType::PLANE)]
                = &intersection::calculatePenetration<Plane, Plane>;
            calculatePenetrationFunctors[std::make_pair(SimpleShapeType::PLANE, SimpleShapeType::SPHERE)]
                = &intersection::calculatePenetration<Plane, Sphere>;
            calculatePenetrationFunctors[std::make_pair(SimpleShapeType::PLANE, SimpleShapeType::BOX)]
                = &intersection::calculatePenetration<Plane, Box>;
            calculatePenetrationFunctors[std::make_pair(SimpleShapeType::SPHERE, SimpleShapeType::PLANE)]
                = &intersection::calculatePenetration<Sphere, Plane>;
            calculatePenetrationFunctors[std::make_pair(SimpleShapeType::SPHERE, SimpleShapeType::SPHERE)]
                = &intersection::calculatePenetration<Sphere, Sphere>;
            calculatePenetrationFunctors[std::make_pair(SimpleShapeType::SPHERE, SimpleShapeType::BOX)]
                = &intersection::calculatePenetration<Sphere, Box>;
            calculatePenetrationFunctors[std::make_pair(SimpleShapeType::BOX, SimpleShapeType::PLANE)]
                = &intersection::calculatePenetration<Box, Plane>;
            calculatePenetrationFunctors[std::make_pair(SimpleShapeType::BOX, SimpleShapeType::SPHERE)]
                = &intersection::calculatePenetration<Box, Sphere>;
            calculatePenetrationFunctors[std::make_pair(SimpleShapeType::BOX, SimpleShapeType::BOX)]
                = &intersection::calculatePenetration<Box, Box>;
        }

        void initialize(SimpleShape const * a, SimpleShape const * b)
        {
            initializeFunctors[std::make_pair(a->type, b->type)](
                a, b, intersectionCaches[std::make_pair(a->type, b->type)].get());
        }

        bool overlap(SimpleShape const* a, SimpleShape const * b)
        {
            return overlapFunctors[std::make_pair(a->type, b->type)](
                a, b, intersectionCaches[std::make_pair(a->type, b->type)].get());
        }

        Vector3 calculateContactNormal(SimpleShape const * a, SimpleShape const * b)
        {
            return calculateContactNormalFunctors[std::make_pair(a->type, b->type)](
                a, b, intersectionCaches[std::make_pair(a->type, b->type)].get());
        }

        double calculatePenetration(SimpleShape const * a, SimpleShape const * b)
        {
            return calculatePenetrationFunctors[std::make_pair(a->type, b->type)](
                a, b, intersectionCaches[std::make_pair(a->type, b->type)].get());
        }
    };

} // namespace geometry
} // namespace pegasus

#endif // PEGASUS_GEOMETRY_HPP
