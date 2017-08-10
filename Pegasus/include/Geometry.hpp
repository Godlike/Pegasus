/*
* Copyright (C) 2017 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#ifndef PEGASUS_GEOMETRY_HPP
#define PEGASUS_GEOMETRY_HPP

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/norm.hpp>
#include <glm/gtx/optimum_pow.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/glm.hpp>

#include <algorithm>
#include <utility>
#include <limits>
#include <memory>
#include <array>
#include <vector>
#include <unordered_map>

#include "Pegasus/include/Math.hpp"

namespace pegasus
{
namespace geometry
{
/**
 * @brief Base shape class
 */
class Shape
{
public:
    glm::dvec3 centerOfMass;

    Shape() = default;

    explicit Shape(glm::dvec3 const& centerOfMass);
};

/**
 * @brief Generic class for representing shapes that could be described as a simple parametric or quadric surface
 */
class SimpleShape : public Shape
{
public:
    enum class Type : uint8_t
    {
        RAY,
        PLANE,
        TRIANGLE,
        SPHERE,
        CONE,
        CYLINDER,
        CAPSULE,
        BOX,
        NONE
    };

    Type type = Type::NONE;

    SimpleShape() = default;

    explicit SimpleShape(Type type)
        : type(type)
    {
    }

    SimpleShape(glm::dvec3 const& centerOfMass, Type type);
};

/** Ray data storage class */
class Ray : public SimpleShape
{
public:
    glm::dvec3 direction;

    Ray();

    Ray(glm::dvec3 const& centerOfMass, glm::dvec3 const& normal);   
};

/** Plane data storage class */
class Plane : public SimpleShape
{
public:
    glm::dvec3 normal;

    Plane();

    Plane(glm::dvec3 const& centerOfMass, glm::dvec3 const& normal);
};

/** Triangle data storage class */
class Triangle : public SimpleShape
{
public:
    glm::dvec3 aVertex;
    glm::dvec3 bVertex;
    glm::dvec3 cVertex;
    glm::dvec3 normal;

    Triangle();

    Triangle(glm::dvec3 const& centerOfMass, glm::dvec3 const& a, glm::dvec3 const& b, glm::dvec3 const& c);

    /** Calculates normal from member vertices and writes it to the normal member field */
    void CalculateNormal();
};

/** Sphere data storage class */
class Sphere : public SimpleShape
{
public:
    double radius;

    Sphere();

    Sphere(glm::dvec3 const& centerOfMass, double r);    
};

/** Cone data storage class */
class Cone : public SimpleShape
{
public:
    glm::dvec3 apex;
    double radius;

    Cone();

    Cone(glm::dvec3 const& centerOfMass, glm::dvec3 const& a, double r);
};

/** Capsule data storage class */
class Capsule : public SimpleShape
{
public:
    glm::dvec3 halfHeight;
    double radius;

    Capsule();

    Capsule(glm::dvec3 const& centerOfMass, glm::dvec3 const& halfHeight, double r);   
};

class Cylinder : public SimpleShape
{
public:
    glm::dvec3 halfHeight;
    double radius;

    Cylinder();

    Cylinder(glm::dvec3 const& centerOfMass, glm::dvec3 const& halfHeight, double r);
};

/** Box data storage class */
class Box : public SimpleShape
{
public:
    glm::dvec3 iAxis;
    glm::dvec3 jAxis;
    glm::dvec3 kAxis;

    Box();

    Box(glm::dvec3 const& centerOfMass, glm::dvec3 const& i, glm::dvec3 const& j, glm::dvec3 const& k);    
};

namespace intersection
{
/** Base cache data structure for shapes intersection queries */
struct IntersectionCacheBase
{
};

/** Specialized cache data structure for shapes intersection queries */
template <typename ShapeA, typename ShapeB>
struct IntersectionCache : IntersectionCacheBase
{
};

template <>
struct IntersectionCache<Ray, Ray> : IntersectionCacheBase
{
    glm::dvec3 aRayNormal;
    glm::dvec3 aRayPoint;
    glm::dvec3 bRayNormal;
    glm::dvec3 bRayPoint;
    double denominator;
    glm::dvec3 aClosestApproach;
    glm::dvec3 bClosestApproach;
};

template <>
struct IntersectionCache<Ray, Plane> : IntersectionCacheBase
{
    glm::dvec3 rayNormal;
    glm::dvec3 rayOrigin;
    glm::dvec3 planeNormal;
    glm::dvec3 planePoint;
    math::HyperPlane hp;
    glm::dvec3 intersection;
};

template <>
struct IntersectionCache<Ray, Sphere> : IntersectionCacheBase
{
    glm::dvec3 rayNormal;
    glm::dvec3 rayOrigin;
    glm::dvec3 sphereCenter;
    glm::dvec3 sphereContactNormal;
    double sphereRadius;
    bool intersection;
    double penetration;
    glm::dvec3 inPoint;
    glm::dvec3 outPoint;
};

template <>
struct IntersectionCache<Ray, Box> : IntersectionCacheBase
{
    glm::dvec3 rayNormalBoxSpace;
    glm::dvec3 rayOriginBoxSpace;
    glm::dvec3 boxMassCenter;
    glm::dmat3 boxModelMatrix;
    glm::dmat3 boxModelMatrixInverse;
    glm::dmat3 boxAxesWorldSpace;
    glm::dmat3 boxAxesModelSpace;
    glm::dvec3 boxMaxPoint;
    glm::dvec3 boxMinPoint;
    glm::dvec3 inPoint;
    glm::dvec3 outPoint;
    glm::dvec3 boxContactNormal;
    double rayIntersectionFactorMin;
    double rayIntersectionFactorMax;
    double penetrationDepth;
};

template <>
struct IntersectionCache<Plane, Ray> : IntersectionCacheBase
{
    IntersectionCache<Ray, Plane> rpCache;
};

template <>
struct IntersectionCache<Plane, Plane> : IntersectionCacheBase
{
    glm::dvec3 aNormal;
    glm::dvec3 bNormal;
    glm::dvec3 crossProduct;
};

template <>
struct IntersectionCache<Plane, Sphere> : IntersectionCacheBase
{
    glm::dvec3 planeMassCenter;
    glm::dvec3 planeNormal;
    glm::dvec3 sphereMassCenter;
    double sphereRadius;
    double penetration;
};

template <>
struct IntersectionCache<Plane, Box> : IntersectionCacheBase
{
    glm::dvec3 boxMassCenter;
    std::array<glm::dvec3, 3> boxAxes;
    std::array<glm::dvec3, 8> boxVertices;
    std::array<glm::dvec3, 6> boxFaces;
    std::array<double, 6> boxFaceDistances;
    std::array<double, 8> boxPenetrations;
    glm::dvec3 planeNormal;
    double planeDistance;
};

template <>
struct IntersectionCache<Sphere, Ray> : IntersectionCacheBase
{
    IntersectionCache<Ray, Sphere> rsCache;
};

template <>
struct IntersectionCache<Sphere, Plane> : IntersectionCacheBase
{
    IntersectionCache<Plane, Sphere> psCache;
};

template <>
struct IntersectionCache<Sphere, Sphere> : IntersectionCacheBase
{
    glm::dvec3 bMassCenter;
    glm::dvec3 baVector;
    double bRadius;
    glm::dvec3 aMassCenter;
    double aRadius;
    double radiusSum;
};

template <>
struct IntersectionCache<Sphere, Box> : IntersectionCacheBase
{
    glm::dvec3 boxMassCenter;
    std::array<glm::dvec3, 6> boxAxes;
    std::array<glm::dvec3, 6> boxNormals;
    std::array<glm::dvec3, 6> boxFaces;
    std::array<double, 6> boxFaceDistances;
    std::array<glm::dvec3, 8> boxVertices;
    std::array<double, 8> boxVerticesProjections;
    std::array<glm::dvec3, 4> separatingAxes;
    glm::dvec3 boxContactNormal;
    glm::dvec3 boxContactAxis;
    glm::dvec3 boxSphereVector;
    glm::dvec3 sphereContactNormal;
    glm::dvec3 sphereMassCenter;
    double sphereRadius;
    glm::dvec3 boxContactPoint;
    glm::dvec3 sphereContactPoint;
};

template <>
struct IntersectionCache<Box, Ray> : IntersectionCacheBase
{
    IntersectionCache<Ray, Box> rbCache;
};

template <>
struct IntersectionCache<Box, Plane> : IntersectionCacheBase
{
    IntersectionCache<Plane, Box> pbCache;
    glm::dvec3 planeMassCenter;
    glm::dvec3 boxContactNormal;
};

template <>
struct IntersectionCache<Box, Sphere> : IntersectionCacheBase
{
    IntersectionCache<Sphere, Box> sbCache;
};

template <>
struct IntersectionCache<Box, Box> : IntersectionCacheBase
{
    glm::dvec3 aMassCenter;
    glm::dvec3 bMassCenter;
    std::array<glm::dvec3, 8> aBoxVertices, bBoxVertices;
    std::array<glm::dvec3, 6> aBoxAxes, bBoxAxes;
    std::array<glm::dvec3, 6> aBoxFaces, bBoxFaces;
    std::vector<glm::dvec3> separatingAxes;
    std::array<double, 6> aBoxFaceDistances, bBoxFaceDistances;
    std::array<double, 8> aBoxVerticesDistances, bBoxVerticesDistances;
    glm::dvec3 contactNormal;
    double penetration = 0;
};

/**
 * @brief Performs initial calculations and writes it into the cache object
 * 
 * Required to be called before any other proximity query function, otherwise results are undefined
 * @tparam ShapeA SimpleShape or SimpleShape derived object
 * @tparam ShapeB SimpleShape or SimpleShape derived object
 * @param[in] a pointer to the ShapeA type object
 * @param[in] b pointer to the ShapeB type object
 * @param[out] cacheBase pointer to the IntersectionCacheBase or IntersectionCacheBase derived object
 */
template <typename ShapeA, typename ShapeB>
void Initialize(SimpleShape const* a, SimpleShape const* b, IntersectionCacheBase* cacheBase);

/**
 * @brief Performs intersection test using shapes and cache data and returns true if shapes are intersecting
 * 
 * Must be called strictly after the corresponding Initialize function call, otherwise result is undefined
 * @tparam ShapeA SimpleShape or SimpleShape derived object
 * @tparam ShapeB SimpleShape or SimpleShape derived object
 * @param[in] a pointer to the ShapeA type object
 * @param[in] b pointer to the ShapeB type object
 * @param[in, out] cacheBase pointer to the IntersectionCacheBase or IntersectionCacheBase derived object 
 * @return @c true if there is intersection, @c false otherwise
 */
template <typename ShapeA, typename ShapeB>
bool CalculateIntersection(SimpleShape const* a, SimpleShape const* b, IntersectionCacheBase* cacheBase);

/**
 * @brief Calculates surface contact normal of b shape using shapes and cache data and returns it
 * 
 * Must be called strictly after the corresponding CalculateIntersection function call, otherwise result is undefined
 * @tparam ShapeA SimpleShape or SimpleShape derived object
 * @tparam ShapeB SimpleShape or SimpleShape derived object
 * @param[in] a pointer to the ShapeA type object
 * @param[in] b pointer to the ShapeB type object
 * @param[in, out] cacheBase pointer to the IntersectionCacheBase or IntersectionCacheBase derived object 
 * @return surface contact normal of the b object
 */
template <typename ShapeA, typename ShapeB>
glm::dvec3 CalculateContactNormal(SimpleShape const* a, SimpleShape const* b, IntersectionCacheBase* cacheBase);

/**
 * @brief Calculates penetration depth using shapes and cache data and returns it
 * 
 * Must be called strictly after the corresponding CalculateContactNormal function call, 
 * otherwise result is undefined
 * @tparam ShapeA SimpleShape or SimpleShape derived object
 * @tparam ShapeB SimpleShape or SimpleShape derived object
 * @param[in] a pointer to the ShapeA type object
 * @param[in] b pointer to the ShapeB type object
 * @param[in, out] cacheBase pointer to the IntersectionCacheBase or IntersectionCacheBase derived object 
 * @return penetration depth
 */
template <typename ShapeA, typename ShapeB>
double CalculatePenetration(SimpleShape const* a, SimpleShape const* b, IntersectionCacheBase* cacheBase);

/** Ray, Ray Initialize specialization */
template <>
inline void Initialize<Ray, Ray>(SimpleShape const* a, SimpleShape const* b, IntersectionCacheBase* cacheBase)
{
    auto cache = static_cast<IntersectionCache<Ray, Ray>*>(cacheBase);
    auto aRay = static_cast<Ray const*>(a);
    auto bRay = static_cast<Ray const*>(b);

    cache->aRayPoint = aRay->centerOfMass;
    cache->aRayNormal = aRay->direction;
    cache->bRayPoint = bRay->centerOfMass;
    cache->bRayNormal = bRay->direction;
}

/** Ray, Ray CalculateIntersection specialization */
template <>
inline bool CalculateIntersection<Ray, Ray>(SimpleShape const* a, SimpleShape const* b, IntersectionCacheBase* cacheBase)
{
    auto cache = static_cast<IntersectionCache<Ray, Ray>*>(cacheBase);

    glm::dmat3 const aNominator{
        cache->bRayPoint - cache->aRayPoint,
        cache->bRayNormal,
        glm::cross(cache->aRayPoint, cache->bRayPoint)
    };
    glm::dmat3 const bNominator{
        cache->bRayPoint - cache->aRayPoint,
        cache->aRayNormal,
        glm::cross(cache->aRayPoint, cache->bRayPoint)
    };
    cache->denominator = glm::pow2(glm::length(aNominator[2]));

    if (cache->denominator == 0.0)
    {
        return false;
    }

    cache->aClosestApproach =
        cache->aRayPoint + (aNominator / cache->denominator) * cache->aRayNormal;
    cache->bClosestApproach =
        cache->bRayPoint + (bNominator / cache->denominator) * cache->bRayNormal;

    return glm::length2(cache->aClosestApproach - cache->bClosestApproach) < 1e-10;
}

/** Ray, Ray CalculateContactNormal specialization */
template <>
inline glm::dvec3 CalculateContactNormal<Ray, Ray>(SimpleShape const* a, SimpleShape const* b, IntersectionCacheBase* cacheBase)
{
    auto cache = static_cast<IntersectionCache<Ray, Ray>*>(cacheBase);

    return glm::normalize(glm::cross(glm::cross(cache->bRayNormal, cache->aRayNormal), cache->bRayNormal));
}

/** Ray, Ray CalculatePenetration specialization */
template <>
inline double CalculatePenetration<Ray, Ray>(SimpleShape const* a, SimpleShape const* b, IntersectionCacheBase* cacheBase)
{
    return 0;
}

/** Ray, Plane Initialize specialization */
template <>
inline void Initialize<Ray, Plane>(SimpleShape const* a, SimpleShape const* b, IntersectionCacheBase* cacheBase)
{
    auto cache = static_cast<IntersectionCache<Ray, Plane>*>(cacheBase);
    auto ray = static_cast<Ray const*>(a);
    auto plane = static_cast<Plane const*>(b);

    cache->rayNormal = ray->direction;
    cache->rayOrigin = ray->centerOfMass;
    cache->planePoint = plane->centerOfMass;
    cache->planeNormal = plane->normal;
    cache->hp = math::HyperPlane{cache->planeNormal, cache->planePoint};
}

/** Ray, Plane CalculateIntersection specialization */
template <>
inline bool CalculateIntersection<Ray, Plane>(SimpleShape const* a, SimpleShape const* b, IntersectionCacheBase* cacheBase)
{
    auto cache = static_cast<IntersectionCache<Ray, Plane>*>(cacheBase);

    return cache->hp.RayIntersection(cache->rayNormal, cache->rayOrigin, cache->intersection);
}

/** Ray, Plane CalculateContactNormal specialization */
template <>
inline glm::dvec3 CalculateContactNormal<Ray, Plane>(SimpleShape const* a, SimpleShape const* b, IntersectionCacheBase* cacheBase)
{
    auto cache = static_cast<IntersectionCache<Ray, Plane>*>(cacheBase);

    return cache->rayNormal;
}

/** Ray, Plane CalculatePenetration specialization */
template <>
inline double CalculatePenetration<Ray, Plane>(SimpleShape const* a, SimpleShape const* b, IntersectionCacheBase* cacheBase)
{
    return std::numeric_limits<double>::max();
}

/** Ray, Sphere Initialize specialization */
template <>
inline void Initialize<Ray, Sphere>(SimpleShape const* a, SimpleShape const* b, IntersectionCacheBase* cacheBase)
{
    auto cache = static_cast<IntersectionCache<Ray, Sphere>*>(cacheBase);
    auto ray = static_cast<Ray const*>(a);
    auto sphere = static_cast<Sphere const*>(b);

    cache->rayOrigin = ray->centerOfMass;
    cache->rayNormal = ray->direction;
    cache->sphereRadius = sphere->radius;
    cache->sphereCenter = sphere->centerOfMass;

    glm::dvec3 const L = cache->sphereCenter - cache->rayOrigin;
    double const tc = glm::dot(L, cache->rayNormal);
    double const dd = glm::dot(L, L) - tc * tc;
    cache->intersection = dd > 0;

    if (cache->intersection)
    {
        double const d = glm::sqrt(dd);
        double const t1c = glm::sqrt(glm::pow2(cache->sphereRadius) - d * d);
        double const t1 = tc - t1c;
        double const t2 = tc + t1c;
        cache->intersection = (cache->sphereRadius - d) >= 0.0;
        cache->inPoint = cache->rayOrigin + cache->rayNormal * t1;
        cache->outPoint = cache->rayOrigin + cache->rayNormal * t2;
        cache->penetration = glm::length(cache->inPoint - cache->outPoint);
        cache->sphereContactNormal = glm::normalize(cache->inPoint - cache->sphereCenter);
    }
}

/** Ray, Sphere CalculateIntersection specialization */
template <>
inline bool CalculateIntersection<Ray, Sphere>(SimpleShape const* a, SimpleShape const* b, IntersectionCacheBase* cacheBase)
{
    auto cache = static_cast<IntersectionCache<Ray, Sphere>*>(cacheBase);

    return cache->intersection;
}

/** Ray, Sphere CalculateContactNormal specialization */
template <>
inline glm::dvec3 CalculateContactNormal<Ray, Sphere>(SimpleShape const* a, SimpleShape const* b, IntersectionCacheBase* cacheBase)
{
    auto cache = static_cast<IntersectionCache<Ray, Sphere>*>(cacheBase);

    return cache->sphereContactNormal;
}

/** Ray, Sphere CalculatePenetration specialization */
template <>
inline double CalculatePenetration<Ray, Sphere>(SimpleShape const* a, SimpleShape const* b, IntersectionCacheBase* cacheBase)
{
    auto cache = static_cast<IntersectionCache<Ray, Sphere>*>(cacheBase);

    return cache->penetration;
}

/** Ray, Box Initialize specialization */
template <>
inline void Initialize<Ray, Box>(SimpleShape const* a, SimpleShape const* b, IntersectionCacheBase* cacheBase)
{
    auto cache = static_cast<IntersectionCache<Ray, Box>*>(cacheBase);
    auto ray = static_cast<Ray const*>(a);
    auto box = static_cast<Box const*>(b);

    cache->boxAxesWorldSpace = { box->iAxis, box->jAxis, box->kAxis };
    cache->boxMassCenter = box->centerOfMass;
    cache->boxModelMatrix = glm::dmat3{
        glm::normalize(cache->boxAxesWorldSpace[0]),
        glm::normalize(cache->boxAxesWorldSpace[1]),
        glm::normalize(cache->boxAxesWorldSpace[2])
    };
    cache->boxModelMatrixInverse = glm::inverse(cache->boxModelMatrix);
    cache->boxAxesModelSpace = glm::dmat3{
        cache->boxModelMatrixInverse * cache->boxAxesWorldSpace[0],
        cache->boxModelMatrixInverse * cache->boxAxesWorldSpace[1],
        cache->boxModelMatrixInverse * cache->boxAxesWorldSpace[2]
    };
    cache->rayNormalBoxSpace = cache->boxModelMatrixInverse * ray->direction;
    cache->rayOriginBoxSpace = cache->boxModelMatrixInverse * (ray->centerOfMass - cache->boxMassCenter);

    auto const findMaxAbs = [](double a, double b) { return std::abs(a) < std::abs(b); };

    cache->boxMaxPoint = glm::dvec3{
        glm::abs(*std::max_element(
            glm::value_ptr(cache->boxAxesModelSpace[0]), glm::value_ptr(cache->boxAxesModelSpace[0]) + 3, findMaxAbs)
        ),
        glm::abs(*std::max_element(
            glm::value_ptr(cache->boxAxesModelSpace[1]), glm::value_ptr(cache->boxAxesModelSpace[1]) + 3, findMaxAbs)
        ),
        glm::abs(*std::max_element(
            glm::value_ptr(cache->boxAxesModelSpace[2]), glm::value_ptr(cache->boxAxesModelSpace[2]) + 3, findMaxAbs)
        )
    };

    cache->boxMinPoint = -cache->boxMaxPoint;
}

/** Ray, Box CalculateIntersection specialization */
template <>
inline bool CalculateIntersection<Ray, Box>(SimpleShape const* a, SimpleShape const* b, IntersectionCacheBase* cacheBase)
{
    auto cache = static_cast<IntersectionCache<Ray, Box>*>(cacheBase);

    double const t1 = (cache->boxMinPoint.x - cache->rayOriginBoxSpace.x) / cache->rayNormalBoxSpace.x;
    double const t2 = (cache->boxMaxPoint.x - cache->rayOriginBoxSpace.x) / cache->rayNormalBoxSpace.x;
    double const t3 = (cache->boxMinPoint.y - cache->rayOriginBoxSpace.y) / cache->rayNormalBoxSpace.y;
    double const t4 = (cache->boxMaxPoint.y - cache->rayOriginBoxSpace.y) / cache->rayNormalBoxSpace.y;
    double const t5 = (cache->boxMinPoint.z - cache->rayOriginBoxSpace.z) / cache->rayNormalBoxSpace.z;
    double const t6 = (cache->boxMaxPoint.z - cache->rayOriginBoxSpace.z) / cache->rayNormalBoxSpace.z;

    double tmin = glm::max(glm::max(glm::min(t1, t2), glm::min(t3, t4)), glm::min(t5, t6));
    double tmax = glm::min(glm::min(glm::max(t1, t2), glm::max(t3, t4)), glm::max(t5, t6));

    // if tmax < 0, ray (line) is intersecting AABB, but the whole AABB is behind us
    if (tmax < 0)
    {
        cache->rayIntersectionFactorMin = tmax;
        return false;
    }

    // if tmin > tmax, ray doesn't intersect AABB
    if (tmin > tmax)
    {
        cache->rayIntersectionFactorMin = tmax;
        return false;
    }

    cache->rayIntersectionFactorMin = tmin;
    cache->rayIntersectionFactorMax = tmax;
    return true;
}

/** Ray, Box CalculateContactNormal specialization */
template <>
inline glm::dvec3 CalculateContactNormal<Ray, Box>(SimpleShape const* a, SimpleShape const* b, IntersectionCacheBase* cacheBase)
{
    auto cache = static_cast<IntersectionCache<Ray, Box>*>(cacheBase);

    //Calculating intersection points in the obb model space
    cache->inPoint = cache->rayOriginBoxSpace + cache->rayNormalBoxSpace * cache->rayIntersectionFactorMin;
    cache->outPoint = cache->rayOriginBoxSpace + cache->rayNormalBoxSpace * cache->rayIntersectionFactorMax;

    std::array<double, 6> const faces = {
        cache->boxMaxPoint[0], cache->boxMaxPoint[1], cache->boxMaxPoint[2],
        cache->boxMinPoint[0], cache->boxMinPoint[1], cache->boxMinPoint[2]
    };

    std::array<double, 6> const deltas = {
        faces[0] - cache->inPoint[0], faces[1] - cache->inPoint[1], faces[2] - cache->inPoint[2],
        faces[3] - cache->inPoint[0], faces[4] - cache->inPoint[1], faces[5] - cache->inPoint[2],
    };

    size_t const contactFaceIndex = std::distance(deltas.begin(),
        std::min_element(deltas.begin(), deltas.end(), [](double a, double b) {
            return glm::abs(a) < glm::abs(b);
    }));

    //Transforming intersection points in the world space
    cache->inPoint = cache->boxModelMatrix * cache->inPoint + cache->boxMassCenter;
    cache->outPoint = cache->boxModelMatrix * cache->outPoint + cache->boxMassCenter;

    //Calculating box contact normal
    cache->boxContactNormal = {};
    cache->boxContactNormal[contactFaceIndex % 3] = faces[contactFaceIndex];
    cache->boxContactNormal = cache->boxModelMatrix * cache->boxContactNormal;

    return cache->boxContactNormal;
}

/** Ray, Box CalculatePenetration specialization */
template <>
inline double CalculatePenetration<Ray, Box>(SimpleShape const* a, SimpleShape const* b, IntersectionCacheBase* cacheBase)
{
    auto cache = static_cast<IntersectionCache<Ray, Box>*>(cacheBase);
    cache->penetrationDepth = glm::length(cache->outPoint - cache->inPoint);

    return cache->penetrationDepth;
}

/** Plane, Ray Initialize specialization */
template <>
inline void Initialize<Plane, Ray>(SimpleShape const* a, SimpleShape const* b, IntersectionCacheBase* cacheBase)
{
    auto cache = static_cast<IntersectionCache<Plane, Ray>*>(cacheBase);
    Initialize<Ray, Plane>(b, a, &cache->rpCache);
}

/** Plane, Ray CalculateIntersection specialization */
template <>
inline bool CalculateIntersection<Plane, Ray>(SimpleShape const* a, SimpleShape const* b, IntersectionCacheBase* cacheBase)
{
    auto cache = static_cast<IntersectionCache<Plane, Ray>*>(cacheBase);
    return CalculateIntersection<Ray, Plane>(b, a, &cache->rpCache);
}

/** Plane, Ray CalculateContactNormal specialization */
template <>
inline glm::dvec3 CalculateContactNormal<Plane, Ray>(SimpleShape const* a, SimpleShape const* b, IntersectionCacheBase* cacheBase)
{
    auto cache = static_cast<IntersectionCache<Plane, Ray>*>(cacheBase);
    CalculateContactNormal<Ray, Plane>(b, a, &cache->rpCache);
    return -cache->rpCache.rayNormal;
}

/** Plane, Ray CalculatePenetration specialization */
template <>
inline double CalculatePenetration<Plane, Ray>(SimpleShape const* a, SimpleShape const* b, IntersectionCacheBase* cacheBase)
{
    auto cache = static_cast<IntersectionCache<Plane, Ray>*>(cacheBase);
    return CalculatePenetration<Ray, Plane>(b, a, &cache->rpCache);
}

/** Plane, Plane Initialize specialization */
template <>
inline void Initialize<Plane, Plane>(SimpleShape const* a, SimpleShape const* b, IntersectionCacheBase* cacheBase)
{
    auto aPlane = static_cast<Plane const*>(a);
    auto bPlane = static_cast<Plane const*>(b);
    auto cache = static_cast<IntersectionCache<Plane, Plane>*>(cacheBase);

    cache->aNormal = aPlane->normal;
    cache->bNormal = bPlane->normal;
    cache->crossProduct = glm::cross(cache->aNormal, cache->bNormal);
}

/** Plane, Plane CalculateIntersection specialization */
template <>
inline bool CalculateIntersection<Plane, Plane>(SimpleShape const* a, SimpleShape const* b, IntersectionCacheBase* cacheBase)
{
    auto cache = static_cast<IntersectionCache<Plane, Plane>*>(cacheBase);
    return glm::length2(cache->crossProduct) != 0;
}

/** Plane, Plane CalculateContactNormal specialization */
template <>
inline glm::dvec3 CalculateContactNormal<Plane, Plane>(SimpleShape const* a, SimpleShape const* b, IntersectionCacheBase* cacheBase)
{
    auto cache = static_cast<IntersectionCache<Plane, Plane>*>(cacheBase);
    return cache->bNormal;
}

/** Plane, Plane CalculatePenetration specialization */
template <>
inline double CalculatePenetration<Plane, Plane>(SimpleShape const* a, SimpleShape const* b, IntersectionCacheBase* cacheBase)
{
    return std::numeric_limits<double>::max();
}

/** Plane, Sphere Initialize specialization */
template <>
inline void Initialize<Plane, Sphere>(SimpleShape const* a, SimpleShape const* b, IntersectionCacheBase* cacheBase)
{
    auto plane = static_cast<Plane const*>(a);
    auto sphere = static_cast<Sphere const*>(b);
    auto cache = static_cast<IntersectionCache<Plane, Sphere>*>(cacheBase);

    cache->planeNormal = plane->normal;
    cache->planeMassCenter = plane->centerOfMass;
    cache->sphereMassCenter = sphere->centerOfMass;
    cache->sphereRadius = sphere->radius;
    cache->penetration = cache->sphereRadius -
        (glm::dot(cache->sphereMassCenter, cache->planeNormal)
            - glm::dot(cache->planeMassCenter, cache->planeNormal));
}

/** Plane, Sphere CalculateIntersection specialization */
template <>
inline bool CalculateIntersection<Plane, Sphere>(SimpleShape const* a, SimpleShape const* b, IntersectionCacheBase* cacheBase)
{
    auto cache = static_cast<IntersectionCache<Plane, Sphere>*>(cacheBase);
    return cache->penetration >= 0.0;
}

/** Plane, Sphere CalculateContactNormal specialization */
template <>
inline glm::dvec3 CalculateContactNormal<Plane, Sphere>(SimpleShape const* a, SimpleShape const* b, IntersectionCacheBase* cacheBase)
{
    auto cache = static_cast<IntersectionCache<Plane, Sphere>*>(cacheBase);
    return -cache->planeNormal;
}

/** Plane, Sphere CalculatePenetration specialization */
template <>
inline double CalculatePenetration<Plane, Sphere>(SimpleShape const* a, SimpleShape const* b, IntersectionCacheBase* cacheBase)
{
    auto cache = static_cast<IntersectionCache<Plane, Sphere>*>(cacheBase);
    return cache->penetration;
}

/** Plane, Box Initialize specialization */
template <>
inline void Initialize<Plane, Box>(SimpleShape const* a, SimpleShape const* b, IntersectionCacheBase* cacheBase)
{
    auto plane = static_cast<Plane const*>(a);
    auto box = static_cast<Box const*>(b);
    auto cache = static_cast<IntersectionCache<Plane, Box>*>(cacheBase);

    cache->planeNormal = plane->normal;
    cache->planeDistance = glm::dot(plane->centerOfMass, cache->planeNormal);

    cache->boxAxes = { box->iAxis, box->jAxis, box->kAxis };
    cache->boxMassCenter = box->centerOfMass;
    math::CalculateBoxVertices(cache->boxAxes[0], cache->boxAxes[1], cache->boxAxes[2], cache->boxVertices.begin());
    cache->boxFaces = {
        cache->boxAxes[0], cache->boxAxes[1], cache->boxAxes[2],
        -cache->boxAxes[0], -cache->boxAxes[1], -cache->boxAxes[2]
    };
    std::for_each(cache->boxVertices.begin(), cache->boxVertices.end(), [cache](auto& n)
    {
        n += cache->boxMassCenter;
    });
    std::transform(cache->boxVertices.begin(), cache->boxVertices.end(), cache->boxPenetrations.begin(),
        [cache](glm::dvec3 const& p)
    {
        return cache->planeDistance - glm::dot(p, cache->planeNormal);
    });
}

/** Plane, Box CalculateIntersection specialization */
template <>
inline bool CalculateIntersection<Plane, Box>(SimpleShape const* a, SimpleShape const* b, IntersectionCacheBase* cacheBase)
{
    auto cache = static_cast<IntersectionCache<Plane, Box>*>(cacheBase);
    return *std::max_element(cache->boxPenetrations.begin(), cache->boxPenetrations.end()) >= 0;
}

/** Plane, Box CalculateContactNormal specialization */
template <>
inline glm::dvec3 CalculateContactNormal<Plane, Box>(SimpleShape const* a, SimpleShape const* b, IntersectionCacheBase* cacheBase)
{
    auto cache = static_cast<IntersectionCache<Plane, Box>*>(cacheBase);

    std::transform(cache->boxFaces.begin(), cache->boxFaces.end(), cache->boxFaceDistances.begin(),
        [cache](glm::dvec3 const& v)
    {
        return glm::dot(v, cache->planeNormal);
    });
    auto const minIndex = std::distance(cache->boxFaceDistances.begin(),
        std::min_element(cache->boxFaceDistances.begin(), cache->boxFaceDistances.end()));

    return glm::normalize(cache->boxFaces[minIndex]);
}

/** Plane, Box CalculatePenetration specialization */
template <>
inline double CalculatePenetration<Plane, Box>(SimpleShape const* a, SimpleShape const* b, IntersectionCacheBase* cacheBase)
{
    auto cache = static_cast<IntersectionCache<Plane, Box>*>(cacheBase);
    return *std::max_element(cache->boxPenetrations.begin(), cache->boxPenetrations.end());
}

/** Sphere, Plane Initialize specialization */
template <>
inline void Initialize<Sphere, Plane>(SimpleShape const* a, SimpleShape const* b, IntersectionCacheBase* cacheBase)
{
    auto cache = static_cast<IntersectionCache<Sphere, Plane>*>(cacheBase);
    Initialize<Plane, Sphere>(b, a, &cache->psCache);
}

/** Sphere, Plane CalculateIntersection specialization */
template <>
inline bool CalculateIntersection<Sphere, Plane>(SimpleShape const* a, SimpleShape const* b, IntersectionCacheBase* cacheBase)
{
    auto cache = static_cast<IntersectionCache<Sphere, Plane>*>(cacheBase);
    return CalculateIntersection<Plane, Sphere>(b, a, &cache->psCache);
}

/** Sphere, Plane CalculateContactNormal specialization */
template <>
inline glm::dvec3 CalculateContactNormal<Sphere, Plane>(SimpleShape const* a, SimpleShape const* b, IntersectionCacheBase* cacheBase)
{
    auto cache = static_cast<IntersectionCache<Sphere, Plane>*>(cacheBase);
    return -CalculateContactNormal<Plane, Sphere>(b, a, &cache->psCache);
}

/** Sphere, Plane CalculatePenetration specialization */
template <>
inline double CalculatePenetration<Sphere, Plane>(SimpleShape const* a, SimpleShape const* b, IntersectionCacheBase* cacheBase)
{
    auto cache = static_cast<IntersectionCache<Sphere, Plane>*>(cacheBase);
    return CalculatePenetration<Plane, Sphere>(b, a, &cache->psCache);
}

/** Sphere, Ray Initialize specialization */
template <>
inline void Initialize<Sphere, Ray>(SimpleShape const* a, SimpleShape const* b, IntersectionCacheBase* cacheBase)
{
    auto cache = static_cast<IntersectionCache<Sphere, Ray>*>(cacheBase);
    Initialize<Ray, Sphere>(b, a, &cache->rsCache);
}

/** Sphere, Ray CalculateIntersection specialization */
template <>
inline bool CalculateIntersection<Sphere, Ray>(SimpleShape const* a, SimpleShape const* b, IntersectionCacheBase* cacheBase)
{
    auto cache = static_cast<IntersectionCache<Sphere, Ray>*>(cacheBase);
    return CalculateIntersection<Ray, Sphere>(b, a, &cache->rsCache);
}

/** Sphere, Ray CalculateContactNormal specialization */
template <>
inline glm::dvec3 CalculateContactNormal<Sphere, Ray>(SimpleShape const* a, SimpleShape const* b, IntersectionCacheBase* cacheBase)
{
    auto cache = static_cast<IntersectionCache<Sphere, Ray>*>(cacheBase);
    CalculateContactNormal<Ray, Sphere>(b, a, &cache->rsCache);
    return -cache->rsCache.rayNormal;
}

/** Sphere, Ray CalculatePenetration specialization */
template <>
inline double CalculatePenetration<Sphere, Ray>(SimpleShape const* a, SimpleShape const* b, IntersectionCacheBase* cacheBase)
{
    auto cache = static_cast<IntersectionCache<Sphere, Ray>*>(cacheBase);
    return CalculatePenetration<Ray, Sphere>(b, a, &cache->rsCache);
}

/** Sphere, Sphere Initialize specialization */
template <>
inline void Initialize<Sphere, Sphere>(SimpleShape const* a, SimpleShape const* b, IntersectionCacheBase* cacheBase)
{
    auto aSphere = static_cast<Sphere const*>(a);
    auto bSphere = static_cast<Sphere const*>(b);
    auto cache = static_cast<IntersectionCache<Sphere, Sphere>*>(cacheBase);

    cache->aRadius = aSphere->radius;
    cache->aMassCenter = aSphere->centerOfMass;
    cache->bRadius = bSphere->radius;
    cache->bMassCenter = bSphere->centerOfMass;
    cache->baVector = cache->aMassCenter - cache->bMassCenter;
    cache->radiusSum = cache->aRadius + cache->bRadius;
}

/** Sphere, Sphere CalculateIntersection specialization */
template <>
inline bool CalculateIntersection<Sphere, Sphere>(SimpleShape const* a, SimpleShape const* b, IntersectionCacheBase* cacheBase)
{
    auto cache = static_cast<IntersectionCache<Sphere, Sphere>*>(cacheBase);
    return glm::pow2(cache->radiusSum) > glm::length2(cache->baVector);
}

/** Sphere, Sphere CalculateContactNormal specialization */
template <>
inline glm::dvec3 CalculateContactNormal<Sphere, Sphere>(SimpleShape const* a, SimpleShape const* b, IntersectionCacheBase* cacheBase)
{
    auto cache = static_cast<IntersectionCache<Sphere, Sphere>*>(cacheBase);
    return glm::normalize(cache->baVector);
}

/** Sphere, Sphere CalculatePenetration specialization */
template <>
inline double CalculatePenetration<Sphere, Sphere>(SimpleShape const* a, SimpleShape const* b, IntersectionCacheBase* cacheBase)
{
    auto cache = static_cast<IntersectionCache<Sphere, Sphere>*>(cacheBase);
    return cache->radiusSum - glm::length(cache->baVector);
}

/** Sphere, Box Initialize specialization */
template <>
inline void Initialize<Sphere, Box>(SimpleShape const* a, SimpleShape const* b, IntersectionCacheBase* cacheBase)
{
    auto sphere = static_cast<Sphere const*>(a);
    auto box = static_cast<Box const*>(b);
    auto cache = static_cast<IntersectionCache<Sphere, Box>*>(cacheBase);

    cache->sphereMassCenter = sphere->centerOfMass;
    cache->sphereRadius = sphere->radius;

    cache->boxMassCenter = box->centerOfMass;
    cache->boxAxes = { box->iAxis, box->jAxis, box->kAxis };
    math::CalculateBoxVertices(cache->boxAxes[0], cache->boxAxes[1], cache->boxAxes[2], cache->boxVertices.begin());
    std::for_each(cache->boxVertices.begin(), cache->boxVertices.end(), [cache](auto& n)
    {
        n += cache->boxMassCenter;
    });
    cache->boxNormals = {
        cache->boxAxes[0], cache->boxAxes[1], cache->boxAxes[2],
        -cache->boxAxes[0], -cache->boxAxes[1], -cache->boxAxes[2]
    };
    cache->boxAxes = cache->boxNormals;

    for (uint32_t i = 0; i < cache->boxNormals.size(); ++i)
    {
        cache->boxFaces[i] = cache->boxNormals[i] + cache->boxMassCenter;
        cache->boxNormals[i] = glm::normalize(cache->boxNormals[i]);
        cache->boxFaceDistances[i] = glm::length(cache->boxFaces[i] - cache->sphereMassCenter);
    }
}

/** Sphere, Box CalculateIntersection specialization */
template <>
inline bool CalculateIntersection<Sphere, Box>(SimpleShape const* a, SimpleShape const* b, IntersectionCacheBase* cacheBase)
{
    auto cache = static_cast<IntersectionCache<Sphere, Box>*>(cacheBase);

    cache->boxSphereVector = cache->sphereMassCenter - cache->boxMassCenter;

    if (glm::length2(cache->boxSphereVector))
    {
        cache->boxContactPoint = cache->boxMassCenter;

        for (uint32_t i = 0; i < 3; ++i)
        {
            double d = glm::dot(cache->boxSphereVector, cache->boxNormals[i]);
            double axisNorm = glm::length(cache->boxAxes[i]);

            if (d > axisNorm)
            {
                d = axisNorm;
            }
            else if (d < -axisNorm)
            {
                d = -axisNorm;
            }

            cache->boxContactPoint += cache->boxNormals[i] * d;
        }
    }
    else
    {
        cache->boxContactPoint = cache->boxFaces.front();
        cache->boxSphereVector = cache->boxAxes.front();
    }

    cache->sphereContactNormal = glm::normalize(cache->boxContactPoint - cache->sphereMassCenter);
    cache->sphereContactPoint = cache->sphereContactNormal * cache->sphereRadius + cache->sphereMassCenter;

    return glm::length2(cache->sphereMassCenter - cache->boxContactPoint) <= glm::pow2(cache->sphereRadius);
}

/** Sphere, Box CalculateContactNormal specialization */
template <>
inline glm::dvec3 CalculateContactNormal<Sphere, Box>(SimpleShape const* a, SimpleShape const* b, IntersectionCacheBase* cacheBase)
{
    auto cache = static_cast<IntersectionCache<Sphere, Box>*>(cacheBase);

    auto minIt = std::min_element(cache->boxFaceDistances.begin(), cache->boxFaceDistances.end());
    auto minIndex = std::distance(cache->boxFaceDistances.begin(), minIt);
    cache->boxContactAxis = cache->boxAxes[minIndex];
    cache->boxContactNormal = cache->boxNormals[minIndex];

    if (cache->boxContactPoint == cache->sphereMassCenter)
    {
        cache->boxContactPoint = cache->boxMassCenter + glm::normalize(cache->boxSphereVector)
            * glm::dot(cache->boxAxes[minIndex], glm::normalize(cache->boxSphereVector));
        cache->sphereContactNormal = glm::normalize(cache->boxContactPoint - cache->sphereMassCenter);
        cache->sphereContactPoint = cache->sphereContactNormal * cache->sphereRadius + cache->sphereMassCenter;
    }

    return cache->boxContactNormal;
}

/** Sphere, Box CalculatePenetration specialization */
template <>
inline double CalculatePenetration<Sphere, Box>(SimpleShape const* a, SimpleShape const* b, IntersectionCacheBase* cacheBase)
{
    auto cache = static_cast<IntersectionCache<Sphere, Box>*>(cacheBase);

    if (cache->boxContactPoint == cache->boxMassCenter)
    {
        return glm::length(cache->boxAxes.front());
    }

    return glm::length(cache->sphereContactPoint - cache->boxContactPoint);
}

/** Box, Ray Initialize specialization */
template <>
inline void Initialize<Box, Ray>(SimpleShape const* a, SimpleShape const* b, IntersectionCacheBase* cacheBase)
{
    auto cache = static_cast<IntersectionCache<Box, Ray>*>(cacheBase);
    Initialize<Ray, Box>(b, a, &cache->rbCache);
}

/** Box, Ray CalculateIntersection specialization */
template <>
inline bool CalculateIntersection<Box, Ray>(SimpleShape const* a, SimpleShape const* b, IntersectionCacheBase* cacheBase)
{
    auto cache = static_cast<IntersectionCache<Box, Ray>*>(cacheBase);
    return CalculateIntersection<Ray, Box>(b, a, &cache->rbCache);
}

/** Box, Ray CalculateContactNormal specialization */
template <>
inline glm::dvec3 CalculateContactNormal<Box, Ray>(SimpleShape const* a, SimpleShape const* b, IntersectionCacheBase* cacheBase)
{
    auto cache = static_cast<IntersectionCache<Box, Ray>*>(cacheBase);
    auto ray = static_cast<Ray const*>(b);
    CalculateContactNormal<Ray, Box>(b, a, &cache->rbCache);
    return ray->direction;
}

/** Box, Ray CalculatePenetration specialization */
template <>
inline double CalculatePenetration<Box, Ray>(SimpleShape const* a, SimpleShape const* b, IntersectionCacheBase* cacheBase)
{
    auto cache = static_cast<IntersectionCache<Box, Ray>*>(cacheBase);
    return CalculatePenetration<Ray, Box>(b, a, &cache->rbCache);
}

/** Box, Plane Initialize specialization */
template <>
inline void Initialize<Box, Plane>(SimpleShape const* a, SimpleShape const* b, IntersectionCacheBase* cacheBase)
{
    auto cache = static_cast<IntersectionCache<Box, Plane>*>(cacheBase);
    Initialize<Plane, Box>(b, a, &cache->pbCache);
}

/** Box, Plane CalculateIntersection specialization */
template <>
inline bool CalculateIntersection<Box, Plane>(SimpleShape const* a, SimpleShape const* b, IntersectionCacheBase* cacheBase)
{
    auto cache = static_cast<IntersectionCache<Box, Plane>*>(cacheBase);
    return CalculateIntersection<Plane, Box>(b, a, &cache->pbCache);
}

/** Box, Plane CalculateContactNormal specialization */
template <>
inline glm::dvec3 CalculateContactNormal<Box, Plane>(SimpleShape const* a, SimpleShape const* b, IntersectionCacheBase* cacheBase)
{
    auto plane = static_cast<Plane const*>(b);
    return plane->normal;
}

/** Box, Plane CalculatePenetration specialization */
template <>
inline double CalculatePenetration<Box, Plane>(SimpleShape const* a, SimpleShape const* b, IntersectionCacheBase* cacheBase)
{
    auto cache = static_cast<IntersectionCache<Box, Plane>*>(cacheBase);
    return CalculatePenetration<Plane, Box>(b, a, &cache->pbCache);
}

/** Box, Sphere Initialize specialization */
template <>
inline void Initialize<Box, Sphere>(SimpleShape const* a, SimpleShape const* b, IntersectionCacheBase* cacheBase)
{
    auto cache = static_cast<IntersectionCache<Box, Sphere>*>(cacheBase);
    Initialize<Sphere, Box>(b, a, &cache->sbCache);
}

/** Box, Sphere CalculateIntersection specialization */
template <>
inline bool CalculateIntersection<Box, Sphere>(SimpleShape const* a, SimpleShape const* b, IntersectionCacheBase* cacheBase)
{
    auto cache = static_cast<IntersectionCache<Box, Sphere>*>(cacheBase);
    return CalculateIntersection<Sphere, Box>(b, a, &cache->sbCache);
}

/** Box, Sphere CalculateContactNormal specialization */
template <>
inline glm::dvec3 CalculateContactNormal<Box, Sphere>(SimpleShape const* a, SimpleShape const* b, IntersectionCacheBase* cacheBase)
{
    auto cache = static_cast<IntersectionCache<Box, Sphere>*>(cacheBase);
    CalculateContactNormal<Sphere, Box>(b, a, &cache->sbCache);

    return cache->sbCache.sphereContactNormal;
}

/** Box, Sphere CalculatePenetration specialization */
template <>
inline double CalculatePenetration<Box, Sphere>(SimpleShape const* a, SimpleShape const* b, IntersectionCacheBase* cacheBase)
{
    auto cache = static_cast<IntersectionCache<Box, Sphere>*>(cacheBase);
    return CalculatePenetration<Sphere, Box>(b, a, &cache->sbCache);
}

/** Box, Box Initialize specialization */
template <>
inline void Initialize<Box, Box>(SimpleShape const* a, SimpleShape const* b, IntersectionCacheBase* cacheBase)
{
    auto aBox = static_cast<Box const*>(a);
    auto bBox = static_cast<Box const*>(b);
    auto cache = static_cast<IntersectionCache<Box, Box>*>(cacheBase);

    cache->aMassCenter = aBox->centerOfMass;
    cache->aBoxAxes = {
        aBox->iAxis, aBox->jAxis, aBox->kAxis, -aBox->iAxis, -aBox->jAxis, -aBox->kAxis
    };
    math::CalculateBoxVertices(cache->aBoxAxes[0], cache->aBoxAxes[1], cache->aBoxAxes[2], cache->aBoxVertices.begin());
    std::for_each(cache->aBoxVertices.begin(), cache->aBoxVertices.end(), [cache](auto& v)
    {
        v += cache->aMassCenter;
    });
    std::transform(cache->aBoxAxes.begin(), cache->aBoxAxes.end(), cache->aBoxFaces.begin(),
        [cache](glm::dvec3 const& v)
    {
        return v + cache->aMassCenter;
    });

    cache->bMassCenter = bBox->centerOfMass;
    cache->bBoxAxes = {
        bBox->iAxis, bBox->jAxis, bBox->kAxis, -bBox->iAxis, -bBox->jAxis, -bBox->kAxis
    };
    math::CalculateBoxVertices(cache->bBoxAxes[0], cache->bBoxAxes[1], cache->bBoxAxes[2], cache->bBoxVertices.begin());
    std::for_each(cache->bBoxVertices.begin(), cache->bBoxVertices.end(), [cache](auto& v)
    {
        v += cache->bMassCenter;
    });
    std::transform(cache->bBoxAxes.begin(), cache->bBoxAxes.end(), cache->bBoxFaces.begin(),
        [cache](glm::dvec3 const& v)
    {
        return v + cache->bMassCenter;
    });

    cache->separatingAxes = {
        glm::normalize(cache->aBoxAxes[0]), glm::normalize(cache->aBoxAxes[1]), glm::normalize(cache->aBoxAxes[2]),
        glm::normalize(cache->bBoxAxes[0]), glm::normalize(cache->bBoxAxes[1]), glm::normalize(cache->bBoxAxes[2])
    };
    math::CalculateCrossProductForeach(cache->aBoxAxes.begin(), cache->aBoxAxes.begin() + 3,
        cache->bBoxAxes.begin(), cache->bBoxAxes.begin() + 3,
        back_inserter(cache->separatingAxes));

    cache->aBoxVerticesDistances = {};
    cache->bBoxVerticesDistances = {};
}

/** Box, Box CalculateIntersection specialization */
template <>
inline bool CalculateIntersection<Box, Box>(SimpleShape const* a, SimpleShape const* b, IntersectionCacheBase* cacheBase)
{
    auto cache = static_cast<IntersectionCache<Box, Box>*>(cacheBase);

    for (glm::dvec3 const& axis : cache->separatingAxes)
    {
        math::CalculateDotProductForeach(axis, cache->aBoxVertices.begin(), cache->aBoxVertices.end(), 
            cache->aBoxVerticesDistances.begin());
        math::CalculateDotProductForeach(axis, cache->bBoxVertices.begin(), cache->bBoxVertices.end(), 
            cache->bBoxVerticesDistances.begin());
        std::sort(cache->aBoxVerticesDistances.begin(), cache->aBoxVerticesDistances.end());
        std::sort(cache->bBoxVerticesDistances.begin(), cache->bBoxVerticesDistances.end());

        if (cache->aBoxVerticesDistances.back() < cache->bBoxVerticesDistances.back())
        {
            if (cache->aBoxVerticesDistances.back() < cache->bBoxVerticesDistances.front())
            {
                return false;
            }
        }
        else if (cache->bBoxVerticesDistances.back() < cache->aBoxVerticesDistances.front())
        {
            return false;
        }
    }

    return true;
}

/** Box, Box CalculateContactNormal specialization */
template <>
inline glm::dvec3 CalculateContactNormal<Box, Box>(SimpleShape const* a, SimpleShape const* b, IntersectionCacheBase* cacheBase)
{
    auto cache = static_cast<IntersectionCache<Box, Box>*>(cacheBase);

    std::array<double, 6> distances;
    for (uint32_t i = 0; i < distances.size(); ++i)
    {
        distances[i] = glm::length(cache->aMassCenter - cache->bBoxFaces[i]);
    }

    auto const minIt = std::min_element(distances.begin(), distances.end());
    auto const minIndex = std::distance(distances.begin(), minIt);
    cache->contactNormal = glm::normalize(cache->bBoxAxes[minIndex]);

    return cache->contactNormal;
}

/** Box, Box CalculatePenetration specialization */
template <>
inline double CalculatePenetration<Box, Box>(SimpleShape const* a, SimpleShape const* b, IntersectionCacheBase* cacheBase)
{
    auto cache = static_cast<IntersectionCache<Box, Box>*>(cacheBase);

    math::CalculateDotProductForeach(cache->contactNormal, cache->aBoxVertices.begin(),
        cache->aBoxVertices.end(), cache->aBoxVerticesDistances.begin());
    math::CalculateDotProductForeach(cache->contactNormal, cache->bBoxVertices.begin(),
        cache->bBoxVertices.end(), cache->bBoxVerticesDistances.begin());

    double bMaxVertexDistance = *std::max_element(cache->bBoxVerticesDistances.begin(), cache->bBoxVerticesDistances.end());
    double aMinVertexDistance = *std::min_element(cache->aBoxVerticesDistances.begin(), cache->aBoxVerticesDistances.end());
    cache->penetration = bMaxVertexDistance - aMinVertexDistance;

    return cache->penetration;
}
} // namespace intersection

/**
 * @brief Provides generic interface for the runtime simple shape intersection detections
 */
class SimpleShapeIntersectionDetector
{
public:
    SimpleShapeIntersectionDetector();

    SimpleShapeIntersectionDetector(SimpleShapeIntersectionDetector const&) = delete;

    SimpleShapeIntersectionDetector& operator=(SimpleShapeIntersectionDetector const&) = delete;

    /**
     * @brief Performs initial calculations for the given pare of shapes
     * 
     * Required to be called before any other proximity query function, otherwise results are undefined
     * @param[in] a input shape
     * @param[in] b input shape
     */
    void Initialize(SimpleShape const* a, SimpleShape const* b);

    /**
     * @brief Performs intersection test of two shapes and returns true if shapes are intersecting
     *
     * Must be called strictly after Initialize function call, otherwise result is undefined
     * @param[in] a input shape
     * @param[in] b input shape
     * @return @c true if there is intersection, @c false otherwise
     */
    bool CalculateIntersection(SimpleShape const* a, SimpleShape const* b);

    /**
     * @brief Calculates surface contact normal of b shape
     *
     * Must be called strictly after CalculateIntersection function call, otherwise result is undefined
     * @param[in] a input shape
     * @param[in] b input shape
     * @return contact normal
     */
    glm::dvec3 CalculateContactNormal(SimpleShape const* a, SimpleShape const* b);

    /**
     * @brief Calculates penetration depth of two shapes
     * 
     * Must be called strictly after CalculateContactNormal function call, 
     * otherwise result is undefined
     * @param[in] a input shape
     * @param[in] b input shape
     * @return penetration depth
     */
    double CalculatePenetration(SimpleShape const* a, SimpleShape const* b);

private:
    using ShapeTypePair = std::pair<SimpleShape::Type, SimpleShape::Type>;

    /** Hasher for ShapeTypePair objects */
    struct ShapeTypePairHasher
    {
        size_t operator()(ShapeTypePair const& p) const;
    };

    static constexpr uint32_t s_unorderedMapInitialPrimeSize = 11;

    std::unordered_map<ShapeTypePair,
                       std::unique_ptr<intersection::IntersectionCacheBase>,
                       ShapeTypePairHasher>
    m_intersectionCaches;

    std::unordered_map<ShapeTypePair,
                       void(*)(SimpleShape const*, SimpleShape const*, intersection::IntersectionCacheBase*),
                       ShapeTypePairHasher>
    m_initializeFunctors;

    std::unordered_map<ShapeTypePair,
                       bool(*)(SimpleShape const*, SimpleShape const*, intersection::IntersectionCacheBase*),
                       ShapeTypePairHasher>
    m_calculateIntersectionFunctors;

    std::unordered_map<ShapeTypePair,
                       glm::dvec3(*)(SimpleShape const*, SimpleShape const*, intersection::IntersectionCacheBase*),
                       ShapeTypePairHasher>
    m_calculateContactNormalFunctors;

    std::unordered_map<ShapeTypePair,
                       double(*)(SimpleShape const*, SimpleShape const*, intersection::IntersectionCacheBase*),
                       ShapeTypePairHasher>
    m_calculatePenetrationFunctors;
};
} // namespace geometry
} // namespace pegasus

#endif // PEGASUS_GEOMETRY_HPP
