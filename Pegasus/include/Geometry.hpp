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
#include <glm/glm.hpp>

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

#include "Pegasus/include/Math.hpp"
#include "glm/gtc/type_ptr.hpp"

namespace pegasus
{
namespace geometry
{
class Shape
{
public:
    Shape() = default;

    explicit Shape(glm::dvec3 const& centerOfMass);

    void SetCenterOfMass(glm::dvec3 const& centerOfMass);

    glm::dvec3 const& GetCenterOfMass() const;

private:
    glm::dvec3 m_centerOfMass;
};

enum class SimpleShapeType : uint32_t
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

class SimpleShape : public Shape
{
public:
    SimpleShapeType type = SimpleShapeType::NONE;

    SimpleShape() = default;

    explicit SimpleShape(SimpleShapeType type)
        : type(type)
    {
    }

    SimpleShape(glm::dvec3 const& centerOfMass, SimpleShapeType type);
};

class Ray : public SimpleShape
{
public:
    Ray();

    Ray(glm::dvec3 const& centerOfMass, glm::dvec3 const& normal);

    void SetNormal(glm::dvec3 const& normal);

    glm::dvec3 const& GetNormal() const;

private:
    glm::dvec3 m_normal;
};

class Plane : public SimpleShape
{
public:
    Plane();

    Plane(glm::dvec3 const& centerOfMass, glm::dvec3 const& normal);

    void SetNormal(glm::dvec3 const& normal);

    glm::dvec3 const& GetNormal() const;

private:
    glm::dvec3 m_normal;
};

class Triangle : public SimpleShape
{
public:
    Triangle();

    Triangle(glm::dvec3 const& centerOfMass, glm::dvec3 const& a, glm::dvec3 const& b, glm::dvec3 const& c);

    void SetAxes(glm::dvec3 const& a, glm::dvec3 const& b, glm::dvec3 const& c);

    void GetAxes(glm::dvec3& a, glm::dvec3& b, glm::dvec3& c) const;

    glm::dvec3 const& GetNormal() const;

private:
    glm::dvec3 m_aVertex;
    glm::dvec3 m_bVertex;
    glm::dvec3 m_cVertex;
    glm::dvec3 m_normal;

    void CalculateNormal();
};

class Sphere : public SimpleShape
{
public:
    Sphere();

    Sphere(glm::dvec3 const& centerOfMass, double r);

    void SetRadius(double r);

    double GetRadius() const;

private:
    double m_radius;
};

class Cone : public SimpleShape
{
public:
    Cone();

    Cone(glm::dvec3 const& centerOfMass, glm::dvec3 const& a, double r);

    void SetAppex(glm::dvec3 const& a);

    glm::dvec3 const& GetAppex() const;

    void SetRadius(double r);

    double GetRadius() const;

private:
    glm::dvec3 m_appex;
    double m_radius;
};

class Capsule : public SimpleShape
{
public:
    Capsule();

    Capsule(glm::dvec3 const& centerOfMass, glm::dvec3 const& halfHeight, double r);

    void SetHalfHeight(glm::dvec3 const& halfHeight);

    glm::dvec3 const& GetHalfHeight() const;

    void SetRadius(double r);

    double GetRadius() const;

private:
    glm::dvec3 m_halfHeight;
    double m_radius;
};

class Cylinder : public Capsule
{
public:
    Cylinder();

    Cylinder(glm::dvec3 const& centerOfMass, glm::dvec3 const& halfHeight, double r);
};

class Box : public SimpleShape
{
public:
    Box();

    Box(glm::dvec3 const& centerOfMass, glm::dvec3 const& a, glm::dvec3 const& b, glm::dvec3 const& c);

    void SetAxes(glm::dvec3 const& a, glm::dvec3 const& b, glm::dvec3 const& c);

    void GetAxes(glm::dvec3& a, glm::dvec3& b, glm::dvec3& c) const;

private:
    glm::dvec3 m_aAxis;
    glm::dvec3 m_bAxis;
    glm::dvec3 m_cAxis;
};

namespace intersection
{
// Utility functions

/**
 * @brief Calculates box vertices in the model coordinate space from a given orthogonal basis
 *
 * Writes output vertices to the container starting with @p verticesBeginIterator. There must
 * be at least 7 more elements following given iterator.
 * @tparam VerticesContainerIt Random access iterator
 * @param[in] i box axis vector
 * @param[in] j box axis vector
 * @param[in] k box axis vector
 * @param[in] verticesBeginIterator iterator to the container that is able to store 8 vertices
 */
template <typename VerticesContainerIt>
void CalculateBoxVertices(
        glm::dvec3 const& i, glm::dvec3 const& j, glm::dvec3 const& k,
        VerticesContainerIt verticesBeginIterator
    )
{
    *(verticesBeginIterator + 0) = ( i + j + k);
    *(verticesBeginIterator + 1) = ( i - j + k);
    *(verticesBeginIterator + 2) = ( j - i + k);
    *(verticesBeginIterator + 3) = (-i - j + k);
    *(verticesBeginIterator + 4) = ( i + j - k);
    *(verticesBeginIterator + 5) = ( i - j - k);
    *(verticesBeginIterator + 6) = ( j - i - k);
    *(verticesBeginIterator + 7) = (-i - j - k);
}

template <typename SrcIt1, typename SrcIt2, typename DestIt>
void CalculateSeparatingAxes(SrcIt1 srcBegin1, SrcIt1 srcEnd1, SrcIt2 srcBegin2, SrcIt2 srcEnd2,
    std::back_insert_iterator<DestIt> destBegin)
{
    for (auto it1 = srcBegin1; it1 != srcEnd1; ++it1)
    {
        for (auto it2 = srcBegin2; it2 != srcEnd2; ++it2)
        {
            auto const axis = glm::normalize(glm::cross(*it1, *it2));
            if (glm::length2(axis) != 0.0)
            {
                destBegin++ = axis;
            }
        }
    }
}

template <typename Vector, typename VertIt, typename ProjIt>
void ProjectAllVertices(
    Vector const& axisNormal, VertIt srcBegin, VertIt srcEnd, ProjIt destBegin)
{
    while (srcBegin != srcEnd)
    {
        *destBegin++ = glm::dot(axisNormal, (*srcBegin++));
    }
}

// Intersection query computation cacheBases
struct CacheBase
{
};

template <typename ShapeA, typename ShapeB>
struct Cache : CacheBase
{
};

template <>
struct Cache<Ray, Ray> : CacheBase
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
struct Cache<Ray, Plane> : CacheBase
{
    glm::dvec3 rayNormal;
    glm::dvec3 rayOrigin;
    glm::dvec3 planeNormal;
    glm::dvec3 planePoint;
    math::HyperPlane hp;
    glm::dvec3 intersection;
};

template <>
struct Cache<Ray, Sphere> : CacheBase
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
struct Cache<Ray, Box> : CacheBase
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
struct Cache<Plane, Ray> : CacheBase
{
    Cache<Ray, Plane> rpCache;
};

template <>
struct Cache<Plane, Plane> : CacheBase
{
    glm::dvec3 aNormal;
    glm::dvec3 bNormal;
    glm::dvec3 crossProduct;
};

template <>
struct Cache<Plane, Sphere> : CacheBase
{
    glm::dvec3 planeMassCenter;
    glm::dvec3 planeNormal;
    glm::dvec3 sphereMassCenter;
    double sphereRadius;
    double penetration;
};

template <>
struct Cache<Plane, Box> : CacheBase
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
struct Cache<Sphere, Ray> : CacheBase
{
    Cache<Ray, Sphere> rsCache;
};

template <>
struct Cache<Sphere, Plane> : CacheBase
{
    Cache<Plane, Sphere> psCache;
};

template <>
struct Cache<Sphere, Sphere> : CacheBase
{
    glm::dvec3 bMassCenter;
    glm::dvec3 baVector;
    double bRadius;
    glm::dvec3 aMassCenter;
    double aRadius;
    double radiusSum;
};

template <>
struct Cache<Sphere, Box> : CacheBase
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
struct Cache<Box, Ray> : CacheBase
{
    Cache<Ray, Box> rbCache;
};

template <>
struct Cache<Box, Plane> : CacheBase
{
    Cache<Plane, Box> pbCache;
    glm::dvec3 planeMassCenter;
    glm::dvec3 boxContactNormal;
};

template <>
struct Cache<Box, Sphere> : CacheBase
{
    Cache<Sphere, Box> sbCache;
};

template <>
struct Cache<Box, Box> : CacheBase
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

// General intersection queries
template <typename ShapeA, typename ShapeB>
void Initialize(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase);

template <typename ShapeA, typename ShapeB>
bool Overlap(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase);

template <typename ShapeA, typename ShapeB>
glm::dvec3 CalculateContactNormal(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase);

template <typename ShapeA, typename ShapeB>
double CalculatePenetration(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase);

// Ray, Ray
template <>
inline void Initialize<Ray, Ray>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto cache = static_cast<Cache<Ray, Ray>*>(cacheBase);
    auto aRay = static_cast<Ray const*>(a);
    auto bRay = static_cast<Ray const*>(b);

    cache->aRayPoint = aRay->GetCenterOfMass();
    cache->aRayNormal = aRay->GetNormal();
    cache->bRayPoint = bRay->GetCenterOfMass();
    cache->bRayPoint = bRay->GetNormal();
}

template <>
inline bool Overlap<Ray, Ray>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto cache = static_cast<Cache<Ray, Ray>*>(cacheBase);

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

template <>
inline glm::dvec3 CalculateContactNormal<Ray, Ray>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto cache = static_cast<Cache<Ray, Ray>*>(cacheBase);

    return glm::normalize(glm::cross(glm::cross(cache->bRayNormal, cache->aRayNormal), cache->bRayNormal));
}

template <>
inline double CalculatePenetration<Ray, Ray>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    return 0;
}

// Ray, Plane
template <>
inline void Initialize<Ray, Plane>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto cache = static_cast<Cache<Ray, Plane>*>(cacheBase);
    auto ray = static_cast<Ray const*>(a);
    auto plane = static_cast<Plane const*>(b);

    cache->rayNormal = ray->GetNormal();
    cache->rayOrigin = ray->GetCenterOfMass();
    cache->planePoint = plane->GetCenterOfMass();
    cache->planeNormal = plane->GetNormal();
    cache->hp = math::HyperPlane{cache->planeNormal, cache->planePoint};
}

template <>
inline bool Overlap<Ray, Plane>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto cache = static_cast<Cache<Ray, Plane>*>(cacheBase);

    return cache->hp.RayIntersection(cache->rayNormal, cache->rayOrigin, cache->intersection);
}

template <>
inline glm::dvec3 CalculateContactNormal<Ray, Plane>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto cache = static_cast<Cache<Ray, Plane>*>(cacheBase);

    return cache->rayNormal;
}

template <>
inline double CalculatePenetration<Ray, Plane>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    return std::numeric_limits<double>::max();
}

// Ray, Sphere
template <>
inline void Initialize<Ray, Sphere>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto cache = static_cast<Cache<Ray, Sphere>*>(cacheBase);
    auto ray = static_cast<Ray const*>(a);
    auto sphere = static_cast<Sphere const*>(b);

    cache->rayOrigin = ray->GetCenterOfMass();
    cache->rayNormal = ray->GetNormal();
    cache->sphereRadius = sphere->GetRadius();
    cache->sphereCenter = sphere->GetCenterOfMass();

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

template <>
inline bool Overlap<Ray, Sphere>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto cache = static_cast<Cache<Ray, Sphere>*>(cacheBase);

    return cache->intersection;
}

template <>
inline glm::dvec3 CalculateContactNormal<Ray, Sphere>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto cache = static_cast<Cache<Ray, Sphere>*>(cacheBase);

    return cache->sphereContactNormal;
}

template <>
inline double CalculatePenetration<Ray, Sphere>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto cache = static_cast<Cache<Ray, Sphere>*>(cacheBase);

    return cache->penetration;
}

// Ray, Box
template <>
inline void Initialize<Ray, Box>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto cache = static_cast<Cache<Ray, Box>*>(cacheBase);
    auto ray = static_cast<Ray const*>(a);
    auto box = static_cast<Box const*>(b);

    box->GetAxes(cache->boxAxesWorldSpace[0], cache->boxAxesWorldSpace[1], cache->boxAxesWorldSpace[2]);
    cache->boxMassCenter = box->GetCenterOfMass();
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
    cache->rayNormalBoxSpace = cache->boxModelMatrixInverse * ray->GetNormal();
    cache->rayOriginBoxSpace = cache->boxModelMatrixInverse * (ray->GetCenterOfMass() - cache->boxMassCenter);

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

template <>
inline bool Overlap<Ray, Box>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto cache = static_cast<Cache<Ray, Box>*>(cacheBase);

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

template <>
inline glm::dvec3 CalculateContactNormal<Ray, Box>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto cache = static_cast<Cache<Ray, Box>*>(cacheBase);

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

template <>
inline double CalculatePenetration<Ray, Box>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto cache = static_cast<Cache<Ray, Box>*>(cacheBase);
    cache->penetrationDepth = glm::length(cache->outPoint - cache->inPoint);

    return cache->penetrationDepth;
}

// Plane, Ray
template <>
inline void Initialize<Plane, Ray>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto cache = static_cast<Cache<Plane, Ray>*>(cacheBase);
    Initialize<Ray, Plane>(b, a, &cache->rpCache);
}

template <>
inline bool Overlap<Plane, Ray>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto cache = static_cast<Cache<Plane, Ray>*>(cacheBase);
    return Overlap<Ray, Plane>(b, a, &cache->rpCache);
}

template <>
inline glm::dvec3 CalculateContactNormal<Plane, Ray>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto cache = static_cast<Cache<Plane, Ray>*>(cacheBase);
    CalculateContactNormal<Ray, Plane>(b, a, &cache->rpCache);
    return -cache->rpCache.rayNormal;
}

template <>
inline double CalculatePenetration<Plane, Ray>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto cache = static_cast<Cache<Plane, Ray>*>(cacheBase);
    return CalculatePenetration<Ray, Plane>(b, a, &cache->rpCache);
}

// Plane, Plane
template <>
inline void Initialize<Plane, Plane>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto aPlane = static_cast<Plane const*>(a);
    auto bPlane = static_cast<Plane const*>(b);
    auto cache = static_cast<Cache<Plane, Plane>*>(cacheBase);

    cache->aNormal = aPlane->GetNormal();
    cache->bNormal = bPlane->GetNormal();
    cache->crossProduct = glm::cross(cache->aNormal, cache->bNormal);
}

template <>
inline bool Overlap<Plane, Plane>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto cache = static_cast<Cache<Plane, Plane>*>(cacheBase);
    return glm::length2(cache->crossProduct) != 0;
}

template <>
inline glm::dvec3 CalculateContactNormal<Plane, Plane>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto cache = static_cast<Cache<Plane, Plane>*>(cacheBase);
    return cache->bNormal;
}

template <>
inline double CalculatePenetration<Plane, Plane>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    return std::numeric_limits<double>::max();
}

// Plane, Sphere
template <>
inline void Initialize<Plane, Sphere>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto plane = static_cast<Plane const*>(a);
    auto sphere = static_cast<Sphere const*>(b);
    auto cache = static_cast<Cache<Plane, Sphere>*>(cacheBase);

    cache->planeNormal = plane->GetNormal();
    cache->planeMassCenter = plane->GetCenterOfMass();
    cache->sphereMassCenter = sphere->GetCenterOfMass();
    cache->sphereRadius = sphere->GetRadius();
    cache->penetration = cache->sphereRadius -
        (glm::dot(cache->sphereMassCenter, cache->planeNormal)
            - glm::dot(cache->planeMassCenter, cache->planeNormal));
}

template <>
inline bool Overlap<Plane, Sphere>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto cache = static_cast<Cache<Plane, Sphere>*>(cacheBase);
    return cache->penetration >= 0.0;
}

template <>
inline glm::dvec3 CalculateContactNormal<Plane, Sphere>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto cache = static_cast<Cache<Plane, Sphere>*>(cacheBase);
    return cache->planeNormal * -1.0;
}

template <>
inline double CalculatePenetration<Plane, Sphere>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto cache = static_cast<Cache<Plane, Sphere>*>(cacheBase);
    return cache->penetration;
}

// Plane, Box
template <>
inline void Initialize<Plane, Box>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto plane = static_cast<Plane const*>(a);
    auto box = static_cast<Box const*>(b);
    auto cache = static_cast<Cache<Plane, Box>*>(cacheBase);

    cache->planeNormal = plane->GetNormal();
    cache->planeDistance = glm::dot(plane->GetCenterOfMass(), cache->planeNormal);

    box->GetAxes(cache->boxAxes[0], cache->boxAxes[1], cache->boxAxes[2]);
    cache->boxMassCenter = box->GetCenterOfMass();
    CalculateBoxVertices(cache->boxAxes[0], cache->boxAxes[1], cache->boxAxes[2], cache->boxVertices.begin());
    cache->boxFaces = {
        cache->boxAxes[0], cache->boxAxes[1], cache->boxAxes[2],
        cache->boxAxes[0] * -1.0, cache->boxAxes[1] * -1.0, cache->boxAxes[2] * -1.0
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

template <>
inline bool Overlap<Plane, Box>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto cache = static_cast<Cache<Plane, Box>*>(cacheBase);
    return *std::max_element(cache->boxPenetrations.begin(), cache->boxPenetrations.end()) >= 0;
}

template <>
inline glm::dvec3 CalculateContactNormal<Plane, Box>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto cache = static_cast<Cache<Plane, Box>*>(cacheBase);

    std::transform(cache->boxFaces.begin(), cache->boxFaces.end(), cache->boxFaceDistances.begin(),
        [cache](glm::dvec3 const& v)
    {
        return glm::dot(v, cache->planeNormal);
    });
    auto const minIndex = std::distance(cache->boxFaceDistances.begin(),
        std::min_element(cache->boxFaceDistances.begin(), cache->boxFaceDistances.end()));

    return glm::normalize(cache->boxFaces[minIndex]);
}

template <>
inline double CalculatePenetration<Plane, Box>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto cache = static_cast<Cache<Plane, Box>*>(cacheBase);
    return *std::max_element(cache->boxPenetrations.begin(), cache->boxPenetrations.end());
}

// Sphere, Plane
template <>
inline void Initialize<Sphere, Plane>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto cache = static_cast<Cache<Sphere, Plane>*>(cacheBase);
    Initialize<Plane, Sphere>(b, a, &cache->psCache);
}

template <>
inline bool Overlap<Sphere, Plane>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto cache = static_cast<Cache<Sphere, Plane>*>(cacheBase);
    return Overlap<Plane, Sphere>(b, a, &cache->psCache);
}

template <>
inline glm::dvec3 CalculateContactNormal<Sphere, Plane>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto cache = static_cast<Cache<Sphere, Plane>*>(cacheBase);
    return CalculateContactNormal<Plane, Sphere>(b, a, &cache->psCache) * -1.0;
}

template <>
inline double CalculatePenetration<Sphere, Plane>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto cache = static_cast<Cache<Sphere, Plane>*>(cacheBase);
    return CalculatePenetration<Plane, Sphere>(b, a, &cache->psCache);
}

// Sphere, Ray
template <>
inline void Initialize<Sphere, Ray>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto cache = static_cast<Cache<Sphere, Ray>*>(cacheBase);
    Initialize<Ray, Sphere>(b, a, &cache->rsCache);
}

template <>
inline bool Overlap<Sphere, Ray>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto cache = static_cast<Cache<Sphere, Ray>*>(cacheBase);
    return Overlap<Ray, Sphere>(b, a, &cache->rsCache);
}

template <>
inline glm::dvec3 CalculateContactNormal<Sphere, Ray>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto cache = static_cast<Cache<Sphere, Ray>*>(cacheBase);
    CalculateContactNormal<Ray, Sphere>(b, a, &cache->rsCache);
    return -cache->rsCache.rayNormal;
}

template <>
inline double CalculatePenetration<Sphere, Ray>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto cache = static_cast<Cache<Sphere, Ray>*>(cacheBase);
    return CalculatePenetration<Ray, Sphere>(b, a, &cache->rsCache);
}

// Sphere, Sphere
template <>
inline void Initialize<Sphere, Sphere>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto aSphere = static_cast<Sphere const*>(a);
    auto bSphere = static_cast<Sphere const*>(b);
    auto cache = static_cast<Cache<Sphere, Sphere>*>(cacheBase);

    cache->aRadius = aSphere->GetRadius();
    cache->aMassCenter = aSphere->GetCenterOfMass();
    cache->bRadius = bSphere->GetRadius();
    cache->bMassCenter = bSphere->GetCenterOfMass();
    cache->baVector = cache->aMassCenter - cache->bMassCenter;
    cache->radiusSum = cache->aRadius + cache->bRadius;
}

template <>
inline bool Overlap<Sphere, Sphere>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto cache = static_cast<Cache<Sphere, Sphere>*>(cacheBase);
    return glm::pow2(cache->radiusSum) > glm::length2(cache->baVector);
}

template <>
inline glm::dvec3 CalculateContactNormal<Sphere, Sphere>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto cache = static_cast<Cache<Sphere, Sphere>*>(cacheBase);
    return glm::normalize(cache->baVector);
}

template <>
inline double CalculatePenetration<Sphere, Sphere>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto cache = static_cast<Cache<Sphere, Sphere>*>(cacheBase);
    return cache->radiusSum - glm::length(cache->baVector);
}

// Sphere, Box
template <>
inline void Initialize<Sphere, Box>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto sphere = static_cast<Sphere const*>(a);
    auto box = static_cast<Box const*>(b);
    auto cache = static_cast<Cache<Sphere, Box>*>(cacheBase);

    cache->sphereMassCenter = sphere->GetCenterOfMass();
    cache->sphereRadius = sphere->GetRadius();

    cache->boxMassCenter = box->GetCenterOfMass();
    box->GetAxes(cache->boxAxes[0], cache->boxAxes[1], cache->boxAxes[2]);
    CalculateBoxVertices(cache->boxAxes[0], cache->boxAxes[1], cache->boxAxes[2], cache->boxVertices.begin());
    std::for_each(cache->boxVertices.begin(), cache->boxVertices.end(), [cache](auto& n)
    {
        n += cache->boxMassCenter;
    });
    cache->boxNormals = {cache->boxAxes[0], cache->boxAxes[1], cache->boxAxes[2],
        cache->boxAxes[0] * -1.0, cache->boxAxes[1] * -1.0, cache->boxAxes[2] * -1.0};
    cache->boxAxes = cache->boxNormals;

    for (uint32_t i = 0; i < cache->boxNormals.size(); ++i)
    {
        cache->boxFaces[i] = cache->boxNormals[i] + cache->boxMassCenter;
        cache->boxNormals[i] = glm::normalize(cache->boxNormals[i]);
        cache->boxFaceDistances[i] = glm::length(cache->boxFaces[i] - cache->sphereMassCenter);
    }
}

template <>
inline bool Overlap<Sphere, Box>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto cache = static_cast<Cache<Sphere, Box>*>(cacheBase);

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

template <>
inline glm::dvec3 CalculateContactNormal<Sphere, Box>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto cache = static_cast<Cache<Sphere, Box>*>(cacheBase);

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

template <>
inline double CalculatePenetration<Sphere, Box>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto cache = static_cast<Cache<Sphere, Box>*>(cacheBase);

    if (cache->boxContactPoint == cache->boxMassCenter)
    {
        return glm::length(cache->boxAxes.front());
    }

    return glm::length(cache->sphereContactPoint - cache->boxContactPoint);
}

// Box, Ray
template <>
inline void Initialize<Box, Ray>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto cache = static_cast<Cache<Box, Ray>*>(cacheBase);
    Initialize<Ray, Box>(b, a, &cache->rbCache);
}

template <>
inline bool Overlap<Box, Ray>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto cache = static_cast<Cache<Box, Ray>*>(cacheBase);
    return Overlap<Ray, Box>(b, a, &cache->rbCache);
}

template <>
inline glm::dvec3 CalculateContactNormal<Box, Ray>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto cache = static_cast<Cache<Box, Ray>*>(cacheBase);
    auto ray = static_cast<Ray const*>(b);
    CalculateContactNormal<Ray, Box>(b, a, &cache->rbCache);
    return ray->GetNormal();
}

template <>
inline double CalculatePenetration<Box, Ray>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto cache = static_cast<Cache<Box, Ray>*>(cacheBase);
    return CalculatePenetration<Ray, Box>(b, a, &cache->rbCache);
}

// Box, Plane
template <>
inline void Initialize<Box, Plane>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto cache = static_cast<Cache<Box, Plane>*>(cacheBase);
    Initialize<Plane, Box>(b, a, &cache->pbCache);
}

template <>
inline bool Overlap<Box, Plane>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto cache = static_cast<Cache<Box, Plane>*>(cacheBase);
    return Overlap<Plane, Box>(b, a, &cache->pbCache);
}

template <>
inline glm::dvec3 CalculateContactNormal<Box, Plane>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto plane = static_cast<Plane const*>(b);
    return plane->GetNormal();
}

template <>
inline double CalculatePenetration<Box, Plane>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto cache = static_cast<Cache<Box, Plane>*>(cacheBase);
    return CalculatePenetration<Plane, Box>(b, a, &cache->pbCache);
}

// Box, Sphere
template <>
inline void Initialize<Box, Sphere>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto cache = static_cast<Cache<Box, Sphere>*>(cacheBase);
    Initialize<Sphere, Box>(b, a, &cache->sbCache);
}

template <>
inline bool Overlap<Box, Sphere>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto cache = static_cast<Cache<Box, Sphere>*>(cacheBase);
    return Overlap<Sphere, Box>(b, a, &cache->sbCache);
}

template <>
inline glm::dvec3 CalculateContactNormal<Box, Sphere>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto cache = static_cast<Cache<Box, Sphere>*>(cacheBase);
    CalculateContactNormal<Sphere, Box>(b, a, &cache->sbCache);

    return cache->sbCache.sphereContactNormal;
}

template <>
inline double CalculatePenetration<Box, Sphere>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto cache = static_cast<Cache<Box, Sphere>*>(cacheBase);
    return CalculatePenetration<Sphere, Box>(b, a, &cache->sbCache);
}

// Box, Box
template <>
inline void Initialize<Box, Box>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto aBox = static_cast<Box const*>(a);
    auto bBox = static_cast<Box const*>(b);
    auto cache = static_cast<Cache<Box, Box>*>(cacheBase);

    cache->aMassCenter = aBox->GetCenterOfMass();
    aBox->GetAxes(cache->aBoxAxes[0], cache->aBoxAxes[1], cache->aBoxAxes[2]);
    cache->aBoxAxes = {cache->aBoxAxes[0], cache->aBoxAxes[1], cache->aBoxAxes[2],
        cache->aBoxAxes[0] * -1.0, cache->aBoxAxes[1] * -1.0, cache->aBoxAxes[2] * -1.0};
    CalculateBoxVertices(cache->aBoxAxes[0], cache->aBoxAxes[1], cache->aBoxAxes[2], cache->aBoxVertices.begin());
    std::for_each(cache->aBoxVertices.begin(), cache->aBoxVertices.end(), [cache](auto& v)
    {
        v += cache->aMassCenter;
    });
    std::transform(cache->aBoxAxes.begin(), cache->aBoxAxes.end(), cache->aBoxFaces.begin(),
        [cache](glm::dvec3 const& v)
    {
        return v + cache->aMassCenter;
    });

    cache->bMassCenter = bBox->GetCenterOfMass();
    bBox->GetAxes(cache->bBoxAxes[0], cache->bBoxAxes[1], cache->bBoxAxes[2]);
    cache->bBoxAxes = {cache->bBoxAxes[0], cache->bBoxAxes[1], cache->bBoxAxes[2],
        cache->bBoxAxes[0] * -1.0, cache->bBoxAxes[1] * -1.0, cache->bBoxAxes[2] * -1.0};
    CalculateBoxVertices(cache->bBoxAxes[0], cache->bBoxAxes[1], cache->bBoxAxes[2], cache->bBoxVertices.begin());
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
    CalculateSeparatingAxes(cache->aBoxAxes.begin(), cache->aBoxAxes.begin() + 3,
        cache->bBoxAxes.begin(), cache->bBoxAxes.begin() + 3,
        back_inserter(cache->separatingAxes));

    cache->aBoxVerticesDistances = {};
    cache->bBoxVerticesDistances = {};
}

template <>
inline bool Overlap<Box, Box>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto cache = static_cast<Cache<Box, Box>*>(cacheBase);

    for (auto axis : cache->separatingAxes)
    {
        ProjectAllVertices(axis, cache->aBoxVertices.begin(), cache->aBoxVertices.end(), cache->aBoxVerticesDistances.begin());
        ProjectAllVertices(axis, cache->bBoxVertices.begin(), cache->bBoxVertices.end(), cache->bBoxVerticesDistances.begin());
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

template <>
inline glm::dvec3 CalculateContactNormal<Box, Box>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto cache = static_cast<Cache<Box, Box>*>(cacheBase);

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

template <>
inline double CalculatePenetration<Box, Box>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto cache = static_cast<Cache<Box, Box>*>(cacheBase);

    ProjectAllVertices(cache->contactNormal, cache->aBoxVertices.begin(),
        cache->aBoxVertices.end(), cache->aBoxVerticesDistances.begin());
    ProjectAllVertices(cache->contactNormal, cache->bBoxVertices.begin(),
        cache->bBoxVertices.end(), cache->bBoxVerticesDistances.begin());

    double bMaxVertexDistance = *std::max_element(cache->bBoxVerticesDistances.begin(), cache->bBoxVerticesDistances.end());
    double aMinVertexDistance = *std::min_element(cache->aBoxVerticesDistances.begin(), cache->aBoxVerticesDistances.end());
    cache->penetration = bMaxVertexDistance - aMinVertexDistance;

    return cache->penetration;
}
} // namespace intersection

// General intersection
using ShapeTypePair = std::pair<SimpleShapeType, SimpleShapeType>;

struct ShapeTypePairHash
{
    size_t operator()(ShapeTypePair const& p) const;
};

class IntersectionQuery
{
public:
    IntersectionQuery()
        : m_intersectionCaches(s_unorderedMapInitialPrimeSize, ShapeTypePairHash())
        , m_initializeFunctors(s_unorderedMapInitialPrimeSize, ShapeTypePairHash())
        , m_overlapFunctors(s_unorderedMapInitialPrimeSize, ShapeTypePairHash())
        , m_calculateContactNormalFunctors(s_unorderedMapInitialPrimeSize, ShapeTypePairHash())
        , m_calculatePenetrationFunctors(s_unorderedMapInitialPrimeSize, ShapeTypePairHash())
    {
        m_intersectionCaches[std::make_pair(SimpleShapeType::PLANE, SimpleShapeType::PLANE)]
            = std::make_unique<intersection::Cache<Plane, Plane>>();
        m_intersectionCaches[std::make_pair(SimpleShapeType::PLANE, SimpleShapeType::SPHERE)]
            = std::make_unique<intersection::Cache<Plane, Sphere>>();
        m_intersectionCaches[std::make_pair(SimpleShapeType::PLANE, SimpleShapeType::BOX)]
            = std::make_unique<intersection::Cache<Plane, Box>>();
        m_intersectionCaches[std::make_pair(SimpleShapeType::SPHERE, SimpleShapeType::PLANE)]
            = std::make_unique<intersection::Cache<Sphere, Plane>>();
        m_intersectionCaches[std::make_pair(SimpleShapeType::SPHERE, SimpleShapeType::SPHERE)]
            = std::make_unique<intersection::Cache<Sphere, Sphere>>();
        m_intersectionCaches[std::make_pair(SimpleShapeType::SPHERE, SimpleShapeType::BOX)]
            = std::make_unique<intersection::Cache<Sphere, Box>>();
        m_intersectionCaches[std::make_pair(SimpleShapeType::BOX, SimpleShapeType::PLANE)]
            = std::make_unique<intersection::Cache<Box, Plane>>();
        m_intersectionCaches[std::make_pair(SimpleShapeType::BOX, SimpleShapeType::SPHERE)]
            = std::make_unique<intersection::Cache<Box, Sphere>>();
        m_intersectionCaches[std::make_pair(SimpleShapeType::BOX, SimpleShapeType::BOX)]
            = std::make_unique<intersection::Cache<Box, Box>>();

        m_initializeFunctors[std::make_pair(SimpleShapeType::PLANE, SimpleShapeType::PLANE)]
            = intersection::Initialize<Plane, Plane>;
        m_initializeFunctors[std::make_pair(SimpleShapeType::PLANE, SimpleShapeType::SPHERE)]
            = intersection::Initialize<Plane, Sphere>;
        m_initializeFunctors[std::make_pair(SimpleShapeType::PLANE, SimpleShapeType::BOX)]
            = intersection::Initialize<Plane, Box>;
        m_initializeFunctors[std::make_pair(SimpleShapeType::SPHERE, SimpleShapeType::PLANE)]
            = intersection::Initialize<Sphere, Plane>;
        m_initializeFunctors[std::make_pair(SimpleShapeType::SPHERE, SimpleShapeType::SPHERE)]
            = intersection::Initialize<Sphere, Sphere>;
        m_initializeFunctors[std::make_pair(SimpleShapeType::SPHERE, SimpleShapeType::BOX)]
            = intersection::Initialize<Sphere, Box>;
        m_initializeFunctors[std::make_pair(SimpleShapeType::BOX, SimpleShapeType::PLANE)]
            = intersection::Initialize<Box, Plane>;
        m_initializeFunctors[std::make_pair(SimpleShapeType::BOX, SimpleShapeType::SPHERE)]
            = intersection::Initialize<Box, Sphere>;
        m_initializeFunctors[std::make_pair(SimpleShapeType::BOX, SimpleShapeType::BOX)]
            = intersection::Initialize<Box, Box>;

        m_overlapFunctors[std::make_pair(SimpleShapeType::PLANE, SimpleShapeType::PLANE)]
            = intersection::Overlap<Plane, Plane>;
        m_overlapFunctors[std::make_pair(SimpleShapeType::PLANE, SimpleShapeType::SPHERE)]
            = intersection::Overlap<Plane, Sphere>;
        m_overlapFunctors[std::make_pair(SimpleShapeType::PLANE, SimpleShapeType::BOX)]
            = intersection::Overlap<Plane, Box>;
        m_overlapFunctors[std::make_pair(SimpleShapeType::SPHERE, SimpleShapeType::PLANE)]
            = intersection::Overlap<Sphere, Plane>;
        m_overlapFunctors[std::make_pair(SimpleShapeType::SPHERE, SimpleShapeType::SPHERE)]
            = intersection::Overlap<Sphere, Sphere>;
        m_overlapFunctors[std::make_pair(SimpleShapeType::SPHERE, SimpleShapeType::BOX)]
            = intersection::Overlap<Sphere, Box>;
        m_overlapFunctors[std::make_pair(SimpleShapeType::BOX, SimpleShapeType::PLANE)]
            = intersection::Overlap<Box, Plane>;
        m_overlapFunctors[std::make_pair(SimpleShapeType::BOX, SimpleShapeType::SPHERE)]
            = intersection::Overlap<Box, Sphere>;
        m_overlapFunctors[std::make_pair(SimpleShapeType::BOX, SimpleShapeType::BOX)]
            = intersection::Overlap<Box, Box>;

        m_calculateContactNormalFunctors[std::make_pair(SimpleShapeType::PLANE, SimpleShapeType::PLANE)]
            = intersection::CalculateContactNormal<Plane, Plane>;
        m_calculateContactNormalFunctors[std::make_pair(SimpleShapeType::PLANE, SimpleShapeType::SPHERE)]
            = intersection::CalculateContactNormal<Plane, Sphere>;
        m_calculateContactNormalFunctors[std::make_pair(SimpleShapeType::PLANE, SimpleShapeType::BOX)]
            = intersection::CalculateContactNormal<Plane, Box>;
        m_calculateContactNormalFunctors[std::make_pair(SimpleShapeType::SPHERE, SimpleShapeType::PLANE)]
            = intersection::CalculateContactNormal<Sphere, Plane>;
        m_calculateContactNormalFunctors[std::make_pair(SimpleShapeType::SPHERE, SimpleShapeType::SPHERE)]
            = intersection::CalculateContactNormal<Sphere, Sphere>;
        m_calculateContactNormalFunctors[std::make_pair(SimpleShapeType::SPHERE, SimpleShapeType::BOX)]
            = intersection::CalculateContactNormal<Sphere, Box>;
        m_calculateContactNormalFunctors[std::make_pair(SimpleShapeType::BOX, SimpleShapeType::PLANE)]
            = intersection::CalculateContactNormal<Box, Plane>;
        m_calculateContactNormalFunctors[std::make_pair(SimpleShapeType::BOX, SimpleShapeType::SPHERE)]
            = intersection::CalculateContactNormal<Box, Sphere>;
        m_calculateContactNormalFunctors[std::make_pair(SimpleShapeType::BOX, SimpleShapeType::BOX)]
            = intersection::CalculateContactNormal<Box, Box>;

        m_calculatePenetrationFunctors[std::make_pair(SimpleShapeType::PLANE, SimpleShapeType::PLANE)]
            = intersection::CalculatePenetration<Plane, Plane>;
        m_calculatePenetrationFunctors[std::make_pair(SimpleShapeType::PLANE, SimpleShapeType::SPHERE)]
            = intersection::CalculatePenetration<Plane, Sphere>;
        m_calculatePenetrationFunctors[std::make_pair(SimpleShapeType::PLANE, SimpleShapeType::BOX)]
            = intersection::CalculatePenetration<Plane, Box>;
        m_calculatePenetrationFunctors[std::make_pair(SimpleShapeType::SPHERE, SimpleShapeType::PLANE)]
            = intersection::CalculatePenetration<Sphere, Plane>;
        m_calculatePenetrationFunctors[std::make_pair(SimpleShapeType::SPHERE, SimpleShapeType::SPHERE)]
            = intersection::CalculatePenetration<Sphere, Sphere>;
        m_calculatePenetrationFunctors[std::make_pair(SimpleShapeType::SPHERE, SimpleShapeType::BOX)]
            = intersection::CalculatePenetration<Sphere, Box>;
        m_calculatePenetrationFunctors[std::make_pair(SimpleShapeType::BOX, SimpleShapeType::PLANE)]
            = intersection::CalculatePenetration<Box, Plane>;
        m_calculatePenetrationFunctors[std::make_pair(SimpleShapeType::BOX, SimpleShapeType::SPHERE)]
            = intersection::CalculatePenetration<Box, Sphere>;
        m_calculatePenetrationFunctors[std::make_pair(SimpleShapeType::BOX, SimpleShapeType::BOX)]
            = intersection::CalculatePenetration<Box, Box>;
    }

    void Initialize(SimpleShape const* a, SimpleShape const* b)
    {
        m_initializeFunctors[std::make_pair(a->type, b->type)](
            a, b, m_intersectionCaches[std::make_pair(a->type, b->type)].get());
    }

    bool Overlap(SimpleShape const* a, SimpleShape const* b)
    {
        return m_overlapFunctors[std::make_pair(a->type, b->type)](
            a, b, m_intersectionCaches[std::make_pair(a->type, b->type)].get());
    }

    glm::dvec3 CalculateContactNormal(SimpleShape const* a, SimpleShape const* b)
    {
        return m_calculateContactNormalFunctors[std::make_pair(a->type, b->type)](
            a, b, m_intersectionCaches[std::make_pair(a->type, b->type)].get());
    }

    double CalculatePenetration(SimpleShape const* a, SimpleShape const* b)
    {
        return m_calculatePenetrationFunctors[std::make_pair(a->type, b->type)](
            a, b, m_intersectionCaches[std::make_pair(a->type, b->type)].get());
    }

private:
    static constexpr uint32_t s_unorderedMapInitialPrimeSize = 11;

    std::unordered_map<ShapeTypePair,
                       std::unique_ptr<intersection::CacheBase>,
                       ShapeTypePairHash>
    m_intersectionCaches;

    std::unordered_map<ShapeTypePair,
                       void(*)(SimpleShape const*, SimpleShape const*, intersection::CacheBase*),
                       ShapeTypePairHash>
    m_initializeFunctors;

    std::unordered_map<ShapeTypePair,
                       bool(*)(SimpleShape const*, SimpleShape const*, intersection::CacheBase*),
                       ShapeTypePairHash>
    m_overlapFunctors;

    std::unordered_map<ShapeTypePair,
                       glm::dvec3(*)(SimpleShape const*, SimpleShape const*, intersection::CacheBase*),
                       ShapeTypePairHash>
    m_calculateContactNormalFunctors;

    std::unordered_map<ShapeTypePair,
                       double(*)(SimpleShape const*, SimpleShape const*, intersection::CacheBase*),
                       ShapeTypePairHash>
    m_calculatePenetrationFunctors;
};
} // namespace geometry
} // namespace pegasus

#endif // PEGASUS_GEOMETRY_HPP
