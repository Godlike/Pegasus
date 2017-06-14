/*
* Copyright (C) 2017 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#ifndef PEGASUS_GEOMETRY_HPP
#define PEGASUS_GEOMETRY_HPP

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

namespace pegasus
{
namespace geometry
{
//Shapes
class Shape
{
public:
    explicit Shape(glm::dvec3 const& centerOfMass);

    void setCenterOfMass(glm::dvec3 const& centerOfMass);
    glm::dvec3 const& getCenterOfMass() const;

private:
    glm::dvec3 m_centerOfMass;
};

enum class SimpleShapeType : uint32_t
{
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
    SimpleShapeType type;
public:
    SimpleShape(glm::dvec3 const& centerOfMass, SimpleShapeType type);
};

class Plane : public SimpleShape
{
public:
    Plane(glm::dvec3 const& centerOfMass, glm::dvec3 const& normal);

    void SetNormal(glm::dvec3 const& normal);
    glm::dvec3 const& GetNormal() const;

private:
    glm::dvec3 m_normal;
};

class Triangle : public SimpleShape
{
public:
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
    Sphere(glm::dvec3 const& centerOfMass, double r);

    void SetRadius(double r);
    double GetRadius() const;

private:
    double m_radius;
};

class Cone : public SimpleShape
{
public:
    Cone(glm::dvec3 const& centerOfMass, glm::dvec3 const& a, double r);

    void SetAppex(glm::dvec3 const& a);
    glm::dvec3 const& GetAppex() const;

    void SetRadius(double r);
    double GetRadius() const;

private:
    glm::dvec3 m_appex;
    double m_radius
    ;
};

class Capsule : public SimpleShape
{
public:
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
    Cylinder(glm::dvec3 const& centerOfMass, glm::dvec3 const& halfHeight, double r);
};

class Box : public SimpleShape
{
public:
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
bool IsSameSidePoint(
    glm::dvec3 const& p1, glm::dvec3 const& p2, glm::dvec3 const& a, glm::dvec3 const& b);

template <typename VerticesContainerIt>
void CalculateBoxVertices(
    glm::dvec3 const& i, glm::dvec3 const& j, glm::dvec3 const& k, VerticesContainerIt verticesIterator)
{
    *verticesIterator = (i + j + k);
    *(verticesIterator + 1) = (i - j + k);
    *(verticesIterator + 2) = (j - i + k);
    *(verticesIterator + 3) = (i * -1.0 - j + k);
    *(verticesIterator + 4) = (i + j - k);
    *(verticesIterator + 5) = (i - j - k);
    *(verticesIterator + 6) = (j - i - k);
    *(verticesIterator + 7) = (i * -1.0 - j - k);
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
            if (glm::length(axis) != 0.0f)
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

// Plane, Plane
template <>
inline void Initialize<Plane, Plane>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto aPlane = static_cast<Plane const *>(a);
    auto bPlane = static_cast<Plane const *>(b);
    auto cache = static_cast<Cache<Plane, Plane>*>(cacheBase);

    cache->aNormal = aPlane->GetNormal();
    cache->bNormal = bPlane->GetNormal();
    cache->crossProduct = glm::cross(cache->aNormal, cache->bNormal);
}

template <>
inline bool Overlap<Plane, Plane>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto cache = static_cast<Cache<Plane, Plane>*>(cacheBase);
    return glm::length(cache->crossProduct) != 0;
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
    auto plane = static_cast<Plane const *>(a);
    auto sphere = static_cast<Sphere const *>(b);
    auto cache = static_cast<Cache<Plane, Sphere>*>(cacheBase);

    cache->planeNormal = plane->GetNormal();
    cache->planeMassCenter = plane->getCenterOfMass();
    cache->sphereMassCenter = sphere->getCenterOfMass();
    cache->sphereRadius = sphere->GetRadius();
    cache->penetration = cache->sphereRadius - 
        (glm::dot(cache->sphereMassCenter, cache->planeNormal) - glm::dot(cache->planeMassCenter, cache->planeNormal));
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
    auto plane = static_cast<Plane const *>(a);
    auto box = static_cast<Box const *>(b);
    auto cache = static_cast<Cache<Plane, Box>*>(cacheBase);

    cache->planeNormal = plane->GetNormal();
    cache->planeDistance = glm::dot(plane->getCenterOfMass(), cache->planeNormal);

    box->GetAxes(cache->boxAxes[0], cache->boxAxes[1], cache->boxAxes[2]);
    cache->boxMassCenter = box->getCenterOfMass();
    CalculateBoxVertices(cache->boxAxes[0], cache->boxAxes[1], cache->boxAxes[2], cache->boxVertices.begin());
    cache->boxFaces = {
        cache->boxAxes[0], cache->boxAxes[1], cache->boxAxes[2],
        cache->boxAxes[0] * -1.0, cache->boxAxes[1] * -1.0, cache->boxAxes[2] * -1.0
    };
    std::for_each(cache->boxVertices.begin(), cache->boxVertices.end(), [cache](auto& n) { n += cache->boxMassCenter; });
    std::transform(cache->boxVertices.begin(), cache->boxVertices.end(), cache->boxPenetrations.begin(),
                   [cache](glm::dvec3 const& p) { return cache->planeDistance - glm::dot(p, cache->planeNormal); });
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
                   [cache](glm::dvec3 const& v) { return glm::dot(v, cache->planeNormal); });
    auto const minIndex = std::distance(cache->boxFaceDistances.begin(),
                                        std::min_element(cache->boxFaceDistances.begin(), cache->boxFaceDistances.end()));

    return glm::normalize(cache->boxFaces[minIndex]);
}

template <>
inline double CalculatePenetration<Plane, Box>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto cache = static_cast<Cache<Plane, Box>*>(cacheBase);
    return (*std::max_element(cache->boxPenetrations.begin(), cache->boxPenetrations.end()));
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

// Sphere, Sphere
template <>
inline void Initialize<Sphere, Sphere>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto aSphere = static_cast<Sphere const *>(a);
    auto bSphere = static_cast<Sphere const *>(b);
    auto cache = static_cast<Cache<Sphere, Sphere>*>(cacheBase);

    cache->aRadius = aSphere->GetRadius();
    cache->aMassCenter = aSphere->getCenterOfMass();
    cache->bRadius = bSphere->GetRadius();
    cache->bMassCenter = bSphere->getCenterOfMass();
    cache->baVector = cache->aMassCenter - cache->bMassCenter;
    cache->radiusSum = cache->aRadius + cache->bRadius;
}

template <>
inline bool Overlap<Sphere, Sphere>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto cache = static_cast<Cache<Sphere, Sphere>*>(cacheBase);
    return cache->radiusSum > glm::length(cache->baVector);
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
    auto sphere = static_cast<Sphere const *>(a);
    auto box = static_cast<Box const *>(b);
    auto cache = static_cast<Cache<Sphere, Box>*>(cacheBase);

    cache->sphereMassCenter = sphere->getCenterOfMass();
    cache->sphereRadius = sphere->GetRadius();

    cache->boxMassCenter = box->getCenterOfMass();
    box->GetAxes(cache->boxAxes[0], cache->boxAxes[1], cache->boxAxes[2]);
    CalculateBoxVertices(cache->boxAxes[0], cache->boxAxes[1], cache->boxAxes[2], cache->boxVertices.begin());
    std::for_each(cache->boxVertices.begin(), cache->boxVertices.end(), [cache](auto& n) { n += cache->boxMassCenter; });
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

    if (glm::length(cache->boxSphereVector))
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

    return glm::length(cache->sphereMassCenter - cache->boxContactPoint) <= cache->sphereRadius;
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
    auto plane = static_cast<Plane const *>(b);
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
    auto aBox = static_cast<Box const *>(a);
    auto bBox = static_cast<Box const *>(b);
    auto cache = static_cast<Cache<Box, Box>*>(cacheBase);

    cache->aMassCenter = aBox->getCenterOfMass();
    aBox->GetAxes(cache->aBoxAxes[0], cache->aBoxAxes[1], cache->aBoxAxes[2]);
    cache->aBoxAxes = {cache->aBoxAxes[0], cache->aBoxAxes[1], cache->aBoxAxes[2],
        cache->aBoxAxes[0] * -1.0, cache->aBoxAxes[1] * -1.0, cache->aBoxAxes[2] * -1.0};
    CalculateBoxVertices(cache->aBoxAxes[0], cache->aBoxAxes[1], cache->aBoxAxes[2], cache->aBoxVertices.begin());
    std::for_each(cache->aBoxVertices.begin(), cache->aBoxVertices.end(), [cache](auto& v) { v += cache->aMassCenter; });
    std::transform(cache->aBoxAxes.begin(), cache->aBoxAxes.end(), cache->aBoxFaces.begin(),
                   [cache](glm::dvec3 const & v) { return v + cache->aMassCenter; });

    cache->bMassCenter = bBox->getCenterOfMass();
    bBox->GetAxes(cache->bBoxAxes[0], cache->bBoxAxes[1], cache->bBoxAxes[2]);
    cache->bBoxAxes = {cache->bBoxAxes[0], cache->bBoxAxes[1], cache->bBoxAxes[2],
        cache->bBoxAxes[0] * -1.0, cache->bBoxAxes[1] * -1.0, cache->bBoxAxes[2] * -1.0};
    CalculateBoxVertices(cache->bBoxAxes[0], cache->bBoxAxes[1], cache->bBoxAxes[2], cache->bBoxVertices.begin());
    std::for_each(cache->bBoxVertices.begin(), cache->bBoxVertices.end(), [cache](auto& v) { v += cache->bMassCenter; });
    std::transform(cache->bBoxAxes.begin(), cache->bBoxAxes.end(), cache->bBoxFaces.begin(),
                   [cache](glm::dvec3 const& v) { return v + cache->bMassCenter; });

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
