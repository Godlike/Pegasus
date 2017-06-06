/*
* Copyright (C) 2017 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#ifndef PEGASUS_GEOMETRY_HPP
#define PEGASUS_GEOMETRY_HPP

#include "Pegasus/include/Math.hpp"

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
    explicit Shape(Vector3 const& centerOfMass);

    void setCenterOfMass(Vector3 const& centerOfMass);
    Vector3 const& getCenterOfMass() const;

private:
    Vector3 m_centerOfMass;
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
    SimpleShape(Vector3 const& centerOfMass, SimpleShapeType type);
};

class Plane : public SimpleShape
{
public:
    Plane(Vector3 const& centerOfMass, Vector3 const& normal);

    void SetNormal(Vector3 const& normal);
    Vector3 const& GetNormal() const;

private:
    Vector3 m_normal;
};

class Triangle : public SimpleShape
{
public:
    Triangle(Vector3 const& centerOfMass, Vector3 const& a, Vector3 const& b, Vector3 const& c);

    void SetAxes(Vector3 const& a, Vector3 const& b, Vector3 const& c);
    void GetAxes(Vector3& a, Vector3& b, Vector3& c) const;
    Vector3 const& GetNormal() const;

private:
    Vector3 m_aVertex;
    Vector3 m_bVertex;
    Vector3 m_cVertex;
    Vector3 m_normal;

    void CalculateNormal();
};

class Sphere : public SimpleShape
{
public:
    Sphere(Vector3 const& centerOfMass, double r);

    void SetRadius(double r);
    double GetRadius() const;

private:
    double m_radius;
};

class Cone : public SimpleShape
{
public:
    Cone(Vector3 const& centerOfMass, Vector3 const& a, double r);

    void SetAppex(Vector3 const& a);
    Vector3 const& GetAppex() const;

    void SetRadius(double r);
    double GetRadius() const;

private:
    Vector3 m_appex;
    double m_radius
    ;
};

class Capsule : public SimpleShape
{
public:
    Capsule(Vector3 const& centerOfMass, Vector3 const& halfHeight, double r);

    void SetHalfHeight(Vector3 const& halfHeight);
    Vector3 const& GetHalfHeight() const;

    void SetRadius(double r);
    double GetRadius() const;

private:
    Vector3 m_halfHeight;
    double m_radius;
};

class Cylinder : public Capsule
{
public:
    Cylinder(Vector3 const& centerOfMass, Vector3 const& halfHeight, double r);
};

class Box : public SimpleShape
{
public:
    Box(Vector3 const& centerOfMass, Vector3 const& a, Vector3 const& b, Vector3 const& c);

    void SetAxes(Vector3 const& a, Vector3 const& b, Vector3 const& c);
    void GetAxes(Vector3& a, Vector3& b, Vector3& c) const;

private:
    Vector3 m_aAxis;
    Vector3 m_bAxis;
    Vector3 m_cAxis;
};

namespace intersection
{
// Utility functions
bool IsSameSidePoint(
    Vector3 const& p1, Vector3 const& p2, Vector3 const& a, Vector3 const& b);

template <typename Vector, typename VerticesContainer>
void CalculateBoxVertices(
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
void CalculateSeparatingAxes(SrcIt1 srcBegin1, SrcIt1 srcEnd1, SrcIt2 srcBegin2, SrcIt2 srcEnd2,
                             std::back_insert_iterator<DestIt> destBegin)
{
    for (auto it1 = srcBegin1; it1 != srcEnd1; ++it1)
    {
        for (auto it2 = srcBegin2; it2 != srcEnd2; ++it2)
        {
            auto const axis = it1->VectorProduct(*it2).Unit();
            if (axis.SquareMagnitude() != 0.0f)
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
        *destBegin++ = axisNormal.ScalarProduct(*srcBegin++);
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
    Vector3 boxSphereVector;
    Vector3 sphereContactNormal;
    Vector3 sphereMassCenter;
    double sphereRadius;
    Vector3 boxContactPoint;
    Vector3 sphereContactPoint;
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
template <typename ShapeA, typename ShapeB>
void Initialize(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase);

template <typename ShapeA, typename ShapeB>
bool Overlap(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase);

template <typename ShapeA, typename ShapeB>
Vector3 CalculateContactNormal(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase);

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
    cache->crossProduct = cache->aNormal % cache->bNormal;
}

template <>
inline bool Overlap<Plane, Plane>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto cache = static_cast<Cache<Plane, Plane>*>(cacheBase);
    return cache->crossProduct.SquareMagnitude() != 0;
}

template <>
inline Vector3 CalculateContactNormal<Plane, Plane>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
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
        (cache->sphereMassCenter * cache->planeNormal - cache->planeMassCenter * cache->planeNormal);
}

template <>
inline bool Overlap<Plane, Sphere>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto cache = static_cast<Cache<Plane, Sphere>*>(cacheBase);
    return cache->penetration >= 0.0;
}

template <>
inline Vector3 CalculateContactNormal<Plane, Sphere>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto cache = static_cast<Cache<Plane, Sphere>*>(cacheBase);
    return cache->planeNormal.Inverse();
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
    cache->planeDistance = plane->getCenterOfMass() * cache->planeNormal;

    box->GetAxes(cache->boxAxes[0], cache->boxAxes[1], cache->boxAxes[2]);
    cache->boxMassCenter = box->getCenterOfMass();
    CalculateBoxVertices(cache->boxAxes[0], cache->boxAxes[1], cache->boxAxes[2], cache->boxVertices);
    cache->boxFaces = {cache->boxAxes[0], cache->boxAxes[1], cache->boxAxes[2],
        cache->boxAxes[0].Inverse(), cache->boxAxes[1].Inverse(), cache->boxAxes[2].Inverse()};
    std::for_each(cache->boxVertices.begin(), cache->boxVertices.end(), [cache](auto& n) { n += cache->boxMassCenter; });
    std::transform(cache->boxVertices.begin(), cache->boxVertices.end(), cache->boxPenetrations.begin(),
                   [cache](auto const& p) { return cache->planeDistance - p * cache->planeNormal; });
}

template <>
inline bool Overlap<Plane, Box>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto cache = static_cast<Cache<Plane, Box>*>(cacheBase);
    return *std::max_element(cache->boxPenetrations.begin(), cache->boxPenetrations.end()) >= 0;
}

template <>
inline Vector3 CalculateContactNormal<Plane, Box>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto cache = static_cast<Cache<Plane, Box>*>(cacheBase);

    std::transform(cache->boxFaces.begin(), cache->boxFaces.end(), cache->boxFaceDistances.begin(),
                   [cache](auto const& v) { return v * cache->planeNormal; });
    auto const minIndex = std::distance(cache->boxFaceDistances.begin(),
                                        std::min_element(cache->boxFaceDistances.begin(), cache->boxFaceDistances.end()));

    return cache->boxFaces[minIndex].Unit();
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
inline Vector3 CalculateContactNormal<Sphere, Plane>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto cache = static_cast<Cache<Sphere, Plane>*>(cacheBase);
    return CalculateContactNormal<Plane, Sphere>(b, a, &cache->psCache).Inverse();
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
    return pow(cache->radiusSum, 2) > cache->baVector.SquareMagnitude();
}

template <>
inline Vector3 CalculateContactNormal<Sphere, Sphere>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto cache = static_cast<Cache<Sphere, Sphere>*>(cacheBase);
    return cache->baVector.Unit();
}

template <>
inline double CalculatePenetration<Sphere, Sphere>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto cache = static_cast<Cache<Sphere, Sphere>*>(cacheBase);
    return (cache->radiusSum - cache->baVector.Magnitude());
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
    CalculateBoxVertices(cache->boxAxes[0], cache->boxAxes[1], cache->boxAxes[2], cache->boxVertices);
    std::for_each(cache->boxVertices.begin(), cache->boxVertices.end(), [cache](auto& n) { n += cache->boxMassCenter; });
    cache->boxNormals = {cache->boxAxes[0], cache->boxAxes[1], cache->boxAxes[2],
        cache->boxAxes[0].Inverse(), cache->boxAxes[1].Inverse(), cache->boxAxes[2].Inverse()};
    cache->boxAxes = cache->boxNormals;

    for (uint32_t i = 0; i < cache->boxNormals.size(); ++i)
    {
        cache->boxFaces[i] = cache->boxNormals[i] + cache->boxMassCenter;
        cache->boxNormals[i].Normalize();
        cache->boxFaceDistances[i] = (cache->boxFaces[i] - cache->sphereMassCenter).Magnitude();
    }
}

template <>
inline bool Overlap<Sphere, Box>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto cache = static_cast<Cache<Sphere, Box>*>(cacheBase);

    cache->boxSphereVector = cache->sphereMassCenter - cache->boxMassCenter;

    if (cache->boxSphereVector.SquareMagnitude())
    {
        cache->boxContactPoint = cache->boxMassCenter;

        for (uint32_t i = 0; i < 3; ++i)
        {
            double d = cache->boxSphereVector * cache->boxNormals[i];
            double axisNorm = cache->boxAxes[i].Magnitude();

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

    cache->sphereContactNormal = (cache->boxContactPoint - cache->sphereMassCenter).Unit();
    cache->sphereContactPoint = cache->sphereContactNormal * cache->sphereRadius + cache->sphereMassCenter;

    return (cache->sphereMassCenter - cache->boxContactPoint).SquareMagnitude()
            <= std::pow(cache->sphereRadius, 2);
}

template <>
inline Vector3 CalculateContactNormal<Sphere, Box>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto cache = static_cast<Cache<Sphere, Box>*>(cacheBase);

    auto minIt = std::min_element(cache->boxFaceDistances.begin(), cache->boxFaceDistances.end());
    auto minIndex = std::distance(cache->boxFaceDistances.begin(), minIt);
    cache->boxContactAxis = cache->boxAxes[minIndex];
    cache->boxContactNormal = cache->boxNormals[minIndex];

    if (cache->boxContactPoint == cache->sphereMassCenter)
    {
        cache->boxContactPoint = cache->boxMassCenter + cache->boxSphereVector.Unit() 
            * (cache->boxAxes[minIndex] * cache->boxSphereVector.Unit());
        cache->sphereContactNormal = (cache->boxContactPoint - cache->sphereMassCenter).Unit();
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
        return cache->boxAxes.front().Magnitude();
    }

    return (cache->sphereContactPoint - cache->boxContactPoint).Magnitude();
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
inline Vector3 CalculateContactNormal<Box, Plane>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
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
inline Vector3 CalculateContactNormal<Box, Sphere>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
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
        cache->aBoxAxes[0].Inverse(), cache->aBoxAxes[1].Inverse(), cache->aBoxAxes[2].Inverse()};
    CalculateBoxVertices(cache->aBoxAxes[0], cache->aBoxAxes[1], cache->aBoxAxes[2], cache->aBoxVertices);
    std::for_each(cache->aBoxVertices.begin(), cache->aBoxVertices.end(), [cache](auto& v) { v += cache->aMassCenter; });
    std::transform(cache->aBoxAxes.begin(), cache->aBoxAxes.end(), cache->aBoxFaces.begin(),
                   [cache](Vector3 const& v) { return v += cache->aMassCenter; });

    cache->bMassCenter = bBox->getCenterOfMass();
    bBox->GetAxes(cache->bBoxAxes[0], cache->bBoxAxes[1], cache->bBoxAxes[2]);
    cache->bBoxAxes = {cache->bBoxAxes[0], cache->bBoxAxes[1], cache->bBoxAxes[2],
        cache->bBoxAxes[0].Inverse(), cache->bBoxAxes[1].Inverse(), cache->bBoxAxes[2].Inverse()};
    CalculateBoxVertices(cache->bBoxAxes[0], cache->bBoxAxes[1], cache->bBoxAxes[2], cache->bBoxVertices);
    std::for_each(cache->bBoxVertices.begin(), cache->bBoxVertices.end(), [cache](auto& v) { v += cache->bMassCenter; });
    std::transform(cache->bBoxAxes.begin(), cache->bBoxAxes.end(), cache->bBoxFaces.begin(),
                   [cache](Vector3 const& v) { return v += cache->bMassCenter; });

    cache->separatingAxes = {cache->aBoxAxes[0].Unit(), cache->aBoxAxes[1].Unit(), cache->aBoxAxes[2].Unit(),
        cache->bBoxAxes[0].Unit(), cache->bBoxAxes[1].Unit(), cache->bBoxAxes[2].Unit()};
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
inline Vector3 CalculateContactNormal<Box, Box>(SimpleShape const* a, SimpleShape const* b, CacheBase* cacheBase)
{
    auto cache = static_cast<Cache<Box, Box>*>(cacheBase);

    std::array<double, 6> distances;
    for (uint32_t i = 0; i < distances.size(); ++i)
    {
        distances[i] = (cache->aMassCenter - cache->bBoxFaces[i]).Magnitude();
    }

    auto const minIt = std::min_element(distances.begin(), distances.end());
    auto const minIndex = std::distance(distances.begin(), minIt);
    cache->contactNormal = cache->bBoxAxes[minIndex].Unit();

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

    Vector3 CalculateContactNormal(SimpleShape const* a, SimpleShape const* b)
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
            Vector3(*)(SimpleShape const*, SimpleShape const*, intersection::CacheBase*),
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
