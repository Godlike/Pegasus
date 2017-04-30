#ifndef PEGAS_GEOMETRY_HPP
#define PEGAS_GEOMETRY_HPP

#include <algorithm>
#include <array>
#include <limits>
#include <memory>

#include "Pegasus/include/math.hpp"
#include <vector>

namespace pegasus {
namespace geometry {

    class Geometry {
    public:
        virtual ~Geometry();
    };

    class Shape : public Geometry {
    public:
        explicit Shape(Vector3 const& centerOfMass);

        void setCenterOfMass(Vector3 const& centerOfMass);
        Vector3 getCenterOfMass() const;

    private:
        Vector3 mCenterOfMass;
    };

    class SimpleShape : public Shape {
    public:
        explicit SimpleShape(Vector3 const& centerOfMass);
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
        Cylinder(Vector3 const& centerOfMass, Vector3 const& halfHeight, real const r)
            : Capsule(centerOfMass, halfHeight, r)
        {
        }
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
    void calculateSeparatingAxes(
        SrcIt1 srcBegin1, SrcIt1 srcEnd1, SrcIt2 srcBegin2, SrcIt2 srcEnd2, DestIt destBegin)
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

    template <typename ShapeA, typename ShapeB, typename IntersectionSpecific>
    class IntersectionQueriesBase {
    protected:
        ShapeA const* a;
        ShapeB const* b;
        bool initialized = false;

    public:
        explicit IntersectionQueriesBase(ShapeA const* _a = nullptr, ShapeB const* _b = nullptr)
        {
            setShapes(_a, _b);
        }

        void set(ShapeA const* _a, ShapeB const* _b)
        {
            setShapes(_a, _b);

            if (initialized) {
                IntersectionSpecific::calculate();
            }
        }

    private:
        void setShapes(ShapeA const* _a, ShapeB const* _b)
        {
            initialized = _a != nullptr && _b != nullptr;
            if (initialized) {
                a = _a;
                b = _b;
            }
        }
    };

    template <typename ShapeA, typename ShapeB>
    class IntersectionQueries : IntersectionQueriesBase<ShapeA, ShapeB, IntersectionQueries<ShapeA, ShapeB> > {
    };

    //Plane tests
    template <>
    class IntersectionQueries<Plane, Plane>
        : IntersectionQueriesBase<Plane, Plane, IntersectionQueries<Plane, Plane> > {
    private:
        using Base = IntersectionQueriesBase<Plane, Plane, IntersectionQueries<Plane, Plane> >;
        Vector3 aNormal;
        Vector3 bNormal;
        Vector3 crossProduct;

    public:
        IntersectionQueries(Plane const* _a, Plane const* _b)
            : Base(_a, _b)
        {
            if (initialized) {
                calculate();
            }
        }

        bool overlap() const
        {
            if (initialized) {
                return crossProduct.squareMagnitude() != static_cast<real>(0);
            }

            return false;
        }

        Vector3 calculateContactNormal() const
        {
            if (initialized) {
                return b->getNormal();
            }

            return {};
        }

        real calculatePenetration() const
        {
            if (initialized) {
                return std::numeric_limits<pegasus::real>::max();
            }

            return 0;
        }

    private:
        void calculate()
        {
            aNormal = a->getNormal();
            bNormal = b->getNormal();
            crossProduct = aNormal % bNormal;
        }
    };

    template <>
    class IntersectionQueries<Plane, Triangle>
        : IntersectionQueriesBase<Plane, Triangle, IntersectionQueries<Plane, Triangle> > {
    private:
        using Base = IntersectionQueriesBase<Plane, Triangle, IntersectionQueries<Plane, Triangle> >;
        Vector3 i, j, k;
        Vector3 aNormal;
        std::array<real, 3> projections;

    public:
        IntersectionQueries(Plane const* a, Triangle const* b)
            : Base(a, b)
        {
            if (initialized) {
                calculate();
            }
        }

        bool overlap() const
        {
            if (initialized) {
                auto const d = b->getCenterOfMass() * aNormal;

                return (projections[0] < d) || (projections[1] < d) || (projections[2] < d);
            }

            return false;
        }

        Vector3 calculateContactNormal() const
        {
            if (initialized) {
                return b->getNormal().unit();
            }

            return {};
        }

        real calculatePenetration() const
        {
            if (initialized) {
                return *std::min_element(projections.begin(), projections.end());
            }

            return 0;
        }

    private:
        void calculate()
        {
            b->getAxes(i, j, k);
            aNormal = a->getNormal();
            projections = { i * aNormal, j * aNormal, k * aNormal };
        }
    };

    template <>
    class IntersectionQueries<Plane, Sphere>
        : IntersectionQueriesBase<Plane, Sphere, IntersectionQueries<Plane, Sphere> > {
    private:
        using Base = IntersectionQueriesBase<Plane, Sphere, IntersectionQueries<Plane, Sphere> >;
        Vector3 aMassCenter;
        Vector3 bMassCenter;
        Vector3 aNormal;
        real bRadius;
        real penetration;

    public:
        IntersectionQueries(Plane const* a, Sphere const* b)
            : Base(a, b)
        {
            if (initialized) {
                calculate();
            }
        }

        bool overlap() const
        {
            if (initialized) {
                return penetration >= real(0);
            }

            return false;
        }

        Vector3 calculateContactNormal() const
        {
            if (initialized) {
                return (aNormal - bMassCenter).unit();
            }

            return {};
        }

        real calculatePenetration() const
        {
            if (initialized) {
                return penetration;
            }

            return 0;
        }

    private:
        void calculate()
        {
            aNormal = a->getNormal();
            aMassCenter = a->getCenterOfMass();
            bMassCenter = b->getCenterOfMass();
            bRadius = b->getRadius();
            penetration = bMassCenter * aNormal - aMassCenter * aNormal;
        }
    };

    template <>
    class IntersectionQueries<Plane, Cone>
        : IntersectionQueriesBase<Plane, Cone, IntersectionQueries<Plane, Cone> > {
    private:
        using Base = IntersectionQueriesBase<Plane, Cone, IntersectionQueries<Plane, Cone> >;
        Vector3 aMassCenter;
        Vector3 aNormal;
        real aDistance;
        real bRadius;
        Vector3 bMassCenter;
        Vector3 bAppex;
        Vector3 intersectionTriangleNormal;
        Vector3 intersectionTriangleRadiusVector;
        mutable std::array<Vector3, 3> intersectionTriangleVertices;
        mutable std::array<Vector3, 3> intersectionTriangleAxes;
        mutable std::array<real, 3> intersectionTriangleAxesDistances;
        mutable std::array<real, 3> intersectionTriangleVerticesDistances;

    public:
        IntersectionQueries(Plane const* a, Cone const* b)
            : Base(a, b)
        {
            if (initialized) {
                calculate();
            }
        }

        bool overlap() const
        {
            if (initialized) {
                return *std::min_element(intersectionTriangleVerticesDistances.begin(),
                           intersectionTriangleVerticesDistances.end())
                    <= 0;
            }

            return false;
        }

        Vector3 calculateContactNormal() const
        {
            if (initialized) {
                intersectionTriangleAxes = { ((intersectionTriangleVertices[1] - intersectionTriangleAxes[0]) % intersectionTriangleNormal).unit(),
                    ((intersectionTriangleVertices[2] - intersectionTriangleAxes[0]) % intersectionTriangleNormal).unit(),
                    ((intersectionTriangleVertices[1] - intersectionTriangleAxes[2]) % intersectionTriangleNormal).unit() };
                std::transform(intersectionTriangleAxes.begin(), intersectionTriangleAxes.end(),
                    intersectionTriangleAxesDistances.begin(),
                    [this](auto const& v) { return (v + bMassCenter) * aNormal; });
                auto minIndex = std::distance(intersectionTriangleAxesDistances.begin(), std::min_element(intersectionTriangleAxesDistances.begin(),
                                                                                             intersectionTriangleAxesDistances.end()));
                return intersectionTriangleAxes[minIndex];
            }

            return {};
        }

        real calculatePenetration() const
        {
            if (initialized) {
                return *std::min_element(intersectionTriangleVerticesDistances.begin(),
                           intersectionTriangleVerticesDistances.end())
                    * -1;
            }

            return 0;
        }

    private:
        void calculate()
        {
            aNormal = a->getNormal();
            aMassCenter = a->getCenterOfMass();
            aDistance = aMassCenter * aNormal;
            bMassCenter = b->getCenterOfMass();
            bRadius = b->getRadius();
            bAppex = b->getAppex();
            intersectionTriangleNormal = ((aNormal - bAppex) % bAppex).unit();
            intersectionTriangleRadiusVector = (bAppex % intersectionTriangleNormal).unit() * bRadius;
            intersectionTriangleVertices = { intersectionTriangleRadiusVector + bMassCenter,
                intersectionTriangleRadiusVector.inverse() + bMassCenter,
                bAppex + bMassCenter };
            std::transform(intersectionTriangleVertices.begin(), intersectionTriangleVertices.end(),
                intersectionTriangleVerticesDistances.begin(),
                [this](auto const& v) { return (v * aNormal) - aDistance; });
        }
    };

    template <>
    class IntersectionQueries<Plane, Cylinder>
        : IntersectionQueriesBase<Plane, Cylinder, IntersectionQueries<Plane, Cylinder> > {
    private:
        using Base = IntersectionQueriesBase<Plane, Cylinder, IntersectionQueries<Plane, Cylinder> >;
        Vector3 aMassCenter;
        Vector3 aNormal;
        real aDistance;
        Vector3 bMassCenter;
        Vector3 bHalfHeight;
        Vector3 bRadiusVector;
        mutable std::array<Vector3, 4> intersectionRectangleAxes;
        mutable std::array<real, 4> intersectionRectangleAxesDistances;
        std::array<Vector3, 4> intersectionRectangleVertices;
        std::array<real, 4> intersectionRectangleDistances;
        real bRadius;

    public:
        IntersectionQueries(Plane const* a, Cylinder const* b)
            : Base(a, b)
        {
            if (initialized) {
                calculate();
            }
        }

        bool overlap() const
        {
            if (initialized) {
                return *std::min_element(intersectionRectangleDistances.begin(), intersectionRectangleDistances.end()) <= 0;
            }

            return false;
        }

        Vector3 calculateContactNormal() const
        {
            if (initialized) {
                intersectionRectangleAxes = { bHalfHeight, bHalfHeight.inverse(), bRadiusVector, bRadiusVector.inverse() };
                for (auto& v : intersectionRectangleAxes) {
                    v += bMassCenter;
                }
                std::transform(intersectionRectangleAxes.begin(), intersectionRectangleAxes.end(), intersectionRectangleAxesDistances.begin(),
                    [this](auto const& v) { return (v * aNormal) - aDistance; });
                auto min_index = std::distance(intersectionRectangleAxesDistances.begin(),
                    std::min_element(intersectionRectangleAxesDistances.begin(),
                                                   intersectionRectangleAxesDistances.end()));
                return intersectionRectangleAxes[min_index];
            }

            return {};
        }

        real calculatePenetration() const
        {
            if (initialized) {
                return *std::min_element(intersectionRectangleDistances.begin(), intersectionRectangleDistances.end()) * -1;
            }

            return 0;
        }

    private:
        void calculate()
        {
            aNormal = a->getNormal();
            aMassCenter = a->getCenterOfMass();
            aDistance = aMassCenter * aNormal;
            bMassCenter = b->getCenterOfMass();
            bRadius = b->getRadius();
            bHalfHeight = b->getHalfHeight();
            bRadiusVector = ((aNormal % bHalfHeight) % bHalfHeight).unit() * bRadius;
            intersectionRectangleVertices = { 
                bRadiusVector - bHalfHeight + bMassCenter,
                bRadiusVector.inverse() - bHalfHeight + bMassCenter,
                bRadiusVector + bHalfHeight + bMassCenter,
                bRadiusVector.inverse() + bHalfHeight + bMassCenter 
            };
            std::transform(intersectionRectangleVertices.begin(), intersectionRectangleVertices.end(), intersectionRectangleDistances.begin(),
                [this](auto const& v) { return (v * aNormal) - aDistance; });
        }
    };

    template <>
    class IntersectionQueries<Plane, Capsule>
        : IntersectionQueriesBase<Plane, Capsule, IntersectionQueries<Plane, Capsule> > {
    private:
        using Base = IntersectionQueriesBase<Plane, Capsule, IntersectionQueries<Plane, Capsule> >;
        Vector3 bMassCenter;
        Vector3 bHalfHeight;
        real bRadius;
        Sphere s1, s2;
        Cylinder c;
        mutable bool overlapS1 = false;
        mutable bool overlapS2 = false;
        mutable bool overlapC = false;
        IntersectionQueries<Plane, Sphere> psIntresection1;
        IntersectionQueries<Plane, Sphere> psIntresection2;
        IntersectionQueries<Plane, Cylinder> pcIntersection;

    public:
        IntersectionQueries(Plane const* a, Capsule const* b)
            : Base(a, b)
            , s1({}, 0)
            , s2({}, 0)
            , c({}, {}, 0)
            , psIntresection1(a, &s1)
            , psIntresection2(a, &s2)
            , pcIntersection(a, &c)
        {
            if (initialized) {
                calculate();
            }
        }

        bool overlap() const
        {
            if (initialized) {
                overlapC = pcIntersection.overlap();
                if (overlapC) {
                    return true;
                }
                
                overlapS1 = psIntresection1.overlap();
                if (overlapS1) {
                    return true;
                }

                return overlapS2 = psIntresection2.overlap();
            }

            return false;
        }

        Vector3 calculateContactNormal() const
        {
            if (initialized) {
                if (overlapC) {
                    return pcIntersection.calculateContactNormal();
                }
                if (overlapS1) {
                    return psIntresection1.calculateContactNormal();
                }
                if (overlapS2) {
                    return psIntresection2.calculateContactNormal();
                }

                return pcIntersection.calculateContactNormal();
            }

            return {};
        }

        real calculatePenetration() const
        {
            if (initialized) {
                if (overlapC) {
                    return pcIntersection.calculatePenetration();
                }
                if (overlapS1) {
                    return psIntresection1.calculatePenetration();
                }
                if (overlapS2) {
                    return psIntresection2.calculatePenetration();
                }

                return pcIntersection.calculatePenetration();
            }

            return 0;
        }

    private:
        void calculate()
        {
            bMassCenter = b->getCenterOfMass();
            bRadius = b->getRadius();
            bHalfHeight = b->getHalfHeight();
            s1 = Sphere(bHalfHeight + bMassCenter, bRadius);
            s2 = Sphere(bHalfHeight.inverse() + bMassCenter, bRadius);
            c = Cylinder(bMassCenter, bHalfHeight, bRadius);
            psIntresection1 = IntersectionQueries<Plane, Sphere>(a, &s1);
            psIntresection2 = IntersectionQueries<Plane, Sphere>(a, &s2);
            pcIntersection = IntersectionQueries<Plane, Cylinder>(a, &c);
        }
    };

    template <>
    class IntersectionQueries<Plane, Box>
        : IntersectionQueriesBase<Plane, Box, IntersectionQueries<Plane, Box> > {
    private:
        using Base = IntersectionQueriesBase<Plane, Box, IntersectionQueries<Plane, Box> >;
        Vector3 i, j, k;
        Vector3 bMassCenter;
        std::array<Vector3, 8> boxVertices;
        std::array<Vector3, 6> boxAxes;
        mutable std::array<real, 6> boxAxesDistances;
        std::array<real, 8> boxPenetrations;
        Vector3 aNormal;
        real aDistance;

    public:
        IntersectionQueries(Plane const* a, Box const* b)
            : Base(a, b)
        {
            if (initialized) {
                calculate();
            }
        }

        bool overlap() const
        {
            if (initialized) {
                return *std::min_element(boxPenetrations.begin(), boxPenetrations.end()) >= 0;
            }

            return false;
        }

        Vector3 calculateContactNormal() const
        {
            if (initialized) {
                std::transform(boxAxes.begin(), boxAxes.end(), boxAxesDistances.begin(), [this](auto const& v) { return v * aNormal; });
                auto const minIndex = std::distance(boxAxesDistances.begin(),
                    std::min_element(boxAxesDistances.begin(), boxAxesDistances.end()));

                return boxAxes[minIndex].unit();
            }

            return {};
        }

        real calculatePenetration() const
        {
            if (initialized) {
                return (*std::min_element(boxPenetrations.begin(), boxPenetrations.end()));
            }

            return 0;
        }

    private:
        void calculate()
        {
            aNormal = a->getNormal();
            aDistance = a->getCenterOfMass() * aNormal;
            b->getAxes(i, j, k);
            bMassCenter = b->getCenterOfMass();
            calculateBoxVertices(i, j, k, boxVertices);
            boxAxes = { i, j, k, i.inverse(), j.inverse(), k.inverse() };
            std::for_each(boxVertices.begin(), boxVertices.end(), [this](auto& n) { n += bMassCenter; });
            std::transform(boxVertices.begin(), boxVertices.end(), boxPenetrations.begin(),
                [this](auto const& p) { return p * aNormal - aDistance; });
        }
    };

    //Sphere tests
    template <>
    class IntersectionQueries<Sphere, Plane>
        : IntersectionQueriesBase<Sphere, Plane, IntersectionQueries<Sphere, Plane> > {
    private:
        using Base = IntersectionQueriesBase<Sphere, Plane, IntersectionQueries<Sphere, Plane> >;
        IntersectionQueries<Plane, Sphere> planeSphereIntersection;
        Vector3 bNormal;

    public:
        IntersectionQueries(Sphere const* a, Plane const* b)
            : Base(a, b)
            , planeSphereIntersection(b, a)
        {
            if (initialized) {
                calculate();
            }
        }

        bool overlap() const
        {
            if (initialized) {
                return planeSphereIntersection.overlap();
            }

            return false;
        }

        Vector3 calculateContactNormal() const
        {
            if (initialized) {
                return bNormal;
            }

            return {};
        }

        real calculatePenetration() const
        {
            if (initialized) {
                return planeSphereIntersection.calculatePenetration();
            }

            return 0;
        }

    private:
        void calculate()
        {
            bNormal = b->getNormal();
        }
    };

    template <>
    class IntersectionQueries<Sphere, Triangle>
        : IntersectionQueriesBase<Sphere, Triangle, IntersectionQueries<Sphere, Triangle> > {
    private:
        using Base = IntersectionQueriesBase<Sphere, Triangle, IntersectionQueries<Sphere, Triangle> >;
        std::array<Vector3, 3> triangleVetices;
        mutable std::array<real, 3> triangleVerticesSqrDistances;
        Vector3 bMassCenter;
        Vector3 bNormal;
        Vector3 aMassCenter;
        real aRadius;

    public:
        IntersectionQueries(Sphere const* a, Triangle const* b)
            : Base(a, b)
        {
            if (initialized) {
                calculate();
            }
        }

        bool overlap() const
        {
            if (initialized) {
                std::transform(triangleVetices.begin(), triangleVetices.end(), triangleVerticesSqrDistances.begin(),
                    [this](auto const& p) { return (p - aMassCenter).squareMagnitude(); });

                std::sort(triangleVerticesSqrDistances.begin(), triangleVerticesSqrDistances.end());
                if (triangleVerticesSqrDistances.front() < std::pow(aRadius, 2)) {
                    return true;
                }

                auto const spherePlaneScalarProjection = triangleVetices[0] * bNormal - aMassCenter * bNormal;
                if (std::abs(spherePlaneScalarProjection) < aRadius) {
                    return true;
                }

                auto const c = bNormal * (triangleVetices[0] - aMassCenter) / bNormal.squareMagnitude();
                auto const sphereCenterPlaneProjection = aMassCenter + bNormal * c;

                return isPointOnSameSide(sphereCenterPlaneProjection, triangleVetices[0], triangleVetices[1], triangleVetices[2])
                    && isPointOnSameSide(sphereCenterPlaneProjection, triangleVetices[1], triangleVetices[0], triangleVetices[2])
                    && isPointOnSameSide(sphereCenterPlaneProjection, triangleVetices[2], triangleVetices[0], triangleVetices[1]);
            }

            return false;
        }

        Vector3 calculateContactNormal() const
        {
            if (initialized) {
                auto const contactNormalScalar = (aMassCenter - triangleVetices[0]) * bNormal;
                return bNormal * contactNormalScalar;
            }

            return {};
        }

        real calculatePenetration() const
        {
            if (initialized) {
                //Note: Works as a plane penetration test, but should it work like that?
                auto const planeDistance = bMassCenter * bNormal;
                auto const sphereDistance = aMassCenter * bNormal;

                return planeDistance - (sphereDistance - aRadius);
            }

            return 0;
        }

    private:
        void calculate()
        {
            aMassCenter = a->getCenterOfMass();
            bMassCenter = b->getCenterOfMass();
            bNormal = b->getNormal();
            b->getAxes(triangleVetices[0], triangleVetices[1], triangleVetices[2]);
            std::for_each(triangleVetices.begin(), triangleVetices.end(),
                [this](auto& p) { p += bMassCenter; });
        }
    };

    template <>
    class IntersectionQueries<Sphere, Sphere>
        : IntersectionQueriesBase<Sphere, Sphere, IntersectionQueries<Sphere, Sphere> > {
    private:
        using Base = IntersectionQueriesBase<Sphere, Sphere, IntersectionQueries<Sphere, Sphere> >;
        Vector3 aMassCenter;
        Vector3 bMassCenter;
        Vector3 baVector;
        real aRadius;
        real bRadius;
        real radiusSum;

    public:
        IntersectionQueries(Sphere const* a, Sphere const* b)
            : Base(a, b)
        {
            if (initialized) {
                calculate();
            }
        }

        bool overlap() const
        {
            if (initialized) {
                return std::pow(radiusSum, 2) > baVector.squareMagnitude();
            }

            return false;
        }

        Vector3 calculateContactNormal() const
        {
            if (initialized) {
                return baVector.unit();
            }

            return {};
        }

        real calculatePenetration() const
        {
            if (initialized) {
                return (radiusSum - baVector.magnitude());
            }

            return 0;
        }

    private:
        void calculate()
        {
            aMassCenter = a->getCenterOfMass();
            bMassCenter = b->getCenterOfMass();
            aRadius = a->getRadius();
            bRadius = b->getRadius();
            radiusSum = aRadius + bRadius;
            baVector = aMassCenter - bMassCenter;
        }
    };

    template <>
    class IntersectionQueries<Sphere, Cone>
        : IntersectionQueriesBase<Sphere, Cone, IntersectionQueries<Sphere, Cone> > {
    private:
        using Base = IntersectionQueriesBase<Sphere, Cone, IntersectionQueries<Sphere, Cone> >;
        Vector3 aMassCenter;
        Vector3 bMassCenter;
        Vector3 bAppex;
        Vector3 intersectionPlaneVector;
        real aRadius;
        Triangle intersectionTriangle;
        Vector3 intersectionTriangleNormal;
        std::array<Vector3, 3> intersectionTriangleVertices;
        IntersectionQueries<Sphere, Triangle> intersection;

    public:
        IntersectionQueries(Sphere const* a, Cone const* b)
            : Base(a, b)
            , intersectionTriangle({}, {}, {}, {})
            , intersection(a, &intersectionTriangle)
        {
            if (initialized) {
                calculate();
            }
        }

        bool overlap() const
        {
            if (initialized) {
                return intersection.overlap();
            }

            return false;
        }

        Vector3 calculateContactNormal() const
        {
            if (initialized) {
                auto const t = intersectionTriangleNormal * (intersectionTriangleVertices[0] - aMassCenter)
                    / intersectionTriangleNormal.squareMagnitude();
                auto const sphereCenterPlaneProjection = aMassCenter + intersectionTriangleNormal * t;

                if (!isPointOnSameSide(sphereCenterPlaneProjection, intersectionTriangleVertices[0],
                        intersectionTriangleVertices[1], intersectionTriangleVertices[2])) {
                    return intersectionTriangleNormal;
                }
                if (!isPointOnSameSide(sphereCenterPlaneProjection, intersectionTriangleVertices[1],
                        intersectionTriangleVertices[0], intersectionTriangleVertices[2])) {
                    return intersectionTriangleNormal.inverse();
                }

                return bAppex.unit().inverse();
            }

            return {};
        }

        real calculatePenetration() const
        {
            if (initialized) {
                auto const contactNormal = intersection.calculateContactNormal();

                if ((contactNormal % bAppex).squareMagnitude() == static_cast<real>(0)) {
                    return (aRadius - (aMassCenter - bMassCenter).magnitude()) - bAppex.magnitude() * real(0.25);
                }

                auto const edgeDistance = (bAppex + bMassCenter) * contactNormal;
                auto const sphereDistance = aMassCenter * contactNormal;

                return edgeDistance - (sphereDistance - aRadius);
            }

            return 0;
        }

    private:
        void calculate()
        {
            aMassCenter = a->getCenterOfMass();
            bMassCenter = b->getCenterOfMass();
            aRadius = a->getRadius();
            bAppex = b->getAppex();
            intersectionPlaneVector = (bAppex % (bAppex % aMassCenter.unit())).unit();
            intersectionTriangle = Triangle(bMassCenter, intersectionPlaneVector * aRadius, intersectionPlaneVector * -aRadius, bAppex);
            intersectionTriangleNormal = intersectionTriangle.getNormal();
            intersectionTriangle.getAxes(intersectionTriangleVertices[0], intersectionTriangleVertices[1], intersectionTriangleVertices[2]);
            intersection = IntersectionQueries<Sphere, Triangle>(a, &intersectionTriangle);
        }
    };

    template <>
    class IntersectionQueries<Sphere, Cylinder>
        : IntersectionQueriesBase<Sphere, Cylinder, IntersectionQueries<Sphere, Cylinder> > {
    private:
        using Base = IntersectionQueriesBase<Sphere, Cylinder, IntersectionQueries<Sphere, Cylinder> >;
        Vector3 bMassCenter;
        Vector3 bHalfHeight;
        real bRadius;
        Vector3 aMassCenter;
        real aRadius;
        Vector3 baNormal;
        Vector3 intersectionPlaneNormal;
        Vector3 intersectionPlaneVector;
        Vector3 aMassCenterProjection;
        Vector3 bMassCenterProjection;

    public:
        IntersectionQueries(Sphere const* a, Cylinder const* b)
            : Base(a, b)
        {
            if (initialized) {
                calculate();
            }
        }

        bool overlap() const
        {
            if (initialized) {
                return calculatePenetration() > 0;
            }

            return false;
        }

        Vector3 calculateContactNormal() const
        {
            if (initialized) {

                if (std::abs(aMassCenterProjection.x - bMassCenterProjection.x) < bRadius
                    && (aMassCenterProjection.y > bMassCenterProjection.y + bHalfHeight.magnitude()
                           || aMassCenterProjection.y < bMassCenterProjection.y - bHalfHeight.magnitude())) {
                    return (aMassCenterProjection.y > bMassCenterProjection.y ? bHalfHeight : bHalfHeight.inverse()).unit();
                }

                return (aMassCenterProjection.x > bMassCenterProjection.x
                               ? intersectionPlaneVector
                               : intersectionPlaneVector.inverse())
                    .unit();
            }

            return {};
        }

        real calculatePenetration() const
        {
            if (initialized) {
                auto const xPenetration = std::abs(bMassCenterProjection.x - aMassCenterProjection.x) - bRadius - aRadius;
                auto const yPenetration = std::abs(bMassCenterProjection.y - aMassCenterProjection.y) - bHalfHeight.magnitude() - aRadius;

                return xPenetration < yPenetration ? xPenetration : yPenetration;
            }

            return 0;
        }

    private:
        void calculate()
        {
            bMassCenter = b->getCenterOfMass();
            bHalfHeight = b->getHalfHeight();
            bRadius = b->getRadius();
            aMassCenter = a->getCenterOfMass();
            aRadius = a->getRadius();
            baNormal = (aMassCenter - bMassCenter).unit();
            intersectionPlaneNormal = baNormal % bHalfHeight;
            intersectionPlaneVector = (bHalfHeight % intersectionPlaneNormal).unit() * bRadius;
            aMassCenterProjection = { aMassCenter * intersectionPlaneVector.unit(),
                aMassCenter * bHalfHeight.unit(), real(0) };
            bMassCenterProjection = { bMassCenter * intersectionPlaneVector.unit(),
                bMassCenter * bHalfHeight.unit(), real(0) };
        }
    };

    template <>
    class IntersectionQueries<Sphere, Capsule>
        : IntersectionQueriesBase<Sphere, Capsule, IntersectionQueries<Sphere, Capsule> > {
    private:
        using Base = IntersectionQueriesBase<Sphere, Capsule, IntersectionQueries<Sphere, Capsule> >;
        Vector3 aMassCenter;
        real aRadius;
        Vector3 bMassCenter;
        Vector3 bHalfHeight;
        real bRadius;
        Sphere s1, s2;
        Cylinder c;
        IntersectionQueries<Sphere, Sphere> ssIntersection1;
        IntersectionQueries<Sphere, Sphere> ssIntersection2;
        IntersectionQueries<Sphere, Cylinder> scIntersection;

    public:
        IntersectionQueries(Sphere const* a, Cylinder const* b)
            : Base(a, b)
            , s1({}, 0)
            , s2({}, 0)
            , c({}, {}, 0)
            , ssIntersection1(a, &s1)
            , ssIntersection2(a, &s2)
            , scIntersection(a, &c)
        {
            if (initialized) {
                calculate();
            }
        }

        bool overlap() const
        {
            if (initialized) {
                if (scIntersection.overlap()) {
                    return true;
                }

                if (ssIntersection1.overlap()) {
                    return true;
                }

                return ssIntersection2.overlap();
            }

            return false;
        }

        Vector3 calculateContactNormal() const
        {
            if (initialized) {
                if (scIntersection.overlap()) {
                    return scIntersection.calculateContactNormal();
                }

                if (ssIntersection1.overlap()) {
                    return ssIntersection1.calculateContactNormal();
                }

                if (ssIntersection2.overlap()) {
                    return ssIntersection2.calculateContactNormal();
                }

                return scIntersection.calculateContactNormal();
            }

            return {};
        }

        real calculatePenetration() const
        {
            if (initialized) {
                if (ssIntersection1.overlap()) {
                    return ssIntersection1.calculatePenetration();
                }

                if (ssIntersection2.overlap()) {
                    return ssIntersection2.calculatePenetration();
                }

                return scIntersection.calculatePenetration();
            }

            return 0;
        }

    private:
        void calculate()
        {
            aMassCenter = a->getCenterOfMass();
            aRadius = a->getRadius();
            bMassCenter = b->getCenterOfMass();
            bRadius = b->getRadius();
            bHalfHeight = b->getHalfHeight();
            s1 = Sphere(bMassCenter + bHalfHeight, bRadius);
            s2 = Sphere(bMassCenter - bHalfHeight, bRadius);
            c = Cylinder(bMassCenter, bHalfHeight, bRadius);
            ssIntersection1 = IntersectionQueries<Sphere, Sphere>(a, &s1);
            ssIntersection2 = IntersectionQueries<Sphere, Sphere>(a, &s2);
            scIntersection = IntersectionQueries<Sphere, Cylinder>(a, &c);
        }
    };

    template <>
    class IntersectionQueries<Sphere, Box>
        : IntersectionQueriesBase<Sphere, Box, IntersectionQueries<Sphere, Box> > {
    private:
        using Base = IntersectionQueriesBase<Sphere, Box, IntersectionQueries<Sphere, Box> >;
        std::array<Vector3, 3> boxAxes;
        std::array<Vector3, 6> boxNormals;
        std::array<Vector3, 6> boxFaces;
        std::array<real, 6> boxFaceDistances;
        std::array<Vector3, 8> boxVertices;
        std::array<real, 8> boxVerticesProjections;
        std::array<Vector3, 4> separatingAxes;
        Vector3 aMassCenter;
        real aRadius;
        Vector3 bMassCenter;

    public:
        IntersectionQueries(Sphere const* a, Box const* b)
            : Base(a, b)
        {
            if (initialized) {
                calculate();
            }
        }

        bool overlap() const
        {
            if (initialized) {
                if (std::abs((boxAxes[0] + bMassCenter) * separatingAxes[0] - aMassCenter * separatingAxes[0]) <= aRadius) {
                    return true;
                }

                if (std::abs((boxAxes[1] + bMassCenter) * separatingAxes[1] - aMassCenter * separatingAxes[1]) <= aRadius) {
                    return true;
                }

                if (std::abs((boxAxes[2] + bMassCenter) * separatingAxes[2] - aMassCenter * separatingAxes[2]) <= aRadius) {
                    return true;
                }

                auto const sphereMassCenterProjection = aMassCenter * separatingAxes[3];

                return std::abs(sphereMassCenterProjection - boxVerticesProjections.front()) < aRadius
                    || std::abs(sphereMassCenterProjection - boxVerticesProjections.back()) < aRadius;
            }

            return false;
        }

        Vector3 calculateContactNormal() const
        {
            if (initialized) {
                auto minIt = std::min_element(boxFaceDistances.begin(), boxFaceDistances.end());

                return boxNormals[std::distance(boxFaceDistances.begin(), minIt)].unit();
            }

            return {};
        }

        real calculatePenetration() const
        {
            if (initialized) {
                auto minIt = std::min_element(boxFaceDistances.begin(), boxFaceDistances.end());

                return aRadius - *minIt;
            }

            return 0;
        }

    private:
        void calculate()
        {
            b->getAxes(boxAxes[0], boxAxes[1], boxAxes[2]);
            bMassCenter = b->getCenterOfMass();
            aMassCenter = a->getCenterOfMass();
            aRadius = a->getRadius();
            separatingAxes = { boxAxes[0].unit(), boxAxes[1].unit(), boxAxes[2].unit(), (aMassCenter - bMassCenter).unit() };
            calculateBoxVertices(boxAxes[0], boxAxes[1], boxAxes[2], boxVertices);
            std::for_each(boxVertices.begin(), boxVertices.end(), [this](auto& n) { n += bMassCenter; });
            std::transform(boxVertices.begin(), boxVertices.end(), boxVerticesProjections.begin(),
                [this](auto const& v) { return v * separatingAxes[3]; });
            std::sort(boxVerticesProjections.begin(), boxVerticesProjections.end());
            boxNormals = { boxAxes[0], boxAxes[1], boxAxes[2], boxAxes[0].inverse(), boxAxes[1].inverse(), boxAxes[2].inverse() };
            boxFaces = boxNormals;
            std::for_each(boxFaces.begin(), boxFaces.end(), [this](auto& n) { n += bMassCenter; });
            for (unsigned index = 0; index < boxFaceDistances.size(); ++index) {
                boxFaceDistances[index] = std::abs(boxFaces[index] * boxNormals[index].unit() - aMassCenter * boxNormals[index].unit());
            }
        }
    };

    //Box tests
    template <>
    class IntersectionQueries<Box, Plane>
        : IntersectionQueriesBase<Box, Plane, IntersectionQueries<Box, Plane> > {
    private:
        using Base = IntersectionQueriesBase<Box, Plane, IntersectionQueries<Box, Plane> >;
        IntersectionQueries<Plane, Box> intersection;

    public:
        IntersectionQueries(Box const* a, Plane const* b)
            : Base(a, b)
            , intersection(b, a)
        {
        }

        bool overlap() const
        {
            if (initialized) {
                return intersection.overlap();
            }

            return false;
        }

        Vector3 calculateContactNormal() const
        {
            if (initialized) {
                return b->getNormal();
            }

            return {};
        }

        real calculatePenetration() const
        {
            if (initialized) {
                return intersection.calculatePenetration();
            }

            return 0;
        }
    };

    template <>
    class IntersectionQueries<Box, Triangle>
        : IntersectionQueriesBase<Box, Triangle, IntersectionQueries<Box, Triangle> > {
    private:
        using Base = IntersectionQueriesBase<Box, Triangle, IntersectionQueries<Box, Triangle> >;
        Vector3 boxMassCenter;
        std::array<Vector3, 8> boxVertices;
        std::array<Vector3, 6> boxFaces;

        Vector3 triangleMassCenter;
        Vector3 triangleNormal;
        std::array<Vector3, 3> triangleVertices;
        std::array<Vector3, 3> triangleEdges;
        std::vector<Vector3> separatingAxes;
        mutable real penetration = 0;

    public:
        IntersectionQueries(Box const* a, Triangle const* b)
            : Base(a, b)
        {
            if (initialized) {
                calculate();
            }
        }

        bool overlap() const
        {
            if (initialized) {
                std::array<real, 8> boxProjections;
                std::array<real, 3> triangleProjections;

                for (auto axis : separatingAxes) {
                    projectAllVertices(axis, boxVertices.begin(), boxVertices.end(), boxProjections.begin());
                    projectAllVertices(axis, triangleVertices.begin(), triangleVertices.end(), triangleProjections.begin());
                    std::sort(boxProjections.begin(), boxProjections.end());
                    std::sort(triangleProjections.begin(), triangleProjections.end());

                    if (boxProjections.back() < triangleProjections.back()) {
                        if (boxProjections.back() > triangleProjections.front()) {
                            penetration = boxProjections.back() - triangleProjections.front();
                            return true;
                        }
                    } else if (triangleProjections.back() > boxProjections.front()) {
                        penetration = triangleProjections.back() - boxProjections.front();
                        return true;
                    }
                }
            }

            return false;
        }

        Vector3 calculateContactNormal() const
        {
            if (initialized) {
                auto const t = (boxMassCenter - triangleMassCenter) * triangleNormal;
                return (triangleNormal * t).unit();
            }

            return {};
        }

        real calculatePenetration() const
        {
            if (initialized) {
                return penetration;
            }

            return 0;
        }

    private:
        void calculate()
        {
            a->getAxes(boxFaces[0], boxFaces[1], boxFaces[2]);
            boxFaces = { boxFaces[0], boxFaces[1], boxFaces[2], boxFaces[0].inverse(), boxFaces[1].inverse(), boxFaces[2].inverse() };
            boxMassCenter = a->getCenterOfMass();
            calculateBoxVertices(boxFaces[0], boxFaces[1], boxFaces[2], boxVertices);
            std::for_each(boxVertices.begin(), boxVertices.end(), [this](auto& v) { v += boxMassCenter; });

            b->getAxes(triangleVertices[0], triangleVertices[1], triangleVertices[2]);
            triangleMassCenter = b->getCenterOfMass();
            triangleNormal = b->getNormal();
            triangleEdges = { triangleVertices[2] - triangleVertices[0],
                              triangleVertices[2] - triangleVertices[1],
                              triangleVertices[1] - triangleVertices[0] };
            std::for_each(triangleVertices.begin(), triangleVertices.end(), [this](auto& v) { v += triangleMassCenter; });

            separatingAxes = { boxFaces[0].unit(), boxFaces[1].unit(), boxFaces[2].unit(), triangleNormal };
            calculateSeparatingAxes(boxFaces.begin(), boxFaces.begin() + 3,
                triangleEdges.begin(), triangleEdges.end(), std::back_inserter(separatingAxes));
        }
    };

    template <>
    class IntersectionQueries<Box, Sphere>
        : IntersectionQueriesBase<Box, Sphere, IntersectionQueries<Box, Sphere> > {
    private:
        using Base = IntersectionQueriesBase<Box, Sphere, IntersectionQueries<Box, Sphere> >;
        IntersectionQueries<Sphere, Box> intersection;

    public:
        IntersectionQueries(Box const* a, Sphere const* b)
            : Base(a, b)
            , intersection(b, a)
        {
        }

        bool overlap() const
        {
            if (initialized) {
                return intersection.overlap();
            }

            return false;
        }

        Vector3 calculateContactNormal() const
        {
            if (initialized) {
                return intersection.calculateContactNormal().inverse();
            }

            return {};
        }

        real calculatePenetration() const
        {
            if (initialized) {
                return intersection.calculatePenetration();
            }

            return 0;
        }
    };

    template <>
    class IntersectionQueries<Box, Cone>
        : IntersectionQueriesBase<Box, Cone, IntersectionQueries<Box, Cone> > {
    private:
        using Base = IntersectionQueriesBase<Box, Cone, IntersectionQueries<Box, Cone> >;
        Triangle intersectionTriangle;
        Plane intersectionBoxEdge;
        IntersectionQueries<Box, Triangle> btIntersection;
        IntersectionQueries<Plane, Triangle> ptIntersection;
        Vector3 boxMassCenter;
        std::array<Vector3, 8> boxVertices;
        std::array<Vector3, 6> boxAxes;
        Vector3 coneMassCenter;
        Vector3 coneAppex;
        real coneRadius;

    public:
        IntersectionQueries(Box const* a, Cone const* b)
            : Base(a, b)
            , intersectionTriangle({}, {}, {}, {})
            , intersectionBoxEdge({}, {})
            , btIntersection(a, &intersectionTriangle)
            , ptIntersection(&intersectionBoxEdge, &intersectionTriangle)
        {
            if (initialized) {
                calculate();
            }
        }

        bool overlap() const
        {
            if (initialized) {
                return btIntersection.overlap();
            }

            return false;
        }

        Vector3 calculateContactNormal() const
        {
            if (initialized) {
                ptIntersection.calculateContactNormal();
            }

            return {};
        }

        real calculatePenetration() const
        {
            if (initialized) {
                return btIntersection.calculatePenetration();
            }

            return 0;
        }

    private:
        void calculate()
        {
            boxMassCenter = a->getCenterOfMass();
            a->getAxes(boxAxes[0], boxAxes[1], boxAxes[2]);
            boxAxes = { boxAxes[0], boxAxes[1], boxAxes[2], boxAxes[3].inverse(), boxAxes[4].inverse(), boxAxes[5].inverse() };
            calculateBoxVertices(boxAxes[0], boxAxes[1], boxAxes[2], boxVertices);
            
            coneMassCenter = b->getCenterOfMass();
            coneAppex = b->getAppex();
            coneRadius = b->getRadius();

            std::array<real, 6> boxConeVerticeSqrDistances;
            std::transform(boxAxes.begin(), boxAxes.end(), boxConeVerticeSqrDistances.begin(),
                [this](auto const & v) { return ((v + boxMassCenter) - coneMassCenter).squareMagnitude(); });
            auto minIt = std::min_element(boxConeVerticeSqrDistances.begin(), boxConeVerticeSqrDistances.end());
            auto closesAxes = boxAxes[std::distance(boxConeVerticeSqrDistances.begin(), minIt)] - boxMassCenter;
            auto coneBase = ((closesAxes % coneAppex) % coneAppex).unit() * coneRadius;
            intersectionTriangle = Triangle(coneMassCenter, coneBase, coneBase.inverse(), coneAppex);
            intersectionBoxEdge = Plane(closesAxes + boxMassCenter, closesAxes.unit());
            btIntersection = IntersectionQueries<Box, Triangle>(a, &intersectionTriangle);
            ptIntersection = IntersectionQueries<Plane, Triangle>(&intersectionBoxEdge, &intersectionTriangle);
        }
    };

    template <>
    class IntersectionQueries<Box, Box>
        : IntersectionQueriesBase<Box, Box, IntersectionQueries<Box, Box> > {
    private:
        using Base = IntersectionQueriesBase<Box, Box, IntersectionQueries<Box, Box> >;
        Vector3 aMassCenter;
        Vector3 bMassCenter;
        std::array<Vector3, 8> aBoxVertices, bBoxVertices;
        std::array<Vector3, 6> aBoxFaces, bBoxFaces;
        std::vector<Vector3> separatingAxes;
        mutable real penetration = 0;

    public:
        IntersectionQueries(Box const* a, Box const* b)
            : Base(a, b)
        {
            if (initialized) {
                calculate();
            }
        }

        bool overlap() const
        {
            if (initialized) {
                std::array<real, 8> aBoxProjections, bBoxProjections;

                for (auto axis : separatingAxes) {
                    projectAllVertices(axis, aBoxVertices.begin(), aBoxVertices.end(), aBoxProjections.begin());
                    projectAllVertices(axis, bBoxVertices.begin(), bBoxVertices.end(), bBoxProjections.begin());
                    std::sort(aBoxProjections.begin(), aBoxProjections.end());
                    std::sort(bBoxProjections.begin(), bBoxProjections.end());

                    if (aBoxProjections.back() < bBoxProjections.back()) {
                        if (aBoxProjections.back() < bBoxProjections.front()) {
                            return false;
                        }
                        penetration = aBoxProjections.back() - bBoxProjections.front();
                    } else {
                        if (bBoxProjections.back() < aBoxProjections.front()) {
                            return false;
                        }
                        penetration = bBoxProjections.back() - aBoxProjections.front();
                    }
                }
            }

            return true;
        }

        Vector3 calculateContactNormal() const
        {
            if (initialized) {
                std::array<real, 6> distances;
                for (unsigned i = 0; i < distances.size(); ++i) {
                    distances[i] = (aMassCenter - bMassCenter - bBoxFaces[i]).squareMagnitude();
                }

                auto const minIt = std::min_element(distances.begin(), distances.end());
                auto const minIndex = std::distance(distances.begin(), minIt);

                return bBoxFaces[minIndex].unit();
            }

            return {};
        }

        real calculatePenetration() const
        {
            if (initialized) {
                return penetration;
            }

            return 0;
        }

    private:
        void calculate()
        {
            aMassCenter = a->getCenterOfMass();
            a->getAxes(aBoxFaces[0], aBoxFaces[1], aBoxFaces[2]);
            aBoxFaces = { aBoxFaces[0], aBoxFaces[1], aBoxFaces[2], aBoxFaces[0].inverse(), aBoxFaces[1].inverse(), aBoxFaces[2].inverse() };
            bMassCenter = b->getCenterOfMass();
            b->getAxes(bBoxFaces[0], bBoxFaces[1], bBoxFaces[2]);
            bBoxFaces = { bBoxFaces[0], bBoxFaces[1], bBoxFaces[2], bBoxFaces[0].inverse(), bBoxFaces[1].inverse(), bBoxFaces[2].inverse() };

            calculateBoxVertices(aBoxFaces[0], aBoxFaces[1], aBoxFaces[2], aBoxVertices);
            calculateBoxVertices(bBoxFaces[0], bBoxFaces[1], bBoxFaces[2], bBoxVertices);
            std::for_each(aBoxVertices.begin(), aBoxVertices.end(), [this](auto& v) { v += aMassCenter; });
            std::for_each(bBoxVertices.begin(), bBoxVertices.end(), [this](auto& v) { v += bMassCenter; });
            separatingAxes = { aBoxFaces[0].unit(), aBoxFaces[1].unit(), aBoxFaces[2].unit(), 
                               bBoxFaces[0].unit(), bBoxFaces[1].unit(), bBoxFaces[2].unit() };
            calculateSeparatingAxes(aBoxFaces.begin(), aBoxFaces.begin() + 3, 
                                    bBoxFaces.begin(), bBoxFaces.begin() + 3, 
                                    std::back_inserter(separatingAxes));
        }
    };

    template <>
    class IntersectionQueries<Box, Cylinder>
        : IntersectionQueriesBase<Box, Cylinder, IntersectionQueries<Box, Cylinder> > {
    private:
        using Base = IntersectionQueriesBase<Box, Cylinder, IntersectionQueries<Box, Cylinder> >;
        Box intersectionBox;
        IntersectionQueries<Box, Box> intersection;
        Vector3 aMassCenter;
        std::array<Vector3, 8> aVertices, bIntersectionBoxVertices;
        std::array<Vector3, 6> aAxes, bIntersectionBoxAxes;
        Vector3 bMassCenter;
        Vector3 bHalgHeight;
        real bRadius;

    public:
        IntersectionQueries(Box const* a, Cylinder const* b)
            : Base(a, b)
            , intersectionBox({}, {}, {}, {})
            , intersection(a, &intersectionBox)
        {
            if (initialized) {
                calculate();
            }
        }

        bool overlap() const
        {
            if (initialized) {
                return intersection.overlap();
            }

            return false;
        }

        Vector3 calculateContactNormal() const
        {
            if (initialized) {
                intersection.calculateContactNormal();
            }

            return {};
        }

        real calculatePenetration() const
        {
            if (initialized) {
                return intersection.calculatePenetration();
            }

            return 0;
        }

    private:
        void calculate()
        {
            aMassCenter = a->getCenterOfMass();
            a->getAxes(aAxes[0], aAxes[1], aAxes[2]);
            aAxes = { aAxes[0], aAxes[1], aAxes[2], aAxes[3].inverse(), aAxes[4].inverse(), aAxes[5].inverse() };
            calculateBoxVertices(aAxes[0], aAxes[1], aAxes[2], aVertices);

            bMassCenter = b->getCenterOfMass();
            bHalgHeight = b->getHalfHeight();
            bRadius = b->getRadius();

            std::array<real, 6> boxConeVerticeSqrDistances;
            std::transform(aAxes.begin(), aAxes.end(), boxConeVerticeSqrDistances.begin(),
                [this](auto const & v) { return ((v + aMassCenter) - bMassCenter).squareMagnitude(); });
            auto minIt = std::min_element(boxConeVerticeSqrDistances.begin(), boxConeVerticeSqrDistances.end());
            auto const closesAxes = aAxes[std::distance(boxConeVerticeSqrDistances.begin(), minIt)] - aMassCenter;
            auto const intersectionPlaneNormal = closesAxes % bHalgHeight;
            auto const bBase = (intersectionPlaneNormal % bHalgHeight).unit() * bRadius;
            intersectionBox = Box(bMassCenter, bHalgHeight, bBase, intersectionPlaneNormal.unit() * 0.001f);
            intersection = IntersectionQueries<Box, Box>(a, &intersectionBox);
        }
    };

    template <>
    class IntersectionQueries<Box, Capsule>
        : IntersectionQueriesBase<Box, Capsule, IntersectionQueries<Box, Capsule> > {
    private:
        using Base = IntersectionQueriesBase<Box, Capsule, IntersectionQueries<Box, Capsule> >;
        Vector3 aMassCenter;
        real aRadius;
        Vector3 bMassCenter;
        Vector3 bHalfHeight;
        real bRadius;
        Sphere s1, s2;
        Cylinder c;
        IntersectionQueries<Box, Sphere> bsIntersection1;
        IntersectionQueries<Box, Sphere> bsIntersection2;
        IntersectionQueries<Box, Cylinder> bcIntersection;

    public:
        IntersectionQueries(Box const* a, Cylinder const* b)
            : Base(a, b)
            , s1({}, 0)
            , s2({}, 0)
            , c({}, {}, 0)
            , bsIntersection1(a, &s1)
            , bsIntersection2(a, &s2)
            , bcIntersection(a, &c)
        {
            if (initialized) {
                calculate();
            }
        }

        bool overlap() const
        {
            if (initialized) {
                if (bcIntersection.overlap()) {
                    return true;
                }

                if (bsIntersection1.overlap()) {
                    return true;
                }

                return bsIntersection2.overlap();
            }

            return false;
        }

        Vector3 calculateContactNormal() const
        {
            if (initialized) {
                if (bcIntersection.overlap()) {
                    return bcIntersection.calculateContactNormal();
                }

                if (bsIntersection1.overlap()) {
                    return bsIntersection1.calculateContactNormal();
                }

                if (bsIntersection2.overlap()) {
                    return bsIntersection2.calculateContactNormal();
                }

                return bcIntersection.calculateContactNormal();
            }

            return {};
        }

        real calculatePenetration() const
        {
            if (initialized) {
                if (bsIntersection1.overlap()) {
                    return bsIntersection1.calculatePenetration();
                }

                if (bsIntersection2.overlap()) {
                    return bsIntersection2.calculatePenetration();
                }

                return bcIntersection.calculatePenetration();
            }

            return 0;
        }

    private:
        void calculate()
        {
            aMassCenter = a->getCenterOfMass();
            bMassCenter = b->getCenterOfMass();
            bRadius = b->getRadius();
            bHalfHeight = b->getHalfHeight();
            s1 = Sphere(bMassCenter + bHalfHeight, bRadius);
            s2 = Sphere(bMassCenter - bHalfHeight, bRadius);
            c = Cylinder(bMassCenter, bHalfHeight, bRadius);
            bsIntersection1 = IntersectionQueries<Box, Sphere>(a, &s1);
            bsIntersection2 = IntersectionQueries<Box, Sphere>(a, &s2);
            bcIntersection  = IntersectionQueries<Box, Cylinder>(a, &c);
        }
    };

} // namespace geometry
} // namespace pegasus
#endif // PEGAS_GEOMETRY_HPP
