#ifndef PEGAS_GEOMETRY_HPP
#define PEGAS_GEOMETRY_HPP

#include <algorithm>
#include <array>
#include <limits>
#include <memory>

#include "Pegasus/include/math.hpp"

namespace pegas {
namespace gmt {

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
    bool isPointOnSameSide(T const& p1, T const& p2, T const& a, T const& b)
    {
        auto const ab = b - a;
        auto const cp1 = ab.vectorProduct(p1 - a);
        auto const cp2 = ab.vectorProduct(p2 - a);
        return cp1.scalarProduct(cp2) >= 0;
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

        void set(Plane const* _a, Plane const* _b)
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
    class IntersectionQueries : public IntersectionQueriesBase<ShapeA, ShapeB, IntersectionQueries<ShapeA, ShapeB> > {
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
                return std::numeric_limits<pegas::real>::max();
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
                return *std::min_element(intersectionRectangleDistances.begin(), intersectionRectangleDistances.end()) > 0;
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
            intersectionRectangleVertices = { (bRadiusVector - bHalfHeight) + bMassCenter,
                (bRadiusVector.inverse() - bHalfHeight) + bMassCenter,
                (bRadiusVector + bHalfHeight) + bMassCenter,
                (bRadiusVector.inverse() + bHalfHeight) + bMassCenter };
            std::transform(intersectionRectangleVertices.begin(), intersectionRectangleVertices.end(), intersectionRectangleDistances.begin(),
                [this](auto const& v) { return (v * aNormal) - aDistance; });
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
            boxVertices = { (i + j + k), (i - j + k), (j - i + k), (i * -1 - j + k),
                (i + j - k), (i - j - k), (j - i - k), (i * -1 - j - k) };
            boxAxes = { i, j, k, i * -1, j * -1, k * -1 };
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
        IntersectionQueries<Sphere, Triangle> intersection;
        Vector3 aMassCenter;
        Vector3 bMassCenter;
        Vector3 bAppex;
        Vector3 intersectionPlaneVector;
        real aRadius;
        Triangle intersectionTriangle;
        Vector3 intersectionTriangleNormal;
        std::array<Vector3, 3> intersectionTriangleVertices;

    public:
        IntersectionQueries(Sphere const* a, Cone const* b)
            : Base(a, b)
            , intersection(a, &intersectionTriangle)
            , intersectionTriangle({}, {}, {}, {})
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
            boxVertices = { (boxAxes[0] + boxAxes[1] + boxAxes[2]),
                (boxAxes[0] - boxAxes[1] + boxAxes[2]),
                (boxAxes[1] - boxAxes[0] + boxAxes[2]),
                (boxAxes[0].inverse() - boxAxes[1] + boxAxes[2]),
                (boxAxes[0] + boxAxes[1] - boxAxes[2]),
                (boxAxes[0] - boxAxes[1] - boxAxes[2]),
                (boxAxes[1] - boxAxes[0] - boxAxes[2]),
                (boxAxes[0].inverse() - boxAxes[1] - boxAxes[2]) };
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

} // namespace gmt
} // namespace pegas
#endif // PEGAS_GEOMETRY_HPP
