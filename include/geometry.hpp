#ifndef PEGAS_GEOMETRY_HPP
#define PEGAS_GEOMETRY_HPP

#include <algorithm>
#include <memory>

#include "Pegas/include/math.hpp"

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
        Triangle(Vector3 const& a, Vector3 const& b, Vector3 const& c);

        void setAxes(Vector3 const& a, Vector3 const& b, Vector3 const& c);
        void getAxes(Vector3& a, Vector3& b, Vector3& c) const;

    private:
        Vector3 mA;
        Vector3 mB;
        Vector3 mC;
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
        Cone(Vector3 const& centerOfMass, Vector3 const& a, real const h, real const r);

        void setAppex(Vector3 const& a);
        Vector3 getAppex() const;

        void setHeight(real const h);
        real getHeight() const;

        void setRadius(real const r);
        real getRadius() const;

    private:
        Vector3 mA;
        real mH;
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

    bool overlap(Plane const& p, Plane const& s);
    Vector3 calculateContactNormal(Plane const& p, Plane const& s);
    real calculatePenetration(Plane const& p, Plane const& s);

    bool overlap(Plane const& p, Triangle const& t);
    Vector3 calculateContactNormal(Plane const& p, Triangle const& t);
    real calculatePenetration(Plane const& p, Triangle const& t);

    bool overlap(Plane const& p, Sphere const& s);
    Vector3 calculateContactNormal(Plane const& p, Sphere const& s);
    real calculatePenetration(Plane const& p, Sphere const& s);

    bool overlap(Plane const& p, Box const& b);
    Vector3 calculateContactNormal(Plane const& p, Box const& b);
    real calculatePenetration(Plane const& p, Box const& b);

    bool overlap(Triangle const& t, Plane const& p);
    Vector3 calculateContactNormal(Triangle const& t, Plane const& p);
    real calculatePenetration(Triangle const& t, Plane const& p);

    bool overlap(Sphere const& s, Plane const& p);
    Vector3 calculateContactNormal(Sphere const& s, Plane const& p);
    real calculatePenetration(Sphere const& s, Plane const& p);

    bool overlap(const Sphere& s, const Triangle& t);
    Vector3 calculateContactNormal(Sphere const& s, Triangle const& t);
    real calculatePenetration(Sphere const& s, Triangle const& t);

    bool overlap(Sphere const& a, Sphere const& b);
    Vector3 calculateContactNormal(Sphere const& a, Sphere const& b);
    real calculatePenetration(Sphere const& a, Sphere const& b);

    bool overlap(Box const& b, Plane const& p);
    Vector3 calculateContactNormal(Box const& b, Plane const& p);
    real calculatePenetration(Box const& b, Plane const& p);

} // namespace gmt
} // namespace pegas
#endif // PEGAS_GEOMETRY_HPP
