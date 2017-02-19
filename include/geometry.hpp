#ifndef PEGAS_GEOMETRY_HPP
#define PEGAS_GEOMETRY_HPP

#include <cmath>
#include <memory>

#include "Pegas/include/math.hpp"

namespace pegas {


class Geometry {
public:
    virtual ~Geometry();
};


class Shape : public Geometry {
public:
    Shape(Vector3 const& centerOfMass);

    void setCenterOfMass(Vector3 const& centerOfMass);

    Vector3 getCenterOfMass() const;

private:
    Vector3 mCenterOfMass;
};


class SimpleShape : public Shape {
public:
    SimpleShape(Vector3 const& centerOfMass);
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

    Sphere(Vector3 const& centerOfMass, real const r);

    void setRadius(real const r);

    real getRadius() const;

	bool overlap(Sphere::Ptr const & other) const
	{
		Vector3 const & cm1 = getCenterOfMass();
		Vector3 const & cm2 = other->getCenterOfMass();
		auto const a = (cm2 - cm1).squareMagnitude();
		auto const b = (mR + other->mR) * (mR + other->mR);
		return b > a;
	}

	Vector3 calculateContactNormal(Sphere::Ptr const & other) const
	{
		Vector3 const cm1 = getCenterOfMass();
		Vector3 const cm2 = other->getCenterOfMass();
		Vector3 result(cm2 - cm1);
		result.normalize();
		return result;
	}

	real calculatePenetration(Sphere::Ptr const & other) const
	{
		Vector3 const & cm1 = getCenterOfMass();
		Vector3 const & cm2 = other->getCenterOfMass();
		return mR + other->mR - (cm2 - cm1).magnitude();
	}

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
} // namespace pegas

#endif // PEGAS_GEOMETRY_HPP
