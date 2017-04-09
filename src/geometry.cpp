#include "Pegas/include/geometry.hpp"

pegas::gmt::Geometry::~Geometry() {}


pegas::gmt::Shape::Shape(pegas::Vector3 const& centerOfMass)
    : mCenterOfMass(centerOfMass)
{
}

void pegas::gmt::Shape::setCenterOfMass(pegas::Vector3 const& centerOfMass)
{
    mCenterOfMass = centerOfMass;
}

pegas::Vector3 pegas::gmt::Shape::getCenterOfMass() const
{
    return mCenterOfMass;
}

pegas::gmt::SimpleShape::SimpleShape(pegas::Vector3 const& centerOfMass)
    : Shape(centerOfMass)
{
}


pegas::gmt::Plane::Plane(pegas::Vector3 const& centerOfMass, pegas::Vector3 const& normal)
    : SimpleShape(centerOfMass)
    , mNormal(normal)
{
}

void pegas::gmt::Plane::setNormal(pegas::Vector3 const& normal)
{
    mNormal = normal;
}

pegas::Vector3 pegas::gmt::Plane::getNormal() const
{
    return mNormal;
}


pegas::gmt::Triangle::Triangle(pegas::Vector3 const & a, pegas::Vector3 const & b, pegas::Vector3 const & c)
    : SimpleShape((a + b + c) * static_cast<real>(1.0f / 3.0f))
    , mA(a)
    , mB(b)
    , mC(c)
{
}

void pegas::gmt::Triangle::setAxes(pegas::Vector3 const & a, pegas::Vector3 const & b, pegas::Vector3 const & c)
{
    mA = a;
    mB = b;
    mC = c;
}

void pegas::gmt::Triangle::getAxes(pegas::Vector3& a, pegas::Vector3& b, pegas::Vector3& c) const
{
    a = mA;
    b = mB;
    c = mC;
}

pegas::gmt::Sphere::Sphere(pegas::Vector3 const& centerOfMass, pegas::real const r)
    : SimpleShape(centerOfMass)
    , mR(r)
{
}

void pegas::gmt::Sphere::setRadius(pegas::real const r)
{
    mR = r;
}

pegas::real pegas::gmt::Sphere::getRadius() const
{
    return mR;
}


pegas::gmt::Cone::Cone(pegas::Vector3 const & centerOfMass, pegas::Vector3 const & a, pegas::real const h, pegas::real const r)
    : SimpleShape(centerOfMass)
    , mA(a)
    , mH(h)
    , mR(r)
{
}

void pegas::gmt::Cone::setAppex(pegas::Vector3 const & a)
{
    mA = a;
}

pegas::Vector3 pegas::gmt::Cone::getAppex() const
{
    return mA;
}

void pegas::gmt::Cone::setHeight(pegas::real const h)
{
    mH = h;
}

pegas::real pegas::gmt::Cone::getHeight() const
{
    return mH;
}

void pegas::gmt::Cone::setRadius(pegas::real const r)
{
    mR = r;
}

pegas::real pegas::gmt::Cone::getRadius() const
{
    return mR;
}


pegas::gmt::Capsule::Capsule(pegas::Vector3 const & centerOfMass, pegas::Vector3 const & halfHeight, pegas::real const r)
    : SimpleShape(centerOfMass)
    , mHalfHeight(halfHeight)
    , mR(r)
{
}

void pegas::gmt::Capsule::setHalfHeight(pegas::Vector3 const & halfHeight)
{
    mHalfHeight = halfHeight;
}

pegas::Vector3 pegas::gmt::Capsule::getHalfHeight() const
{
    return mHalfHeight;
}

void pegas::gmt::Capsule::setRadius(pegas::real const r)
{
    mR = r;
}

pegas::real pegas::gmt::Capsule::getRadius() const
{
    return mR;
}


pegas::gmt::Box::Box(pegas::Vector3 const & centerOfMass, pegas::Vector3 const & a, pegas::Vector3 const & b, pegas::Vector3 const & c)
    : SimpleShape(centerOfMass)
    , mA(a)
    , mB(b)
    , mC(c)
{
}


void pegas::gmt::Box::setAxes(pegas::Vector3 const & a, pegas::Vector3 const & b, pegas::Vector3 const & c)
{
    mA = a;
    mB = b;
    mC = c;
}

void pegas::gmt::Box::getAxes(pegas::Vector3& a, pegas::Vector3& b, pegas::Vector3& c) const
{
    a = mA;
    b = mB;
    c = mC;
}


bool pegas::gmt::overlap(Plane const & a, Plane const & b)
{
	auto const & aNormal = a.getNormal();
	auto const & bNormal = b.getNormal();
	auto const & crossProduct = aNormal % bNormal;

	return crossProduct.squareMagnitude() != static_cast<real>(0);
}

pegas::Vector3 pegas::gmt::calculateContactNormal(Plane const & a, Plane const & b)
{
	auto result(a.getNormal() - b.getNormal());
	result.normalize();
	return result;
}

pegas::real pegas::gmt::calculatePenetration(Plane const & a, Plane const & b)
{
	return std::numeric_limits<pegas::real>::max();
}


bool pegas::gmt::overlap(Plane const& p, Sphere const& s)
{
	return -calculatePenetration(p, s) > s.getRadius();
}

pegas::Vector3 pegas::gmt::calculateContactNormal(Plane const& p, Sphere const& s)
{
	auto const & pNormal = p.getNormal();
	auto const & pPoint = p.getCenterOfMass();
	auto const & sMassCenter = s.getCenterOfMass();

	auto result(pNormal * pPoint.magnitude() - sMassCenter);
	result.normalize();
	return result;
}

pegas::real pegas::gmt::calculatePenetration(Plane const& p, Sphere const& s)
{
	auto const& sMassCenter = s.getCenterOfMass();
	auto const& pNormal = p.getNormal();
	auto const dist = (pNormal.x * sMassCenter.x + pNormal.y * sMassCenter.y + pNormal.z * sMassCenter.z)
		+ p.getCenterOfMass().magnitude();

	return -dist;
}


bool pegas::gmt::overlap(Sphere const& a, Sphere const& b)
{
	auto const& aMassCenter = a.getCenterOfMass();
	auto const& bMassCenter = b.getCenterOfMass();
	auto const distSqr = (aMassCenter - bMassCenter).squareMagnitude();
	auto const radiusSqr = std::pow(a.getRadius() + b.getRadius(), 2);
	return radiusSqr > distSqr;
}

pegas::Vector3 pegas::gmt::calculateContactNormal(Sphere const& a, Sphere const& b)
{
	auto const& aMassCenter = a.getCenterOfMass();
	auto const& bMassCenter = b.getCenterOfMass();
	auto result(aMassCenter - bMassCenter);
	result.normalize();
	return result;
}

pegas::real pegas::gmt::calculatePenetration(Sphere const& a, Sphere const& b)
{
	auto const& aMassCenter = a.getCenterOfMass();
	auto const& bMassCenter = b.getCenterOfMass();
	return (a.getRadius() + b.getRadius() - (aMassCenter - bMassCenter).magnitude());
}

