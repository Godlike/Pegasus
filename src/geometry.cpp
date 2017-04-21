#include "Pegas/include/geometry.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

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

pegas::gmt::Triangle::Triangle(pegas::Vector3 const& a, pegas::Vector3 const& b, pegas::Vector3 const& c)
    : SimpleShape((a + b + c) * static_cast<real>(1.0f / 3.0f))
    , mA(a)
    , mB(b)
    , mC(c)
{
}

void pegas::gmt::Triangle::setAxes(pegas::Vector3 const& a, pegas::Vector3 const& b, pegas::Vector3 const& c)
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

pegas::gmt::Cone::Cone(pegas::Vector3 const& centerOfMass, pegas::Vector3 const& a, pegas::real const h, pegas::real const r)
    : SimpleShape(centerOfMass)
    , mA(a)
    , mH(h)
    , mR(r)
{
}

void pegas::gmt::Cone::setAppex(pegas::Vector3 const& a)
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

pegas::gmt::Capsule::Capsule(pegas::Vector3 const& centerOfMass, pegas::Vector3 const& halfHeight, pegas::real const r)
    : SimpleShape(centerOfMass)
    , mHalfHeight(halfHeight)
    , mR(r)
{
}

void pegas::gmt::Capsule::setHalfHeight(pegas::Vector3 const& halfHeight)
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

pegas::gmt::Box::Box(pegas::Vector3 const& centerOfMass, pegas::Vector3 const& a, pegas::Vector3 const& b, pegas::Vector3 const& c)
    : SimpleShape(centerOfMass)
    , mA(a)
    , mB(b)
    , mC(c)
{
}

void pegas::gmt::Box::setAxes(pegas::Vector3 const& a, pegas::Vector3 const& b, pegas::Vector3 const& c)
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

bool pegas::gmt::overlap(Plane const& a, Plane const& b)
{
    auto const& aNormal = a.getNormal();
    auto const& bNormal = b.getNormal();
    auto const& crossProduct = aNormal % bNormal;

    return crossProduct.squareMagnitude() != static_cast<real>(0);
}

pegas::Vector3 pegas::gmt::calculateContactNormal(Plane const& a, Plane const& b)
{
    return b.getNormal();
}

pegas::real pegas::gmt::calculatePenetration(Plane const& a, Plane const& b)
{
    return std::numeric_limits<pegas::real>::max();
}

bool pegas::gmt::overlap(Plane const& p, Triangle const& t)
{
    auto const pNormal = p.getNormal();
    auto const d = p.getCenterOfMass() * pNormal;

    Vector3 a, b, c;
    t.getAxes(a, b, c);

    return (a * pNormal < d) || (b * pNormal < d) || (c * pNormal < d);
}

pegas::Vector3 pegas::gmt::calculateContactNormal(Plane const& p, Triangle const& t)
{
    Vector3 a, b, c;
    t.getAxes(a, b, c);

    auto const dB = b - a;
    auto const dC = c - a;
    auto const tNormal = dB % dC;

    return tNormal.unit();
}

pegas::real pegas::gmt::calculatePenetration(Plane const& p, Triangle const& t)
{
    Vector3 a, b, c;
    t.getAxes(a, b, c);

    auto const pNormal = p.getNormal();
    auto const d = { pNormal * a, pNormal * b, pNormal * c };

    return *std::min_element(d.begin(), d.end());
}

bool pegas::gmt::overlap(Plane const& p, Sphere const& s)
{
    return -calculatePenetration(p, s) > s.getRadius();
}

pegas::Vector3 pegas::gmt::calculateContactNormal(Plane const& p, Sphere const& s)
{
    auto const normal = p.getNormal() - s.getCenterOfMass();
    return normal.unit();
}

pegas::real pegas::gmt::calculatePenetration(Plane const& p, Sphere const& s)
{
    auto const pNormal = p.getNormal();
    auto const a = s.getCenterOfMass() * pNormal;
    auto const b = p.getCenterOfMass() * pNormal;

    return a - b;
}

bool pegas::gmt::overlap(Plane const& p, Box const& b)
{
    Vector3 i, j, k;
    b.getAxes(i, j, k);

    std::array<Vector3, 8> points{(i + j + k), (i - j + k), (j - i + k), (i * -1 - j + k), (i + j - k), (i - j - k), (j - i - k), (i * -1 - j - k)};
    std::for_each(points.begin(), points.end(), [&b](auto & n) { n += b.getCenterOfMass(); });

    auto const pNormal = p.getNormal();
    auto const d = p.getCenterOfMass() * pNormal;

    return std::find_if(points.begin(), points.end(), [&d, &pNormal](auto const& point) { return point * pNormal < d; }) != points.end();
}

pegas::Vector3 pegas::gmt::calculateContactNormal(Plane const& p, Box const& b)
{
    std::array<Vector3, 6> axes;
    b.getAxes(axes[0], axes[1], axes[2]);
    axes[3] = axes[0] * -1;
    axes[4] = axes[1] * -1;
    axes[5] = axes[2] * -1;

    auto const pNormal = p.getNormal();
    auto const d = { axes[0] * pNormal, axes[1] * pNormal, axes[2] * pNormal, axes[3] * pNormal, axes[4] * pNormal, axes[5] * pNormal };

    return axes[std::distance(d.begin(), std::min_element(d.begin(), d.end()))].unit();
}

pegas::real pegas::gmt::calculatePenetration(Plane const& p, Box const& b)
{
    Vector3 i, j, k;
    b.getAxes(i, j, k);

    auto const pNormal = p.getNormal();
    std::array<Vector3, 8> points{(i + j + k), (i - j + k), (j - i + k), (i * -1 - j + k), (i + j - k), (i - j - k), (j - i - k), (i * -1 - j - k)};
    std::for_each(points.begin(), points.end(), [&b](auto & n) { n += b.getCenterOfMass(); });

    std::array<real, 8> penetrations;
    auto const d = p.getCenterOfMass() * p.getNormal();
    std::transform(points.begin(), points.end(), penetrations.begin(), [&pNormal, &d](auto const& p) { return p * pNormal - d; });

    return (*std::min_element(penetrations.begin(), penetrations.end())) * -1;
}

bool pegas::gmt::overlap(Triangle const& t, Plane const& p)
{
    return overlap(p, t);
}

pegas::Vector3 pegas::gmt::calculateContactNormal(Triangle const& t, Plane const& p)
{
    return p.getNormal();
}

pegas::real pegas::gmt::calculatePenetration(Triangle const& t, Plane const& p)
{
    return calculatePenetration(p, t);
}

bool pegas::gmt::overlap(Sphere const& s, Plane const& p)
{
    return overlap(p, s);
}

pegas::Vector3 pegas::gmt::calculateContactNormal(Sphere const& s, Plane const& p)
{
    return p.getNormal();
}

pegas::real pegas::gmt::calculatePenetration(Sphere const& s, Plane const& p)
{
    return calculatePenetration(p, s);
}

bool pegas::gmt::overlap(Sphere const& s, Triangle const& t)
{
    std::array<Vector3, 3> trianglePoints;
    t.getAxes(trianglePoints[0], trianglePoints[1], trianglePoints[2]);
    auto const triangleMassCenter = t.getCenterOfMass();
    std::for_each(trianglePoints.begin(), trianglePoints.end(), [&triangleMassCenter](auto& p) { p += triangleMassCenter; });

    auto const sphereMassCenter = s.getCenterOfMass();
    std::array<real, 3> trianglePointsToSphereSqrDistances;
    std::transform(trianglePoints.begin(), trianglePoints.end(), trianglePointsToSphereSqrDistances.begin(),
                   [&sphereMassCenter](auto const& p) { return (p - sphereMassCenter).squareMagnitude(); });

    std::sort(trianglePointsToSphereSqrDistances.begin(), trianglePointsToSphereSqrDistances.end());
    if (trianglePointsToSphereSqrDistances.front() < std::pow(s.getRadius(), 2)) {
        return true;
    }

    auto const planeNormal = (trianglePoints[1] - trianglePoints[0]) % (trianglePoints[2] - trianglePoints[0]);
    auto const spherePlaneScalarProjection = trianglePoints[0] * planeNormal.unit() - sphereMassCenter * planeNormal.unit();
    if (std::abs(spherePlaneScalarProjection) < s.getRadius())
    {
        return true;
    }

    auto const c = planeNormal * (trianglePoints[0] - sphereMassCenter) / planeNormal.squareMagnitude();
    auto const sphereCenterPlaneProjection = sphereMassCenter + planeNormal * c;

    static auto sameSide = [](auto const & p1, auto const &p2, auto const &a, auto const &b) {
        auto const ab = b - a;
        auto const cp1 = ab.vectorProduct(p1 - a);
        auto const cp2 = ab.vectorProduct(p2 - a);
        return cp1.scalarProduct(cp2) >= 0;
    };

    return sameSide(sphereCenterPlaneProjection, trianglePoints[0], trianglePoints[1], trianglePoints[2])
        && sameSide(sphereCenterPlaneProjection, trianglePoints[1], trianglePoints[0], trianglePoints[2])
        && sameSide(sphereCenterPlaneProjection, trianglePoints[2], trianglePoints[0], trianglePoints[1]);
}

pegas::Vector3 pegas::gmt::calculateContactNormal(Sphere const& s, Triangle const & t)
{
    std::array<Vector3, 3> trianglePoints;
    t.getAxes(trianglePoints[0], trianglePoints[1], trianglePoints[2]);

    auto const planeNormal = ((trianglePoints[1] - trianglePoints[0]) % (trianglePoints[2] - trianglePoints[0])).unit();
    auto const contactNormalScalar = (s.getCenterOfMass() - trianglePoints[0]) * planeNormal;
    auto const contactNormal = planeNormal * contactNormalScalar;

    return contactNormal;
}

pegas::real pegas::gmt::calculatePenetration(Sphere const& s, Triangle const& t)
{
    std::array<Vector3, 3> trianglePoints;
    t.getAxes(trianglePoints[0], trianglePoints[1], trianglePoints[2]);

    auto const planeNormal = (trianglePoints[1] - trianglePoints[0]) % (trianglePoints[2] - trianglePoints[0]);
    auto const planeDistance = t.getCenterOfMass() * planeNormal.unit();
    auto const sphereDistance = s.getCenterOfMass() * planeNormal.unit();

    return planeDistance - (sphereDistance - s.getRadius());
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
    return (aMassCenter - bMassCenter).unit();
}

pegas::real pegas::gmt::calculatePenetration(Sphere const& a, Sphere const& b)
{
    auto const& aMassCenter = a.getCenterOfMass();
    auto const& bMassCenter = b.getCenterOfMass();
    return (a.getRadius() + b.getRadius() - (aMassCenter - bMassCenter).magnitude());
}

bool pegas::gmt::overlap(Box const& b, Plane const& p)
{
    return overlap(p, b);
}

pegas::Vector3 pegas::gmt::calculateContactNormal(Box const& b, Plane const& p)
{
    return p.getNormal();
}

pegas::real pegas::gmt::calculatePenetration(Box const& b, Plane const& p)
{
    return calculatePenetration(p, b);
}