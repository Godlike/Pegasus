/*
 * Copyright (C) 2017 by Godlike
 * This code is licensed under the MIT license (MIT)
 * (http://opensource.org/licenses/MIT)
 */
#define CATCH_CONFIG_MAIN
#include <catch.hpp>

#include <geometry/Shape.hpp>
#include <geometry/SimpleShapeIntersectionDetector.hpp>

TEST_CASE("Sphere-Sphere Collision", "[collision][sphere]")
{
    using namespace pegasus::geometry;

    SimpleShapeIntersectionDetector detector;
    Sphere aSphere(glm::dvec3(0, 0, 0), 2.0);
    Sphere bSphere(glm::dvec3(0, 0, 1), 2.0);

    REQUIRE(true == detector.CalculateIntersection(&aSphere, &bSphere));
}
