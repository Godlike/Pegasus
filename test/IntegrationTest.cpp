/*
 * Copyright (C) 2017 by Godlike
 * This code is licensed under the MIT license (MIT)
 * (http://opensource.org/licenses/MIT)
 */
#define CATCH_CONFIG_MAIN
#include <catch.hpp>

#include <pegasus/Integration.hpp>
#include <Epona/FloatingPoint.hpp>

TEST_CASE("Integration", "[integration]")
{
    glm::dvec3 const force = pegasus::integration::IntegrateForce({0, 0, 0}, {1, 1, 1});
    REQUIRE(
        true == epona::fp::IsZero(glm::distance(force, glm::dvec3{1, 1, 1}))
    );
}
