/*
 * Copyright (C) 2018 by Godlike
 * This code is licensed under the MIT license (MIT)
 * (http://opensource.org/licenses/MIT)
 */
#define CATCH_CONFIG_MAIN
#include <catch.hpp>

#include <pegasus/Body.hpp>
#include <pegasus/Integration.hpp>
#include <Epona/FloatingPoint.hpp>
#include <glm/glm.hpp>

TEST_CASE("Integration", "[integration]")
{
    glm::vec3 const force = pegasus::integration::IntegrateForce({0, 0, 0}, {1, 1, 1});
    REQUIRE(
        true == epona::fp::IsZero(glm::distance(force, glm::vec3{1, 1, 1}))
    );
}

TEST_CASE("Angular integration torque", "[integration]")
{
    glm::vec3 const torque{ 0, 1, 0 };
    pegasus::mechanics::Body body;
    body.angularMotion.torque = pegasus::integration::IntegrateTorque(glm::vec3{ 0 }, torque);
    pegasus::integration::Integrate(body, 1.0);

    REQUIRE(
        true == epona::fp::IsZero(glm::length(body.angularMotion.torque))
    );
    REQUIRE(
        true == epona::fp::IsZero(glm::distance(body.angularMotion.velocity, torque))
    );
}
