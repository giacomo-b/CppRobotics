#include <doctest/doctest.h>
#include <robotics/path-planning/lqr-planner.h>

#include <string>

TEST_CASE("Robotics")
{
    using namespace Robotics::PathPlanning;

    LQRPlanner planner("Tests");

    CHECK(planner.greet(LanguageCode::EN) == "Hello, Tests!");
    CHECK(planner.greet(LanguageCode::DE) == "Hallo Tests!");
    CHECK(planner.greet(LanguageCode::ES) == "¡Hola Tests!");
    CHECK(planner.greet(LanguageCode::FR) == "Bonjour Tests!");
}