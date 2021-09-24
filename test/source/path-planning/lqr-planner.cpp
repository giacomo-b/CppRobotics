#include <doctest/doctest.h>
#include <robotics/path-planning/lqr-planner.h>

#include <string>

TEST_CASE("Robotics") {
  using namespace robotics;

  Robotics robotics("Tests");

  CHECK(robotics.greet(LanguageCode::EN) == "Hello, Tests!");
  CHECK(robotics.greet(LanguageCode::DE) == "Hallo Tests!");
  CHECK(robotics.greet(LanguageCode::ES) == "¡Hola Tests!");
  CHECK(robotics.greet(LanguageCode::FR) == "Bonjour Tests!");
}