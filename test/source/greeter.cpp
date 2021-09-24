#include <doctest/doctest.h>
#include <robotics/robotics.h>
#include <robotics/version.h>

#include <string>

TEST_CASE("Robotics") {
  using namespace robotics;

  Robotics robotics("Tests");

  CHECK(robotics.greet(LanguageCode::EN) == "Hello, Tests!");
  CHECK(robotics.greet(LanguageCode::DE) == "Hallo Tests!");
  CHECK(robotics.greet(LanguageCode::ES) == "¡Hola Tests!");
  CHECK(robotics.greet(LanguageCode::FR) == "Bonjour Tests!");
}

TEST_CASE("Robotics version") {
  static_assert(std::string_view(GREETER_VERSION) == std::string_view("1.0"));
  CHECK(std::string(GREETER_VERSION) == std::string("1.0"));
}
