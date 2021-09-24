#include <doctest/doctest.h>
#include <robotics/version.h>

#include <string>

TEST_CASE("Robotics version") {
  static_assert(std::string_view(GREETER_VERSION) == std::string_view("1.0"));
  CHECK(std::string(GREETER_VERSION) == std::string("1.0"));
}