if(NOT BUILD_TESTING)
  return()
endif()

option(ENABLE_TEST_COVERAGE "Enable test coverage" OFF)

CPMAddPackage("gh:onqtam/doctest#2.4.5")
CPMAddPackage("gh:TheLartians/Format.cmake@1.7.0")

if(CMAKE_CXX_COMPILER_ID MATCHES "(Clang|GNU)")
  add_compile_options(-Werror -Wall -Wextra -Wpedantic)
elseif(MSVC)
  add_compile_options(/WX /W4 /permissive-)
endif()

add_executable(RoboticsTests main.cpp robotics.cpp linear_control/lqr.cpp)
target_link_libraries(RoboticsTests PRIVATE Robotics::Robotics doctest::doctest)
if(MSVC)
  target_compile_definitions(RoboticsTests PRIVATE DOCTEST_CONFIG_USE_STD_HEADERS)
endif()

include(${doctest_SOURCE_DIR}/scripts/cmake/doctest.cmake)
doctest_discover_tests(RoboticsTests)

if(ENABLE_TEST_COVERAGE)
  target_compile_options(Robotics INTERFACE -O0 -g -fprofile-arcs -ftest-coverage)
  target_link_options(Robotics INTERFACE -fprofile-arcs -ftest-coverage)
endif()
