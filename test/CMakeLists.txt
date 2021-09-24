cmake_minimum_required(VERSION 3.14 FATAL_ERROR)

project(RoboticsTests LANGUAGES CXX)

# ---- Options ----

option(ENABLE_TEST_COVERAGE "Enable test coverage" OFF)
option(TEST_INSTALLED_VERSION "Test the version found by find_package" OFF)

# --- Import tools ----

include(../cmake/tools.cmake)

# ---- Dependencies ----

include(../cmake/CPM.cmake)

CPMAddPackage("gh:onqtam/doctest#2.4.5")
CPMAddPackage("gh:TheLartians/Format.cmake@1.7.0")

if(TEST_INSTALLED_VERSION)
  find_package(Robotics REQUIRED)
else()
  CPMAddPackage(NAME Robotics SOURCE_DIR ${CMAKE_CURRENT_LIST_DIR}/..)
endif()

# ---- Create binary ----

file(GLOB_RECURSE sources CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/source/*.cpp)
add_executable(RoboticsTests ${sources})
target_link_libraries(RoboticsTests doctest::doctest Robotics::Robotics)
set_target_properties(RoboticsTests PROPERTIES CXX_STANDARD 17)

# enable compiler warnings
if(NOT TEST_INSTALLED_VERSION)
  if(CMAKE_CXX_COMPILER_ID MATCHES "Clang" OR CMAKE_CXX_COMPILER_ID MATCHES "GNU")
    target_compile_options(Robotics INTERFACE -Wall -Wpedantic -Wextra -Werror)
  elseif(MSVC)
    target_compile_options(Robotics INTERFACE /W4 /WX)
    target_compile_definitions(RoboticsTests INTERFACE DOCTEST_CONFIG_USE_STD_HEADERS)
  endif()
endif()

# ---- Add RoboticsTests ----

enable_testing()

# Note: doctest and similar testing frameworks can automatically configure CMake tests. For other
# testing frameworks add the tests target instead: add_test(NAME roboticsTests COMMAND
# RoboticsTests)

include(${doctest_SOURCE_DIR}/scripts/cmake/doctest.cmake)
doctest_discover_tests(RoboticsTests)

# ---- code coverage ----

if(ENABLE_TEST_COVERAGE)
  target_compile_options(Robotics PUBLIC -O0 -g -fprofile-arcs -ftest-coverage)
  target_link_options(Robotics PUBLIC -fprofile-arcs -ftest-coverage)
endif()
