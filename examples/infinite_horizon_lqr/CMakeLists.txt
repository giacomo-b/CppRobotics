cmake_minimum_required(VERSION 3.14 FATAL_ERROR)

project(InfiniteHorizonLQR LANGUAGES CXX)

# --- Import tools ----

include(../../cmake/tools.cmake)

# ---- Dependencies ----

include(../../cmake/CPM.cmake)

CPMAddPackage(
  NAME matplotplusplus
  GITHUB_REPOSITORY alandefreitas/matplotplusplus
  GIT_TAG origin/master
)

CPMAddPackage(NAME Robotics SOURCE_DIR ${CMAKE_CURRENT_LIST_DIR}/../..)

# ---- Create standalone executable ----

file(GLOB sources CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)

add_executable(InfiniteHorizonLQR ${sources})

set_target_properties(InfiniteHorizonLQR PROPERTIES CXX_STANDARD 17 OUTPUT_NAME "main")

target_link_libraries(InfiniteHorizonLQR Robotics::Robotics matplot)
