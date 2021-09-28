cmake_minimum_required(VERSION 3.14)
project(
  Robotics
  VERSION 1.0
  LANGUAGES CXX
)

if(PROJECT_SOURCE_DIR STREQUAL PROJECT_BINARY_DIR)
  message(
    FATAL_ERROR
      "In-source builds not allowed. Please make a new directory (called a build directory) and run CMake from there."
  )
endif()

include(cmake/tools.cmake)
include(cmake/CPM.cmake)

# FIXME: doesn't seem to be caching packages
add_compile_definitions(CPM_SOURCE_CACHE="${CMAKE_BINARY_DIR}/cache/CPM")

CPMAddPackage(
  NAME Eigen3
  GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
  GIT_TAG 3.3.9
  # URL https://gitlab.com/libeigen/eigen/-/archive/3.3.9/eigen-3.3.9.tar.gz Eigen3's CMakelists are
  # not intended for library use
  DOWNLOAD_ONLY YES
)
add_library(Eigen3::Eigen INTERFACE IMPORTED)
target_include_directories(Eigen3::Eigen INTERFACE ${Eigen3_SOURCE_DIR})

add_library(Robotics INTERFACE)
target_include_directories(
  Robotics INTERFACE $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
                     $<INSTALL_INTERFACE:include/${PROJECT_NAME}-${PROJECT_VERSION}>
)
target_link_libraries(Robotics INTERFACE Eigen3::Eigen)
target_compile_features(Robotics INTERFACE cxx_std_17)

CPMAddPackage("gh:TheLartians/PackageProject.cmake@1.6.0")
string(TOLOWER ${PROJECT_NAME}/version.h VERSION_HEADER_LOCATION)

packageProject(
  NAME ${PROJECT_NAME}
  VERSION ${PROJECT_VERSION}
  NAMESPACE ${PROJECT_NAME}
  BINARY_DIR ${PROJECT_BINARY_DIR}
  INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include
  INCLUDE_DESTINATION include/${PROJECT_NAME}-${PROJECT_VERSION}
  VERSION_HEADER "${VERSION_HEADER_LOCATION}"
  COMPATIBILITY SameMajorVersion
  DEPENDENCIES "Eigen3 3.3.9"
)

if(NOT CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
  return()
endif()

include(CTest)
add_subdirectory(documentation)
add_subdirectory(examples)
add_subdirectory(tests)
