option(BUILD_EXAMPLES "Build the examples" ON)

if(NOT BUILD_EXAMPLES)
  return()
endif()

CPMAddPackage(
  NAME matplotplusplus
  GITHUB_REPOSITORY alandefreitas/matplotplusplus
  GIT_TAG master
  OPTIONS "BUILD_EXAMPLES OFF" "BUILD_TESTS OFF" "BUILD_INSTALLER OFF" "BUILD_PACKAGE OFF"
)

if(CMAKE_CXX_COMPILER_ID MATCHES "(Clang|GNU)")
  add_compile_options(-Werror -Wall -Wextra -Wpedantic)
elseif(MSVC)
  add_compile_options(/WX /W4 /permissive-)
endif()

add_subdirectory(extended_kalman_filter)
add_subdirectory(infinite_horizon_lqr)
add_subdirectory(infinite_horizon_lqr_plot)
add_subdirectory(pid)
