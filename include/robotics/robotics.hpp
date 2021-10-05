#pragma once

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32)
#    define _USE_MATH_DEFINES
#endif

#include <robotics/classical-control/classical-control.hpp>
#include <robotics/common.hpp>
#include <robotics/estimation/estimation.hpp>
#include <robotics/linear-control/linear-control.hpp>
#include <robotics/system/system.hpp>