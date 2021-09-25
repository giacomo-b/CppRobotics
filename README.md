<!-- [![Actions Status](https://github.com/TheLartians/ModernCppStarter/workflows/MacOS/badge.svg)](https://github.com/TheLartians/ModernCppStarter/actions)
[![Actions Status](https://github.com/TheLartians/ModernCppStarter/workflows/Windows/badge.svg)](https://github.com/TheLartians/ModernCppStarter/actions)
[![Actions Status](https://github.com/TheLartians/ModernCppStarter/workflows/Ubuntu/badge.svg)](https://github.com/TheLartians/ModernCppStarter/actions)
[![Actions Status](https://github.com/TheLartians/ModernCppStarter/workflows/Style/badge.svg)](https://github.com/TheLartians/ModernCppStarter/actions)
[![Actions Status](https://github.com/TheLartians/ModernCppStarter/workflows/Install/badge.svg)](https://github.com/TheLartians/ModernCppStarter/actions)
[![codecov](https://codecov.io/gh/TheLartians/ModernCppStarter/branch/master/graph/badge.svg)](https://codecov.io/gh/TheLartians/ModernCppStarter)

<p align="center">
  <img src="https://repository-images.githubusercontent.com/254842585/4dfa7580-7ffb-11ea-99d0-46b8fe2f4170" height="175" width="auto" />
</p> -->

<img src="https://github.com/AtsushiSakai/PythonRobotics/raw/master/icon.png?raw=true" align="right" width="300" alt="header pic"/>

# CppRobotics

<!-- ![GitHub_Action_Linux_CI](https://github.com/AtsushiSakai/PythonRobotics/workflows/Linux_CI/badge.svg)
![GitHub_Action_MacOS_CI](https://github.com/AtsushiSakai/PythonRobotics/workflows/MacOS_CI/badge.svg)
[![Build Status](https://travis-ci.org/AtsushiSakai/PythonRobotics.svg?branch=master)](https://travis-ci.org/AtsushiSakai/PythonRobotics)
[![Documentation Status](https://readthedocs.org/projects/pythonrobotics/badge/?version=latest)](https://pythonrobotics.readthedocs.io/en/latest/?badge=latest)
[![Build status](https://ci.appveyor.com/api/projects/status/sb279kxuv1be391g?svg=true)](https://ci.appveyor.com/project/AtsushiSakai/pythonrobotics)
[![codecov](https://codecov.io/gh/AtsushiSakai/PythonRobotics/branch/master/graph/badge.svg)](https://codecov.io/gh/AtsushiSakai/PythonRobotics)
[![Language grade: Python](https://img.shields.io/lgtm/grade/python/g/AtsushiSakai/PythonRobotics.svg?logo=lgtm&logoWidth=18)](https://lgtm.com/projects/g/AtsushiSakai/PythonRobotics/context:python)
[![CodeFactor](https://www.codefactor.io/repository/github/atsushisakai/pythonrobotics/badge/master)](https://www.codefactor.io/repository/github/atsushisakai/pythonrobotics/overview/master)
[![tokei](https://tokei.rs/b1/github/AtsushiSakai/PythonRobotics)](https://github.com/AtsushiSakai/PythonRobotics) -->

C++ algorithms for robotics, based on [AtsushiSakai/PythonRobotics](https://github.com/AtsushiSakai/PythonRobotics).

Add a star if you like the repo, and make sure to add one to the original Python repo as well! :smiley:.

This is a work in progress, and any help or suggestion would be greatly appreciated!

## Table of Contents

* [What is this?](#what-is-this)
* [Requirements](#requirements)
* [Documentation](#documentation)
* [How to use](#how-to-use)
* [TODOs](#todos)

## What is this?

This is a C++ code collection of robotics algorithms.

Features:

1. Easy to read for understanding each algorithm's basic idea.

2. Widely used and practical algorithms are selected.

3. Minimum dependency.

See this paper for more details on the original Python repo:

- [\[1808\.10703\] PythonRobotics: a Python code collection of robotics algorithms](https://arxiv.org/abs/1808.10703) ([BibTeX](https://github.com/AtsushiSakai/PythonRoboticsPaper/blob/master/python_robotics.bib))


## Requirements

* A C++11 compiler

* CMake

* The dependencies will be obtained automatically by [CPM.cmake](https://github.com/TheLartians/CPM.cmake)

## Documentation

This README only shows some examples of this project. 

If you are interested in other examples or mathematical backgrounds of each algorithm, 

You can check the full documentation online: [https://pythonrobotics.readthedocs.io/](https://pythonrobotics.readthedocs.io/)

All animation gifs are stored here: [AtsushiSakai/PythonRoboticsGifs: Animation gifs of PythonRobotics](https://github.com/AtsushiSakai/PythonRoboticsGifs)

## How to use

### Clone this repo

> git clone https://github.com/giacomo-b/CppRobotics.git

### Build and run the standalone target

Use the following commands to build and run the examples:

```bash
cmake -S examples/infinite_horizon_lqr -B build/infinite_horizon_lqr
cmake --build build/infinite_horizon_lqr
./build/infinite_horizon_lqr/main --help
```

### Build and run test suite

Use the following commands from the project's root directory to run the test suite.

```bash
cmake -S test -B build/test
cmake --build build/test
CTEST_OUTPUT_ON_FAILURE=1 cmake --build build/test --target test

# or simply call the executable: 
./build/test/RoboticsTests
```

To collect code coverage information, run CMake with the `-DENABLE_TEST_COVERAGE=1` option.

### Run clang-format

Use the following commands from the project's root directory to check and fix C++ and CMake source style.
This requires _clang-format_, _cmake-format_ and _pyyaml_ to be installed on the current system.

```bash
cmake -S test -B build/test

# view changes
cmake --build build/test --target format

# apply changes
cmake --build build/test --target fix-format
```

See [Format.cmake](https://github.com/TheLartians/Format.cmake) for details.

### Build the documentation

The documentation is automatically built and [published](https://thelartians.github.io/ModernCppStarter) whenever a [GitHub Release](https://help.github.com/en/github/administering-a-repository/managing-releases-in-a-repository) is created.
To manually build documentation, call the following command.

```bash
cmake -S documentation -B build/doc
cmake --build build/doc --target GenerateDocs
# view the docs
open build/doc/doxygen/html/index.html
```

To build the documentation locally, you will need Doxygen, jinja2 and Pygments on installed your system.

### Build everything at once

The project also includes an `all` directory that allows building all targets at the same time.
This is useful during development, as it exposes all subprojects to your IDE and avoids redundant builds of the library.

```bash
cmake -S all -B build
cmake --build build

# run tests
./build/test/RoboticsTests
# format code
cmake --build build --target fix-format
# run standalone
./build/standalone/Robotics --help
# build docs
cmake --build build --target GenerateDocs
```

### Additional tools

The test and standalone subprojects include the [tools.cmake](cmake/tools.cmake) file which is used to import additional tools on-demand through CMake configuration arguments.
The following are currently supported.

#### Sanitizers

Sanitizers can be enabled by configuring CMake with `-DUSE_SANITIZER=<Address | Memory | MemoryWithOrigins | Undefined | Thread | Leak | 'Address;Undefined'>`.

#### Static Analyzers

Static Analyzers can be enabled by setting `-DUSE_STATIC_ANALYZER=<clang-tidy | iwyu | cppcheck>`, or a combination of those in quotation marks, separated by semicolons.
By default, analyzers will automatically find configuration files such as `.clang-format`.
Additional arguments can be passed to the analyzers by setting the `CLANG_TIDY_ARGS`, `IWYU_ARGS` or `CPPCHECK_ARGS` variables.

#### Ccache

Ccache can be enabled by configuring with `-DUSE_CCACHE=<ON | OFF>`.

## TODOs

- [ ] Localization  
  - [ ] Extended Kalman Filter localization  
  - [ ] Particle filter localization  
  - [ ] Histogram filter localization  
- [ ] Mapping  
  - [ ] Gaussian grid map  
  - [ ] Ray casting grid map  
  - [ ] Lidar to grid map
  - [ ] k-means object clustering  
  - [ ] Rectangle fitting  
- [ ] SLAM  
  - [ ] Iterative Closest Point (ICP) Matching  
  - [ ] FastSLAM 1.0


- [ ] Path Planning  
  - [ ] Dynamic Window Approach  
  - [ ] Grid based search  
    - [ ] Dijkstra algorithm  
    - [ ] A\* algorithm  
    - [ ] D\* algorithm  
    - [ ] D\* Lite algorithm  
    - [ ] Potential Field algorithm  
    - [ ] Grid based coverage path planning  
  - [ ] State Lattice Planning  
    - [ ] Biased polar sampling  
    - [ ] Lane sampling  
  - [ ] Probabilistic Road-Map (PRM) planning   
  - [ ] Rapidly-Exploring Random Trees (RRT)  
    - [ ] RRT\*  
    - [ ] RRT\* with reeds-shepp path  
    - [ ] LQR-RRT\*  
  - [ ] Quintic polynomials planning  
  - [ ] Reeds Shepp planning  
  - [ ] LQR based path planning  
  - [ ] Optimal Trajectory in a Frenet Frame  
- [ ] Path Tracking  
  - [ ] move to a pose control  
  - [ ] Stanley control  
  - [ ] Rear wheel feedback control  
  - [ ] Linear–quadratic regulator (LQR) speed and steering control  
  - [ ] Model predictive speed and steering control  
  - [ ] Nonlinear Model predictive control with C-GMRES  
- [ ] Arm Navigation  
  - [ ] N joint arm to point control  
  - [ ] Arm navigation with obstacle avoidance  
- [ ] Aerial Navigation  
  - [ ] Drone 3D trajectory following  
  - [ ] Rocket powered landing  
- [ ] Bipedal  
  - [ ] bipedal planner with inverted pendulum  

