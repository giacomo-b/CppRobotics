[![Actions Status](https://github.com/giacomo-b/CppRobotics/workflows/MacOS/badge.svg)](https://github.com/giacomo/CppRobotics/actions)
[![Actions Status](https://github.com/giacomo-b/CppRobotics/workflows/Windows/badge.svg)](https://github.com/giacomo/CppRobotics/actions)
[![Actions Status](https://github.com/giacomo-b/CppRobotics/workflows/Ubuntu/badge.svg)](https://github.com/giacomo/CppRobotics/actions)
[![Actions Status](https://github.com/giacomo-b/CppRobotics/workflows/Style/badge.svg)](https://github.com/giacomo/CppRobotics/actions)
![CodeFactor Grade](https://img.shields.io/codefactor/grade/github/giacomo-b/CppRobotics/master)
<!--[![codecov](https://codecov.io/gh/TheLartians/ModernCppStarter/branch/master/graph/badge.svg)](https://codecov.io/gh/TheLartians/CppRobotics)

 <p align="center">
  <img src="https://repository-images.githubusercontent.com/254842585/4dfa7580-7ffb-11ea-99d0-46b8fe2f4170" height="175" width="auto" />
</p> -->

<img src="https://github.com/AtsushiSakai/PythonRobotics/raw/master/icon.png?raw=true" align="right" width="300" alt="header pic"/>

# CppRobotics

A header-only, fully-templated C++ library for robotics algorithms.

This repo is inspired by [AtsushiSakai/PythonRobotics](https://github.com/AtsushiSakai/PythonRobotics). Instead of being just educational, this repo focuses on runtime speed and modularity. The `examples` folder contains short tutorials for each implemented algorithm.

While this is still a work in progress, I would really appreciate it if you left any suggestion or just a star. Any help is greatly appreciated!

* [Goal](#goal)
* [Requirements](#requirements)
* [Getting started](#getting-started)
* [TODOs](#todos)
* [References](#references)

## Goal

The main goal for this library is to:

1. Implement as many robotics algorithms as possible, without sacrificing quality. These include, e.g., control, path planning, and estimation algorithms.

2. Be easy to use and to get started with, thus the header-only and minimal external dependencies.

3. Be fast: by making use of templates and by relying on `Eigen`, most algorithms use static-size data structures known at compilation time.

## Requirements

* C++11 compiler

* CMake 3.14+ (if using Visual Studio on Windows you should be able to import the project as a CMake project)

* The C++ dependencies will be obtained automatically by [CPM.cmake](https://github.com/TheLartians/CPM.cmake). Note that the library only depends on `Eigen`, but `matplotplusplus` is used in the `examples` folder to plot the results.

## Getting started

Following are some examples to get started. The `examples` folder contains several examples that are useful to get started.

CppRobotics aims to be modular, which means:

* Once you define a dynamical system, most algorithms will be readily available for it to use

* Data should flow seamlessly between objects (e.g. estimator -> controller)

* Once you setup an algorithm, you should be able to change the dynamical system and integrate it directly

### Clone this repo

> git clone <https://github.com/giacomo-b/CppRobotics.git>

### Building and running the examples

Given a generic `EXAMPLE` that you want to run, the following commands build it:

```bash
cmake -S examples/EXAMPLE -B build/EXAMPLE
cmake --build build/EXAMPLE
```

On Windows, this will default to a Debug configuration. To build the project in release mode, you can add `--config=Release` after the first command.

To run the example on Linux, macOS and most Unix-based systems:

```bash
./build/EXAMPLE/main
```

On Windows:

```bash
./build/EXAMPLE/CONFIG_TYPE/main
```

where `CONFIG_TYPE` is either `Debug` or `Release`, depending on how you configured the project.

### Using the library in your own projects

#### Importing the library

```cpp
#include <robotics/robotics.h>
```

#### System definition

`SystemBase` represents a generic dynamical system. In most cases, you will be using either a `LinearSystem` or `NonlinearSystem`.

Since the library is templated, in order to define a system you need to define:

* The number of states

* The number of inputs (control actions)

* The number of outputs

e.g.:

```cpp
static constexpr int N = 2; // Number of states
static constexpr int M = 1; // Number of inputs
static constexpr int P = 2; // Number of outputs
```

Some of the following type aliases can be helpful to simplify your code and to avoid mixing up the wrong dimensions:

```cpp
using State = Robotics::ColumnVector<N>;
using Input = Robotics::ColumnVector<M>;
using Output = Robotics::ColumnVector<P>;

using StateMatrix = Robotics::SquareMatrix<N>;
using InputMatrix = Robotics::Matrix<N, M>;
using OutputMatrix = Robotics::Matrix<P, N>;
using FeedthroughMatrix = Robotics::Matrix<P, N>;
```

Let's define a linear system whose state form is

```
x' = A * x + B * u
y  = C * x + D * u
```

To setup a `LinearSystem`:

```cpp
StateMatrix A;
A << 1, 0
     0, 1;

InputMatrix B;
B << 1, 0;

OutputMatrix C;
C << 1, 0,
     0, 1;
```

Note that having templates not only improves runtime performance, but allows to carry out compile-time checks on the types you define. If you initialize some of the matrices above with the wrong number of elements, the compiler will throw an error.

Matrices C and D are not required: they will be null by default if not provided. In this case, D will be null.
To define the system:

```cpp
Robotics::Model::LinearSystem<N, M, P> system(A, B, C);
```

The initial state is zero by default. You can set a custom initial state as follows:

```cpp
system.SetInitialState(State(1.0, -1.0));
```

### Library features

Check out [TheLartians/ModernCppStarter](https://github.com/TheLartians/ModernCppStarter) if you want to include these features in your project.

#### Running tests

From the root directory:

```bash
cmake -S test -B build/test
cmake --build build/test
CTEST_OUTPUT_ON_FAILURE=1 cmake --build build/test --target test

# or simply call the executable: 
./build/test/RoboticsTests
```

To also collect code coverage information, run CMake with the `-DENABLE_TEST_COVERAGE=1` option.

#### Running clang-format for autoformatting

This requires _clang-format_, _cmake-format_ and _pyyaml_ to be installed on the current system.

```bash
cmake -S test -B build/test

# view changes
cmake --build build/test --target format

# apply changes
cmake --build build/test --target fix-format
```

See [Format.cmake](https://github.com/TheLartians/Format.cmake) for details.

#### Build the documentation

The documentation is automatically built and [published](https://giacomo-b.github.io/CppRobotics) whenever a [GitHub Release](https://help.github.com/en/github/administering-a-repository/managing-releases-in-a-repository) is created.
To manually build documentation, call the following command.

```bash
cmake -S documentation -B build/doc
cmake --build build/doc --target GenerateDocs
# view the docs
open build/doc/doxygen/html/index.html
```

To build the documentation locally, you will need _Doxygen_, _jinja2_ and _Pygments_ installed your system.

#### Build everything at once

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

#### Additional tools

The test and standalone subprojects include the [tools.cmake](cmake/tools.cmake) file which is used to import additional tools on-demand through CMake configuration arguments.
The following are currently supported.

##### Sanitizers

Sanitizers can be enabled by configuring CMake with `-DUSE_SANITIZER=<Address | Memory | MemoryWithOrigins | Undefined | Thread | Leak | 'Address;Undefined'>`.

##### Static Analyzers

Static Analyzers can be enabled by setting `-DUSE_STATIC_ANALYZER=<clang-tidy | iwyu | cppcheck>`, or a combination of those in quotation marks, separated by semicolons.
By default, analyzers will automatically find configuration files such as `.clang-format`.
Additional arguments can be passed to the analyzers by setting the `CLANG_TIDY_ARGS`, `IWYU_ARGS` or `CPPCHECK_ARGS` variables.

##### Ccache

Ccache can be enabled by configuring with `-DUSE_CCACHE=<ON | OFF>`.

## TODOs

* [ ] Add more algorithms  
* [ ] Add support for nonlinear systems automatic differentiation, so that the Jacobians are automatically computed (see [autodiff](https://autodiff.github.io/tutorials/))  
* [ ] Add a README.md to each example folder, to explain the theory  
* [ ] Cache the packages downloaded by CPM.CMake (currently everything is re-downloaded everytime a new example is built)
* [ ] Many more, feel free to add your ideas!

## References

As mentioned above, this repo was originally inspired by [AtsushiSakai/PythonRobotics](https://github.com/AtsushiSakai/PythonRobotics). So go check it out if you want to see more algorithms (or if you want to help port a few of them!).
