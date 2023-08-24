
# Physman

Demo: https://physman.captain.birler.co/

Simplest physics engine I could write. Supports point dynamics, forces & constraints.

Is (very) janky but it at least looks like physics. I am sure this could be used in a game. 

Based on the Braff, Witkin, Kass tutorial https://web.mat.upc.edu/toni.susin/files/BaraffWitkinKass2001.pdf .

# Logic

* Compute forces to be applied on points (usual forces that the application would define such as gravity etc.).
* Compute response forces due to constraints (What force do we have to apply such that constraints aren't violated and system gains no energy, i.e. no work is done).
* Compute the state for the next time step.

Response force computation describes the requirements as a set of (sparse) linear equations which are solved with CGD (conjugate gradient descent).
To compute the state for the next time step, the Runge Kutta algorithm is used as the numeric ODE (ordinary differential equation) solver.

The physics step function is around 50 lines of code (src/math/Physics.cpp).
The CGD and ODE solvers are around 30 lines of code (src/math/Algorithm.cpp).
The collision constraint is under 10 lines of code (src/math/Constraint.cpp).
I don't know if this can get simpler.

The infrastructure is more lines of code that the actual logic. When defining constraints, we use an auto-differentiator (src/math/Val.hpp).
Normally, many derivatives by many variables need to be computed for the constraints. We do not want to compute these by hand.
So we simply build expression trees that are automatically differenciated with the required variables.

# Dependencies

Package management is done with vcpkg. See vcpkg.json for all dependencies.
Uses the amazing raylib for visualization.
Uses catch2 for tests.
The rest are simple C++ utilities.

# Usage:

Default CMake settings:
```
-DCMAKE_TOOLCHAIN_FILE=<path_to_vcpkg>/scripts/buildsystems/vcpkg.cmake
-DCMAKE_TOOLCHAIN_FILE=C:\git\vcpkg\scripts\buildsystems\vcpkg.cmake
```

Emscripten CMake settings:
```
"-DVCPKG_CHAINLOAD_TOOLCHAIN_FILE=<fullpath_to_emsdk>/upstream/emscripten/cmake/Modules/Platform/Emscripten.cmake" "-DCMAKE_TOOLCHAIN_FILE=<path_to_vcpkg>/scripts/buildsystems/vcpkg.cmake" "-DVCPKG_TARGET_TRIPLET=wasm32-emscripten"
"-DVCPKG_CHAINLOAD_TOOLCHAIN_FILE=C:\git\emsdk\upstream\emscripten\cmake\Modules\Platform\Emscripten.cmake" "-DCMAKE_TOOLCHAIN_FILE=C:\git\vcpkg\scripts\buildsystems\vcpkg.cmake" "-DVCPKG_TARGET_TRIPLET=wasm32-emscripten"
```

(I needed to add emscripten PATH to system PATH to use with Clion)

Additional docs:
https://github.com/raysan5/raylib/wiki/Working-for-Web-(HTML5)#building-with-clioncmakeemscripten-for-web

# Docker

You can also use Docker to build HTML via Emscripten.

# TODO

* Optimize constraint processing
* Add proper rendering abstractions & entity component system.
