cmake_minimum_required(VERSION 3.26)
project(physman)

set(CMAKE_CXX_STANDARD 23)

add_executable(main
        src/Game.cpp
        src/Js.cpp
        src/main.cpp
        src/math/Val.cpp
        src/math/Algorithm.cpp
        src/math/Constraint.cpp
        src/math/Force.cpp
        src/math/Physics.cpp
)

find_package(fmt CONFIG REQUIRED)
target_link_libraries(main PRIVATE fmt::fmt)

find_package(raylib CONFIG REQUIRED)
target_link_libraries(main PRIVATE raylib)

find_package(Catch2 CONFIG REQUIRED)
target_link_libraries(main PRIVATE Catch2::Catch2)

find_package(EnTT CONFIG REQUIRED)
target_link_libraries(main PRIVATE EnTT::EnTT)

target_include_directories(main PRIVATE ${CMAKE_SOURCE_DIR}/src)
target_compile_options(main PRIVATE -Wno-unknown-attributes -Wno-unqualified-std-cast-call)

if (EMSCRIPTEN)
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -s USE_GLFW=3 -s ASSERTIONS=1 -s WASM=1 -s ASYNCIFY --preload-file ${CMAKE_SOURCE_DIR}/resources@resources")
    set(CMAKE_EXECUTABLE_SUFFIX ".html") # This line is used to set your executable to build with the emscripten html template so that you can directly open it.
endif ()
