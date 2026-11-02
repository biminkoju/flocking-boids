cmake_minimum_required(VERSION 3.16)
project(boids LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Locally built raylib (see README.md for the bootstrap step).
set(RAYLIB_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/third_party/raylib")

if(NOT EXISTS "${RAYLIB_ROOT}/include/raylib.h")
  message(FATAL_ERROR "raylib not found at ${RAYLIB_ROOT}. See README.md for build instructions.")
endif()

add_library(raylib SHARED IMPORTED)
set_target_properties(raylib PROPERTIES
  IMPORTED_LOCATION "${RAYLIB_ROOT}/lib/libraylib.so"
  INTERFACE_INCLUDE_DIRECTORIES "${RAYLIB_ROOT}/include"
)

add_executable(boids src/main.cpp src/sim.cpp src/ui.cpp src/selftest.cpp src/evolve.cpp)
target_include_directories(boids PRIVATE src)
target_link_libraries(boids PRIVATE raylib)
target_compile_options(boids PRIVATE -Wall -Wextra -Wformat=2)

# Parallel force evaluation via std::execution::par_unseq. OFF by default: it
# needs a stdlib backend (libstdc++ uses TBB) and exact-output tests are the only
# way to catch a race. Turn on with -DBOIDS_PARALLEL=ON.
option(BOIDS_PARALLEL "Parallelise the per-boid force loop" OFF)
if(BOIDS_PARALLEL)
  target_compile_definitions(boids PRIVATE BOIDS_PARALLEL)
  find_library(TBB_LIB tbb)
  if(TBB_LIB)
    target_link_libraries(boids PRIVATE ${TBB_LIB})
  else()
    message(WARNING "BOIDS_PARALLEL is on but libtbb was not found; linking may fail")
  endif()
endif()

if(UNIX AND NOT APPLE)
  target_link_libraries(boids PRIVATE m pthread dl rt)
endif()
