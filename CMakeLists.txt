cmake_minimum_required(VERSION 3.18)
project(cppjsbsim CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)

# Upstream defaults these ON; they pull in Cython/Doxygen and are not needed.
set(BUILD_PYTHON_MODULE OFF CACHE BOOL "" FORCE)
set(BUILD_DOCS         OFF CACHE BOOL "" FORCE)
set(BUILD_SHARED_LIBS  OFF CACHE BOOL "" FORCE)

FetchContent_Declare(jsbsim
  GIT_REPOSITORY https://github.com/JSBSim-Team/jsbsim.git
  GIT_TAG        v1.3.1
  GIT_SHALLOW    TRUE)
FetchContent_MakeAvailable(jsbsim)

add_executable(sim main.cpp)
target_link_libraries(sim PRIVATE libJSBSim)

# aircraft/, engine/, systems/ live in the JSBSim source tree.
target_compile_definitions(sim PRIVATE JSBSIM_ROOT="${jsbsim_SOURCE_DIR}")
