cmake_minimum_required(VERSION 3.18)
project(geopf VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

option(GEOPF_FETCH_DEPS "Download Eigen / yaml-cpp if they are not installed" ON)

include(FetchContent)

# --- Eigen (header only) -----------------------------------------------------
find_package(Eigen3 3.3 QUIET NO_MODULE)
if(NOT TARGET Eigen3::Eigen)
  if(NOT GEOPF_FETCH_DEPS)
    message(FATAL_ERROR "Eigen3 not found (install it or set -DGEOPF_FETCH_DEPS=ON)")
  endif()
  message(STATUS "Eigen3 not found - downloading 3.4.0")
  FetchContent_Declare(eigen
    GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
    GIT_TAG        3.4.0
    GIT_SHALLOW    TRUE
    # Only the headers are needed: point SOURCE_SUBDIR at a directory without
    # a CMakeLists.txt so Eigen's own CMake project is not configured.
    SOURCE_SUBDIR  headers-only)
  FetchContent_MakeAvailable(eigen)
  add_library(Eigen3::Eigen INTERFACE IMPORTED)
  set_target_properties(Eigen3::Eigen PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${eigen_SOURCE_DIR}")
endif()

# --- yaml-cpp ----------------------------------------------------------------
find_package(yaml-cpp QUIET)
if(NOT TARGET yaml-cpp AND NOT TARGET yaml-cpp::yaml-cpp)
  if(NOT GEOPF_FETCH_DEPS)
    message(FATAL_ERROR "yaml-cpp not found (install it or set -DGEOPF_FETCH_DEPS=ON)")
  endif()
  message(STATUS "yaml-cpp not found - downloading 0.9.0")
  set(YAML_CPP_BUILD_TESTS OFF CACHE BOOL "" FORCE)
  set(YAML_CPP_BUILD_TOOLS OFF CACHE BOOL "" FORCE)
  set(YAML_CPP_BUILD_CONTRIB OFF CACHE BOOL "" FORCE)
  set(YAML_CPP_INSTALL OFF CACHE BOOL "" FORCE)
  set(YAML_BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
  FetchContent_Declare(yaml-cpp
    GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
    GIT_TAG        yaml-cpp-0.9.0
    GIT_SHALLOW    TRUE)
  FetchContent_MakeAvailable(yaml-cpp)
endif()
if(TARGET yaml-cpp::yaml-cpp)
  set(GEOPF_YAML_TARGET yaml-cpp::yaml-cpp)
else()
  set(GEOPF_YAML_TARGET yaml-cpp)
endif()

# --- GeoPF force library (no robotics / simulator dependencies) -------------
add_library(geopf_core STATIC
  src/force_attr.cpp
  src/force_boundary.cpp
  src/force_sphere.cpp
  src/force_line.cpp
  src/force_plane.cpp
  src/force_cubic.cpp
  src/force_cylinder.cpp
  src/IndexMinMax.cpp)
target_include_directories(geopf_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(geopf_core PUBLIC Eigen3::Eigen)

# --- Command-line tool -------------------------------------------------------
add_executable(geopf src/geopf_cli.cpp)
target_compile_definitions(geopf PRIVATE GEOPF_VERSION="${PROJECT_VERSION}")
target_link_libraries(geopf PRIVATE geopf_core ${GEOPF_YAML_TARGET})

install(TARGETS geopf RUNTIME DESTINATION bin)

# The original research programs (SingleAgent, MockupAgent, ...) are not built
# here: they need DQ Robotics and contain hard-coded paths.
