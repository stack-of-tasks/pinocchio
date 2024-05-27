cmake_minimum_required(VERSION 3.11)

project(ExtraLib CXX)

message(STATUS "PINOCCHIO_GIT_REPOSITORY: $ENV{PINOCCHIO_GIT_REPOSITORY}")
message(STATUS "PINOCCHIO_GIT_TAG: $ENV{PINOCCHIO_GIT_TAG}")

include(FetchContent)
FetchContent_Declare(
  # FetchContent_Populate(
  Pinocchio
  GIT_REPOSITORY $ENV{PINOCCHIO_GIT_REPOSITORY}
  GIT_TAG $ENV{PINOCCHIO_GIT_TAG}
  GIT_SUBMODULES cmake
  GIT_SHALLOW ON)

FetchContent_GetProperties(Pinocchio)
if(NOT pinocchio_POPULATED) # name is lowercased
  FetchContent_Populate(Pinocchio)
  message(STATUS "pinocchio source dir: ${pinocchio_SOURCE_DIR}")
  message(STATUS "pinocchio binary dir: ${pinocchio_BINARY_DIR}")
  set(BUILD_PYTHON_INTERFACE
      OFF
      CACHE INTERNAL "Build Python bindings")
  set(BUILD_TESTING
      OFF
      CACHE INTERNAL "Build unit tests")
  add_subdirectory(${pinocchio_SOURCE_DIR} ${pinocchio_BINARY_DIR}) # name is lowercased
endif()
# or simply FetchContent_MakeAvailable(Pinocchio)

set(PINOCCHIO_MODEL_DIR "${PROJECT_SOURCE_DIR}/../../../models")
add_definitions('-DPINOCCHIO_MODEL_DIR=\"${PINOCCHIO_MODEL_DIR}\"')

add_executable(run_rnea ../run_rnea.cpp)
target_link_libraries(run_rnea PUBLIC pinocchio)

if(DEFINED PINOCCHIO_USE_URDFDOM)
  add_executable(load_urdf ../load_urdf.cpp)
  target_link_libraries(load_urdf PUBLIC pinocchio)
endif(DEFINED PINOCCHIO_USE_URDFDOM)
