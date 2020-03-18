cmake_minimum_required(VERSION 2.6)

project(ExtraLib CXX)
find_package(pinocchio REQUIRED)
find_package(PythonInterp REQUIRED)
find_package(PythonLibs REQUIRED)

set(PINOCCHIO_MODEL_DIR "${PROJECT_SOURCE_DIR}/../../models")
add_definitions('-DPINOCCHIO_MODEL_DIR=\"${PINOCCHIO_MODEL_DIR}\"')

add_executable(run_rnea run_rnea.cpp)
target_link_libraries(run_rnea PUBLIC pinocchio::pinocchio)

if(DEFINED PINOCCHIO_USE_URDFDOM)
  add_executable(load_urdf load_urdf.cpp)
  target_link_libraries(load_urdf PUBLIC pinocchio::pinocchio)
endif(DEFINED PINOCCHIO_USE_URDFDOM)
