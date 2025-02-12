cmake_minimum_required(VERSION 3.12)

project(ExtraLib CXX)
find_package(PkgConfig REQUIRED)
pkg_check_modules(PINOCCHIO REQUIRED pinocchio)

add_executable(run_rnea ../run_rnea.cpp)
target_link_libraries(run_rnea PRIVATE ${PINOCCHIO_LDFLAGS})
target_include_directories(run_rnea SYSTEM PRIVATE ${PINOCCHIO_INCLUDE_DIRS})
target_compile_definitions(run_rnea PRIVATE ${PINOCCHIO_CFLAGS_OTHER})

if("-DPINOCCHIO_WITH_URDFDOM" IN_LIST PINOCCHIO_CFLAGS)
  set(PINOCCHIO_MODEL_DIR "${PROJECT_SOURCE_DIR}/../../../models")

  add_executable(load_urdf ../load_urdf.cpp)
  target_link_libraries(load_urdf PRIVATE ${PINOCCHIO_LDFLAGS})
  target_include_directories(load_urdf SYSTEM PRIVATE ${PINOCCHIO_INCLUDE_DIRS})
  target_compile_definitions(
    load_urdf
    PRIVATE ${PINOCCHIO_CFLAGS_OTHER} "PINOCCHIO_MODEL_DIR=\"${PINOCCHIO_MODEL_DIR}\""
  )
endif()

message(STATUS "LDFLAGS: ${PINOCCHIO_LDFLAGS}")
message(STATUS "CFLAGS: ${PINOCCHIO_CFLAGS_OTHER}")
message(STATUS "INCLUDE: ${PINOCCHIO_INCLUDE_DIRS}")
