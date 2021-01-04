cmake_minimum_required(VERSION 2.6)

project(ExtraLib CXX)
find_package(PkgConfig REQUIRED)
pkg_check_modules(PINOCCHIO REQUIRED pinocchio)

include_directories(SYSTEM ${PINOCCHIO_INCLUDE_DIRS})
add_executable(run_rnea ../run_rnea.cpp)
target_link_libraries(run_rnea PUBLIC ${PINOCCHIO_LDFLAGS})
message(STATUS "${PINOCCHIO_LDFLAGS}")
