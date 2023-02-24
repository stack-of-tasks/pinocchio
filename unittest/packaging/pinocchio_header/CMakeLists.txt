cmake_minimum_required(VERSION 2.6)

project(ExtraLib CXX)
find_package(pinocchio REQUIRED)

add_executable(run_fk ../run_fk.cpp)
target_link_libraries(run_fk PUBLIC pinocchio::pinocchio_headers)
