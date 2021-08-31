cmake_minimum_required(VERSION 3.4)
project(test_boost_pybind11 LANGUAGES CXX)

find_package(pybind11 REQUIRED)
find_package(pinocchio REQUIRED)

execute_process(COMMAND ${PYTHON_EXECUTABLE} -c "import sys; sys.stdout.write(sys.version[:3])"
  OUTPUT_VARIABLE PYTHON_MAJOR_MINOR_VERSION)
set(BOOST_PYTHON python${PYTHON_MAJOR_MINOR_VERSION})
find_package(Boost 1.70 REQUIRED COMPONENTS ${BOOST_PYTHON})

pybind11_add_module(cpp2pybind11 cpp2pybind11.cpp)
target_link_libraries(cpp2pybind11 PRIVATE pinocchio::pinocchio Boost::${BOOST_PYTHON})
