find_package(pybind11 REQUIRED)

pybind11_add_module(cpp2pybind11 cpp2pybind11.cpp)
target_link_libraries(cpp2pybind11 PRIVATE pinocchio_pywrap)
  
if(CMAKE_CXX_STANDARD LESS 14)
  message(STATUS "CXX_STANDARD for cpp2pybind11 set changed from ${CMAKE_CXX_STANDARD} to 14")
  set_target_properties(cpp2pybind11 PROPERTIES CXX_STANDARD 14)
endif()
