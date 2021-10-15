if(CMAKE_VERSION VERSION_GREATER 3.11)
  include(FetchContent)
  FetchContent_Declare(pybind11
    GIT_REPOSITORY https://github.com/pybind/pybind11
    GIT_TAG v2.8.0)
  FetchContent_GetProperties(pybind11)
  if(NOT pybind11_POPULATED)
    FetchContent_Populate(pybind11)
    add_subdirectory(${pybind11_SOURCE_DIR} ${pybind11_BINARY_DIR})

    pybind11_add_module(cpp2pybind11 cpp2pybind11.cpp)
    target_link_libraries(cpp2pybind11 PRIVATE pinocchio_pywrap)

    if(CMAKE_CXX_STANDARD LESS 14)
      message(STATUS "CXX_STANDARD for cpp2pybind11 set changed from ${CMAKE_CXX_STANDARD} to 14")
      set_target_properties(cpp2pybind11 PROPERTIES CXX_STANDARD 14)
    endif()

    if(WIN32)
      target_compile_definitions(cpp2pybind11 PRIVATE -DNOMINMAX)
    endif(WIN32)
  endif()
endif()
