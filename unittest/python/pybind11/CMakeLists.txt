if(CMAKE_VERSION VERSION_GREATER 4.11)
  include(FetchContent)
  FetchContent_Declare(
    pybind11
    GIT_REPOSITORY https://github.com/pybind/pybind11
    GIT_TAG v2.10.0)
  FetchContent_GetProperties(pybind11)
  if(NOT pybind11_POPULATED)
    FetchContent_Populate(pybind11)
    add_subdirectory(${pybind11_SOURCE_DIR} ${pybind11_BINARY_DIR})

    # pybind11_add_module(cpp2pybind11 cpp2pybind11.cpp)
    #
    # BUG: might not work out of the box on OSX with conda:
    # https://github.com/pybind/pybind11/issues/3081
    if(NOT BUILD_TESTING)
      add_library(cpp2pybind11 MODULE EXCLUDE_FROM_ALL cpp2pybind11.cpp)
    else()
      add_library(cpp2pybind11 MODULE cpp2pybind11.cpp)
    endif()
    add_dependencies(build_tests cpp2pybind11)
    target_link_libraries(cpp2pybind11 PRIVATE pinocchio_pywrap_default pybind11::module)
    target_include_directories(cpp2pybind11 SYSTEM PUBLIC ${EIGEN3_INCLUDE_DIR})
    set_target_properties(cpp2pybind11 PROPERTIES PREFIX "" SUFFIX ${PYTHON_EXT_SUFFIX})

    if(CMAKE_CXX_STANDARD LESS 14)
      message(STATUS "CXX_STANDARD for cpp2pybind11 changed from ${CMAKE_CXX_STANDARD} to 14")
      set_target_properties(cpp2pybind11 PROPERTIES CXX_STANDARD 14)
    endif()

    if(WIN32)
      target_compile_definitions(cpp2pybind11 PRIVATE -DNOMINMAX)
    endif(WIN32)

    add_python_unit_test("test-py-cpp2pybind11" "unittest/python/pybind11/test-cpp2pybind11.py"
                         "bindings/python" "unittest/python/pybind11")
  endif()
endif()
