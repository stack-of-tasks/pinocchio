find_package(pybind11 CONFIG QUIET)
if(NOT TARGET pybind11::module)
    include(FetchContent)
    FetchContent_Declare(
        pybind11
        GIT_REPOSITORY https://github.com/pybind/pybind11
        GIT_TAG v3.0.1
    )
    FetchContent_GetProperties(pybind11)
    if(NOT pybind11_POPULATED)
        FetchContent_Populate(pybind11)
        add_subdirectory(${pybind11_SOURCE_DIR} ${pybind11_BINARY_DIR})
    endif()
endif()

# pybind11_add_module(cpp2pybind11 cpp2pybind11.cpp)
#
# BUG: might not work out of the box on OSX with conda:
# https://github.com/pybind/pybind11/issues/3081
if(BUILD_TESTING)
    set(_exclude)
else()
    set(_exclude EXCLUDE_FROM_ALL)
endif()

python_add_library(cpp2pybind11 MODULE ${_exclude} WITH_SOABI cpp2pybind11.cpp)
add_dependencies(pinocchio-tests cpp2pybind11)
add_dependencies(cpp2pybind11 pinocchio_pywrap_default)
target_link_libraries(
    cpp2pybind11
    PRIVATE
        pinocchio_pywrap_headers
        eigenpy::eigenpy
        Boost::python
        pybind11::module
        Eigen3::Eigen
)

if(CMAKE_CXX_STANDARD LESS 14)
    message(
        STATUS
        "CXX_STANDARD for cpp2pybind11 changed from ${CMAKE_CXX_STANDARD} to 14"
    )
    set_target_properties(cpp2pybind11 PROPERTIES CXX_STANDARD 14)
endif()

if(WIN32)
    target_compile_definitions(cpp2pybind11 PRIVATE -DNOMINMAX)
endif(WIN32)

pinocchio_add_python_test(test-cpp2pybind11)
set_tests_properties(
    "pinocchio-test-py-test-cpp2pybind11"
    PROPERTIES
        ENVIRONMENT_MODIFICATION
            "PYTHONPATH=path_list_prepend:$<TARGET_FILE_DIR:cpp2pybind11>"
)
