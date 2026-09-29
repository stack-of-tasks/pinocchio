#
# Copyright (c) 2015-2018 CNRS
# Copyright (c) 2018-2026 INRIA
# Copyright (c) 2015 Wandercraft, 86 rue de Paris 91400 Orsay, France.
#

add_library(pinocchio-tests-resources INTERFACE)
file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/serialization-data)
target_compile_definitions(
    pinocchio-tests-resources
    INTERFACE
        TEST_SERIALIZATION_FOLDER=\"${CMAKE_CURRENT_BINARY_DIR}/serialization-data\"
        PINOCCHIO_MODEL_DIR=\"${PINOCCHIO_MODEL_DIR}\"
)

add_custom_target(pinocchio-tests)

# ----------------------------------------------------
# --- MACROS  ----------------------------------------
# ----------------------------------------------------

function(pinocchio_add_cpp_test_or_example target_name file_name)
    add_executable(${target_name} ${file_name}.cpp)
    target_link_libraries(
        ${target_name}
        PRIVATE
            Boost::unit_test_framework
            pinocchio
            pinocchio-tests-resources
            example-robot-data::example-robot-data
    )

    jrl_target_set_default_compile_options(${target_name} PRIVATE)
    jrl_target_enforce_msvc_conformance(${target_name} PRIVATE)

    add_test(NAME ${target_name} COMMAND ${target_name})
endfunction()

function(pinocchio_add_cpp_test name)
    set(test_name "pinocchio-test-cpp-${name}")
    pinocchio_add_cpp_test_or_example(${test_name} ${name})
    add_dependencies(pinocchio-tests ${test_name})
endfunction()

# CPP tests
add_subdirectory(cpp)

# Python tests
if(BUILD_PYTHON_INTERFACE)
    add_subdirectory(python)
endif()
