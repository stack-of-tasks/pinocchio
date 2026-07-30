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

# TODO rename in pinocchio_add_test
function(add_pinocchio_unit_test name)
    set(test_name "pinocchio-test-cpp-${name}")
    add_executable(${test_name} ${name}.cpp)
    target_link_libraries(
        ${test_name}
        PRIVATE
            Boost::unit_test_framework
            pinocchio
            pinocchio-tests-resources
            example-robot-data::example-robot-data
    )

    jrl_target_set_default_compile_options(${test_name} PRIVATE)
    jrl_target_enforce_msvc_conformance(${test_name} PRIVATE)

    add_test(NAME ${test_name} COMMAND ${test_name})
    add_dependencies(pinocchio-tests ${test_name})
endfunction()

# CPP tests
add_subdirectory(cpp)

# Python tests
if(BUILD_PYTHON_INTERFACE)
    add_subdirectory(python)
endif()
