#
# Copyright (c) 2015-2018 CNRS
# Copyright (c) 2018-2025 INRIA
#

# ----------------------------------------------------
# --- BENCHMARK --------------------------------------
# ----------------------------------------------------
add_library(pinocchio-benchmarks-resources INTERFACE)
target_compile_definitions(
    pinocchio-benchmarks-resources
    INTERFACE
        PINOCCHIO_MODEL_DIR=\"${PINOCCHIO_MODEL_DIR}\"
        EXAMPLE_ROBOT_DATA_MODEL_DIR=\"${EXAMPLE_ROBOT_DATA_MODEL_DIR}\"
)

add_custom_target(pinocchio-benchmarks)

function(pinocchio_add_benchmark benchmark_name)
    set(target_name "pinocchio-benchmark-${benchmark_name}")
    add_executable(${target_name} ${benchmark_name}.cpp)

    target_link_libraries(
        ${target_name}
        PRIVATE pinocchio benchmark::benchmark pinocchio-benchmarks-resources
    )

    jrl_target_set_default_compile_options(${target_name} PRIVATE)
    jrl_target_enforce_msvc_conformance(${target_name} PRIVATE)

    add_dependencies(pinocchio-benchmarks ${target_name})
endfunction()

# modernize_target_link_libraries(
#   ${bench_name}
#   SCOPE PRIVATE
#   TARGETS Boost::filesystem
#   LIBRARIES ${Boost_FILESYSTEM_LIBRARY}
#   INCLUDE_DIRS ${Boost_INCLUDE_DIRS}
# )

pinocchio_add_benchmark(spatial-operations)
# timings-eigen
pinocchio_add_benchmark(timings-eigen)
# timings-linalg
pinocchio_add_benchmark(timings-linalg-inverse)

if(BUILD_WITH_PARSERS_SUPPORT)
    # timings
    pinocchio_add_benchmark(timings)
    # timings cholesky
    pinocchio_add_benchmark(timings-cholesky)
    pinocchio_add_benchmark(timings-loop-constrained-aba)
    # timings derivatives
    pinocchio_add_benchmark(timings-derivatives)
    # timings-jacobian
    pinocchio_add_benchmark(timings-jacobian)
    # timings-contact-dynamics
    pinocchio_add_benchmark(timings-delassus-operations)
    pinocchio_add_benchmark(timings-contact-dynamics)
    pinocchio_add_benchmark(timings-constrained-dynamics-derivatives)
    # timings-impulse-dynamics
    pinocchio_add_benchmark(timings-impulse-dynamics)
    pinocchio_add_benchmark(timings-impulse-dynamics-derivatives)

    # cppacg
    if(BUILD_WITH_CODEGEN_SUPPORT)
        macro(pinocchio_add_cppadcg_benchmark benchmark_name)
            pinocchio_add_benchmark(${benchmark_name})
            target_link_libraries(
                pinocchio-benchmark-${benchmark_name}
                PRIVATE pinocchio_cppadcg
            )
            target_compile_definitions(
                pinocchio-benchmark-${benchmark_name}
                PRIVATE PINOCCHIO_CXX_COMPILER=\"${CMAKE_CXX_COMPILER}\"
            )
        endmacro()
        pinocchio_add_cppadcg_benchmark(timings-cg)
    endif()

    # parallel
    if(BUILD_WITH_OPENMP_SUPPORT)
        pinocchio_add_benchmark(timings-parallel)
    endif()

    # collision
    if(BUILD_WITH_URDF_SUPPORT AND BUILD_WITH_COLLISION_SUPPORT)
        pinocchio_add_benchmark(timings-geometry)
    endif()

    #cppad
    if(BUILD_WITH_AUTODIFF_SUPPORT)
        macro(pinocchio_add_cppad_benchmark benchmark_name)
            pinocchio_add_benchmark(${benchmark_name})
            target_link_libraries(
                pinocchio-benchmark-${benchmark_name}
                PRIVATE pinocchio_cppad
            )
            target_compile_definitions(
                pinocchio-benchmark-${benchmark_name}
                PRIVATE PINOCCHIO_CXX_COMPILER=\"${CMAKE_CXX_COMPILER}\"
            )
        endmacro()
        pinocchio_add_cppad_benchmark(timings-cppad-jit)
    endif()
endif()
