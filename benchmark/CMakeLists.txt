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

    target_link_libraries(${target_name}
        PRIVATE pinocchio benchmark::benchmark pinocchio-benchmarks-resources)
    
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

if (BUILD_WITH_PARSERS_SUPPORT)
    macro(add_parsers_benchmark benchmark_name)
        pinocchio_add_benchmark(${benchmark_name})
        target_link_libraries(pinocchio-benchmark-${benchmark_name}
            PRIVATE pinocchio_parsers)
    endmacro()
    # timings
    add_parsers_benchmark(timings)
    # timings cholesky
    add_parsers_benchmark(timings-cholesky)
    add_parsers_benchmark(timings-loop-constrained-aba)
    # timings derivatives
    add_parsers_benchmark(timings-derivatives)
    # timings-jacobian
    add_parsers_benchmark(timings-jacobian)
    # timings-contact-dynamics
    add_parsers_benchmark(timings-delassus-operations)
    add_parsers_benchmark(timings-contact-dynamics)
    add_parsers_benchmark(timings-constrained-dynamics-derivatives)
    # timings-impulse-dynamics
    add_parsers_benchmark(timings-impulse-dynamics)
    add_parsers_benchmark(timings-impulse-dynamics-derivatives)

    # cppacg
    if(BUILD_WITH_CODEGEN_SUPPORT)
        macro(add_cppadcg_benchmark benchmark_name)
            pinocchio_add_benchmark(${benchmark_name})
            target_link_libraries(pinocchio-benchmark-${benchmark_name}
                PRIVATE pinocchio_parsers)
            target_link_libraries(pinocchio-benchmark-${benchmark_name}
                PRIVATE pinocchio_cppadcg)
            target_compile_definitions(pinocchio-benchmark-${benchmark_name}
                PRIVATE PINOCCHIO_CXX_COMPILER=\"${CMAKE_CXX_COMPILER}\"
        )
        endmacro()
        add_cppadcg_benchmark(timings-cg)
    endif()

    # parallel
    if(BUILD_WITH_OPENMP_SUPPORT)
        macro(add_parallel_benchmark benchmark_name)
            pinocchio_add_benchmark(${benchmark_name})
            target_link_libraries(pinocchio-benchmark-${benchmark_name}
                PRIVATE pinocchio_parsers)
            target_link_libraries(pinocchio-benchmark-${benchmark_name}
                PRIVATE pinocchio_parallel)
        endmacro()
        add_parallel_benchmark(timings-parallel)
    endif()
    
    # collision
    if(BUILD_WITH_URDF_SUPPORT AND BUILD_WITH_COLLISION_SUPPORT)
        macro(add_collision_benchmark benchmark_name)
            pinocchio_add_benchmark(${benchmark_name})
            target_link_libraries(pinocchio-benchmark-${benchmark_name}
                PRIVATE pinocchio_parsers)
            target_link_libraries(pinocchio-benchmark-${benchmark_name}
                PRIVATE pinocchio_collision)
        endmacro()
        add_collision_benchmark(timings-geometry)
    endif()
endif()

#cppad
if(BUILD_WITH_AUTODIFF_SUPPORT)
macro(add_cppad_benchmark benchmark_name)
        pinocchio_add_benchmark(${benchmark_name})
        target_link_libraries(pinocchio-benchmark-${benchmark_name}
            PRIVATE pinocchio_parsers)
        target_link_libraries(pinocchio-benchmark-${benchmark_name}
            PRIVATE pinocchio_cppad)
        target_compile_definitions(pinocchio-benchmark-${benchmark_name}
            PRIVATE PINOCCHIO_CXX_COMPILER=\"${CMAKE_CXX_COMPILER}\"
    )
    endmacro()    
    add_cppad_benchmark(timings-cppad-jit)
endif()