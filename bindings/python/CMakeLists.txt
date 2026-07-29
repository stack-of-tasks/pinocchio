#
# Copyright (c) 2015-2023 CNRS INRIA
# Copyright (c) 2015 Wandercraft, 86 rue de Paris 91400 Orsay, France.

function(pinocchio_add_python_binding scalar_name)
    set(target_name pinocchio_pywrap_${scalar_name})
    jrl_boostpy_add_module(${target_name} ${ARGN})
    jrl_target_enforce_msvc_conformance(${target_name} PRIVATE)
    jrl_check_python_module_name(${target_name})
    target_link_libraries(
        ${target_name}
        PRIVATE eigenpy::eigenpy pinocchio_pywrap_headers
    )
    jrl_target_set_default_compile_options(${target_name} PRIVATE)
    jrl_target_set_output_directory(${target_name} OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib/site-packages/pinocchio)

    target_compile_definitions(
        ${target_name}
        PRIVATE
            PINOCCHIO_PYTHON_CONTEXT_FILE="pinocchio/bindings/python/context/${scalar_name}.hpp"
            PINOCCHIO_PYTHON_MODULE_NAME=${target_name}
    )

    # Links against accelerate
    if(BUILD_WITH_ACCELERATE_SUPPORT)
        target_link_libraries(${target_name} PRIVATE "-framework accelerate")
    endif()

    if(BUILD_WITH_OPENMP_SUPPORT)
        # OpenMP is linked with pinocchio_parallel target
        target_compile_definitions(
            ${target_name}
            PRIVATE -DPINOCCHIO_PYTHON_INTERFACE_WITH_OPENMP
        )
    endif()

    if(BUILD_WITH_URDF_SUPPORT)
        # Link directly against console_bridge since we bind some enums and call
        # console_bridge::setLogLevel function.
        target_link_libraries(${target_name} PUBLIC console_bridge::console_bridge)
    endif()

    if(BUILD_WITH_COLLISION_SUPPORT)
        target_compile_definitions(
            ${target_name}
            PRIVATE PINOCCHIO_PYTHON_INTERFACE_WITH_COLLISION_PYTHON_BINDINGS
        )
    endif()

    # Precompiled headers are used to speed up compilation of the Python bindings. We have also tested
    # precompiling headers for the main library's `pinocchio_default` target, but the time gain there
    # was negligible compared to the time gained in the Python bindings. We also precompile headers
    # for other scalar types as well.
    if(PINOCCHIO_BUILD_BINDING_WITH_PCH)
        target_precompile_headers(
            ${target_name}
            PRIVATE
                ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/pch.hpp
        )
    endif()
endfunction()

# Define header interface
add_library(pinocchio_pywrap_headers INTERFACE)
add_library(pinocchio::pinocchio_pywrap_headers ALIAS pinocchio_pywrap_headers)
target_link_libraries(pinocchio_pywrap_headers INTERFACE pinocchio)
target_include_directories(
    pinocchio_pywrap_headers
    INTERFACE
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

jrl_target_headers(pinocchio_pywrap_headers INTERFACE
  HEADERS
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/spatial/explog.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/spatial/classic-acceleration.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/spatial/symmetric3.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/spatial/motion.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/spatial/force.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/spatial/inertia.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/spatial/se3.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/algorithm/contact-info.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/algorithm/constraint-cholesky.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/algorithm/constraint-solver-base.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/algorithm/delassus-operator.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/algorithm/proximal.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/algorithm/algorithms.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/algorithm/constraints/baumgarte-corrector-param
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/algorithm/constraints/constraint-data-base.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/algorithm/constraints/constraint-data.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/algorithm/constraints/constraint-data-inheritan
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/algorithm/constraints/constraint-model-base.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/algorithm/constraints/constraint-model.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/algorithm/constraints/constraint-model-inherita
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/algorithm/constraints/constraints-datas.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/algorithm/constraints/constraints-models.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/algorithm/constraints/constraints-variant.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/algorithm/constraints/set-base.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/algorithm/constraints/set-box-set.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/algorithm/constraints/set-coulomb-friction-cone
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/algorithm/constraints/set-trivial-cones.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/context.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/pybind11-all.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/pybind11.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/utils/constant.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/utils/version.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/utils/pickle-vector.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/utils/macros.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/utils/path.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/utils/std-vector.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/utils/printable.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/utils/dependencies.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/utils/conversions.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/utils/address.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/utils/copyable.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/utils/registration.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/utils/pickle.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/utils/pickle-map.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/utils/list.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/utils/eigen.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/utils/comparable.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/utils/std-map.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/utils/cast.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/utils/deprecation.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/utils/keep-alive.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/utils/model-checker.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/utils/namespace.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/utils/std-aligned-vector.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/context/cppadcg.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/context/cppad.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/context/casadi.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/context/default.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/context/mpfr.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/context/generic.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/fwd.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/serialization/serialization.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/serialization/serializable.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/math/tridiagonal-matrix.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/math/lanczos-decomposition.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/collision/broadphase-manager-base.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/collision/geometry-functors.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/collision/tree-broadphase-manager.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/multibody/data.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/multibody/frame.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/multibody/geometry-model.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/multibody/joint/joint.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/multibody/model.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/multibody/joint/joint-model.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/multibody/joint/joint-derived.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/multibody/joint/joints-models.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/multibody/joint/joints-variant.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/multibody/joint/joints-liegroup.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/multibody/joint/joints-datas.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/multibody/joint/joint-data.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/multibody/liegroups.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/multibody/geometry-data.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/multibody/geometry-object.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/parsers/urdf.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/parsers/sdf.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/parsers/mjcf.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/parsers/srdf.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/parsers/model-graph.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/parsers/graph/model-configuration-converter.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/extra/extras.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/visualizers/visualizer-visitor.hpp
  BASE_DIRS include
)

if(BUILD_WITH_COMMIT_VERSION)
    tag_library_version(python/context/)
endif()

set(COMMON_SOURCES
    spatial/expose-symmetric3.cpp
    spatial/expose-force.cpp
    spatial/expose-inertia.cpp
    spatial/expose-SE3.cpp
    spatial/expose-motion.cpp
    spatial/expose-explog.cpp
    spatial/expose-skew.cpp
    algorithm/expose-impulse-dynamics.cpp
    algorithm/expose-model.cpp
    algorithm/expose-centroidal.cpp
    algorithm/expose-aba.cpp
    algorithm/expose-algorithms.cpp
    algorithm/expose-com.cpp
    algorithm/expose-frames.cpp
    algorithm/expose-energy.cpp
    algorithm/expose-contact-dynamics.cpp
    algorithm/expose-constrained-dynamics-derivatives.cpp
    algorithm/expose-rnea-derivatives.cpp
    algorithm/expose-kinematics.cpp
    algorithm/expose-geometry.cpp
    algorithm/expose-aba-derivatives.cpp
    algorithm/expose-constraint-solvers.cpp
    algorithm/expose-contact-inverse-dynamics.cpp
    algorithm/expose-joints.cpp
    algorithm/expose-constrained-dynamics.cpp
    algorithm/expose-loop-constrained-aba.cpp
    algorithm/expose-rnea.cpp
    algorithm/expose-contact-jacobian.cpp
    algorithm/constraints/expose-cones.cpp
    algorithm/constraints/expose-constraints.cpp
    algorithm/expose-cholesky.cpp
    algorithm/expose-regressor.cpp
    algorithm/expose-kinematics-derivatives.cpp
    algorithm/expose-cat.cpp
    algorithm/expose-frames-derivatives.cpp
    algorithm/expose-centroidal-derivatives.cpp
    algorithm/expose-jacobian.cpp
    algorithm/expose-delassus.cpp
    algorithm/expose-impulse-dynamics-derivatives.cpp
    algorithm/expose-kinematic-regressor.cpp
    algorithm/expose-crba.cpp
    algorithm/solvers/expose-admm-solver.cpp
    algorithm/solvers/expose-pgs-solver.cpp
    utils/version.cpp
    utils/dependencies.cpp
    utils/conversions.cpp
    utils/path.cpp
    math/expose-linalg.cpp
    math/expose-tridiagonal-matrix.cpp
    math/expose-lanczos-decomposition.cpp
    math/expose-gram-schmidt-orthonormalisation.cpp
    math/expose-rpy.cpp
    math/expose-eigen-types.cpp
    serialization/serialization.cpp
    multibody/expose-model.cpp
    multibody/expose-liegroups.cpp
    multibody/expose-geometry.cpp
    multibody/expose-frame.cpp
    multibody/expose-data.cpp
    multibody/joint/expose-joints.cpp
    multibody/sample-models.cpp
    parsers/sdf/model.cpp
    parsers/sdf/geometry.cpp
    parsers/expose-parsers.cpp
    parsers/urdf/console-bridge.cpp
    parsers/urdf/model.cpp
    parsers/urdf/geometry.cpp
    parsers/srdf.cpp
    parsers/mjcf/model.cpp
    parsers/mjcf/geometry.cpp
    parsers/graph/expose-edges.cpp
    parsers/graph/expose-frames.cpp
    parsers/graph/expose-geometries.cpp
    parsers/graph/expose-model-graph-algo.cpp
    parsers/graph/expose-model-graph.cpp
    parsers/graph/expose-model-configuration-converter.cpp
    extra/expose-extras.cpp
    module.cpp
)

if(BUILD_WITH_EXTRA_SUPPORT)
    list(
        APPEND COMMON_SOURCES
        ${PROJECT_SOURCE_DIR}/bindings/python/extra/expose-reachable-workspace.cpp
    )
endif()

# default scalar
pinocchio_add_python_binding(default ${COMMON_SOURCES})
add_library(pinocchio_pywrap ALIAS pinocchio_pywrap_default)

if(GENERATE_PYTHON_STUBS)
    set(STUBGEN_DEPENDENCIES)
endif()

if(BUILD_WITH_AUTODIFF_SUPPORT)
    pinocchio_add_python_binding(cppad ${COMMON_SOURCES})
    # CPPAD_DEBUG_AND_RELEASE allow to mix debug and release versions of CppAD in the same program.
    # This can happen when Pinocchio is build in Debug mode and pycppad is build in Release mode.
    # See
    # https://coin-or.github.io/CppAD/doc/preprocessor.htm#Documented%20Here.CPPAD_DEBUG_AND_RELEASE.
    target_compile_definitions(
        pinocchio_pywrap_cppad
        PRIVATE
            PYCPPAD_EXCLUDE_EIGEN_NUMTRAITS_SPECIALIZATION
            CPPAD_DEBUG_AND_RELEASE
    )
    target_include_directories(
        pinocchio_pywrap_cppad
        SYSTEM
        PUBLIC $<BUILD_INTERFACE:${cppad_INCLUDE_DIR}>
    )
    target_link_libraries(pinocchio_pywrap_cppad PUBLIC ${cppad_LIBRARY})
    list(APPEND STUBGEN_DEPENDENCIES "pinocchio_pywrap_cppad")
endif()

if(BUILD_WITH_CODEGEN_SUPPORT)
    pinocchio_add_python_binding(cppadcg ${COMMON_SOURCES})
    # On osx, use default visiblitiy because of an issue with thread_local storage defined in
    # cppad/cg/cg.hpp header (see
    # https://github.com/stack-of-tasks/pinocchio/pull/2469#issuecomment-2461845127)
    if(APPLE)
        set_target_properties(
            pinocchio_pywrap_cppadcg
            PROPERTIES CXX_VISIBILITY_PRESET default
        )
    endif()
    # CPPAD_DEBUG_AND_RELEASE allow to mix debug and release versions of CppAD in the same program.
    target_compile_definitions(
        pinocchio_pywrap_cppadcg
        PRIVATE
            PYCPPAD_EXCLUDE_EIGEN_NUMTRAITS_SPECIALIZATION
            CPPAD_DEBUG_AND_RELEASE
    )
    target_include_directories(
        pinocchio_pywrap_cppadcg
        SYSTEM
        PUBLIC $<BUILD_INTERFACE:${cppadcg_INCLUDE_DIR}>
    )
    target_link_libraries(
        pinocchio_pywrap_cppadcg
        PUBLIC ${cppadcg_LIBRARY} ${cppad_LIBRARY}
    )
    list(APPEND STUBGEN_DEPENDENCIES "pinocchio_pywrap_cppadcg")
endif()

if(BUILD_WITH_CASADI_SUPPORT)
    pinocchio_add_python_binding(casadi ${COMMON_SOURCES})
    target_link_libraries(pinocchio_pywrap_casadi PUBLIC casadi::casadi)
    list(APPEND STUBGEN_DEPENDENCIES "pinocchio_pywrap_casadi")
endif()

if(BUILD_PYTHON_BINDINGS_WITH_BOOST_MPFR_SUPPORT)
    jrl_target_headers(pinocchio_pywrap_headers INTERFACE
        HEADERS
        ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/math/multiprecision/boost/number.hpp
        BASE_DIRS include
    )
    pinocchio_add_python_binding(mpfr ${COMMON_SOURCES} ${PROJECT_SOURCE_DIR}/bindings/python/extra/mpfr/boost_number.cpp)
    target_link_libraries(pinocchio_pywrap_mpfr PRIVATE gmp mpfr)
    list(APPEND STUBGEN_DEPENDENCIES "pinocchio_pywrap_mpfr")
endif()

# --- PACKAGING --- #
if(DOXYGEN_FOUND AND DOXYGEN_VERSION VERSION_GREATER 1.8.17)
    set(DOXYGEN_GENERATE_HTML YES)
    set(DOXYGEN_GENERATE_LATEX NO)
    set(DOXYGEN_PROJECT_NAME "Pinocchio PyBind11 helpers.")
    set(_source_headers_root "../../include/${PROJECT_NAME}/bindings/python")
    doxygen_add_docs(
        doc_pybind11
        ${_source_headers_root}/pybind11.hpp
        ${_source_headers_root}/pybind11-all.hpp
        USE_STAMP_FILE
        COMMENT "Generating documentation of the PyBind11 helpers."
    )
endif()

# Generate python stubs
if(GENERATE_PYTHON_STUBS)
    jrl_boostpy_add_stubs(
        pinocchio_pywrap_stubs
        MODULE pinocchio.pinocchio_pywrap_default
        OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib/site-packages
        PYTHON_PATH ${CMAKE_BINARY_DIR}/lib/site-packages
        DEPENDS pinocchio_pywrap ${STUBGEN_DEPENDENCIES}
        VERBOSE
    )
endif()

# Compile and move in build directory all .py file
jrl_python_compile_all(DIRECTORY
    ${CMAKE_BINARY_DIR}/lib/site-packages/pinocchio
    VERBOSE
)

# Install binding in the right location
jrl_python_compute_install_dir(python_install_dir)
install(
    DIRECTORY
        ${CMAKE_BINARY_DIR}/lib/site-packages/pinocchio
        ${CMAKE_CURRENT_SOURCE_DIR}/pinocchio
    DESTINATION ${python_install_dir}
    FILES_MATCHING
    PATTERN "*.py"
    PATTERN "*.pyc"
    PATTERN "*.pyi"
    PATTERN "*.typed"
    PATTERN "*.so"
    PATTERN "*.pyd"
)
