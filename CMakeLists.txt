# Copyright (c) 2015-2025 CNRS INRIA

# TODO: keep Wandercraft copyright ?
# Copyright (c) 2015 Wandercraft, 86 rue de Paris 91400 Orsay, France.
cmake_minimum_required(VERSION 3.22)

project(
    pinocchio
    VERSION 0.1.0
    DESCRIPTION "A fast and flexible implementation of Rigid Body Dynamics algorithms and their analytical derivatives"
    HOMEPAGE_URL "http://github.com/stack-of-tasks/pinocchio"
    LANGUAGES CXX
)

include(cmake/get-jrl-cmakemodules.cmake)

jrl_configure_defaults()

# ----------------------------------------------------
# --- OPTIONS  ---------------------------------------
# ----------------------------------------------------

jrl_option(INSTALL_DOCUMENTATION "Generate and install the documentation" OFF)
jrl_option(PINOCCHIO_DOXYGEN_USE_MATHJAX "Avoid latex at build time" OFF)
#TODO : remove intermediate option
if(PINOCCHIO_DOXYGEN_USE_MATHJAX)
    set(DOXYGEN_USE_MATHJAX YES)
endif()
jrl_option(BUILD_BENCHMARK "Build the benchmarks" OFF)
jrl_option(BUILD_EXAMPLES "Build the examples" ON)
jrl_option(BUILD_UTILS "Build the utils" OFF)
jrl_option(GENERATE_PYTHON_STUBS "Generate the Python stubs associated to the Python library" OFF)
jrl_option(BUILD_WITH_COMMIT_VERSION "Build libraries by setting specific commit version" OFF)
# TODO : remove ? deprecated
jrl_option(BUILD_UNIT_TESTS "BUILD_UNIT_TESTS is deprecated. Use BUILD_TESTING instead.\
    If you are manually building Pinocchio from source in an existing build folder,\
    we suggest that you delete your build folder and make a new one." ON )
jrl_option(BUILD_TESTING "build the tests" ON)
if (BUILD_UNIT_TESTS)
    set(BUILD_TESTING  ${BUILD_UNIT_TESTS})
endif()
jrl_option(BUILD_ADVANCED_TESTING "Build the advanced tests (multiprecision, etc.) of Pinocchio" OFF)
jrl_option(PINOCCHIO_BUILD_MPFR_TESTING "Build MPFR dependent tests" OFF)
jrl_option(BUILD_WITH_URDF_SUPPORT "Build the library with the URDF format support" ON)
jrl_option(BUILD_WITH_SDF_SUPPORT "Build the library with the SDF format support" OFF)
jrl_option(BUILD_WITH_COLLISION_SUPPORT "Build the library with the collision support (required coal)" OFF)
jrl_option(BUILD_WITH_AUTODIFF_SUPPORT "Build the library with the automatic differentiation support (via CppAD)" OFF)
jrl_option( BUILD_WITH_CASADI_SUPPORT "Build the library with the support of CASADI" OFF)
# cppadcodgen don't build well with CL
if(NOT WIN32)
    jrl_option(BUILD_WITH_CODEGEN_SUPPORT "Build the library with the support of code generation (via CppADCodeGen)" OFF)
endif()
jrl_option(BUILD_WITH_OPENMP_SUPPORT "Build the library with the OpenMP support" OFF)
# TODO : why not present with jrl v1 
jrl_option(BUILD_PYTHON_INTERFACE "Build the Python bindings" ON)
jrl_option(BUILD_PYTHON_BINDINGS_WITH_BOOST_MPFR_SUPPORT "Build the Python interface with Boost.Multiprecision MPFR support" 
    OFF
    CONDITION BUILD_PYTHON_INTERFACE
    FALLBACK OFF
)
jrl_option(BUILD_WITH_LIBPYTHON "Build the library with Python format support"
    ON
    CONDITION BUILD_PYTHON_INTERFACE
    FALLBACK OFF
)
if(APPLE)
    jrl_option(BUILD_WITH_ACCELERATE_SUPPORT "Build Pinocchio with the Accelerate support" OFF)
else(APPLE)
    set(BUILD_WITH_ACCELERATE_SUPPORT FALSE)
endif()
jrl_option(INITIALIZE_WITH_NAN "Initialize Eigen entries with NaN" OFF)
jrl_option(ASSERT_ON_EXCEPTION "Turns Pinocchio excpetions into asserts for debug purposes" OFF)
jrl_option(ENABLE_TEMPLATE_INSTANTIATION "Template instantiation of the main library" ON)
jrl_option(PINOCCHIO_BUILD_WITH_TRACY "Build with tracy profiler for performance analysis" OFF)
jrl_option( BUILD_WITH_RTSAN "Build with the Realtime Sanitizer (clang >=20, linux/mac only)" OFF)
jrl_option(PINOCCHIO_INSTALL_DEPRECATED_HEADERS "Use and install deprecated headers" ON)
# PCH and ccache doesn't play well together so we deactivate
# PCH use by default
jrl_option(PINOCCHIO_BUILD_BINDING_WITH_PCH "Use PCH to accelerate binding build" OFF)
jrl_option(PINOCCHIO_BUILD_VISUALIZERS "Build the pinocchio visualizers library" ON)

# ----------------------------------------------------
# --- DEPENDENCIES -----------------------------------
# ----------------------------------------------------

jrl_find_package(Eigen3 CONFIG REQUIRED EXPECTED_TARGETS Eigen3::Eigen)
jrl_find_package(Boost CONFIG REQUIRED COMPONENTS serialization filesystem
 EXPECTED_TARGETS Boost::boost Boost::serialization Boost::filesystem
)

set(PINOCCHIO_MODEL_DIR "${PROJECT_SOURCE_DIR}/models")
jrl_find_package(example-robot-data CONFIG REQUIRED EXPECTED_TARGETS example-robot-data::example-robot-data)

if(BUILD_WITH_COLLISION_SUPPORT)
    jrl_find_package(coal REQUIRED)
endif()
# ----------------------------------------------------
# --- MAIN LIBRARY -----------------------------------
# ----------------------------------------------------

add_library(pinocchio_headers INTERFACE)
target_compile_features(pinocchio_headers INTERFACE cxx_std_17)
jrl_target_generate_config_header(pinocchio_headers INTERFACE FILENAME pinocchio/config.hpp VERSION ${PROJECT_VERSION} LIBRARY_NAME pinocchio)
jrl_target_generate_warning_header(pinocchio_headers INTERFACE FILENAME pinocchio/warning.hpp LIBRARY_NAME pinocchio)
jrl_target_generate_deprecated_header(pinocchio_headers INTERFACE FILENAME pinocchio/deprecated.hpp LIBRARY_NAME pinocchio)
jrl_target_generate_tracy_header(pinocchio_headers INTERFACE FILENAME pinocchio/tracy.hpp  LIBRARY_NAME pinocchio)

target_compile_definitions(pinocchio_headers INTERFACE
    $<$<BOOL:${INITIALIZE_WITH_NAN}>:EIGEN_INITIALIZE_MATRICES_BY_NAN>
)

target_compile_definitions(pinocchio_headers INTERFACE
    INTERFACE
        BOOST_MPL_LIMIT_LIST_SIZE=30
        BOOST_MPL_LIMIT_VECTOR_SIZE=30
        BOOST_MPL_CFG_NO_PREPROCESSED_HEADERS
        BOOST_FUSION_INVOKE_MAX_ARITY=12
        $<BUILD_INTERFACE:PINOCCHIO_DISABLE_UNSUPPORTED_WARNINGS>
)

target_link_libraries(
    pinocchio_headers
    INTERFACE
        Eigen3::Eigen 
        Boost::boost
        Boost::serialization  
)

target_include_directories(
    pinocchio_headers
    INTERFACE
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

# Dependency free headers
# Root, algorithm, common, containers, math, multibody, serialization, spatial, utils
jrl_target_headers(pinocchio_headers PUBLIC
    HEADERS 
    include/pinocchio/algorithm/aba-derivatives.hpp
    include/pinocchio/algorithm/aba.hpp
    include/pinocchio/algorithm/center-of-mass-derivatives.hpp
    include/pinocchio/algorithm/center-of-mass.hpp
    include/pinocchio/algorithm/centroidal-derivatives.hpp
    include/pinocchio/algorithm/centroidal.hpp
    include/pinocchio/algorithm/check-base.hpp
    include/pinocchio/algorithm/check-data.hpp
    include/pinocchio/algorithm/check-model.hpp
    include/pinocchio/algorithm/check.hpp
    include/pinocchio/algorithm/cholesky.hpp
    include/pinocchio/algorithm/compute-all-terms.hpp
    include/pinocchio/algorithm/constrained-dynamics-derivatives.hpp
    include/pinocchio/algorithm/constrained-dynamics.hpp
    include/pinocchio/algorithm/constraint-cholesky-decl.hpp
    include/pinocchio/algorithm/constraint-cholesky.hpp
    include/pinocchio/algorithm/contact-dynamics.hpp
    include/pinocchio/algorithm/contact-inverse-dynamics.hpp
    include/pinocchio/algorithm/copy.hpp
    include/pinocchio/algorithm/crba.hpp
    include/pinocchio/algorithm/default-check.hpp
    include/pinocchio/algorithm/delassus-operator.hpp
    include/pinocchio/algorithm/delassus.hpp
    include/pinocchio/algorithm/diagonal-preconditioner.hpp
    include/pinocchio/algorithm/energy.hpp
    include/pinocchio/algorithm/frames-derivatives.hpp
    include/pinocchio/algorithm/frames.hpp
    include/pinocchio/algorithm/fwd.hpp
    include/pinocchio/algorithm/geometry.hpp
    include/pinocchio/algorithm/impulse-dynamics-derivatives.hpp
    include/pinocchio/algorithm/impulse-dynamics.hpp
    include/pinocchio/algorithm/jacobian.hpp
    include/pinocchio/algorithm/joint-configuration.hpp
    include/pinocchio/algorithm/kinematics-derivatives.hpp
    include/pinocchio/algorithm/kinematics.hpp
    include/pinocchio/algorithm/loop-constrained-aba.hpp
    include/pinocchio/algorithm/model.hpp
    include/pinocchio/algorithm/preconditioner-base.hpp
    include/pinocchio/algorithm/proximal.hpp
    include/pinocchio/algorithm/pv.hpp
    include/pinocchio/algorithm/regressor.hpp
    include/pinocchio/algorithm/rnea-derivatives.hpp
    include/pinocchio/algorithm/rnea-second-order-derivatives.hpp
    include/pinocchio/algorithm/rnea.hpp
    include/pinocchio/algorithm/solvers/admm-solver.hpp
    include/pinocchio/algorithm/solvers/anderson-acceleration.hpp
    include/pinocchio/algorithm/solvers/constraint-solver-base.hpp
    include/pinocchio/algorithm/solvers/constraint-solver-utils.hpp
    include/pinocchio/algorithm/solvers/fwd.hpp
    include/pinocchio/algorithm/solvers/pgs-solver.hpp
    include/pinocchio/algorithm/utils/force.hpp
    include/pinocchio/algorithm/utils/motion.hpp
    include/pinocchio/common/fwd.hpp
    include/pinocchio/common.hpp
    include/pinocchio/constraints/fwd.hpp
    include/pinocchio/constraints.hpp
    include/pinocchio/container/double-entry-container.hpp
    include/pinocchio/container/eigen-storage.hpp
    include/pinocchio/container/matrix-stack.hpp
    include/pinocchio/context.hpp
    include/pinocchio/eigen-common.hpp
    include/pinocchio/fwd.hpp
    include/pinocchio/geometry.hpp
    include/pinocchio/geometry/fwd.hpp
    include/pinocchio/macros.hpp
    include/pinocchio/math/fwd.hpp
    include/pinocchio/math/multiprecision-mpfr.hpp
    include/pinocchio/math/multiprecision.hpp
    include/pinocchio/math.hpp
    include/pinocchio/multibody/fwd.hpp
    include/pinocchio/multibody/joint/fwd.hpp
    include/pinocchio/multibody/joint-motion-subspace.hpp
    include/pinocchio/multibody/joint.hpp
    include/pinocchio/multibody/liegroup/fwd.hpp
    include/pinocchio/multibody/liegroup.hpp
    include/pinocchio/multibody/pool/fwd.hpp
    include/pinocchio/multibody/pool.hpp
    include/pinocchio/multibody/sample-models.hpp
    include/pinocchio/multibody/visitor.hpp
    include/pinocchio/multibody.hpp
    include/pinocchio/serialization/archive.hpp
    include/pinocchio/serialization/fwd.hpp
    include/pinocchio/serialization/eigen.hpp
    include/pinocchio/serialization/serializable.hpp
    include/pinocchio/serialization.hpp
    include/pinocchio/spatial/fwd.hpp
    include/pinocchio/spatial.hpp
    include/pinocchio/unsupported.hpp
    include/pinocchio/utils/alloca.hpp
    include/pinocchio/utils/cast.hpp
    include/pinocchio/utils/check.hpp
    include/pinocchio/utils/eigen-helpers.hpp
    include/pinocchio/utils/file-io.hpp
    include/pinocchio/utils/openmp.hpp
    include/pinocchio/utils/promote-static-eval.hpp
    include/pinocchio/utils/reference.hpp
    include/pinocchio/utils/shared-ptr.hpp
    include/pinocchio/utils/size-in-bytes.hpp
    include/pinocchio/utils/static-if.hpp
    include/pinocchio/utils/std-vector.hpp
    include/pinocchio/utils/string-generator.hpp
    include/pinocchio/utils/string.hpp
    include/pinocchio/utils/timer.hpp
    include/pinocchio/utils/timer2.hpp
    include/pinocchio/utils/version.hpp
    BASE_DIRS include
)

jrl_target_headers(pinocchio_headers PUBLIC
    HEADERS 
    include/pinocchio/src/visualizers/base-visualizer.hxx
    include/pinocchio/src/common/fwd.hxx
    include/pinocchio/src/common/model-entity.hxx
    include/pinocchio/src/common/data-entity.hxx
    include/pinocchio/src/eigen-macros.hxx
    include/pinocchio/src/context/generic.hxx
    include/pinocchio/src/context/casadi.hxx
    include/pinocchio/src/context/cppad.hxx
    include/pinocchio/src/context/default.hxx
    include/pinocchio/src/context/cppadcg.hxx
    include/pinocchio/src/multibody/data.hxx
    include/pinocchio/src/multibody/model-item.hxx
    include/pinocchio/src/multibody/joint-motion-subspace-base.hxx
    include/pinocchio/src/multibody/force-set.hxx
    include/pinocchio/src/multibody/liegroup/vector-space.hxx
    include/pinocchio/src/multibody/model.hxx
    include/pinocchio/src/multibody/liegroup/cartesian-product.hxx
    include/pinocchio/src/multibody/liegroup/liegroup-generic.hxx
    include/pinocchio/src/multibody/liegroup/liegroup-base.hxx
    include/pinocchio/src/multibody/fwd.hxx
    include/pinocchio/src/multibody/joint-motion-subspace-generic.hxx
    include/pinocchio/src/multibody/frame.hxx
    include/pinocchio/src/multibody/sample-models.hxx
    include/pinocchio/src/geometry/fwd.hxx
    include/pinocchio/src/geometry/coal.hxx
    include/pinocchio/src/geometry/instance-filter.hxx
    include/pinocchio/src/geometry/geometry-object.hxx
    include/pinocchio/src/geometry/geometry.hxx
    include/pinocchio/src/geometry/geometry-object-filter.hxx
    include/pinocchio/src/algorithm/delassus-operator-preconditioned.hxx
    include/pinocchio/src/algorithm/crba.hxx
    include/pinocchio/src/algorithm/kinematics.hxx
    include/pinocchio/src/multibody/liegroup/liegroup-algo.hxx
    include/pinocchio/src/math/matrix-inverse-code-generated.hxx
    include/pinocchio/src/math/block-diagonal-matrix.hxx
    include/pinocchio/src/math/quaternion.hxx
    include/pinocchio/src/algorithm/contact-dynamics.hxx
    include/pinocchio/src/math/matrix-block-element-base.hxx
    include/pinocchio/src/math/matrix-inverse.hxx
    include/pinocchio/src/math/block-diagonal-matrix-base.hxx
    include/pinocchio/src/math/taylor-expansion.hxx
    include/pinocchio/src/math/assign-operators.hxx
    include/pinocchio/src/math/lanczos-decomposition.hxx
    include/pinocchio/src/eigen-ref.hxx
    include/pinocchio/src/macros.hxx
    include/pinocchio/src/fwd.hxx
    include/pinocchio/src/algorithm/regressor.hxx
    include/pinocchio/src/algorithm/frames.hxx
    include/pinocchio/src/math/alias.hxx
    include/pinocchio/src/math/eigenvalues.hxx
    include/pinocchio/src/multibody/visitor/joint-unary-visitor.hxx
    include/pinocchio/src/multibody/visitor/joint-binary-visitor.hxx
    include/pinocchio/src/multibody/visitor/fusion.hxx
    include/pinocchio/src/multibody/liegroup/special-orthogonal.hxx
    include/pinocchio/src/multibody/liegroup/special-euclidean.hxx
    include/pinocchio/src/multibody/liegroup/cartesian-product-variant.hxx
    include/pinocchio/src/multibody/liegroup/liegroup-map.hxx
    include/pinocchio/src/multibody/liegroup/liegroup-collection.hxx
    include/pinocchio/src/multibody/liegroup/liegroup-joint.hxx
    include/pinocchio/src/multibody/liegroup/liegroup-variant-visitors.hxx
    include/pinocchio/src/multibody/liegroup/fwd.hxx
    include/pinocchio/src/multibody/liegroup/cartesian-product-variant-fwd.hxx
    include/pinocchio/src/spatial/classic-acceleration.hxx
    include/pinocchio/src/spatial/motion-dense.hxx
    include/pinocchio/src/spatial/se3-base.hxx
    include/pinocchio/src/spatial/cartesian-axis.hxx
    include/pinocchio/src/spatial/symmetric3.hxx
    include/pinocchio/src/spatial/spatial-axis.hxx
    include/pinocchio/src/spatial/log-common.hxx
    include/pinocchio/src/spatial/se3-tpl.hxx
    include/pinocchio/src/spatial/motion-zero.hxx
    include/pinocchio/src/spatial/log.hxx
    include/pinocchio/src/spatial/inertia.hxx
    include/pinocchio/src/spatial/se3-common.hxx
    include/pinocchio/src/algorithm/check-model.hxx
    include/pinocchio/src/algorithm/rnea.hxx
    include/pinocchio/src/math/matrix-block-element.hxx
    include/pinocchio/src/math/matrix-block-type.hxx
    include/pinocchio/src/math/comparison-operators.hxx
    include/pinocchio/src/math/multiprecision.hxx
    include/pinocchio/src/algorithm/joint-configuration.hxx
    include/pinocchio/src/algorithm/center-of-mass.hxx
    include/pinocchio/src/spatial/se3-tpl-interpolate.hxx
    include/pinocchio/src/spatial/force-base.hxx
    include/pinocchio/src/algorithm/delassus-operator-cholesky-expression.hxx
    include/pinocchio/src/algorithm/constraint-cholesky-decl.hxx
    include/pinocchio/src/algorithm/default-check.hxx
    include/pinocchio/src/spatial/motion-common.hxx
    include/pinocchio/src/algorithm/jacobian.hxx
    include/pinocchio/src/algorithm/kinematics-derivatives.hxx
    include/pinocchio/src/serialization/symmetric3.hxx
    include/pinocchio/src/algorithm/contact-inverse-dynamics.hxx
    include/pinocchio/src/algorithm/diagonal-preconditioner.hxx
    include/pinocchio/src/math/matrix-block-element-operations.hxx
    include/pinocchio/src/math/fwd.hxx
    include/pinocchio/src/math/matrix-info.hxx
    include/pinocchio/src/spatial/motion-tpl.hxx
    include/pinocchio/src/serialization/constraints-data.hxx
    include/pinocchio/src/algorithm/rnea-second-order-derivatives.hxx
    include/pinocchio/src/math/matrix-block.hxx
    include/pinocchio/src/math/eigen-helpers.hxx
    include/pinocchio/src/math/matrix-product.hxx
    include/pinocchio/src/spatial/motion-ref.hxx
    include/pinocchio/src/context.hxx
    include/pinocchio/src/eigen-align.hxx
    include/pinocchio/src/spatial/fwd.hxx
    include/pinocchio/src/algorithm/delassus-operator-rigid-body-visitors.hxx
    include/pinocchio/src/math/matrix-block-element-operation.hxx
    include/pinocchio/src/container/aligned-vector.hxx
    include/pinocchio/src/container/matrix-stack.hxx
    include/pinocchio/src/container/double-entry-container.hxx
    include/pinocchio/src/container/eigen-storage.hxx
    include/pinocchio/src/algorithm/utils/force.hxx
    include/pinocchio/src/spatial/act-on-set.hxx
    include/pinocchio/src/spatial/force-dense.hxx
    include/pinocchio/src/spatial/force-ref.hxx
    include/pinocchio/src/spatial/explog-quaternion.hxx
    include/pinocchio/src/spatial/force-common.hxx
    include/pinocchio/src/spatial/motion-base.hxx
    include/pinocchio/src/utils/timer.hxx
    include/pinocchio/src/spatial/force-tpl.hxx
    include/pinocchio/src/spatial/skew.hxx
    include/pinocchio/src/serialization/inertia.hxx
    include/pinocchio/src/serialization/vector.hxx
    include/pinocchio/src/spatial/explog.hxx
    include/pinocchio/src/serialization/force.hxx
    include/pinocchio/src/serialization/joints-data.hxx
    include/pinocchio/src/algorithm/utils/motion.hxx
    include/pinocchio/src/serialization/block-diagonal-matrix.hxx
    include/pinocchio/src/serialization/eigen.hxx
    include/pinocchio/src/algorithm/constraint-cholesky-def.hxx
    include/pinocchio/src/serialization/model.hxx
    include/pinocchio/src/serialization/eigen-storage.hxx
    include/pinocchio/src/algorithm/impulse-dynamics.hxx
    include/pinocchio/src/serialization/motion.hxx
    include/pinocchio/src/serialization/se3.hxx
    include/pinocchio/src/algorithm/preconditioner-base.hxx
    include/pinocchio/src/serialization/static-buffer.hxx
    include/pinocchio/src/math/gram-schmidt-orthonormalisation.hxx
    include/pinocchio/src/algorithm/constrained-dynamics.hxx
    include/pinocchio/src/math/block-diagonal-matrix-inverse.hxx
    include/pinocchio/src/algorithm/model.hxx
    include/pinocchio/src/algorithm/rnea-derivatives.hxx
    include/pinocchio/src/utils/check.hxx
    include/pinocchio/src/algorithm/centroidal.hxx
    include/pinocchio/src/math/multiprecision-mpfr.hxx
    include/pinocchio/src/serialization/frame.hxx
    include/pinocchio/src/serialization/double-entry-container.hxx
    include/pinocchio/src/algorithm/delassus.hxx
    include/pinocchio/src/algorithm/loop-constrained-aba.hxx
    include/pinocchio/src/algorithm/fwd.hxx
    include/pinocchio/src/math/arithmetic-operators.hxx
    include/pinocchio/src/math/tridiagonal-matrix.hxx
    include/pinocchio/src/math/block-diagonal-matrix-sum.hxx
    include/pinocchio/src/math/tensor.hxx
    include/pinocchio/src/serialization/matrix-block-element.hxx
    include/pinocchio/src/serialization/fwd.hxx
    include/pinocchio/src/algorithm/proximal.hxx
    include/pinocchio/src/math/eigenvalues-tridiagonal-matrix.hxx
    include/pinocchio/src/math/block-diagonal-matrix-expression.hxx
    include/pinocchio/src/utils/promote-static-eval.hxx
    include/pinocchio/src/multibody/pool/model.hxx
    include/pinocchio/src/serialization/delassus.hxx
    include/pinocchio/src/algorithm/pv.hxx
    include/pinocchio/src/multibody/pool/fwd.hxx
    include/pinocchio/src/math/rpy.hxx
    include/pinocchio/src/utils/string.hxx
    include/pinocchio/src/serialization/boost-blank.hxx
    include/pinocchio/src/algorithm/delassus-operator-sparse.hxx
    include/pinocchio/src/math/sincos.hxx
    include/pinocchio/src/utils/file-io.hxx
    include/pinocchio/src/utils/version.hxx
    include/pinocchio/src/serialization/joints-motion-subspace.hxx
    include/pinocchio/src/serialization/matrix-stack.hxx
    include/pinocchio/src/algorithm/constrained-dynamics-derivatives.hxx
    include/pinocchio/src/algorithm/delassus-operator-dense.hxx
    include/pinocchio/src/utils/alloca.hxx
    include/pinocchio/src/multibody/pool/geometry.hxx
    include/pinocchio/src/math/matrix-block-element-plain.hxx
    include/pinocchio/src/serialization/geometry.hxx
    include/pinocchio/src/algorithm/energy.hxx
    include/pinocchio/src/math/rotation.hxx
    include/pinocchio/src/utils/string-generator.hxx
    include/pinocchio/src/serialization/archive.hxx
    include/pinocchio/src/math/triangular-matrix.hxx
    include/pinocchio/src/math/matrix.hxx
    include/pinocchio/src/math/sign.hxx
    include/pinocchio/src/utils/size-in-bytes.hxx
    include/pinocchio/src/utils/timer2.hxx
    include/pinocchio/src/utils/cast.hxx
    include/pinocchio/src/utils/eigen-helpers.hxx
    include/pinocchio/src/utils/reference.hxx
    include/pinocchio/src/utils/static-if.hxx
    include/pinocchio/src/multibody/joint/joint-universal.hxx
    include/pinocchio/src/multibody/joint/joint-helical.hxx
    include/pinocchio/src/multibody/joint/joint-ellipsoid.hxx
    include/pinocchio/src/math/details/matrix-inverse-9x9.hxx
    include/pinocchio/src/math/details/matrix-inverse-12x12.hxx
    include/pinocchio/src/math/details/matrix-inverse-11x11.hxx
    include/pinocchio/src/utils/openmp.hxx
    include/pinocchio/src/serialization/joints-motion.hxx
    include/pinocchio/src/serialization/constraints-model.hxx
    include/pinocchio/src/utils/eigen-fix.hxx
    include/pinocchio/src/multibody/joint/joint-composite.hxx
    include/pinocchio/src/math/details/matrix-inverse-4x4.hxx
    include/pinocchio/src/math/details/matrix-inverse-3x3.hxx
    include/pinocchio/src/math/details/matrix-inverse-6x6.hxx
    include/pinocchio/src/math/details/matrix-inverse-10x10.hxx
    include/pinocchio/src/math/details/matrix-inverse-5x5.hxx
    include/pinocchio/src/multibody/joint/joint-revolute-unbounded.hxx
    include/pinocchio/src/serialization/data.hxx
    include/pinocchio/src/serialization/coal.hxx
    include/pinocchio/src/serialization/joints-model.hxx
    include/pinocchio/src/utils/std-vector.hxx
    include/pinocchio/src/utils/shared-ptr.hxx
    include/pinocchio/src/math/details/matrix-inverse-7x7.hxx
    include/pinocchio/src/serialization/constraints-set.hxx
    include/pinocchio/src/math/details/matrix-inverse-2x2.hxx
    include/pinocchio/src/multibody/joint/joint-collection.hxx
    include/pinocchio/src/serialization/serializable.hxx
    include/pinocchio/src/math/details/matrix-inverse-8x8.hxx
    include/pinocchio/src/algorithm/centroidal-derivatives.hxx
    include/pinocchio/src/algorithm/delassus-operator-base.hxx
    include/pinocchio/src/multibody/joint/joint-spherical.hxx
    include/pinocchio/src/math/details/matrix-inverse-1x1.hxx
    include/pinocchio/src/serialization/common.hxx
    include/pinocchio/src/serialization/csv.hxx
    include/pinocchio/src/algorithm/aba-derivatives.hxx
    include/pinocchio/src/algorithm/aba.hxx
    include/pinocchio/src/serialization/joints-transform.hxx
    include/pinocchio/src/algorithm/compute-all-terms.hxx
    include/pinocchio/src/algorithm/delassus-operator-rigid-body.hxx
    include/pinocchio/src/algorithm/center-of-mass-derivatives.hxx
    include/pinocchio/src/algorithm/check-base.hxx
    include/pinocchio/src/algorithm/check-data.hxx
    include/pinocchio/src/algorithm/impulse-dynamics-derivatives.hxx
    include/pinocchio/src/algorithm/copy.hxx
    include/pinocchio/src/algorithm/cholesky.hxx
    include/pinocchio/src/algorithm/frames-derivatives.hxx
    include/pinocchio/src/multibody/joint/joint-spherical-ZYX.hxx
    include/pinocchio/src/multibody/joint/joint-planar.hxx
    include/pinocchio/src/algorithm/geometry.hxx
    include/pinocchio/src/multibody/joint/joint-revolute-unaligned.hxx
    include/pinocchio/src/algorithm/solvers/anderson-acceleration.hxx
    include/pinocchio/src/algorithm/solvers/constraint-solver-base.hxx
    include/pinocchio/src/algorithm/solvers/pgs-solver.hxx
    include/pinocchio/src/algorithm/solvers/admm-solver.hxx
    include/pinocchio/src/algorithm/solvers/constraint-solver-utils.hxx
    include/pinocchio/src/multibody/joint/joint-translation.hxx
    include/pinocchio/src/multibody/joint/joint-prismatic-unaligned.hxx
    include/pinocchio/src/multibody/joint/joint-mimic.hxx
    include/pinocchio/src/multibody/joint/fwd.hxx
    include/pinocchio/src/multibody/joint/joint-free-flyer.hxx
    include/pinocchio/src/multibody/joint/joint-common-operations.hxx
    include/pinocchio/src/multibody/joint/joint-prismatic.hxx
    include/pinocchio/src/multibody/joint/joint-generic.hxx
    include/pinocchio/src/multibody/joint/joint-revolute-unbounded-unaligned.h
    include/pinocchio/src/multibody/joint/joint-helical-unaligned.hxx
    include/pinocchio/src/multibody/joint/joint-revolute.hxx
    include/pinocchio/src/multibody/joint/joint-data-base.hxx
    include/pinocchio/src/multibody/joint/joint-basic-visitors.hxx
    include/pinocchio/src/multibody/joint/joint-model-base.hxx
    include/pinocchio/src/constraints/constraint-data-base.hxx
    include/pinocchio/src/constraints/kinematics-constraint-model-base.hxx
    include/pinocchio/src/constraints/baumgarte-corrector-parameters.hxx
    include/pinocchio/src/constraints/point-anchor-constraint.hxx
    include/pinocchio/src/constraints/constraint-model-common-parameters.hxx
    include/pinocchio/src/constraints/constraint-collection-default.hxx
    include/pinocchio/src/constraints/visitors/constraint-model-visitor.hxx
    include/pinocchio/src/constraints/frame-constraint-data-base.hxx
    include/pinocchio/src/constraints/point-constraint-data-base.hxx
    include/pinocchio/src/constraints/binary-kinematics-constraint-model-base.
    include/pinocchio/src/constraints/constraint-model-base.hxx
    include/pinocchio/src/constraints/blank-constraint.hxx
    include/pinocchio/src/constraints/constraint-ordering.hxx
    include/pinocchio/src/constraints/fwd.hxx
    include/pinocchio/src/constraints/jointwise-constraint-model-base.hxx
    include/pinocchio/src/constraints/point-constraint-model-base.hxx
    include/pinocchio/src/constraints/joint-limit-constraint.hxx
    include/pinocchio/src/constraints/sets/second-order-cone-jordan-operation.
    include/pinocchio/src/constraints/sets/full-space-cone.hxx
    include/pinocchio/src/constraints/sets/orthant-cone.hxx
    include/pinocchio/src/constraints/sets/orthant-cone-jordan-operation.hxx
    include/pinocchio/src/constraints/sets/coulomb-friction-cone.hxx
    include/pinocchio/src/constraints/sets/cone-base.hxx
    include/pinocchio/src/constraints/sets/zero-cone-jordan-operation.hxx
    include/pinocchio/src/constraints/sets/box-set.hxx
    include/pinocchio/src/constraints/sets/zero-cone.hxx
    include/pinocchio/src/constraints/sets/set-base.hxx
    include/pinocchio/src/constraints/sets/jordan-operation-helper.hxx
    include/pinocchio/src/constraints/constraint-data-generic.hxx
    include/pinocchio/src/constraints/utils.hxx
    include/pinocchio/src/constraints/frame-constraint-model-base.hxx
    include/pinocchio/src/constraints/point-contact-constraint.hxx
    include/pinocchio/src/constraints/frame-anchor-constraint.hxx
    include/pinocchio/src/constraints/joint-friction-constraint.hxx
    include/pinocchio/src/constraints/contact-info.hxx
    include/pinocchio/src/constraints/constraint-model-generic.hxx
    include/pinocchio/src/constraints/rigid-constraint-conversion.hxx
  BASE_DIRS include
)

# Target to link against all pinocchio libraries
add_library(pinocchio INTERFACE)
add_library(pinocchio::pinocchio ALIAS pinocchio)

add_library(pinocchio_default INTERFACE)
target_link_libraries(pinocchio_default INTERFACE pinocchio_headers)
target_link_libraries(pinocchio INTERFACE pinocchio_default)

# List of exported targets
set(EXPORTED_TARGETS pinocchio pinocchio_default pinocchio_headers)

if(BUILD_WITH_COLLISION_SUPPORT)
    add_library(pinocchio_collision SHARED)

    target_sources(
        pinocchio_collision
        PRIVATE
            src/collision/collision.cpp
            src/collision/distance.cpp
    )

    jrl_target_headers(pinocchio_collision PUBLIC 
      HEADERS 
        include/pinocchio/collision/broadphase-callbacks.hpp
        include/pinocchio/collision/broadphase-manager-base.hpp
        include/pinocchio/collision/broadphase-manager.hpp
        include/pinocchio/collision/broadphase.hpp
        include/pinocchio/collision/coal-pinocchio-conversions.hpp
        include/pinocchio/collision/collision.hpp
        include/pinocchio/collision/distance.hpp
        include/pinocchio/collision/pool/broadphase-manager.hpp
        include/pinocchio/collision/pool/fwd.hpp
        include/pinocchio/collision/tree-broadphase-manager.hpp
      BASE_DIRS include
    )

    jrl_target_headers(pinocchio_collision PUBLIC
        HEADERS
            include/pinocchio/src/collision/broadphase-callbacks.hxx
            include/pinocchio/src/collision/broadphase-manager.hxx
            include/pinocchio/src/collision/broadphase.hxx
            include/pinocchio/src/collision/collision.hxx
            include/pinocchio/src/collision/distance.hxx
            include/pinocchio/src/collision/pool/broadphase-manager.hxx
            include/pinocchio/src/collision/pool/fwd.hxx
            include/pinocchio/src/collision/tree-broadphase-manager.hxx
        BASE_DIRS include
    )

    target_link_libraries(pinocchio_collision
        PUBLIC pinocchio_default coal::coal)

    target_compile_definitions(pinocchio_collision PUBLIC PINOCCHIO_WITH_COLLISION)
    # DEPRECATED: Setup back compatibility define
    target_compile_definitions(pinocchio_collision PUBLIC PINOCCHIO_WITH_HPP_FCL)
    jrl_target_set_default_compile_options(pinocchio_collision PRIVATE)
    jrl_target_enforce_msvc_conformance(pinocchio_collision PRIVATE)
    jrl_target_generate_config_header(pinocchio_collision INTERFACE FILENAME pinocchio/collision/config.hpp VERSION ${PROJECT_VERSION} LIBRARY_NAME pinocchio_collision)

    target_link_libraries(pinocchio INTERFACE pinocchio_collision)
    list(APPEND EXPORTED_TARGETS pinocchio_collision)
endif()

if(ENABLE_TEMPLATE_INSTANTIATION)
    add_library(pinocchio_template_instantiation_default SHARED)

    target_sources(pinocchio_template_instantiation_default PRIVATE
        src/algorithm/cholesky.cpp
        src/algorithm/aba.cpp
        src/algorithm/regressor.cpp
        src/algorithm/contact-dynamics.cpp
        src/algorithm/frames-derivatives.cpp
        src/algorithm/impulse-dynamics.cpp
        src/algorithm/model.cpp
        src/algorithm/constrained-dynamics.cpp
        src/algorithm/rnea-derivatives.cpp
        src/algorithm/compute-all-terms.cpp
        src/algorithm/jacobian.cpp
        src/algorithm/energy.cpp
        src/algorithm/centroidal-derivatives.cpp
        src/algorithm/frames.cpp
        src/algorithm/constrained-dynamics-derivatives.cpp
        src/algorithm/center-of-mass.cpp
        src/algorithm/geometry.cpp
        src/algorithm/kinematics.cpp
        src/algorithm/rnea.cpp
        src/algorithm/centroidal.cpp
        src/algorithm/aba-derivatives.cpp
        src/algorithm/crba.cpp
        src/algorithm/constraint-cholesky.cpp
        src/algorithm/joint-configuration.cpp
        src/algorithm/center-of-mass-derivatives.cpp
        src/algorithm/solvers/admm-solver.cpp
        src/algorithm/solvers/pgs-solver.cpp
        src/algorithm/kinematics-derivatives.cpp
        src/algorithm/constraints/utils.cpp
        src/multibody/model.cpp
        src/multibody/data.cpp
        src/multibody/sample-models.cpp
    )

    target_link_libraries(
        pinocchio_template_instantiation_default
        PUBLIC pinocchio_headers)

    target_compile_definitions(
        pinocchio_template_instantiation_default
        PUBLIC PINOCCHIO_ENABLE_TEMPLATE_INSTANTIATION
    )

    jrl_target_set_default_compile_options(pinocchio_template_instantiation_default PRIVATE)
    jrl_target_enforce_msvc_conformance(pinocchio_template_instantiation_default PRIVATE)

    target_link_libraries(
        pinocchio_default
        INTERFACE pinocchio_template_instantiation_default
    )

    list(APPEND EXPORTED_TARGETS pinocchio_template_instantiation_default)

endif()

# ----------------------------------------------------
# --- EXPORT  ----------------------------------------
# ----------------------------------------------------
jrl_add_export_component(NAME pinocchio
  TARGETS
    ${EXPORTED_TARGETS}
)
jrl_export_package()

# Print summaries
jrl_print_dependencies_summary()
jrl_print_options_summary()
