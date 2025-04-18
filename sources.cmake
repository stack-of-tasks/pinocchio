# Define Pinocchio sources and headers

set(${PROJECT_NAME}_CORE_SOURCES)

set(${PROJECT_NAME}_CORE_PUBLIC_HEADERS
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/aba-derivatives.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/aba-derivatives.hxx
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/aba.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/aba.hxx
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/admm-solver.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/admm-solver.hxx
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/center-of-mass-derivatives.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/center-of-mass-derivatives.hxx
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/center-of-mass.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/center-of-mass.hxx
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/centroidal-derivatives.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/centroidal-derivatives.hxx
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/centroidal.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/centroidal.hxx
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/check.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/check-base.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/check-data.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/check-data.hxx
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/check-model.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/check-model.hxx
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/cholesky.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/cholesky.hxx
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/compute-all-terms.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/compute-all-terms.hxx
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/constrained-dynamics-derivatives.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/constrained-dynamics-derivatives.hxx
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/constrained-dynamics.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/constrained-dynamics.hxx
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/constrained-problem-data.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/constraints/coulomb-friction-cone.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/constraints/visitors/constraint-model-visitor.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/constraints/constraint-data-base.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/constraints/constraint-data-generic.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/constraints/constraint-model-base.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/constraints/constraint-model-generic.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/constraints/constraints.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/constraints/fwd.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/contact-cholesky.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/contact-cholesky.hxx
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/contact-dynamics.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/contact-dynamics.hxx
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/contact.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/contact.hxx
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/contact-info.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/contact-jacobian.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/contact-jacobian.hxx
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/contact-solver-base.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/contact-solver-utils.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/contact-inverse-dynamics.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/copy.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/crba.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/crba.hxx
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/default-check.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/delassus.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/delassus.hxx
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/delassus-operartor-ref.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/delassus-operator-base.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/delassus-operator-dense.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/delassus-operator-rigid-body.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/delassus-operator-sparse.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/energy.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/energy.hxx
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/frames-derivatives.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/frames-derivatives.hxx
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/frames.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/frames.hxx
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/fwd.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/geometry.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/geometry.hxx
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/impulse-dynamics-derivatives.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/impulse-dynamics-derivatives.hxx
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/impulse-dynamics.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/impulse-dynamics.hxx
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/jacobian.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/jacobian.hxx
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/joint-configuration.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/joint-configuration.hxx
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/kinematics-derivatives.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/kinematics-derivatives.hxx
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/kinematics.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/kinematics.hxx
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/model.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/model.hxx
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/pgs-solver.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/pgs-solver.hxx
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/proximal.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/pv.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/pv.hxx
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/regressor.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/regressor.hxx
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/rnea-derivatives.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/rnea-derivatives.hxx
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/rnea.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/rnea.hxx
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/rnea-second-order-derivatives.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/rnea-second-order-derivatives.hxx
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/utils/force.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/utils/motion.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/autodiff/casadi-algo.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/autodiff/casadi.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/autodiff/casadi/math/matrix.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/autodiff/casadi/math/quaternion.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/autodiff/casadi/math/triangular-matrix.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/autodiff/casadi/spatial/se3-tpl.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/autodiff/casadi/utils/static-if.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/autodiff/cppad/algorithm/aba.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/autodiff/cppad.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/autodiff/cppad/math/eigen_plugin.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/autodiff/cppad/math/quaternion.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/autodiff/cppad/spatial/log.hxx
    ${PROJECT_SOURCE_DIR}/include/pinocchio/autodiff/cppad/spatial/se3-tpl.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/autodiff/cppad/utils/static-if.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/codegen/code-generator-algo.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/codegen/code-generator-base.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/codegen/cppadcg.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/container/aligned-vector.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/container/boost-container-limits.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/context/casadi.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/context/cppad.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/context/cppadcg.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/context/default.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/context/generic.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/context.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/core/binary-op.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/core/unary-op.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/deprecated-macros.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/deprecated-namespaces.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/deprecation.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/eigen-macros.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/fwd.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/macros.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/unsupported.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/math/casadi.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/math/comparison-operators.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/math/cppadcg.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/math/cppad.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/math/eigenvalues.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/math/fwd.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/math/gram-schmidt-orthonormalisation.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/math/lanczos-decomposition.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/math/matrix-block.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/math/matrix.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/math/multiprecision.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/math/multiprecision-mpfr.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/math/quaternion.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/math/rotation.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/math/rpy.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/math/rpy.hxx
    ${PROJECT_SOURCE_DIR}/include/pinocchio/math/sign.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/math/sincos.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/math/taylor-expansion.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/math/tensor.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/math/triangular-matrix.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/math/tridiagonal-matrix.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/multibody/data.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/multibody/data.hxx
    ${PROJECT_SOURCE_DIR}/include/pinocchio/multibody/fcl.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/multibody/force-set.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/multibody/frame.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/multibody/fwd.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/multibody/geometry.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/multibody/geometry.hxx
    ${PROJECT_SOURCE_DIR}/include/pinocchio/multibody/geometry-object-filter.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/multibody/geometry-object.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/multibody/geometry-object.hxx
    ${PROJECT_SOURCE_DIR}/include/pinocchio/multibody/instance-filter.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/multibody/joint/fwd.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/multibody/joint/joint-base.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/multibody/joint/joint-basic-visitors.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/multibody/joint/joint-basic-visitors.hxx
    ${PROJECT_SOURCE_DIR}/include/pinocchio/multibody/joint/joint-collection.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/multibody/joint/joint-common-operations.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/multibody/joint/joint-composite.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/multibody/joint/joint-composite.hxx
    ${PROJECT_SOURCE_DIR}/include/pinocchio/multibody/joint/joint-data-base.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/multibody/joint/joint-free-flyer.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/multibody/joint/joint-generic.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/multibody/joint/joint-helical.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/multibody/joint/joint-helical-unaligned.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/multibody/joint/joint-mimic.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/multibody/joint/joint-model-base.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/multibody/joint/joint-planar.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/multibody/joint/joint-prismatic.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/multibody/joint/joint-prismatic-unaligned.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/multibody/joint/joint-revolute.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/multibody/joint/joint-revolute-unaligned.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/multibody/joint/joint-revolute-unbounded.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/multibody/joint/joint-revolute-unbounded-unaligned.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/multibody/joint/joints.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/multibody/joint/joint-spherical.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/multibody/joint/joint-spherical-ZYX.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/multibody/joint/joint-translation.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/multibody/joint/joint-universal.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/multibody/joint-motion-subspace-base.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/multibody/joint-motion-subspace-generic.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/multibody/joint-motion-subspace.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/multibody/liegroup/cartesian-product.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/multibody/liegroup/cartesian-product-variant.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/multibody/liegroup/cartesian-product-variant.hxx
    ${PROJECT_SOURCE_DIR}/include/pinocchio/multibody/liegroup/fwd.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/multibody/liegroup/liegroup-algo.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/multibody/liegroup/liegroup-algo.hxx
    ${PROJECT_SOURCE_DIR}/include/pinocchio/multibody/liegroup/liegroup-base.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/multibody/liegroup/liegroup-base.hxx
    ${PROJECT_SOURCE_DIR}/include/pinocchio/multibody/liegroup/liegroup-collection.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/multibody/liegroup/liegroup-generic.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/multibody/liegroup/liegroup.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/multibody/liegroup/liegroup-variant-visitors.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/multibody/liegroup/liegroup-variant-visitors.hxx
    ${PROJECT_SOURCE_DIR}/include/pinocchio/multibody/liegroup/special-euclidean.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/multibody/liegroup/special-orthogonal.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/multibody/liegroup/vector-space.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/multibody/model.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/multibody/model.hxx
    ${PROJECT_SOURCE_DIR}/include/pinocchio/multibody/model-item.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/multibody/pool/fwd.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/multibody/pool/geometry.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/multibody/pool/model.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/multibody/visitor/fusion.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/multibody/visitor.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/multibody/visitor/joint-binary-visitor.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/multibody/visitor/joint-unary-visitor.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/multibody/sample-models.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/multibody/sample-models.hxx
    ${PROJECT_SOURCE_DIR}/include/pinocchio/serialization/aligned-vector.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/serialization/archive.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/serialization/data.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/serialization/eigen.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/serialization/csv.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/serialization/fcl.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/serialization/force.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/serialization/frame.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/serialization/fwd.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/serialization/geometry.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/serialization/inertia.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/serialization/joints-data.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/serialization/joints.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/serialization/joints-model.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/serialization/joints-motion.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/serialization/joints-motion-subspace.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/serialization/joints-transform.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/serialization/model.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/serialization/motion.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/serialization/se3.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/serialization/serializable.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/serialization/spatial.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/serialization/static-buffer.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/serialization/symmetric3.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/serialization/vector.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/spatial/act-on-set.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/spatial/act-on-set.hxx
    ${PROJECT_SOURCE_DIR}/include/pinocchio/spatial/cartesian-axis.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/spatial/classic-acceleration.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/spatial/explog.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/spatial/explog-quaternion.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/spatial/force-base.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/spatial/force-dense.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/spatial/force.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/spatial/force-ref.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/spatial/force-tpl.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/spatial/fwd.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/spatial/inertia.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/spatial/log.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/spatial/log.hxx
    ${PROJECT_SOURCE_DIR}/include/pinocchio/spatial/motion-base.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/spatial/motion-dense.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/spatial/motion.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/spatial/motion-ref.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/spatial/motion-tpl.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/spatial/motion-zero.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/spatial/se3-base.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/spatial/se3.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/spatial/se3-tpl.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/spatial/skew.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/spatial/spatial-axis.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/spatial/symmetric3.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/utils/axis-label.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/utils/cast.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/utils/check.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/utils/eigen-fix.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/utils/file-io.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/utils/helpers.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/utils/openmp.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/utils/shared-ptr.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/utils/static-if.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/utils/string-generator.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/utils/string.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/utils/timer2.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/utils/timer.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/utils/version.hpp)

set(${PROJECT_NAME}_PARALLEL_PUBLIC_HEADERS
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/parallel/aba.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/parallel/omp.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/parallel/rnea.hpp)

set(${PROJECT_NAME}_COLLISION_TEMPLATE_INSTANTIATION_SOURCES
    ${PROJECT_SOURCE_DIR}/src/collision/collision.cpp
    ${PROJECT_SOURCE_DIR}/src/collision/distance.cpp)

set(${PROJECT_NAME}_COLLISION_TEMPLATE_INSTANTIATION_PUBLIC_HEADERS
    ${PROJECT_SOURCE_DIR}/include/pinocchio/collision/collision.txx
    ${PROJECT_SOURCE_DIR}/include/pinocchio/collision/distance.txx)

set(${PROJECT_NAME}_COLLISION_PUBLIC_HEADERS
    ${PROJECT_SOURCE_DIR}/include/pinocchio/collision/collision.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/collision/collision.hxx
    ${PROJECT_SOURCE_DIR}/include/pinocchio/collision/broadphase-callbacks.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/collision/broadphase-callbacks.hxx
    ${PROJECT_SOURCE_DIR}/include/pinocchio/collision/broadphase.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/collision/broadphase.hxx
    ${PROJECT_SOURCE_DIR}/include/pinocchio/collision/distance.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/collision/distance.hxx
    ${PROJECT_SOURCE_DIR}/include/pinocchio/collision/broadphase-manager-base.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/collision/broadphase-manager.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/collision/broadphase-manager.hxx
    ${PROJECT_SOURCE_DIR}/include/pinocchio/collision/tree-broadphase-manager.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/collision/tree-broadphase-manager.hxx
    ${PROJECT_SOURCE_DIR}/include/pinocchio/collision/fcl-pinocchio-conversions.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/collision/pool/fwd.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/collision/pool/broadphase-manager.hpp
    # Deprecated header
    ${PROJECT_SOURCE_DIR}/include/pinocchio/spatial/fcl-pinocchio-conversions.hpp)

set(${PROJECT_NAME}_COLLISION_PARALLEL_PUBLIC_HEADERS
    ${PROJECT_SOURCE_DIR}/include/pinocchio/collision/parallel/broadphase.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/collision/parallel/geometry.hpp
    # Deprecated header
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/parallel/geometry.hpp)

set(${PROJECT_NAME}_PARSERS_SOURCES
    ${PROJECT_SOURCE_DIR}/src/utils/file-explorer.cpp
    ${PROJECT_SOURCE_DIR}/src/parsers/mjcf/mjcf-graph.cpp
    ${PROJECT_SOURCE_DIR}/src/parsers/mjcf/mjcf-graph-geom.cpp)

set(${PROJECT_NAME}_PARSERS_PUBLIC_HEADERS
    ${PROJECT_SOURCE_DIR}/include/pinocchio/parsers/meshloader-fwd.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/parsers/srdf.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/parsers/srdf.hxx
    ${PROJECT_SOURCE_DIR}/include/pinocchio/parsers/utils.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/utils/file-explorer.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/parsers/mjcf.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/parsers/mjcf/model.hxx
    ${PROJECT_SOURCE_DIR}/include/pinocchio/parsers/mjcf/geometry.hxx
    ${PROJECT_SOURCE_DIR}/include/pinocchio/parsers/mjcf/mjcf-graph.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/parsers/sample-models.hpp)

set(${PROJECT_NAME}_URDF_SOURCES
    ${PROJECT_SOURCE_DIR}/src/parsers/urdf/model.cpp
    ${PROJECT_SOURCE_DIR}/src/parsers/urdf/geometry.cpp
    ${PROJECT_SOURCE_DIR}/src/parsers/urdf/utils.cpp)

set(${PROJECT_NAME}_URDF_PUBLIC_HEADERS
    ${PROJECT_SOURCE_DIR}/include/pinocchio/parsers/urdf.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/parsers/urdf/model.hxx
    ${PROJECT_SOURCE_DIR}/include/pinocchio/parsers/urdf/geometry.hxx
    ${PROJECT_SOURCE_DIR}/include/pinocchio/parsers/urdf/utils.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/parsers/urdf/types.hpp)

set(${PROJECT_NAME}_SDF_SOURCES ${PROJECT_SOURCE_DIR}/src/parsers/sdf/model.cpp
                                ${PROJECT_SOURCE_DIR}/src/parsers/sdf/geometry.cpp)

set(${PROJECT_NAME}_SDF_PUBLIC_HEADERS
    ${PROJECT_SOURCE_DIR}/include/pinocchio/parsers/sdf.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/parsers/sdf/model.hxx
    ${PROJECT_SOURCE_DIR}/include/pinocchio/parsers/sdf/geometry.hxx)

set(${PROJECT_NAME}_PYTHON_PARSER_PUBLIC_HEADERS # TODO : Equivalent for nanobind (if needed)
    ${PROJECT_SOURCE_DIR}/include/pinocchio/parsers/python.hpp)

set(${PROJECT_NAME}_PYTHON_PARSER_SOURCES ${PROJECT_SOURCE_DIR}/src/parsers/python/model.cpp)

set(${PROJECT_NAME}_EXTRA_SOURCES ${PROJECT_SOURCE_DIR}/src/extra/reachable-workspace.cpp)

set(${PROJECT_NAME}_EXTRA_PUBLIC_HEADERS
    ${PROJECT_SOURCE_DIR}/include/pinocchio/extra/reachable-workspace.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/extra/reachable-workspace.hxx)

set(${PROJECT_NAME}_VISUALIZERS_SOURCES ${PROJECT_SOURCE_DIR}/src/visualizers/base-visualizer.cpp)

set(${PROJECT_NAME}_VISUALIZERS_PUBLIC_HEADERS
    ${PROJECT_SOURCE_DIR}/include/pinocchio/visualizers/base-visualizer.hpp)

set(_binary_headers_root ${${PROJECT_NAME}_BINARY_DIR}/include/pinocchio)
set(${PROJECT_NAME}_CORE_GENERATED_PUBLIC_HEADERS
    ${_binary_headers_root}/config.hpp ${_binary_headers_root}/deprecated.hpp
    ${_binary_headers_root}/warning.hpp)

set(${PROJECT_NAME}_TEMPLATE_INSTANTIATION_PUBLIC_HEADERS
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/aba-derivatives.txx
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/contact-cholesky.txx
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/contact-dynamics.txx
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/frames-derivatives.txx
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/regressor.txx
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/joint-configuration.txx
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/model.txx
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/proximal.txx
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/cholesky.txx
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/rnea.txx
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/jacobian.txx
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/geometry.txx
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/center-of-mass.txx
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/energy.txx
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/aba.txx
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/frames.txx
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/centroidal.txx
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/constrained-dynamics-derivatives.txx
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/contact-jacobian.txx
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/crba.txx
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/constrained-dynamics.txx
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/centroidal-derivatives.txx
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/rnea-derivatives.txx
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/impulse-dynamics.txx
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/compute-all-terms.txx
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/kinematics.txx
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/center-of-mass-derivatives.txx
    ${PROJECT_SOURCE_DIR}/include/pinocchio/algorithm/kinematics-derivatives.txx
    ${PROJECT_SOURCE_DIR}/include/pinocchio/multibody/model.txx
    ${PROJECT_SOURCE_DIR}/include/pinocchio/multibody/data.txx
    ${PROJECT_SOURCE_DIR}/include/pinocchio/multibody/sample-models.txx)

set(${PROJECT_NAME}_TEMPLATE_INSTANTIATION_SOURCES
    ${PROJECT_SOURCE_DIR}/src/algorithm/cholesky.cpp
    ${PROJECT_SOURCE_DIR}/src/algorithm/aba.cpp
    ${PROJECT_SOURCE_DIR}/src/algorithm/regressor.cpp
    ${PROJECT_SOURCE_DIR}/src/algorithm/contact-dynamics.cpp
    ${PROJECT_SOURCE_DIR}/src/algorithm/frames-derivatives.cpp
    ${PROJECT_SOURCE_DIR}/src/algorithm/impulse-dynamics.cpp
    ${PROJECT_SOURCE_DIR}/src/algorithm/model.cpp
    ${PROJECT_SOURCE_DIR}/src/algorithm/constrained-dynamics.cpp
    ${PROJECT_SOURCE_DIR}/src/algorithm/rnea-derivatives.cpp
    ${PROJECT_SOURCE_DIR}/src/algorithm/compute-all-terms.cpp
    ${PROJECT_SOURCE_DIR}/src/algorithm/jacobian.cpp
    ${PROJECT_SOURCE_DIR}/src/algorithm/energy.cpp
    ${PROJECT_SOURCE_DIR}/src/algorithm/centroidal-derivatives.cpp
    ${PROJECT_SOURCE_DIR}/src/algorithm/frames.cpp
    ${PROJECT_SOURCE_DIR}/src/algorithm/constrained-dynamics-derivatives.cpp
    ${PROJECT_SOURCE_DIR}/src/algorithm/center-of-mass.cpp
    ${PROJECT_SOURCE_DIR}/src/algorithm/geometry.cpp
    ${PROJECT_SOURCE_DIR}/src/algorithm/kinematics.cpp
    ${PROJECT_SOURCE_DIR}/src/algorithm/rnea.cpp
    ${PROJECT_SOURCE_DIR}/src/algorithm/centroidal.cpp
    ${PROJECT_SOURCE_DIR}/src/algorithm/aba-derivatives.cpp
    ${PROJECT_SOURCE_DIR}/src/algorithm/crba.cpp
    ${PROJECT_SOURCE_DIR}/src/algorithm/contact-cholesky.cpp
    ${PROJECT_SOURCE_DIR}/src/algorithm/joint-configuration.cpp
    ${PROJECT_SOURCE_DIR}/src/algorithm/center-of-mass-derivatives.cpp
    ${PROJECT_SOURCE_DIR}/src/algorithm/proximal.cpp
    ${PROJECT_SOURCE_DIR}/src/algorithm/kinematics-derivatives.cpp
    ${PROJECT_SOURCE_DIR}/src/algorithm/contact-jacobian.cpp
    ${PROJECT_SOURCE_DIR}/src/multibody/model.cpp
    ${PROJECT_SOURCE_DIR}/src/multibody/data.cpp
    ${PROJECT_SOURCE_DIR}/src/multibody/sample-models.cpp)

# Define Pinocchio Python binding sources and headers (Boost.Python)

set(${PROJECT_NAME}_BINDINGS_PYTHON_PUBLIC_HEADERS
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/spatial/explog.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/spatial/classic-acceleration.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/spatial/symmetric3.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/spatial/motion.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/spatial/force.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/spatial/inertia.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/spatial/se3.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/algorithm/contact-info.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/algorithm/contact-cholesky.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/algorithm/contact-solver-base.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/algorithm/delassus-operator.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/algorithm/proximal.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/algorithm/algorithms.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/algorithm/constraints/coulomb-friction-cone.hpp
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
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/utils/std-aligned-vector.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/utils/eigen.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/utils/comparable.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/utils/std-map.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/utils/cast.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/utils/deprecation.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/utils/model-checker.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/utils/namespace.hpp
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
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/math/multiprecision/boost/number.hpp
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
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/multibody/joint/joints-datas.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/multibody/joint/joint-data.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/multibody/liegroups.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/multibody/geometry-data.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/multibody/geometry-object.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/parsers/urdf.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/parsers/sdf.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/parsers/mjcf.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/parsers/srdf.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/extra/extras.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/visualizers/visualizer-visitor.hpp)

set(${PROJECT_NAME}_BINDINGS_PYTHON_SOURCES
    ${PROJECT_SOURCE_DIR}/bindings/python/spatial/expose-symmetric3.cpp
    ${PROJECT_SOURCE_DIR}/bindings/python/spatial/expose-force.cpp
    ${PROJECT_SOURCE_DIR}/bindings/python/spatial/expose-inertia.cpp
    ${PROJECT_SOURCE_DIR}/bindings/python/spatial/expose-SE3.cpp
    ${PROJECT_SOURCE_DIR}/bindings/python/spatial/expose-motion.cpp
    ${PROJECT_SOURCE_DIR}/bindings/python/spatial/expose-explog.cpp
    ${PROJECT_SOURCE_DIR}/bindings/python/spatial/expose-skew.cpp
    ${PROJECT_SOURCE_DIR}/bindings/python/algorithm/expose-impulse-dynamics.cpp
    ${PROJECT_SOURCE_DIR}/bindings/python/algorithm/expose-model.cpp
    ${PROJECT_SOURCE_DIR}/bindings/python/algorithm/expose-centroidal.cpp
    ${PROJECT_SOURCE_DIR}/bindings/python/algorithm/expose-aba.cpp
    ${PROJECT_SOURCE_DIR}/bindings/python/algorithm/expose-algorithms.cpp
    ${PROJECT_SOURCE_DIR}/bindings/python/algorithm/admm-solver.cpp
    ${PROJECT_SOURCE_DIR}/bindings/python/algorithm/pgs-solver.cpp
    ${PROJECT_SOURCE_DIR}/bindings/python/algorithm/expose-com.cpp
    ${PROJECT_SOURCE_DIR}/bindings/python/algorithm/expose-frames.cpp
    ${PROJECT_SOURCE_DIR}/bindings/python/algorithm/expose-energy.cpp
    ${PROJECT_SOURCE_DIR}/bindings/python/algorithm/expose-contact-dynamics.cpp
    ${PROJECT_SOURCE_DIR}/bindings/python/algorithm/expose-constrained-dynamics-derivatives.cpp
    ${PROJECT_SOURCE_DIR}/bindings/python/algorithm/expose-rnea-derivatives.cpp
    ${PROJECT_SOURCE_DIR}/bindings/python/algorithm/expose-kinematics.cpp
    ${PROJECT_SOURCE_DIR}/bindings/python/algorithm/expose-geometry.cpp
    ${PROJECT_SOURCE_DIR}/bindings/python/algorithm/expose-aba-derivatives.cpp
    ${PROJECT_SOURCE_DIR}/bindings/python/algorithm/expose-contact-solvers.cpp
    ${PROJECT_SOURCE_DIR}/bindings/python/algorithm/expose-contact-inverse-dynamics.cpp
    ${PROJECT_SOURCE_DIR}/bindings/python/algorithm/expose-joints.cpp
    ${PROJECT_SOURCE_DIR}/bindings/python/algorithm/expose-constrained-dynamics.cpp
    ${PROJECT_SOURCE_DIR}/bindings/python/algorithm/expose-rnea.cpp
    ${PROJECT_SOURCE_DIR}/bindings/python/algorithm/expose-contact-jacobian.cpp
    ${PROJECT_SOURCE_DIR}/bindings/python/algorithm/constraints/expose-cones.cpp
    ${PROJECT_SOURCE_DIR}/bindings/python/algorithm/expose-cholesky.cpp
    ${PROJECT_SOURCE_DIR}/bindings/python/algorithm/expose-regressor.cpp
    ${PROJECT_SOURCE_DIR}/bindings/python/algorithm/expose-kinematics-derivatives.cpp
    ${PROJECT_SOURCE_DIR}/bindings/python/algorithm/expose-cat.cpp
    ${PROJECT_SOURCE_DIR}/bindings/python/algorithm/expose-frames-derivatives.cpp
    ${PROJECT_SOURCE_DIR}/bindings/python/algorithm/expose-centroidal-derivatives.cpp
    ${PROJECT_SOURCE_DIR}/bindings/python/algorithm/expose-jacobian.cpp
    ${PROJECT_SOURCE_DIR}/bindings/python/algorithm/expose-delassus.cpp
    ${PROJECT_SOURCE_DIR}/bindings/python/algorithm/expose-impulse-dynamics-derivatives.cpp
    ${PROJECT_SOURCE_DIR}/bindings/python/algorithm/expose-kinematic-regressor.cpp
    ${PROJECT_SOURCE_DIR}/bindings/python/algorithm/expose-crba.cpp
    ${PROJECT_SOURCE_DIR}/bindings/python/module.cpp
    ${PROJECT_SOURCE_DIR}/bindings/python/utils/version.cpp
    ${PROJECT_SOURCE_DIR}/bindings/python/utils/dependencies.cpp
    ${PROJECT_SOURCE_DIR}/bindings/python/utils/conversions.cpp
    ${PROJECT_SOURCE_DIR}/bindings/python/utils/path.cpp
    ${PROJECT_SOURCE_DIR}/bindings/python/math/expose-linalg.cpp
    ${PROJECT_SOURCE_DIR}/bindings/python/math/expose-tridiagonal-matrix.cpp
    ${PROJECT_SOURCE_DIR}/bindings/python/math/expose-lanczos-decomposition.cpp
    ${PROJECT_SOURCE_DIR}/bindings/python/math/expose-rpy.cpp
    ${PROJECT_SOURCE_DIR}/bindings/python/math/expose-eigen-types.cpp
    ${PROJECT_SOURCE_DIR}/bindings/python/serialization/serialization.cpp
    ${PROJECT_SOURCE_DIR}/bindings/python/multibody/expose-model.cpp
    ${PROJECT_SOURCE_DIR}/bindings/python/multibody/expose-liegroups.cpp
    ${PROJECT_SOURCE_DIR}/bindings/python/multibody/expose-geometry.cpp
    ${PROJECT_SOURCE_DIR}/bindings/python/multibody/expose-frame.cpp
    ${PROJECT_SOURCE_DIR}/bindings/python/multibody/expose-data.cpp
    ${PROJECT_SOURCE_DIR}/bindings/python/multibody/joint/expose-joints.cpp
    ${PROJECT_SOURCE_DIR}/bindings/python/multibody/sample-models.cpp
    ${PROJECT_SOURCE_DIR}/bindings/python/parsers/sdf/model.cpp
    ${PROJECT_SOURCE_DIR}/bindings/python/parsers/sdf/geometry.cpp
    ${PROJECT_SOURCE_DIR}/bindings/python/parsers/expose-parsers.cpp
    ${PROJECT_SOURCE_DIR}/bindings/python/parsers/urdf/console-bridge.cpp
    ${PROJECT_SOURCE_DIR}/bindings/python/parsers/urdf/model.cpp
    ${PROJECT_SOURCE_DIR}/bindings/python/parsers/urdf/geometry.cpp
    ${PROJECT_SOURCE_DIR}/bindings/python/parsers/srdf.cpp
    ${PROJECT_SOURCE_DIR}/bindings/python/parsers/mjcf/model.cpp
    ${PROJECT_SOURCE_DIR}/bindings/python/parsers/mjcf/geometry.cpp
    ${PROJECT_SOURCE_DIR}/bindings/python/extra/expose-extras.cpp)

set(${PROJECT_NAME}_BINDINGS_PYTHON_HPP_FCL_SOURCES
    ${PROJECT_SOURCE_DIR}/bindings/python/collision/expose-broadphase.cpp
    ${PROJECT_SOURCE_DIR}/bindings/python/collision/expose-broadphase-callbacks.cpp
    ${PROJECT_SOURCE_DIR}/bindings/python/collision/expose-collision.cpp
    ${PROJECT_SOURCE_DIR}/bindings/python/collision/expose-fcl.cpp)

set(${PROJECT_NAME}_BINDINGS_PYTHON_HPP_FCL_PUBLIC_HEADERS
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/collision/fcl/transform.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/collision/broadphase-manager.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/collision/collision.hpp)

set(${PROJECT_NAME}_BINDINGS_PYTHON_PARALLEL_SOURCES
    ${PROJECT_SOURCE_DIR}/bindings/python/multibody/pool/expose-pool.cpp
    ${PROJECT_SOURCE_DIR}/bindings/python/algorithm/parallel/aba.cpp
    ${PROJECT_SOURCE_DIR}/bindings/python/algorithm/parallel/expose-parallel.cpp
    ${PROJECT_SOURCE_DIR}/bindings/python/algorithm/parallel/rnea.cpp)

set(${PROJECT_NAME}_BINDINGS_PYTHON_PARALLEL_PUBLIC_HEADERS
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/multibody/pool/model.hpp)

set(${PROJECT_NAME}_BINDINGS_PYTHON_HPP_FCL_PARALLEL_SOURCES
    ${PROJECT_SOURCE_DIR}/bindings/python/collision/parallel/expose-parallel.cpp
    ${PROJECT_SOURCE_DIR}/bindings/python/collision/parallel/geometry.cpp
    ${PROJECT_SOURCE_DIR}/bindings/python/collision/parallel/broadphase.cpp
    ${PROJECT_SOURCE_DIR}/bindings/python/collision/pool/expose-pool.cpp)

set(${PROJECT_NAME}_BINDINGS_PYTHON_HPP_FCL_PARALLEL_PUBLIC_HEADERS
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/collision/pool/geometry.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/collision/pool/broadphase-manager.hpp)

set(${PROJECT_NAME}_BINDINGS_PYTHON_EXTRA_SOURCES
    ${PROJECT_SOURCE_DIR}/bindings/python/extra/expose-reachable-workspace.cpp)

set(${PROJECT_NAME}_BINDINGS_PYTHON_EXTRA_MPFR_PUBLIC_HEADERS
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/math/multiprecision/boost/number.hpp)

set(${PROJECT_NAME}_BINDINGS_PYTHON_EXTRA_MPFR_SOURCES
    ${PROJECT_SOURCE_DIR}/bindings/python/extra/mpfr/boost_number.cpp)

# Define Pinocchio Python binding sources and headers (nanobind)

set(${PROJECT_NAME}_BINDINGS_PYTHON_NB_PUBLIC_HEADERS
    # ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/spatial/explog.hpp
    # ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/spatial/classic-acceleration.hpp
    # ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/spatial/symmetric3.hpp
    # ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/spatial/motion.hpp
    # ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/spatial/force.hpp
    # ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/spatial/inertia.hpp
    # ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/spatial/se3.hpp
    # ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/algorithm/contact-info.hpp
    # ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/algorithm/contact-cholesky.hpp
    # ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/algorithm/contact-solver-base.hpp
    # ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/algorithm/delassus-operator.hpp
    # ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/algorithm/proximal.hpp
    # ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/algorithm/algorithms.hpp
    # ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/algorithm/constraints/coulomb-friction-cone.hpp
    # ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/context.hpp
    # ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/pybind11-all.hpp
    # ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/pybind11.hpp
    # ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/utils/constant.hpp
    # ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/utils/version.hpp
    # ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/utils/pickle-vector.hpp
    # ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/utils/macros.hpp
    # ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/utils/path.hpp
    # ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/utils/std-vector.hpp
    # ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/utils/printable.hpp
    # ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/utils/dependencies.hpp
    # ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/utils/conversions.hpp
    # ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/utils/address.hpp
    # ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/utils/copyable.hpp
    # ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/utils/registration.hpp
    # ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/utils/pickle.hpp
    # ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/utils/pickle-map.hpp
    # ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/utils/list.hpp
    # ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/utils/std-aligned-vector.hpp
    # ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/utils/eigen.hpp
    # ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/utils/comparable.hpp
    # ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/utils/std-map.hpp
    # ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/utils/cast.hpp
    # ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/utils/deprecation.hpp
    # ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/utils/model-checker.hpp
    # ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/utils/namespace.hpp
    # ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/context/cppadcg.hpp
    # ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/context/cppad.hpp
    # ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/context/casadi.hpp
    # ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/context/default.hpp
    # ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/context/mpfr.hpp
    # ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/context/generic.hpp
    ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python_nb/fwd.hpp
    # ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/serialization/serialization.hpp
    # ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/serialization/serializable.hpp
    # ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/math/tridiagonal-matrix.hpp
    # ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/math/lanczos-decomposition.hpp
    # ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/math/multiprecision/boost/number.hpp
    # ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/collision/broadphase-manager-base.hpp
    # ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/collision/geometry-functors.hpp
    # ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/collision/tree-broadphase-manager.hpp
    # ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/multibody/data.hpp
    # ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/multibody/frame.hpp
    # ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/multibody/geometry-model.hpp
    # ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/multibody/joint/joint.hpp
    # ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/multibody/model.hpp
    # ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/multibody/joint/joint-model.hpp
    # ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/multibody/joint/joint-derived.hpp
    # ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/multibody/joint/joints-models.hpp
    # ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/multibody/joint/joints-variant.hpp
    # ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/multibody/joint/joints-datas.hpp
    # ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/multibody/joint/joint-data.hpp
    # ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/multibody/liegroups.hpp
    # ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/multibody/geometry-data.hpp
    # ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/multibody/geometry-object.hpp
    # ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/parsers/urdf.hpp
    # ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/parsers/sdf.hpp
    # ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/parsers/mjcf.hpp
    # ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/parsers/srdf.hpp
    # ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/extra/extras.hpp
    # ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/visualizers/visualizer-visitor.hpp
)

set(${PROJECT_NAME}_BINDINGS_PYTHON_NB_SOURCES
    # ${PROJECT_SOURCE_DIR}/bindings/python/spatial/expose-symmetric3.cpp
    # ${PROJECT_SOURCE_DIR}/bindings/python/spatial/expose-force.cpp
    # ${PROJECT_SOURCE_DIR}/bindings/python/spatial/expose-inertia.cpp
    # ${PROJECT_SOURCE_DIR}/bindings/python/spatial/expose-SE3.cpp
    # ${PROJECT_SOURCE_DIR}/bindings/python/spatial/expose-motion.cpp
    # ${PROJECT_SOURCE_DIR}/bindings/python/spatial/expose-explog.cpp
    # ${PROJECT_SOURCE_DIR}/bindings/python/spatial/expose-skew.cpp
    # ${PROJECT_SOURCE_DIR}/bindings/python/algorithm/expose-impulse-dynamics.cpp
    # ${PROJECT_SOURCE_DIR}/bindings/python/algorithm/expose-model.cpp
    # ${PROJECT_SOURCE_DIR}/bindings/python/algorithm/expose-centroidal.cpp
    # ${PROJECT_SOURCE_DIR}/bindings/python/algorithm/expose-aba.cpp
    # ${PROJECT_SOURCE_DIR}/bindings/python/algorithm/expose-algorithms.cpp
    # ${PROJECT_SOURCE_DIR}/bindings/python/algorithm/admm-solver.cpp
    # ${PROJECT_SOURCE_DIR}/bindings/python/algorithm/pgs-solver.cpp
    # ${PROJECT_SOURCE_DIR}/bindings/python/algorithm/expose-com.cpp
    # ${PROJECT_SOURCE_DIR}/bindings/python/algorithm/expose-frames.cpp
    # ${PROJECT_SOURCE_DIR}/bindings/python/algorithm/expose-energy.cpp
    # ${PROJECT_SOURCE_DIR}/bindings/python/algorithm/expose-contact-dynamics.cpp
    # ${PROJECT_SOURCE_DIR}/bindings/python/algorithm/expose-constrained-dynamics-derivatives.cpp
    # ${PROJECT_SOURCE_DIR}/bindings/python/algorithm/expose-rnea-derivatives.cpp
    # ${PROJECT_SOURCE_DIR}/bindings/python/algorithm/expose-kinematics.cpp
    # ${PROJECT_SOURCE_DIR}/bindings/python/algorithm/expose-geometry.cpp
    # ${PROJECT_SOURCE_DIR}/bindings/python/algorithm/expose-aba-derivatives.cpp
    # ${PROJECT_SOURCE_DIR}/bindings/python/algorithm/expose-contact-solvers.cpp
    # ${PROJECT_SOURCE_DIR}/bindings/python/algorithm/expose-contact-inverse-dynamics.cpp
    # ${PROJECT_SOURCE_DIR}/bindings/python/algorithm/expose-joints.cpp
    # ${PROJECT_SOURCE_DIR}/bindings/python/algorithm/expose-constrained-dynamics.cpp
    # ${PROJECT_SOURCE_DIR}/bindings/python/algorithm/expose-rnea.cpp
    # ${PROJECT_SOURCE_DIR}/bindings/python/algorithm/expose-contact-jacobian.cpp
    # ${PROJECT_SOURCE_DIR}/bindings/python/algorithm/constraints/expose-cones.cpp
    # ${PROJECT_SOURCE_DIR}/bindings/python/algorithm/expose-cholesky.cpp
    # ${PROJECT_SOURCE_DIR}/bindings/python/algorithm/expose-regressor.cpp
    # ${PROJECT_SOURCE_DIR}/bindings/python/algorithm/expose-kinematics-derivatives.cpp
    # ${PROJECT_SOURCE_DIR}/bindings/python/algorithm/expose-cat.cpp
    # ${PROJECT_SOURCE_DIR}/bindings/python/algorithm/expose-frames-derivatives.cpp
    # ${PROJECT_SOURCE_DIR}/bindings/python/algorithm/expose-centroidal-derivatives.cpp
    # ${PROJECT_SOURCE_DIR}/bindings/python/algorithm/expose-jacobian.cpp
    # ${PROJECT_SOURCE_DIR}/bindings/python/algorithm/expose-delassus.cpp
    # ${PROJECT_SOURCE_DIR}/bindings/python/algorithm/expose-impulse-dynamics-derivatives.cpp
    # ${PROJECT_SOURCE_DIR}/bindings/python/algorithm/expose-kinematic-regressor.cpp
    # ${PROJECT_SOURCE_DIR}/bindings/python/algorithm/expose-crba.cpp
    # ${PROJECT_SOURCE_DIR}/bindings/python/module.cpp
    ${PROJECT_SOURCE_DIR}/bindings/python_nb/utils/version.cpp
    # ${PROJECT_SOURCE_DIR}/bindings/python/utils/dependencies.cpp
    # ${PROJECT_SOURCE_DIR}/bindings/python/utils/conversions.cpp
    # ${PROJECT_SOURCE_DIR}/bindings/python/utils/path.cpp
    # ${PROJECT_SOURCE_DIR}/bindings/python/math/expose-linalg.cpp
    # ${PROJECT_SOURCE_DIR}/bindings/python/math/expose-tridiagonal-matrix.cpp
    # ${PROJECT_SOURCE_DIR}/bindings/python/math/expose-lanczos-decomposition.cpp
    # ${PROJECT_SOURCE_DIR}/bindings/python/math/expose-rpy.cpp
    # ${PROJECT_SOURCE_DIR}/bindings/python/math/expose-eigen-types.cpp
    # ${PROJECT_SOURCE_DIR}/bindings/python/serialization/serialization.cpp
    # ${PROJECT_SOURCE_DIR}/bindings/python/multibody/expose-model.cpp
    # ${PROJECT_SOURCE_DIR}/bindings/python/multibody/expose-liegroups.cpp
    # ${PROJECT_SOURCE_DIR}/bindings/python/multibody/expose-geometry.cpp
    # ${PROJECT_SOURCE_DIR}/bindings/python/multibody/expose-frame.cpp
    # ${PROJECT_SOURCE_DIR}/bindings/python/multibody/expose-data.cpp
    # ${PROJECT_SOURCE_DIR}/bindings/python/multibody/joint/expose-joints.cpp
    # ${PROJECT_SOURCE_DIR}/bindings/python/multibody/sample-models.cpp
    # ${PROJECT_SOURCE_DIR}/bindings/python/parsers/sdf/model.cpp
    # ${PROJECT_SOURCE_DIR}/bindings/python/parsers/sdf/geometry.cpp
    # ${PROJECT_SOURCE_DIR}/bindings/python/parsers/expose-parsers.cpp
    # ${PROJECT_SOURCE_DIR}/bindings/python/parsers/urdf/console-bridge.cpp
    # ${PROJECT_SOURCE_DIR}/bindings/python/parsers/urdf/model.cpp
    # ${PROJECT_SOURCE_DIR}/bindings/python/parsers/urdf/geometry.cpp
    # ${PROJECT_SOURCE_DIR}/bindings/python/parsers/srdf.cpp
    # ${PROJECT_SOURCE_DIR}/bindings/python/parsers/mjcf/model.cpp
    # ${PROJECT_SOURCE_DIR}/bindings/python/parsers/mjcf/geometry.cpp
    # ${PROJECT_SOURCE_DIR}/bindings/python/extra/expose-extras.cpp
)

set(${PROJECT_NAME}_BINDINGS_PYTHON_NB_HPP_FCL_SOURCES
    # ${PROJECT_SOURCE_DIR}/bindings/python/collision/expose-broadphase.cpp
    # ${PROJECT_SOURCE_DIR}/bindings/python/collision/expose-broadphase-callbacks.cpp
    # ${PROJECT_SOURCE_DIR}/bindings/python/collision/expose-collision.cpp
    # ${PROJECT_SOURCE_DIR}/bindings/python/collision/expose-fcl.cpp
)

set(${PROJECT_NAME}_BINDINGS_PYTHON_NB_HPP_FCL_PUBLIC_HEADERS
    # ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/collision/fcl/transform.hpp
    # ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/collision/broadphase-manager.hpp
    # ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/collision/collision.hpp
)

set(${PROJECT_NAME}_BINDINGS_PYTHON_NB_PARALLEL_SOURCES
    # ${PROJECT_SOURCE_DIR}/bindings/python/multibody/pool/expose-pool.cpp
    # ${PROJECT_SOURCE_DIR}/bindings/python/algorithm/parallel/aba.cpp
    # ${PROJECT_SOURCE_DIR}/bindings/python/algorithm/parallel/expose-parallel.cpp
    # ${PROJECT_SOURCE_DIR}/bindings/python/algorithm/parallel/rnea.cpp
)

set(${PROJECT_NAME}_BINDINGS_PYTHON_NB_PARALLEL_PUBLIC_HEADERS
    # ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/multibody/pool/model.hpp
)

set(${PROJECT_NAME}_BINDINGS_PYTHON_NB_HPP_FCL_PARALLEL_SOURCES
    # ${PROJECT_SOURCE_DIR}/bindings/python/collision/parallel/expose-parallel.cpp
    # ${PROJECT_SOURCE_DIR}/bindings/python/collision/parallel/geometry.cpp
    # ${PROJECT_SOURCE_DIR}/bindings/python/collision/parallel/broadphase.cpp
    # ${PROJECT_SOURCE_DIR}/bindings/python/collision/pool/expose-pool.cpp
)

set(${PROJECT_NAME}_BINDINGS_PYTHON_NB_HPP_FCL_PARALLEL_PUBLIC_HEADERS
    # ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/collision/pool/geometry.hpp
    # ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/collision/pool/broadphase-manager.hpp
)

set(${PROJECT_NAME}_BINDINGS_PYTHON_NB_EXTRA_SOURCES
    # ${PROJECT_SOURCE_DIR}/bindings/python/extra/expose-reachable-workspace.cpp
)

set(${PROJECT_NAME}_BINDINGS_PYTHON_NB_EXTRA_MPFR_PUBLIC_HEADERS
    # ${PROJECT_SOURCE_DIR}/include/pinocchio/bindings/python/math/multiprecision/boost/number.hpp
)

set(${PROJECT_NAME}_BINDINGS_PYTHON_NB_EXTRA_MPFR_SOURCES
    # ${PROJECT_SOURCE_DIR}/bindings/python/extra/mpfr/boost_number.cpp
)
