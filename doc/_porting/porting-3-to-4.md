# Pinocchio 3 to 4 {#md_doc__porting-3-to-4}

This section describes how to port your code from the Pinocchio 3 to Pinocchio 4.

## C++ changes

All Pinocchio 3 deprecated headers had been removed:
- Replace `pinocchio/math/casadi.hpp` by `pinocchio/autodiff/casadi.hpp`
- Replace `pinocchio/algorithm/parallel/geometry.hpp` by `pinocchio/collision/parallel/geometry.hpp`
- Replace `pinocchio/spatial/fcl-pinocchio-conversions.hpp` by `pinocchio/collision/fcl-pinocchio-conversions.hpp`
- Replace `pinocchio/parsers/sample-models.hpp` by `pinocchio/multibody/sample-models.hpp`
- Replace `pinocchio/bindings/python/parsers/python.hpp` by `pinocchio/parsers/python.hpp`
- Replace `pinocchio/math/cppad.hpp` by `pinocchio/autodiff/cppad.hpp`
- Replace `pinocchio/math/cppadcg.hpp` by `pinocchio/codegen/cppadcg.hpp`

All Pinocchio 3 deprecated functions had been removed:
- Replace `FrameTpl::parent` by `FrameTpl::parentJoint`
- Replace `FrameTpl::previousFrame` by `FrameTpl::parentFrame`
- Replace `GeometryObject` constructor argument order has changed

Remove use of `Eigen::aligned_allocator`:
  - Deprecate `PINOCCHIO_ALIGNED_STD_VECTOR` replaced by `std::vector`
  - Deprecate `PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR` replaced by `std::vector`
  - Deprecate `pinocchio::container::aligned_vector<T>` replaced by `std::vector<T, std::allocator<T>>`
  - Deprecate `pinocchio/container/aligned-vector.hpp`
  - Deprecate `pinocchio::python::StdAlignedVectorPythonVisitor`

Eigen 3.4 is now the required minimum version of Eigen:
  - Remove PINOCCHIO_WITH_EIGEN_TENSOR_MODULE define

HPP-FCL has been replaced by coal:
- Deprecate `pinocchio/multibody/fcl.hpp` moved to `pinocchio/multibody/coal.hpp`
- Deprecate `pinocchio/serialization/fcl.hpp` moved to `pinocchio/serialization/coal.hpp`
- Deprecate `pinocchio/collision/fcl-pinocchio-conversions.hpp` moved to `pinocchio/collision/coal-pinocchio-conversions.hpp`
- Deprecate `pinocchio/bindings/python/collision/fcl/transform.hpp` moved at `pinocchio/bindings/python/collision/coal/transform.hpp`
- Deprecate `pinocchio::toFclTransform3f` replaced by `pinocchio::toCoalTransform3s`
- Deprecate `PINOCCHIO_WITH_HPP_FCL` replaced by `PINOCCHIO_WITH_COLLISION`

Constraints API changes:
  - Add `std::vector<ConstraintData, ConstraintDataAllocator> contact_datas` in `initConstraintDynamics` method
  - Change `BaumgarteCorrectorParametersTpl` constructor: scalars are used for the gains instead of vectors
  - `constraint_data` should be computed by `constaint_model.calc` call before calling `getConstraintJacobian`

`RigidConstraintModel` internal API has changed:
  - Remove `colwise_joint1_sparsity`
  - Remove `colwise_joint2_sparsity`
  - Remove `joint1_span_indexes`
  - Remove `joint2_span_indexes`

- Deprecate `ContactCholeskyDecompositionTpl`, replaced by  `ConstraintCholeskyDecompositionTpl`
- Deprecate `DelassusCholeskyExpressionTpl`, replaced by  `DelassusOperatorCholeskyExpressionTpl`

ConstraintCholeskyDecompositionTpl changes :
- Deprecate `ConstraintCholeskyDecompositionTpl::allocate` replaced by `ConstraintCholeskyDecompositionTpl::rebuild`
  - Add `std::vector<ConstraintModel, ConstraintModelAllocator>`
  - Add `std::vector<ConstraintData, ConstraintDataAllocator>`
  - `constraint_datas` should be computed by the associated model
- ConstraintCholeskyDecompositionTpl constructor :
  - Add `DataTpl<S1, O1, JointCollectionTpl>`
  - Add `std::vector<ConstraintData, ConstraintDataAllocator>`
- `ConstraintCholeskyDecompositionTpl::compute`:
  - `constraint_datas` should be computed by the associated model

Utility API changes:
- Remove `gettimeofday` definition on Windows
- Remove `operator-(timeval, timeval)` definition
- Remove `deprecated-macros.hpp` and `deprecated-namespaces.hpp`
- Remove `pinocchio/deprecation.hpp` replaced by `pinocchio/deprecated.hpp`

DataTpl API changes:
- Deprecate `DataTpl::lastChild` replaced by `ModelTpl::children`:
  - If joint `index` doesn't have any child it can be checked with `model.children[index].empty()`
  - In other case `data.lastChild[index]` can be replaced by `model.children[index].back()`

Python bindings API changes:
- Deprecate `pinocchio/bindings/python/multibody/joint/joint.hpp` replaced by `pinocchio/bindings/python/multibody/joint/joint-model.hpp`
- Deprecate Python bindings headers already implemented in eigenpy:
  - `pinocchio/bindings/python/utils/registration.hpp` replaced by `eigenpy/registration.hpp`
  - `pinocchio/bindings/python/utils/copyable.hpp` replaced by `eigenpy/copyable.hpp`
  - `pinocchio/bindings/python/utils/deprecation.hpp` replaced by `eigenpy/deprecation-policy.hpp`

## Python changes

HPP-FCL has been replaced by coal:
- Deprecate `pinocchio.WITH_HPP_FCL` and `pinocchio.WITH_HPP_FCL_BINDINGS` replaced by `pinocchio.WITH_COLLISION`
- Deprecate `pinocchio.hppfcl` replaced by `pinocchio.coal`
- Deprecate `buildModelFromMJCF(filename, root_joint, root_joint_name)` replaced by `buildModelFromMJCFAndRootJoint` and `buildModelAndLegacyConstraintsFromMJCF`
- Deprecate `buildModelFromSdf` replaced by `buildModelAndLegacyConstraintsFromSdf`

Constraints API changes:
  - Add `contact_datas` in `initConstraintDynamics` method
  - Change `BaumgarteCorrectorParameters` constructor scalar are used instead of vectors

- Deprecate `ContactCholeskyDecomposition`, replaced by  `ConstraintCholeskyDecomposition`
- Deprecate `DelassusCholeskyExpressionTpl`, replaced by  `DelassusOperatorCholeskyExpressionTpl`
