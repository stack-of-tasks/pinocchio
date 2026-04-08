// Copyright (c) 2026 INRIA

#include "pinocchio/bindings/python-nb/multibody/liegroups.hpp"

#include <nanobind/eigen/dense.h>

PINOCCHIO_PYTHON_NAMESPACE_BEGIN
namespace nb = nanobind;

template<typename LgType>
auto makeLieGroup()
{
  return CartesianProductOperationVariantTpl<Scalar, Options, LieGroupCollectionDefaultTpl>(
    LgType());
}

auto makeRn(int n)
{
  return CartesianProductOperationVariantTpl<Scalar, Options, LieGroupCollectionDefaultTpl>(
    VectorSpaceOperationTpl<Eigen::Dynamic, Scalar, Options>(n));
}

void exposeLieGroups(nb::module_ m)
{
  // Generic Lie group = Cartesian product
  using LieGroup =
    CartesianProductOperationVariantTpl<Scalar, Options, LieGroupCollectionDefaultTpl>;

  exposeLieGroup<LieGroup>(m, "LieGroup");

  nb::module_ liegroups = m.def_submodule("liegroups");

  liegroups.def(
    "R1", makeLieGroup<VectorSpaceOperationTpl<1, Scalar, Options>>, "Return the R^1 Lie group.");
  liegroups.def(
    "R2", makeLieGroup<VectorSpaceOperationTpl<2, Scalar, Options>>, "Return the R^2 Lie group.");
  liegroups.def(
    "R3", makeLieGroup<VectorSpaceOperationTpl<3, Scalar, Options>>, "Return the R^3 Lie group.");
  liegroups.def("Rn", makeRn, "n"_a, "Return the R^n Lie group.");
  liegroups.def(
    "SO2", makeLieGroup<SpecialOrthogonalOperationTpl<2, Scalar, Options>>,
    "Return the SO(2) Lie group.");
  liegroups.def(
    "SO3", makeLieGroup<SpecialOrthogonalOperationTpl<3, Scalar, Options>>,
    "Return the SO(3) Lie group.");
  liegroups.def(
    "SE2", makeLieGroup<SpecialEuclideanOperationTpl<2, Scalar, Options>>,
    "Return the SE(2) Lie group.");
  liegroups.def(
    "SE3", makeLieGroup<SpecialEuclideanOperationTpl<3, Scalar, Options>>,
    "Return the SE(3) Lie group.");
}

PINOCCHIO_PYTHON_NAMESPACE_END
