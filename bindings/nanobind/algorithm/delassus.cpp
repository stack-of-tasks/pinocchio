// Copyright (c) 2026 INRIA

#include "pinocchio/bindings/python-nb/fwd.hpp"

#include "pinocchio/algorithm/delassus.hpp"

PINOCCHIO_PYTHON_NAMESPACE_BEGIN

void exposeDelassus(nb::module_ m)
{
  using namespace nb::literals;

  m.def(
    "computeDelassusMatrix",
    [](
      const Model & model, Data & data, const VectorXs & q,
      const RigidConstraintModelVector & contact_models, RigidConstraintDataVector & contact_datas,
      const Scalar mu) -> MatrixXs {
      const Eigen::Index constraint_size = getTotalConstraintResidualSize(contact_models);
      MatrixXs delassus(constraint_size, constraint_size);
      computeDelassusMatrix(model, data, q, contact_models, contact_datas, delassus, mu);
      make_symmetric(delassus);
      return delassus;
    },
    "model"_a, "data"_a, "q"_a, "contact_models"_a, "contact_datas"_a, "mu"_a = 0.0,
    "Computes the Delassus matrix associated to a set of given constraints.");

  m.def(
    "computeDampedDelassusMatrixInverse",
    [](
      const Model & model, Data & data, const VectorXs & q,
      const RigidConstraintModelVector & contact_models, RigidConstraintDataVector & contact_datas,
      const Scalar mu, const bool scaled) -> MatrixXs {
      const Eigen::Index constraint_size = getTotalConstraintResidualSize(contact_models);
      MatrixXs delassus_inverse(constraint_size, constraint_size);
      computeDampedDelassusMatrixInverse(
        model, data, q, contact_models, contact_datas, delassus_inverse, mu, scaled);
      make_symmetric(delassus_inverse);
      return delassus_inverse;
    },
    "model"_a, "data"_a, "q"_a, "contact_models"_a, "contact_datas"_a, "mu"_a = 0.0,
    "scaled"_a = false,
    "Computes the inverse of the Delassus matrix associated to a set of given constraints.");
}

PINOCCHIO_PYTHON_NAMESPACE_END
