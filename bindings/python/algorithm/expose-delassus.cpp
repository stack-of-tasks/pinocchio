//
// Copyright (c) 2020-2021 INRIA
//

#include "pinocchio/bindings/python/algorithm/algorithms.hpp"

#include "pinocchio/bindings/python/utils/eigen.hpp"
#include "pinocchio/bindings/python/utils/std-vector.hpp"
#include "pinocchio/bindings/python/utils/model-checker.hpp"

#include "pinocchio/algorithm/delassus.hpp"

namespace bp = boost::python;

namespace pinocchio
{
  namespace python
  {
    typedef PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(context::RigidConstraintModel)
      RigidConstraintModelVector;
    typedef PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(context::RigidConstraintData)
      RigidConstraintDataVector;

    static const context::MatrixXs computeDelassusMatrix_proxy(
      const context::Model & model,
      context::Data & data,
      const context::VectorXs & q,
      const RigidConstraintModelVector & contact_models,
      RigidConstraintDataVector & contact_datas,
      const context::Scalar mu = context::Scalar(0))
    {
      const size_t constraint_size = getTotalConstraintSize(contact_models);
      context::MatrixXs delassus(constraint_size, constraint_size);
      computeDelassusMatrix(model, data, q, contact_models, contact_datas, delassus, mu);
      make_symmetric(delassus);
      return delassus;
    }

    static const context::MatrixXs computeDampedDelassusMatrixInverse_proxy(
      const context::Model & model,
      context::Data & data,
      const context::VectorXs & q,
      const RigidConstraintModelVector & contact_models,
      RigidConstraintDataVector & contact_datas,
      const context::Scalar mu,
      const bool scaled = false)
    {
      const size_t constraint_size = getTotalConstraintSize(contact_models);
      context::MatrixXs delassus_inverse(constraint_size, constraint_size);
      computeDampedDelassusMatrixInverse(
        model, data, q, contact_models, contact_datas, delassus_inverse, mu, scaled);
      make_symmetric(delassus_inverse);
      return delassus_inverse;
    }

    void exposeDelassus()
    {
      using namespace Eigen;

      bp::def(
        "computeDelassusMatrix", computeDelassusMatrix_proxy,
        (bp::arg("model"), bp::arg("data"), bp::arg("q"), bp::arg("contact_models"),
         bp::arg("contact_datas"), bp::arg("mu") = 0),
        "Computes the Delassus matrix associated to a set of given constraints.",
        mimic_not_supported_function<>(0));

      bp::def(
        "computeDampedDelassusMatrixInverse", computeDampedDelassusMatrixInverse_proxy,
        (bp::arg("model"), bp::arg("data"), bp::arg("q"), bp::arg("contact_models"),
         bp::arg("contact_datas"), bp::arg("mu") = 0),
        "Computes the inverse of the Delassus matrix associated to a set of given constraints.",
        mimic_not_supported_function<>(0));
    }
  } // namespace python
} // namespace pinocchio
