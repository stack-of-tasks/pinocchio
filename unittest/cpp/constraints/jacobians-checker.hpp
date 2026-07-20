#pragma once

#include "pinocchio/multibody.hpp"

#include "pinocchio/constraints.hpp"

#include <boost/test/unit_test.hpp>

namespace pinocchio
{

  template<typename ConstraintModelDerived, typename ConstraintDataDerived>
  void check_jacobians_operations(
    const Model & model,
    const Data & data,
    const ConstraintModelBase<ConstraintModelDerived> & cmodel,
    ConstraintDataBase<ConstraintDataDerived> & cdata)
  {
    Data::MatrixXs J_ref = Data::MatrixXs::Zero(cmodel.residualSize(), model.nv);
    getConstraintJacobian(model, data, cmodel, cdata, J_ref);

    // Check Jacobian matrix product
#ifdef NDEBUG
    const int num_tests = int(1e4);
#else
    const int num_tests = int(1e2);
#endif

    const Eigen::Index m = 40;
    for (int k = 0; k < num_tests; ++k)
    {
      const Data::MatrixXs mat = Data::MatrixXs::Random(model.nv, m);
      Data::MatrixXs res(cmodel.residualSize(), m);

      const Data::MatrixXs mat_transpose = Data::MatrixXs::Random(cmodel.residualSize(), m);
      Data::MatrixXs res_transpose(model.nv, m);

      // Set to
      cmodel.jacobianMatrixProduct(model, data, cdata.derived(), mat, res);
      Data::MatrixXs res_ref = J_ref * mat;
      BOOST_CHECK(res.isApprox(res_ref));

      cmodel.jacobianTransposeMatrixProduct(
        model, data, cdata.derived(), mat_transpose, res_transpose);
      Data::MatrixXs res_transpose_ref = J_ref.transpose() * mat_transpose;
      BOOST_CHECK(res_transpose.isApprox(res_transpose_ref));

      // Add to
      res = res_ref.setRandom();
      cmodel.jacobianMatrixProduct(model, data, cdata.derived(), mat, res, AddTo());
      res_ref += J_ref * mat;
      BOOST_CHECK(res.isApprox(res_ref));

      res_transpose = res_transpose_ref.setRandom();
      cmodel.jacobianTransposeMatrixProduct(
        model, data, cdata.derived(), mat_transpose, res_transpose, AddTo());
      res_transpose_ref += J_ref.transpose() * mat_transpose;
      BOOST_CHECK(res_transpose.isApprox(res_transpose_ref));

      // Remove to
      res = res_ref.setRandom();
      cmodel.jacobianMatrixProduct(model, data, cdata.derived(), mat, res, RmTo());
      res_ref -= J_ref * mat;
      BOOST_CHECK(res.isApprox(res_ref));

      res_transpose = res_transpose_ref.setRandom();
      cmodel.jacobianTransposeMatrixProduct(
        model, data, cdata.derived(), mat_transpose, res_transpose, RmTo());
      res_transpose_ref -= J_ref.transpose() * mat_transpose;
      BOOST_CHECK(res_transpose.isApprox(res_transpose_ref));
    }

    {
      const auto identity = Eigen::MatrixXd::Identity(model.nv, model.nv);
      BOOST_CHECK(
        cmodel.jacobianMatrixProduct(model, data, cdata.derived(), identity).isApprox(J_ref));
    }
  }
} // namespace pinocchio
