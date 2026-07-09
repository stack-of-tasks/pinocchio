//
// Copyright (c) 2024-2025 INRIA
//

#define BOOST_TEST_MODULE constraint_jacobian

#include "pinocchio/constraints.hpp"
#include "pinocchio/multibody/sample-models.hpp"

#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"

#include <boost/utility/binary.hpp>

#include <boost/test/unit_test.hpp>

using namespace pinocchio;
using namespace Eigen;

BOOST_AUTO_TEST_CASE(constraint_jacobian_operations)
{

  pinocchio::Model model;
  pinocchio::buildModels::humanoidRandom(model, true);
  Data data(model), data_ref(model);

  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill(1.);
  VectorXd q = randomConfiguration(model);
  computeJointJacobians(model, data, q);
  computeJointJacobians(model, data_ref, q);

  const std::string RF = "rleg6_joint";
  const std::string LF = "lleg6_joint";

  // 3D - LOCAL
  {
    PointAnchorConstraintModel cm_RF_LOCAL(model, model.getJointId(RF), SE3::Random());
    PointAnchorConstraintData cd_RF_LOCAL(cm_RF_LOCAL);
    PointAnchorConstraintModel cm_LF_LOCAL(model, model.getJointId(LF), SE3::Random());
    PointAnchorConstraintData cd_LF_LOCAL(cm_LF_LOCAL);

    const std::vector<PointAnchorConstraintModel> constraints_models{cm_RF_LOCAL, cm_LF_LOCAL};
    std::vector<PointAnchorConstraintData> constraints_datas{cd_RF_LOCAL, cd_LF_LOCAL};
    std::vector<PointAnchorConstraintData> constraints_datas_ref{cd_RF_LOCAL, cd_LF_LOCAL};

    const Eigen::Index m = getTotalConstraintResidualSize(constraints_models);

    Eigen::VectorXd res(model.nv);
    const Eigen::VectorXd rhs = Eigen::VectorXd::Random(m);

    calc(model, data, constraints_models, constraints_datas);
    evalConstraintJacobianTransposeMatrixProduct(
      model, data, constraints_models, constraints_datas, rhs, res);

    // Check Jacobian
    {
      Eigen::VectorXd res_ref = Eigen::VectorXd::Zero(model.nv);
      Data::MatrixXs J_RF_LOCAL_sparse(3, model.nv);
      J_RF_LOCAL_sparse.setZero(); // TODO: change input type when all the API would be refactorized
                                   // with CRTP on contact constraints
      cm_RF_LOCAL.calc(model, data, cd_RF_LOCAL);
      getConstraintJacobian(model, data, cm_RF_LOCAL, cd_RF_LOCAL, J_RF_LOCAL_sparse);
      res_ref += J_RF_LOCAL_sparse.transpose() * rhs.segment<3>(0);

      Data::MatrixXs J_LF_LOCAL_sparse(3, model.nv);
      J_LF_LOCAL_sparse.setZero(); // TODO: change input type when all the API would be refactorized
                                   // with CRTP on contact constraints
      cm_LF_LOCAL.calc(model, data, cd_LF_LOCAL);
      getConstraintJacobian(model, data, cm_LF_LOCAL, cd_LF_LOCAL, J_LF_LOCAL_sparse);
      res_ref += J_LF_LOCAL_sparse.transpose() * rhs.segment<3>(3);

      BOOST_CHECK(res.isApprox(res_ref));
    }

    // Alternative way to compute the Jacobians
    {
      Eigen::MatrixXd J_ref(6, model.nv);
      J_ref.setZero();
      calc(model, data_ref, constraints_models, constraints_datas_ref);
      getConstraintsJacobian(model, data_ref, constraints_models, constraints_datas_ref, J_ref);
      const Eigen::VectorXd res_ref = J_ref.transpose() * rhs;
      BOOST_CHECK(res.isApprox(res_ref));
    }

    // Check that getConstraintJacobian works with Matrix3Xs
    {
      using Matrix3Xs = Eigen::Matrix<Data::Scalar, 3, Eigen::Dynamic, Data::Options>;
      Matrix3Xs J_RF_LOCAL_sparse_3xs(3, model.nv);
      J_RF_LOCAL_sparse_3xs.setZero();
      cm_RF_LOCAL.calc(model, data, cd_RF_LOCAL);
      getConstraintJacobian(model, data, cm_RF_LOCAL, cd_RF_LOCAL, J_RF_LOCAL_sparse_3xs);

      Data::MatrixXs J_RF_LOCAL_sparse_xs(3, model.nv);
      J_RF_LOCAL_sparse_xs.setZero();
      cm_RF_LOCAL.calc(model, data, cd_RF_LOCAL);
      getConstraintJacobian(model, data, cm_RF_LOCAL, cd_RF_LOCAL, J_RF_LOCAL_sparse_xs);

      BOOST_CHECK(J_RF_LOCAL_sparse_3xs.isApprox(J_RF_LOCAL_sparse_xs));
    }

    // Check that getConstraintJacobian works with Matrix6Xs
    {
      using Matrix6Xs = Eigen::Matrix<Data::Scalar, 6, Eigen::Dynamic, Data::Options>;
      Matrix6Xs J_RF_LOCAL_sparse_6xs(6, model.nv);
      J_RF_LOCAL_sparse_6xs.setZero();
      cm_RF_LOCAL.calc(model, data, cd_RF_LOCAL);
      getConstraintJacobian(
        model, data, cm_RF_LOCAL, cd_RF_LOCAL, J_RF_LOCAL_sparse_6xs.topRows(3));

      Data::MatrixXs J_RF_LOCAL_sparse_xs(6, model.nv);
      J_RF_LOCAL_sparse_xs.setZero();
      cm_RF_LOCAL.calc(model, data, cd_RF_LOCAL);
      getConstraintJacobian(model, data, cm_RF_LOCAL, cd_RF_LOCAL, J_RF_LOCAL_sparse_xs.topRows(3));

      BOOST_CHECK(J_RF_LOCAL_sparse_6xs.isApprox(J_RF_LOCAL_sparse_xs));
    }
  }
}
