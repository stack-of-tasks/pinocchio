//
// Copyright (c) 2019-2021 INRIA
//

#include "pinocchio/algorithm/aba.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/centroidal.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/contact-info.hpp"
#include "pinocchio/algorithm/contact-jacobian.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/multibody/sample-models.hpp"
#include "pinocchio/utils/timer.hpp"
#include "pinocchio/spatial/classic-acceleration.hpp"

#include <iostream>

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

using namespace pinocchio;
using namespace Eigen;

template<typename T>
bool within(const T & elt, const std::vector<T> & vec)
{
  typename std::vector<T>::const_iterator it;

  it = std::find(vec.begin(), vec.end(), elt);
  if (it != vec.end())
    return true;
  else
    return false;
}

template<typename Matrix>
bool within(const typename Matrix::Scalar & elt, const Eigen::MatrixBase<Matrix> & mat)
{
  for (DenseIndex i = 0; i < mat.rows(); ++i)
    for (DenseIndex j = 0; j < mat.rows(); ++j)
    {
      if (elt == mat(i, j))
        return true;
    }

  return false;
}

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

BOOST_AUTO_TEST_CASE(contact_models)
{
  pinocchio::Model model;
  pinocchio::buildModels::humanoidRandom(model, true);

  // Check complete constructor
  const SE3 M(SE3::Random());
  RigidConstraintModel cmodel2(CONTACT_3D, model, 0, M);
  BOOST_CHECK(cmodel2.type == CONTACT_3D);
  BOOST_CHECK(cmodel2.joint1_id == 0);
  BOOST_CHECK(cmodel2.joint1_placement.isApprox(M));
  BOOST_CHECK(cmodel2.size() == 3);

  // Check contructor with two arguments
  RigidConstraintModel cmodel2prime(CONTACT_3D, model, 0);
  BOOST_CHECK(cmodel2prime.type == CONTACT_3D);
  BOOST_CHECK(cmodel2prime.joint1_id == 0);
  BOOST_CHECK(cmodel2prime.joint1_placement.isIdentity());
  BOOST_CHECK(cmodel2prime.size() == 3);

  // Check default copy constructor
  RigidConstraintModel cmodel3(cmodel2);
  BOOST_CHECK(cmodel3 == cmodel2);

  // Check complete constructor 6D
  RigidConstraintModel cmodel4(CONTACT_6D, model, 0);
  BOOST_CHECK(cmodel4.type == CONTACT_6D);
  BOOST_CHECK(cmodel4.joint1_id == 0);
  BOOST_CHECK(cmodel4.joint1_placement.isIdentity());
  BOOST_CHECK(cmodel4.size() == 6);
}

void check_A1_and_A2(
  const Model & model,
  const Data & data,
  const RigidConstraintModel & cmodel,
  RigidConstraintData & cdata)
{
  const RigidConstraintModel::Matrix36 A1 = cmodel.getA1(cdata);
  const RigidConstraintModel::Matrix36 A1_ref = cdata.oMc1.toActionMatrixInverse().topRows<3>();

  BOOST_CHECK(A1.isApprox(A1_ref));

  const RigidConstraintModel::Matrix36 A2 = cmodel.getA2(cdata);
  const RigidConstraintModel::Matrix36 A2_ref =
    -cdata.c1Mc2.rotation() * cdata.oMc2.toActionMatrixInverse().topRows<3>();

  BOOST_CHECK(A2.isApprox(A2_ref));

  // Check Jacobian
  Data::MatrixXs J_ref(3, model.nv);
  J_ref.setZero();
  getConstraintJacobian(model, data, cmodel, cdata, J_ref);
  const Data::Matrix6x J1 = getJointJacobian(model, data, cmodel.joint1_id, WORLD);
  const Data::Matrix6x J2 = getJointJacobian(model, data, cmodel.joint2_id, WORLD);
  const Data::Matrix3x J = A1 * J1 + A2 * J2;

  BOOST_CHECK(J.isApprox(J_ref));

  // Check Jacobian matrix product
  const Eigen::DenseIndex m = 40;
  const Data::MatrixXs mat = Data::MatrixXs::Random(model.nv, m);

  Data::MatrixXs res(cmodel.size(), m);
  res.setZero();
  cmodel.jacobian_matrix_product(model, data, cdata, mat, res);

  const Data::MatrixXs res_ref = J_ref * mat;

  BOOST_CHECK(res.isApprox(res_ref));
}

BOOST_AUTO_TEST_CASE(contact_models_sparsity_and_jacobians)
{

  pinocchio::Model model;
  pinocchio::buildModels::humanoidRandom(model, true);
  Data data(model);

  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill(1.);
  VectorXd q = randomConfiguration(model);
  computeJointJacobians(model, data, q);

  const std::string RF = "rleg6_joint";
  const std::string LF = "lleg6_joint";

  // 6D - LOCAL
  {
    RigidConstraintModel cm_RF_LOCAL(CONTACT_6D, model, model.getJointId(RF), SE3::Random(), LOCAL);
    RigidConstraintData cd_RF_LOCAL(cm_RF_LOCAL);
    RigidConstraintModel cm_LF_LOCAL(CONTACT_6D, model, model.getJointId(LF), SE3::Random(), LOCAL);
    RigidConstraintData cd_LF_LOCAL(cm_LF_LOCAL);
    RigidConstraintModel clm_RF_LF_LOCAL(
      CONTACT_6D, model, cm_RF_LOCAL.joint1_id, cm_RF_LOCAL.joint1_placement, cm_LF_LOCAL.joint1_id,
      cm_LF_LOCAL.joint1_placement, LOCAL);
    RigidConstraintData cld_RF_LF_LOCAL(clm_RF_LF_LOCAL);

    Data::Matrix6x J_RF_LOCAL(6, model.nv);
    J_RF_LOCAL.setZero();
    getFrameJacobian(
      model, data, cm_RF_LOCAL.joint1_id, cm_RF_LOCAL.joint1_placement, cm_RF_LOCAL.reference_frame,
      J_RF_LOCAL);
    Data::Matrix6x J_LF_LOCAL(6, model.nv);
    J_LF_LOCAL.setZero();
    getFrameJacobian(
      model, data, cm_LF_LOCAL.joint1_id, cm_LF_LOCAL.joint1_placement, cm_LF_LOCAL.reference_frame,
      J_LF_LOCAL);

    for (DenseIndex k = 0; k < model.nv; ++k)
    {
      BOOST_CHECK(J_RF_LOCAL.col(k).isZero() != cm_RF_LOCAL.colwise_joint1_sparsity[k]);
      BOOST_CHECK(J_LF_LOCAL.col(k).isZero() != cm_LF_LOCAL.colwise_joint1_sparsity[k]);
    }
    BOOST_CHECK(cm_RF_LOCAL.colwise_joint2_sparsity.isZero());
    BOOST_CHECK(cm_LF_LOCAL.colwise_joint2_sparsity.isZero());

    const SE3 oMc1 = data.oMi[clm_RF_LF_LOCAL.joint1_id] * clm_RF_LF_LOCAL.joint1_placement;
    const SE3 oMc2 = data.oMi[clm_RF_LF_LOCAL.joint2_id] * clm_RF_LF_LOCAL.joint2_placement;
    const SE3 c1Mc2 = oMc1.actInv(oMc2);
    const Data::Matrix6x J_clm_LOCAL = J_RF_LOCAL - c1Mc2.toActionMatrix() * J_LF_LOCAL;

    for (DenseIndex k = 0; k < model.nv; ++k)
    {
      BOOST_CHECK(J_clm_LOCAL.col(k).isZero() != within(k, clm_RF_LF_LOCAL.colwise_span_indexes));
    }

    // Check Jacobian
    Data::MatrixXs J_RF_LOCAL_sparse(6, model.nv);
    J_RF_LOCAL_sparse.setZero(); // TODO: change input type when all the API would be refactorized
                                 // with CRTP on contact constraints
    getConstraintJacobian(model, data, cm_RF_LOCAL, cd_RF_LOCAL, J_RF_LOCAL_sparse);
    BOOST_CHECK(J_RF_LOCAL.isApprox(J_RF_LOCAL_sparse));

    Data::MatrixXs J_LF_LOCAL_sparse(6, model.nv);
    J_LF_LOCAL_sparse.setZero(); // TODO: change input type when all the API would be refactorized
                                 // with CRTP on contact constraints
    getConstraintJacobian(model, data, cm_LF_LOCAL, cd_LF_LOCAL, J_LF_LOCAL_sparse);
    BOOST_CHECK(J_LF_LOCAL.isApprox(J_LF_LOCAL_sparse));

    Data::MatrixXs J_clm_LOCAL_sparse(6, model.nv);
    J_clm_LOCAL_sparse.setZero(); // TODO: change input type when all the API would be refactorized
                                  // with CRTP on contact constraints
    getConstraintJacobian(model, data, clm_RF_LF_LOCAL, cld_RF_LF_LOCAL, J_clm_LOCAL_sparse);
    BOOST_CHECK(J_clm_LOCAL.isApprox(J_clm_LOCAL_sparse));
  }

  // 6D - LOCAL_WORLD_ALIGNED
  {
    RigidConstraintModel cm_RF_LWA(
      CONTACT_6D, model, model.getJointId(RF), SE3::Random(), LOCAL_WORLD_ALIGNED);
    RigidConstraintData cd_RF_LWA(cm_RF_LWA);
    RigidConstraintModel cm_LF_LWA(
      CONTACT_6D, model, model.getJointId(LF), SE3::Random(), LOCAL_WORLD_ALIGNED);
    RigidConstraintData cd_LF_LWA(cm_LF_LWA);
    RigidConstraintModel clm_RF_LF_LWA(
      CONTACT_6D, model, cm_RF_LWA.joint1_id, cm_RF_LWA.joint1_placement, cm_LF_LWA.joint1_id,
      cm_LF_LWA.joint1_placement, LOCAL_WORLD_ALIGNED);
    RigidConstraintData cld_RF_LF_LWA(clm_RF_LF_LWA);

    Data::Matrix6x J_RF_LOCAL(6, model.nv);
    J_RF_LOCAL.setZero();
    getFrameJacobian(
      model, data, cm_RF_LWA.joint1_id, cm_RF_LWA.joint1_placement, LOCAL, J_RF_LOCAL);
    Data::Matrix6x J_LF_LOCAL(6, model.nv);
    J_LF_LOCAL.setZero();
    getFrameJacobian(
      model, data, cm_LF_LWA.joint1_id, cm_LF_LWA.joint1_placement, LOCAL, J_LF_LOCAL);

    Data::Matrix6x J_RF_LWA(6, model.nv);
    J_RF_LWA.setZero();
    getFrameJacobian(
      model, data, cm_RF_LWA.joint1_id, cm_RF_LWA.joint1_placement, LOCAL_WORLD_ALIGNED, J_RF_LWA);
    Data::Matrix6x J_LF_LWA(6, model.nv);
    J_LF_LWA.setZero();
    getFrameJacobian(
      model, data, cm_LF_LWA.joint1_id, cm_LF_LWA.joint1_placement, LOCAL_WORLD_ALIGNED, J_LF_LWA);

    for (DenseIndex k = 0; k < model.nv; ++k)
    {
      BOOST_CHECK(J_RF_LWA.col(k).isZero() != cm_RF_LWA.colwise_joint1_sparsity[k]);
      BOOST_CHECK(J_LF_LWA.col(k).isZero() != cm_LF_LWA.colwise_joint1_sparsity[k]);
    }
    BOOST_CHECK(cm_RF_LWA.colwise_joint2_sparsity.isZero());
    BOOST_CHECK(cm_LF_LWA.colwise_joint2_sparsity.isZero());

    const SE3 oMc1 = data.oMi[clm_RF_LF_LWA.joint1_id] * clm_RF_LF_LWA.joint1_placement;
    const SE3 oMc2 = data.oMi[clm_RF_LF_LWA.joint2_id] * clm_RF_LF_LWA.joint2_placement;
    const SE3 c1Mc2 = oMc1.actInv(oMc2);
    const SE3 oMc1_lwa = SE3(oMc1.rotation(), SE3::Vector3::Zero());
    const SE3 oMc2_lwa = oMc1_lwa * c1Mc2;
    const Data::Matrix6x J_clm_LWA =
      oMc1_lwa.toActionMatrix() * J_RF_LOCAL - oMc2_lwa.toActionMatrix() * J_LF_LOCAL;

    for (DenseIndex k = 0; k < model.nv; ++k)
    {
      BOOST_CHECK(J_clm_LWA.col(k).isZero() != within(k, clm_RF_LF_LWA.colwise_span_indexes));
    }

    // Check Jacobian
    Data::MatrixXs J_RF_LWA_sparse(6, model.nv);
    J_RF_LWA_sparse.setZero(); // TODO: change input type when all the API would be refactorized
                               // with CRTP on contact constraints
    getConstraintJacobian(model, data, cm_RF_LWA, cd_RF_LWA, J_RF_LWA_sparse);
    BOOST_CHECK(J_RF_LWA.isApprox(J_RF_LWA_sparse));

    Data::MatrixXs J_LF_LWA_sparse(6, model.nv);
    J_LF_LWA_sparse.setZero(); // TODO: change input type when all the API would be refactorized
                               // with CRTP on contact constraints
    getConstraintJacobian(model, data, cm_LF_LWA, cd_LF_LWA, J_LF_LWA_sparse);
    BOOST_CHECK(J_LF_LWA.isApprox(J_LF_LWA_sparse));

    Data::MatrixXs J_clm_LWA_sparse(6, model.nv);
    J_clm_LWA_sparse.setZero(); // TODO: change input type when all the API would be refactorized
                                // with CRTP on contact constraints
    getConstraintJacobian(model, data, clm_RF_LF_LWA, cld_RF_LF_LWA, J_clm_LWA_sparse);
    BOOST_CHECK(J_clm_LWA.isApprox(J_clm_LWA_sparse));
  }

  // 3D - LOCAL
  {
    RigidConstraintModel cm_RF_LOCAL(CONTACT_3D, model, model.getJointId(RF), SE3::Random(), LOCAL);
    RigidConstraintData cd_RF_LOCAL(cm_RF_LOCAL);
    RigidConstraintModel cm_LF_LOCAL(CONTACT_3D, model, model.getJointId(LF), SE3::Random(), LOCAL);
    RigidConstraintData cd_LF_LOCAL(cm_LF_LOCAL);
    RigidConstraintModel clm_RF_LF_LOCAL(
      CONTACT_3D, model, cm_RF_LOCAL.joint1_id, cm_RF_LOCAL.joint1_placement, cm_LF_LOCAL.joint1_id,
      cm_LF_LOCAL.joint1_placement, LOCAL);
    RigidConstraintData cld_RF_LF_LOCAL(clm_RF_LF_LOCAL);

    Data::Matrix6x J_RF_LOCAL(6, model.nv);
    J_RF_LOCAL.setZero();
    getFrameJacobian(
      model, data, cm_RF_LOCAL.joint1_id, cm_RF_LOCAL.joint1_placement, cm_RF_LOCAL.reference_frame,
      J_RF_LOCAL);
    Data::Matrix6x J_LF_LOCAL(6, model.nv);
    J_LF_LOCAL.setZero();
    getFrameJacobian(
      model, data, cm_LF_LOCAL.joint1_id, cm_LF_LOCAL.joint1_placement, cm_LF_LOCAL.reference_frame,
      J_LF_LOCAL);

    for (DenseIndex k = 0; k < model.nv; ++k)
    {
      BOOST_CHECK(
        J_RF_LOCAL.middleRows<3>(SE3::LINEAR).col(k).isZero()
        != cm_RF_LOCAL.colwise_joint1_sparsity[k]);
      BOOST_CHECK(
        J_LF_LOCAL.middleRows<3>(SE3::LINEAR).col(k).isZero()
        != cm_LF_LOCAL.colwise_joint1_sparsity[k]);
    }
    BOOST_CHECK(cm_RF_LOCAL.colwise_joint2_sparsity.isZero());
    BOOST_CHECK(cm_LF_LOCAL.colwise_joint2_sparsity.isZero());

    const SE3 oMc1 = data.oMi[clm_RF_LF_LOCAL.joint1_id] * clm_RF_LF_LOCAL.joint1_placement;
    const SE3 oMc2 = data.oMi[clm_RF_LF_LOCAL.joint2_id] * clm_RF_LF_LOCAL.joint2_placement;
    const SE3 c1Mc2 = oMc1.actInv(oMc2);
    const Data::Matrix3x J_clm_LOCAL = J_RF_LOCAL.middleRows<3>(SE3::LINEAR)
                                       - c1Mc2.rotation() * J_LF_LOCAL.middleRows<3>(SE3::LINEAR);

    for (DenseIndex k = 0; k < model.nv; ++k)
    {
      BOOST_CHECK(J_clm_LOCAL.col(k).isZero(0) != within(k, clm_RF_LF_LOCAL.colwise_span_indexes));
    }

    // Check Jacobian
    Data::MatrixXs J_RF_LOCAL_sparse(3, model.nv);
    J_RF_LOCAL_sparse.setZero(); // TODO: change input type when all the API would be refactorized
                                 // with CRTP on contact constraints
    getConstraintJacobian(model, data, cm_RF_LOCAL, cd_RF_LOCAL, J_RF_LOCAL_sparse);
    BOOST_CHECK(J_RF_LOCAL.middleRows<3>(SE3::LINEAR).isApprox(J_RF_LOCAL_sparse));

    Data::MatrixXs J_LF_LOCAL_sparse(3, model.nv);
    J_LF_LOCAL_sparse.setZero(); // TODO: change input type when all the API would be refactorized
                                 // with CRTP on contact constraints
    getConstraintJacobian(model, data, cm_LF_LOCAL, cd_LF_LOCAL, J_LF_LOCAL_sparse);
    BOOST_CHECK(J_LF_LOCAL.middleRows<3>(SE3::LINEAR).isApprox(J_LF_LOCAL_sparse));

    Data::MatrixXs J_clm_LOCAL_sparse(3, model.nv);
    J_clm_LOCAL_sparse.setZero(); // TODO: change input type when all the API would be refactorized
                                  // with CRTP on contact constraints
    getConstraintJacobian(model, data, clm_RF_LF_LOCAL, cld_RF_LF_LOCAL, J_clm_LOCAL_sparse);
    BOOST_CHECK(J_clm_LOCAL.isApprox(J_clm_LOCAL_sparse));

    check_A1_and_A2(model, data, cm_RF_LOCAL, cd_RF_LOCAL);
    check_A1_and_A2(model, data, cm_LF_LOCAL, cd_LF_LOCAL);
    check_A1_and_A2(model, data, clm_RF_LF_LOCAL, cld_RF_LF_LOCAL);
  }

  // 3D - LOCAL_WORLD_ALIGNED
  {
    RigidConstraintModel cm_RF_LWA(
      CONTACT_3D, model, model.getJointId(RF), SE3::Random(), LOCAL_WORLD_ALIGNED);
    RigidConstraintData cd_RF_LWA(cm_RF_LWA);
    RigidConstraintModel cm_LF_LWA(
      CONTACT_3D, model, model.getJointId(LF), SE3::Random(), LOCAL_WORLD_ALIGNED);
    RigidConstraintData cd_LF_LWA(cm_LF_LWA);
    RigidConstraintModel clm_RF_LF_LWA(
      CONTACT_3D, model, cm_RF_LWA.joint1_id, cm_RF_LWA.joint1_placement, cm_LF_LWA.joint1_id,
      cm_LF_LWA.joint1_placement, LOCAL_WORLD_ALIGNED);
    RigidConstraintData cld_RF_LF_LWA(clm_RF_LF_LWA);

    Data::Matrix6x J_RF_LOCAL(6, model.nv);
    J_RF_LOCAL.setZero();
    getFrameJacobian(
      model, data, cm_RF_LWA.joint1_id, cm_RF_LWA.joint1_placement, LOCAL, J_RF_LOCAL);
    Data::Matrix6x J_LF_LOCAL(6, model.nv);
    J_LF_LOCAL.setZero();
    getFrameJacobian(
      model, data, cm_LF_LWA.joint1_id, cm_LF_LWA.joint1_placement, LOCAL, J_LF_LOCAL);

    Data::Matrix6x J_RF_LWA(6, model.nv);
    J_RF_LWA.setZero();
    getFrameJacobian(
      model, data, cm_RF_LWA.joint1_id, cm_RF_LWA.joint1_placement, LOCAL_WORLD_ALIGNED, J_RF_LWA);
    Data::Matrix6x J_LF_LWA(6, model.nv);
    J_LF_LWA.setZero();
    getFrameJacobian(
      model, data, cm_LF_LWA.joint1_id, cm_LF_LWA.joint1_placement, LOCAL_WORLD_ALIGNED, J_LF_LWA);

    for (DenseIndex k = 0; k < model.nv; ++k)
    {
      BOOST_CHECK(
        J_RF_LWA.middleRows<3>(SE3::LINEAR).col(k).isZero()
        != cm_RF_LWA.colwise_joint1_sparsity[k]);
      BOOST_CHECK(
        J_LF_LWA.middleRows<3>(SE3::LINEAR).col(k).isZero()
        != cm_LF_LWA.colwise_joint1_sparsity[k]);
    }
    BOOST_CHECK(cm_RF_LWA.colwise_joint2_sparsity.isZero());
    BOOST_CHECK(cm_LF_LWA.colwise_joint2_sparsity.isZero());

    const SE3 oMc1 = data.oMi[clm_RF_LF_LWA.joint1_id] * clm_RF_LF_LWA.joint1_placement;
    const SE3 oMc2 = data.oMi[clm_RF_LF_LWA.joint2_id] * clm_RF_LF_LWA.joint2_placement;
    const SE3 oMc1_lwa = SE3(oMc1.rotation(), SE3::Vector3::Zero());
    const SE3 oMc2_lwa = SE3(oMc2.rotation(), SE3::Vector3::Zero());
    const Data::Matrix3x J_clm_LWA =
      (oMc1_lwa.toActionMatrix() * J_RF_LOCAL - oMc2_lwa.toActionMatrix() * J_LF_LOCAL)
        .middleRows<3>(Motion::LINEAR);

    for (DenseIndex k = 0; k < model.nv; ++k)
    {
      BOOST_CHECK(J_clm_LWA.col(k).isZero(0) != within(k, clm_RF_LF_LWA.colwise_span_indexes));
    }

    // Check Jacobian
    Data::MatrixXs J_RF_LWA_sparse(3, model.nv);
    J_RF_LWA_sparse.setZero(); // TODO: change input type when all the API would be refactorized
                               // with CRTP on contact constraints
    getConstraintJacobian(model, data, cm_RF_LWA, cd_RF_LWA, J_RF_LWA_sparse);
    BOOST_CHECK(J_RF_LWA.middleRows<3>(SE3::LINEAR).isApprox(J_RF_LWA_sparse));

    Data::MatrixXs J_LF_LWA_sparse(3, model.nv);
    J_LF_LWA_sparse.setZero(); // TODO: change input type when all the API would be refactorized
                               // with CRTP on contact constraints
    getConstraintJacobian(model, data, cm_LF_LWA, cd_LF_LWA, J_LF_LWA_sparse);
    BOOST_CHECK(J_LF_LWA.middleRows<3>(SE3::LINEAR).isApprox(J_LF_LWA_sparse));

    Data::MatrixXs J_clm_LWA_sparse(3, model.nv);
    J_clm_LWA_sparse.setZero(); // TODO: change input type when all the API would be refactorized
                                // with CRTP on contact constraints
    getConstraintJacobian(model, data, clm_RF_LF_LWA, cld_RF_LF_LWA, J_clm_LWA_sparse);
    BOOST_CHECK(J_clm_LWA.isApprox(J_clm_LWA_sparse));
  }
}

BOOST_AUTO_TEST_SUITE_END()
