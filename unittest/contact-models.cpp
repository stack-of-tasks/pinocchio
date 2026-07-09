//
// Copyright (c) 2019-2024 INRIA
//

#define BOOST_TEST_MODULE contact_models

#include "pinocchio/spatial.hpp"
#include "pinocchio/constraints.hpp"
#include "pinocchio/multibody/sample-models.hpp"

#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"

#include <boost/utility/binary.hpp>

#include <boost/test/unit_test.hpp>

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
  for (Eigen::Index i = 0; i < mat.rows(); ++i)
    for (Eigen::Index j = 0; j < mat.rows(); ++j)
    {
      if (elt == mat(i, j))
        return true;
    }

  return false;
}

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
  BOOST_CHECK(cmodel2.residualSize() == 3);

  // Check contructor with two arguments
  RigidConstraintModel cmodel2prime(CONTACT_3D, model, 0);
  BOOST_CHECK(cmodel2prime.type == CONTACT_3D);
  BOOST_CHECK(cmodel2prime.joint1_id == 0);
  BOOST_CHECK(cmodel2prime.joint1_placement.isIdentity());
  BOOST_CHECK(cmodel2prime.residualSize() == 3);

  // Check default copy constructor
  RigidConstraintModel cmodel3(cmodel2);
  BOOST_CHECK(cmodel3 == cmodel2);

  // Check complete constructor 6D
  RigidConstraintModel cmodel4(CONTACT_6D, model, 0);
  BOOST_CHECK(cmodel4.type == CONTACT_6D);
  BOOST_CHECK(cmodel4.joint1_id == 0);
  BOOST_CHECK(cmodel4.joint1_placement.isIdentity());
  BOOST_CHECK(cmodel4.residualSize() == 6);
}

void check_A1_and_A2(
  const Model & model,
  const Data & data,
  const RigidConstraintModel & cmodel,
  RigidConstraintData & cdata)
{
  const RigidConstraintModel::Matrix36 A1_world = cmodel.getA1(cdata, WorldFrameTag());
  const RigidConstraintModel::Matrix36 A1_world_ref =
    cdata.oMc1.toActionMatrixInverse().topRows<3>();

  BOOST_CHECK(A1_world.isApprox(A1_world_ref));

  const RigidConstraintModel::Matrix36 A2_world = cmodel.getA2(cdata, WorldFrameTag());
  const RigidConstraintModel::Matrix36 A2_world_ref =
    -cdata.c1Mc2.rotation() * cdata.oMc2.toActionMatrixInverse().topRows<3>();

  BOOST_CHECK(A2_world.isApprox(A2_world_ref));

  const RigidConstraintModel::Matrix36 A1_local = cmodel.getA1(cdata, LocalFrameTag());
  const RigidConstraintModel::Matrix36 A1_local_ref =
    cmodel.joint1_placement.toActionMatrixInverse().topRows<3>();

  BOOST_CHECK(A1_local.isApprox(A1_local_ref));

  const RigidConstraintModel::Matrix36 A2_local = cmodel.getA2(cdata, LocalFrameTag());
  const RigidConstraintModel::Matrix36 A2_local_ref =
    -cdata.c1Mc2.rotation() * cmodel.joint2_placement.toActionMatrixInverse().topRows<3>();

  BOOST_CHECK(A2_local.isApprox(A2_local_ref));

  // Check Jacobians
  cmodel.calc(model, data, cdata);
  Data::MatrixXs J_ref(3, model.nv);
  J_ref.setZero();
  getConstraintJacobian(model, data, cmodel, cdata, J_ref);

  // World
  const Data::Matrix6x J1_world = getJointJacobian(model, data, cmodel.joint1_id, WORLD);
  const Data::Matrix6x J2_world = getJointJacobian(model, data, cmodel.joint2_id, WORLD);
  const Data::Matrix3x J_world = A1_world * J1_world + A2_world * J2_world;

  BOOST_CHECK(J_world.isApprox(J_ref));

  // Local
  const Data::Matrix6x J1_local = getJointJacobian(model, data, cmodel.joint1_id, LOCAL);
  const Data::Matrix6x J2_local = getJointJacobian(model, data, cmodel.joint2_id, LOCAL);
  const Data::Matrix3x J_local = A1_local * J1_local + A2_local * J2_local;

  BOOST_CHECK(J_local.isApprox(J_ref));

  // Check Jacobian matrix product
  const Eigen::Index m = 40;
  const Data::MatrixXs mat = Data::MatrixXs::Random(model.nv, m);

  Data::MatrixXs res(cmodel.residualSize(), m);
  res.setZero();
  cmodel.jacobian_matrix_product(model, data, cdata, mat, res);

  const Data::MatrixXs res_ref = J_ref * mat;

  BOOST_CHECK(res.isApprox(res_ref));
}

BOOST_AUTO_TEST_CASE(constraint3D_basic_operations)
{
  const pinocchio::Model model;
  const pinocchio::Data data(model);
  RigidConstraintModel cm(CONTACT_3D, model, 0, SE3::Random(), LOCAL);
  RigidConstraintData cd(cm);
  cm.calc(model, data, cd);

  const pinocchio::SE3 placement = cm.joint1_placement;

  {
    const Eigen::Vector3d diagonal_inertia(1, 2, 3);

    const pinocchio::SE3::Matrix6 spatial_inertia =
      cm.computeConstraintSpatialInertia(placement, diagonal_inertia);
    BOOST_CHECK(spatial_inertia.transpose().isApprox(spatial_inertia)); // check symmetric matrix

    const auto A1 = cm.getA1(cd, LocalFrameTag());
    const pinocchio::SE3::Matrix6 spatial_inertia_ref =
      A1.transpose() * diagonal_inertia.asDiagonal() * A1;

    BOOST_CHECK(spatial_inertia.isApprox(spatial_inertia_ref));
  }

  // Scalar
  {
    const double constant_value = 10;
    const Eigen::Vector3d diagonal_inertia = Eigen::Vector3d::Constant(constant_value);

    const pinocchio::SE3::Matrix6 spatial_inertia =
      cm.computeConstraintSpatialInertia(placement, diagonal_inertia);
    BOOST_CHECK(spatial_inertia.transpose().isApprox(spatial_inertia)); // check symmetric matrix

    const auto A1 = cm.getA1(cd, LocalFrameTag());
    const pinocchio::SE3::Matrix6 spatial_inertia_ref =
      A1.transpose() * diagonal_inertia.asDiagonal() * A1;

    BOOST_CHECK(spatial_inertia.isApprox(spatial_inertia_ref));

    const Inertia spatial_inertia_ref2(constant_value, placement.translation(), Symmetric3::Zero());
    BOOST_CHECK(spatial_inertia.isApprox(spatial_inertia_ref2.matrix()));
  }
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

    for (Eigen::Index k = 0; k < model.nv; ++k)
    {
      BOOST_CHECK(
        J_RF_LOCAL.col(k).isZero() != model.sparsity_pattern_vector[cm_RF_LOCAL.joint1_id][k]);
      BOOST_CHECK(
        J_LF_LOCAL.col(k).isZero() != model.sparsity_pattern_vector[cm_LF_LOCAL.joint1_id][k]);
    }
    BOOST_CHECK(model.sparsity_pattern_vector[cm_RF_LOCAL.joint2_id].isZero());
    BOOST_CHECK(model.sparsity_pattern_vector[cm_LF_LOCAL.joint2_id].isZero());

    const SE3 oMc1 = data.oMi[clm_RF_LF_LOCAL.joint1_id] * clm_RF_LF_LOCAL.joint1_placement;
    const SE3 oMc2 = data.oMi[clm_RF_LF_LOCAL.joint2_id] * clm_RF_LF_LOCAL.joint2_placement;
    const SE3 c1Mc2 = oMc1.actInv(oMc2);
    const Data::Matrix6x J_clm_LOCAL = J_RF_LOCAL - c1Mc2.toActionMatrix() * J_LF_LOCAL;

    Model::EigenIndexVector colwise_span_indexes;
    clm_RF_LF_LOCAL.getRowIndexes(model, data, cld_RF_LF_LOCAL, 0, colwise_span_indexes);
    for (Eigen::Index k = 0; k < model.nv; ++k)
    {
      if (!within(k, colwise_span_indexes))
        BOOST_CHECK(J_clm_LOCAL.col(k).isZero());
    }

    // Check Jacobian
    Data::MatrixXs J_RF_LOCAL_sparse(6, model.nv);
    J_RF_LOCAL_sparse.setZero(); // TODO: change input type when all the API would be refactorized
                                 // with CRTP on contact constraints
    cm_RF_LOCAL.calc(model, data, cd_RF_LOCAL);
    getConstraintJacobian(model, data, cm_RF_LOCAL, cd_RF_LOCAL, J_RF_LOCAL_sparse);
    BOOST_CHECK(J_RF_LOCAL.isApprox(J_RF_LOCAL_sparse));

    Data::MatrixXs J_LF_LOCAL_sparse(6, model.nv);
    J_LF_LOCAL_sparse.setZero(); // TODO: change input type when all the API would be refactorized
                                 // with CRTP on contact constraints
    cm_LF_LOCAL.calc(model, data, cd_LF_LOCAL);
    getConstraintJacobian(model, data, cm_LF_LOCAL, cd_LF_LOCAL, J_LF_LOCAL_sparse);
    BOOST_CHECK(J_LF_LOCAL.isApprox(J_LF_LOCAL_sparse));

    Data::MatrixXs J_clm_LOCAL_sparse(6, model.nv);
    J_clm_LOCAL_sparse.setZero(); // TODO: change input type when all the API would be refactorized
                                  // with CRTP on contact constraints
    clm_RF_LF_LOCAL.calc(model, data, cld_RF_LF_LOCAL);
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

    for (Eigen::Index k = 0; k < model.nv; ++k)
    {
      BOOST_CHECK(
        J_RF_LWA.col(k).isZero() != model.sparsity_pattern_vector[cm_RF_LWA.joint1_id][k]);
      BOOST_CHECK(
        J_LF_LWA.col(k).isZero() != model.sparsity_pattern_vector[cm_LF_LWA.joint1_id][k]);
    }
    BOOST_CHECK(model.sparsity_pattern_vector[cm_RF_LWA.joint2_id].isZero());
    BOOST_CHECK(model.sparsity_pattern_vector[cm_LF_LWA.joint2_id].isZero());

    const SE3 oMc1 = data.oMi[clm_RF_LF_LWA.joint1_id] * clm_RF_LF_LWA.joint1_placement;
    const SE3 oMc2 = data.oMi[clm_RF_LF_LWA.joint2_id] * clm_RF_LF_LWA.joint2_placement;
    const SE3 c1Mc2 = oMc1.actInv(oMc2);
    const SE3 oMc1_lwa = SE3(oMc1.rotation(), SE3::Vector3::Zero());
    const SE3 oMc2_lwa = oMc1_lwa * c1Mc2;
    const Data::Matrix6x J_clm_LWA =
      oMc1_lwa.toActionMatrix() * J_RF_LOCAL - oMc2_lwa.toActionMatrix() * J_LF_LOCAL;

    Model::EigenIndexVector colwise_span_indexes;
    clm_RF_LF_LWA.getRowIndexes(model, data, cld_RF_LF_LWA, 0, colwise_span_indexes);
    for (Eigen::Index k = 0; k < model.nv; ++k)
    {
      if (!within(k, colwise_span_indexes))
        BOOST_CHECK(J_clm_LWA.col(k).isZero());
    }

    // Check Jacobian
    Data::MatrixXs J_RF_LWA_sparse(6, model.nv);
    J_RF_LWA_sparse.setZero(); // TODO: change input type when all the API would be refactorized
                               // with CRTP on contact constraints
    cm_RF_LWA.calc(model, data, cd_RF_LWA);
    getConstraintJacobian(model, data, cm_RF_LWA, cd_RF_LWA, J_RF_LWA_sparse);
    BOOST_CHECK(J_RF_LWA.isApprox(J_RF_LWA_sparse));

    Data::MatrixXs J_LF_LWA_sparse(6, model.nv);
    J_LF_LWA_sparse.setZero(); // TODO: change input type when all the API would be refactorized
                               // with CRTP on contact constraints
    cm_LF_LWA.calc(model, data, cd_LF_LWA);
    getConstraintJacobian(model, data, cm_LF_LWA, cd_LF_LWA, J_LF_LWA_sparse);
    BOOST_CHECK(J_LF_LWA.isApprox(J_LF_LWA_sparse));

    Data::MatrixXs J_clm_LWA_sparse(6, model.nv);
    J_clm_LWA_sparse.setZero(); // TODO: change input type when all the API would be refactorized
                                // with CRTP on contact constraints
    clm_RF_LF_LWA.calc(model, data, cld_RF_LF_LWA);
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

    for (Eigen::Index k = 0; k < model.nv; ++k)
    {
      BOOST_CHECK(
        J_RF_LOCAL.middleRows<3>(SE3::LINEAR).col(k).isZero()
        != model.sparsity_pattern_vector[cm_RF_LOCAL.joint1_id][k]);
      BOOST_CHECK(
        J_LF_LOCAL.middleRows<3>(SE3::LINEAR).col(k).isZero()
        != model.sparsity_pattern_vector[cm_LF_LOCAL.joint1_id][k]);
    }
    BOOST_CHECK(model.sparsity_pattern_vector[cm_RF_LOCAL.joint2_id].isZero());
    BOOST_CHECK(model.sparsity_pattern_vector[cm_LF_LOCAL.joint2_id].isZero());

    const SE3 oMc1 = data.oMi[clm_RF_LF_LOCAL.joint1_id] * clm_RF_LF_LOCAL.joint1_placement;
    const SE3 oMc2 = data.oMi[clm_RF_LF_LOCAL.joint2_id] * clm_RF_LF_LOCAL.joint2_placement;
    const SE3 c1Mc2 = oMc1.actInv(oMc2);
    const Data::Matrix3x J_clm_LOCAL = J_RF_LOCAL.middleRows<3>(SE3::LINEAR)
                                       - c1Mc2.rotation() * J_LF_LOCAL.middleRows<3>(SE3::LINEAR);

    Model::EigenIndexVector colwise_span_indexes;
    clm_RF_LF_LOCAL.getRowIndexes(model, data, cld_RF_LF_LOCAL, 0, colwise_span_indexes);
    for (Eigen::Index k = 0; k < model.nv; ++k)
    {
      BOOST_CHECK(J_clm_LOCAL.col(k).isZero(0) != within(k, colwise_span_indexes));
    }

    // Check Jacobian
    Data::MatrixXs J_RF_LOCAL_sparse(3, model.nv);
    J_RF_LOCAL_sparse.setZero(); // TODO: change input type when all the API would be refactorized
                                 // with CRTP on contact constraints
    cm_RF_LOCAL.calc(model, data, cd_RF_LOCAL);
    getConstraintJacobian(model, data, cm_RF_LOCAL, cd_RF_LOCAL, J_RF_LOCAL_sparse);
    BOOST_CHECK(J_RF_LOCAL.middleRows<3>(SE3::LINEAR).isApprox(J_RF_LOCAL_sparse));

    Data::MatrixXs J_LF_LOCAL_sparse(3, model.nv);
    J_LF_LOCAL_sparse.setZero(); // TODO: change input type when all the API would be refactorized
                                 // with CRTP on contact constraints
    cm_LF_LOCAL.calc(model, data, cd_LF_LOCAL);
    getConstraintJacobian(model, data, cm_LF_LOCAL, cd_LF_LOCAL, J_LF_LOCAL_sparse);
    BOOST_CHECK(J_LF_LOCAL.middleRows<3>(SE3::LINEAR).isApprox(J_LF_LOCAL_sparse));

    Data::MatrixXs J_clm_LOCAL_sparse(3, model.nv);
    J_clm_LOCAL_sparse.setZero(); // TODO: change input type when all the API would be refactorized
                                  // with CRTP on contact constraints
    clm_RF_LF_LOCAL.calc(model, data, cld_RF_LF_LOCAL);
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

    for (Eigen::Index k = 0; k < model.nv; ++k)
    {
      BOOST_CHECK(
        J_RF_LWA.middleRows<3>(SE3::LINEAR).col(k).isZero()
        != model.sparsity_pattern_vector[cm_RF_LWA.joint1_id][k]);
      BOOST_CHECK(
        J_LF_LWA.middleRows<3>(SE3::LINEAR).col(k).isZero()
        != model.sparsity_pattern_vector[cm_LF_LWA.joint1_id][k]);
    }
    BOOST_CHECK(model.sparsity_pattern_vector[cm_RF_LWA.joint2_id].isZero());
    BOOST_CHECK(model.sparsity_pattern_vector[cm_LF_LWA.joint2_id].isZero());

    const SE3 oMc1 = data.oMi[clm_RF_LF_LWA.joint1_id] * clm_RF_LF_LWA.joint1_placement;
    const SE3 oMc2 = data.oMi[clm_RF_LF_LWA.joint2_id] * clm_RF_LF_LWA.joint2_placement;
    const SE3 oMc1_lwa = SE3(oMc1.rotation(), SE3::Vector3::Zero());
    const SE3 oMc2_lwa = SE3(oMc2.rotation(), SE3::Vector3::Zero());
    const Data::Matrix3x J_clm_LWA =
      (oMc1_lwa.toActionMatrix() * J_RF_LOCAL - oMc2_lwa.toActionMatrix() * J_LF_LOCAL)
        .middleRows<3>(Motion::LINEAR);

    Model::EigenIndexVector colwise_span_indexes;
    clm_RF_LF_LWA.getRowIndexes(model, data, cld_RF_LF_LWA, 0, colwise_span_indexes);
    for (Eigen::Index k = 0; k < model.nv; ++k)
    {
      BOOST_CHECK(J_clm_LWA.col(k).isZero(0) != within(k, colwise_span_indexes));
    }

    // Check Jacobian
    Data::MatrixXs J_RF_LWA_sparse(3, model.nv);
    J_RF_LWA_sparse.setZero(); // TODO: change input type when all the API would be refactorized
                               // with CRTP on contact constraints
    cm_RF_LWA.calc(model, data, cd_RF_LWA);
    getConstraintJacobian(model, data, cm_RF_LWA, cd_RF_LWA, J_RF_LWA_sparse);
    BOOST_CHECK(J_RF_LWA.middleRows<3>(SE3::LINEAR).isApprox(J_RF_LWA_sparse));

    Data::MatrixXs J_LF_LWA_sparse(3, model.nv);
    J_LF_LWA_sparse.setZero(); // TODO: change input type when all the API would be refactorized
                               // with CRTP on contact constraints
    cm_LF_LWA.calc(model, data, cd_LF_LWA);
    getConstraintJacobian(model, data, cm_LF_LWA, cd_LF_LWA, J_LF_LWA_sparse);
    BOOST_CHECK(J_LF_LWA.middleRows<3>(SE3::LINEAR).isApprox(J_LF_LWA_sparse));

    Data::MatrixXs J_clm_LWA_sparse(3, model.nv);
    J_clm_LWA_sparse.setZero(); // TODO: change input type when all the API would be refactorized
                                // with CRTP on contact constraints
    clm_RF_LF_LWA.calc(model, data, cld_RF_LF_LWA);
    getConstraintJacobian(model, data, clm_RF_LF_LWA, cld_RF_LF_LWA, J_clm_LWA_sparse);
    BOOST_CHECK(J_clm_LWA.isApprox(J_clm_LWA_sparse));
  }
}
