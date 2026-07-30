//
// Copyright (c) 2024-2026 INRIA
//

#define BOOST_TEST_MODULE delassus_operator_rigid_body

#include <memory>
#include <algorithm>

#include "pinocchio/constraints.hpp"
#include "pinocchio/multibody/sample-models.hpp"

#include "pinocchio/algorithm/constraint-cholesky.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/delassus-operator.hpp"
#include "pinocchio/algorithm/aba.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/utils/reference.hpp"

#include "utils/model-generator.hpp"

#include <boost/utility/binary.hpp>

#include <boost/test/unit_test.hpp>

using namespace pinocchio;
using namespace Eigen;

template<typename Delassus>
struct DelassusAccessor : Delassus
{
  typedef Delassus Base;
  using Base::m_compliance;
  using Base::m_damping;
  using Base::m_sum_compliance_damping;
  using Base::m_sum_compliance_damping_inverse;
};

template<typename Delassus>
const DelassusAccessor<Delassus> & access(const Delassus & delassus)
{
  return static_cast<const DelassusAccessor<Delassus> &>(delassus);
}

BOOST_AUTO_TEST_CASE(default_constructor_shared_ptr)
{
  typedef PointContactConstraintModelTpl<double> ConstraintModel;
  typedef DelassusOperatorRigidBodySystemsTpl<
    double, 0, JointCollectionDefaultTpl, ConstraintModel, std::shared_ptr>
    DelassusOperatorRigidBodySharedPtr;
  typedef typename DelassusOperatorRigidBodySharedPtr::ConstraintModelVector ConstraintModelVector;
  typedef typename DelassusOperatorRigidBodySharedPtr::ConstraintDataVector ConstraintDataVector;

  std::shared_ptr<Model> model_shared_ptr = std::make_shared<Model>();
  Model * model_ptr = model_shared_ptr.get();
  Model & model = *model_ptr;
  buildModels::humanoidRandom(model, true);

  std::shared_ptr<Data> data_shared_ptr = std::make_shared<Data>(model);
  Data * data_ptr = data_shared_ptr.get();
  //  Data & data = *data_ptr;

  std::shared_ptr<ConstraintModelVector> constraint_models_shared_ptr =
    std::make_shared<ConstraintModelVector>();
  ConstraintModelVector * constraint_models_ptr = constraint_models_shared_ptr.get();
  std::shared_ptr<ConstraintDataVector> constraint_datas_shared_ptr =
    std::make_shared<ConstraintDataVector>();
  ConstraintDataVector * constraint_datas_ptr = constraint_datas_shared_ptr.get();

  DelassusOperatorRigidBodySharedPtr delassus_operator(
    model_shared_ptr, data_shared_ptr, constraint_models_shared_ptr, constraint_datas_shared_ptr);

  BOOST_CHECK(delassus_operator.size() == 0);
  BOOST_CHECK(&delassus_operator.model() == model_ptr);
  BOOST_CHECK(&delassus_operator.data() == data_ptr);
  BOOST_CHECK(&delassus_operator.constraint_models() == constraint_models_ptr);
  BOOST_CHECK(&delassus_operator.constraint_datas() == constraint_datas_ptr);
}

BOOST_AUTO_TEST_CASE(default_constructor_reference_wrapper)
{
  typedef PointContactConstraintModelTpl<double> ConstraintModel;
  typedef DelassusOperatorRigidBodySystemsTpl<
    double, 0, JointCollectionDefaultTpl, ConstraintModel, std::reference_wrapper>
    DelassusOperatorRigidBodyReferenceWrapper;
  typedef
    typename DelassusOperatorRigidBodyReferenceWrapper::ConstraintModelVector ConstraintModelVector;
  typedef
    typename DelassusOperatorRigidBodyReferenceWrapper::ConstraintDataVector ConstraintDataVector;

  Model model;
  std::reference_wrapper<Model> model_ref = model;
  buildModels::humanoidRandom(model, true);

  Data data(model);
  std::reference_wrapper<Data> data_ref = data;

  ConstraintModelVector constraint_models;
  std::reference_wrapper<ConstraintModelVector> constraint_models_ref = constraint_models;
  ConstraintDataVector constraint_datas;
  auto constraint_datas_ref = pinocchio::internal::helper::make_ref(constraint_datas);

  DelassusOperatorRigidBodyReferenceWrapper delassus_operator(
    model_ref, data_ref, constraint_models_ref, constraint_datas_ref);

  const auto csize = residualSize(pinocchio::internal::helper::get_ref(constraint_models_ref));
  BOOST_CHECK(delassus_operator.size() == csize);

  BOOST_CHECK(delassus_operator.size() == 0);
  BOOST_CHECK(&delassus_operator.model() == &model);
  BOOST_CHECK(&delassus_operator.data() == &data);
  BOOST_CHECK(&delassus_operator.constraint_models() == &constraint_models);
  BOOST_CHECK(&delassus_operator.constraint_datas() == &constraint_datas);
}

BOOST_AUTO_TEST_CASE(default_constructor_const_reference_wrapper)
{
  Model model;
  buildModels::humanoidRandom(model, true);
  Data data(model);

  // creating a constraint model (joint limits)
  const JointIndex last_joint_id = Model::JointIndex(model.njoints);
  Model::IndexVector selected_joints;
  for (Model::JointIndex i = 2 /* skip free_flyer*/; i < last_joint_id; ++i)
  {
    selected_joints.push_back(i);
  }
  JointLimitConstraintModel jlcm(model, selected_joints);
  ConstraintModel cmodel(jlcm); // let's put it in a variant just to show it works
  ConstraintData cdata = cmodel.createData();

  // we will wrap the constraint model in a std::reference_wrapper
  // we make it const -> the delassus does not touch the constraint models/datas
  typedef std::reference_wrapper<const ConstraintModel> WrappedConstraintModel;

  // delassus trait automatically infers the correct type for the wrapped constraint data
  typedef DelassusOperatorRigidBodySystemsTpl<
    double, 0, JointCollectionDefaultTpl, WrappedConstraintModel>
    WrappedDelassusOperatorRigidBody;

  typedef
    typename WrappedDelassusOperatorRigidBody::ConstraintModelVector WrappedConstraintModelVector;
  typedef
    typename WrappedDelassusOperatorRigidBody::ConstraintDataVector WrappedConstraintDataVector;
  WrappedConstraintModelVector constraint_models;
  constraint_models.push_back(cmodel);
  WrappedConstraintDataVector constraint_datas;
  constraint_datas.push_back(cdata);

  // we can still call calc on cmodel (but not via its const wrapped handle)
  VectorXd q = neutral(model);
  computeJointJacobians(model, data, q); // fill data's oMi, lMi and J fields
  data.q_in = q;
  cmodel.calc(model, data, cdata); // make constraint data up to date with system state

  WrappedDelassusOperatorRigidBody delassus_operator(
    pinocchio::internal::helper::make_ref(model),             //
    pinocchio::internal::helper::make_ref(data),              //
    pinocchio::internal::helper::make_ref(constraint_models), //
    pinocchio::internal::helper::make_ref(constraint_datas));

  const auto csize = residualSize(pinocchio::internal::helper::get_ref(constraint_models));

  BOOST_CHECK(delassus_operator.size() == csize);
  BOOST_CHECK(&delassus_operator.model() == &model);
  BOOST_CHECK(&delassus_operator.data() == &data);
  BOOST_CHECK(&delassus_operator.constraint_models() == &constraint_models);
  BOOST_CHECK(&delassus_operator.constraint_datas() == &constraint_datas);
}

BOOST_AUTO_TEST_CASE(size_in_bytes)
{
  typedef FrameAnchorConstraintModelTpl<double> ConstraintModel;
  typedef DelassusOperatorRigidBodySystemsTpl<
    double, 0, JointCollectionDefaultTpl, ConstraintModel, std::reference_wrapper>
    DelassusOperatorRigidBodyReferenceWrapper;
  typedef
    typename DelassusOperatorRigidBodyReferenceWrapper::ConstraintModelVector ConstraintModelVector;
  typedef
    typename DelassusOperatorRigidBodyReferenceWrapper::ConstraintDataVector ConstraintDataVector;

  Model model;
  std::reference_wrapper<Model> model_ref = model;
  buildModels::humanoidRandom(model, true);
  model.gravity.setZero();
  const Eigen::VectorXd q_neutral = neutral(model);
  const Eigen::VectorXd v = Eigen::VectorXd::Random(model.nv);
  const Eigen::VectorXd tau = Eigen::VectorXd::Random(model.nv);

  const std::string RF = "rleg6_joint";
  const std::string LF = "lleg6_joint";

  Data data(model);
  std::reference_wrapper<Data> data_ref = data;

  ConstraintModelVector constraint_models;
  ConstraintDataVector constraint_datas;
  const ConstraintModel cm_RF_LF_LOCAL(
    model, model.getJointId(RF), SE3::Random(), model.getJointId(LF), SE3::Random());

  constraint_models.push_back(cm_RF_LF_LOCAL);
  constraint_datas.push_back(cm_RF_LF_LOCAL.createData());
  const ConstraintModel cm_LF_LOCAL(model, model.getJointId(LF), SE3::Random());
  constraint_models.push_back(cm_LF_LOCAL);
  constraint_datas.push_back(cm_LF_LOCAL.createData());

  computeJointJacobians(model, data, q_neutral);
  data.q_in = q_neutral;
  calc(model, data, constraint_models, constraint_datas);

  std::reference_wrapper<ConstraintModelVector> constraint_models_ref = constraint_models;
  std::reference_wrapper<ConstraintDataVector> constraint_datas_ref = constraint_datas;

  const double damping_value = 1e-4;

  const auto delassus_operator = DelassusOperatorRigidBodyReferenceWrapper(
    model_ref, data_ref, constraint_models_ref, constraint_datas_ref, damping_value);

  const auto size_in_bytes = delassus_operator.sizeInBytes();
  BOOST_CHECK(size_in_bytes > delassus_operator.getInternalData().sizeInBytes());
}

BOOST_AUTO_TEST_CASE(general_test_frame_anchor_constraint_model)
{
  typedef FrameAnchorConstraintModelTpl<double> ConstraintModel;
  typedef DelassusOperatorRigidBodySystemsTpl<
    double, 0, JointCollectionDefaultTpl, ConstraintModel, std::reference_wrapper>
    DelassusOperatorRigidBodyReferenceWrapper;
  typedef
    typename DelassusOperatorRigidBodyReferenceWrapper::ConstraintModelVector ConstraintModelVector;
  typedef
    typename DelassusOperatorRigidBodyReferenceWrapper::ConstraintDataVector ConstraintDataVector;

  Model model;
  std::reference_wrapper<Model> model_ref = model;
  buildModels::humanoidRandom(model, true);
  model.gravity.setZero();
  const Eigen::VectorXd q_neutral = neutral(model);
  const Eigen::VectorXd v = Eigen::VectorXd::Random(model.nv);
  const Eigen::VectorXd tau = Eigen::VectorXd::Random(model.nv);

  const std::string RF = "rleg6_joint";
  const std::string LF = "lleg6_joint";

  Data data(model);
  std::reference_wrapper<Data> data_ref = data;

  ConstraintModelVector constraint_models;
  ConstraintDataVector constraint_datas;

  {
    const ConstraintModel cm_RF_LF_LOCAL(
      model, model.getJointId(RF), SE3::Random(), model.getJointId(LF), SE3::Random());
    constraint_models.push_back(cm_RF_LF_LOCAL);
    constraint_datas.push_back(cm_RF_LF_LOCAL.createData());
  }

  {
    const ConstraintModel cm_LF_LOCAL(model, model.getJointId(LF), SE3::Random());
    constraint_models.push_back(cm_LF_LOCAL);
    constraint_datas.push_back(cm_LF_LOCAL.createData());
  }

  computeJointJacobians(model, data, q_neutral);
  data.q_in = q_neutral;
  calc(model, data, constraint_models, constraint_datas);

  //  ConstraintDataVector constraint_datas_gt = constraint_datas;
  //
  std::reference_wrapper<ConstraintModelVector> constraint_models_ref = constraint_models;
  std::reference_wrapper<ConstraintDataVector> constraint_datas_ref = constraint_datas;

  const double damping_value = 1e-4;

  const double mu_inv = damping_value;
  const double mu = 1. / mu_inv;

  // Test operator *
  {
    DelassusOperatorRigidBodyReferenceWrapper delassus_operator(
      model_ref, data_ref, constraint_models_ref, constraint_datas_ref, damping_value);
    delassus_operator.updateDamping(mu_inv);
    delassus_operator.updateCompliance(0);

    {
      BOOST_CHECK(access(delassus_operator).m_damping.matrix().diagonal().isConstant(mu_inv, 0));
      BOOST_CHECK(access(delassus_operator).m_compliance.isConstant(0., 0));
      BOOST_CHECK(access(delassus_operator)
                    .m_sum_compliance_damping.matrix()
                    .diagonal()
                    .isConstant(mu_inv, 0));
    }

    BOOST_CHECK(delassus_operator.isDirty());
    delassus_operator.compute();
    // we could also simply call:
    // delassus_operator.updateDecomposition();
    BOOST_CHECK(!delassus_operator.isDirty());
    {
      BOOST_CHECK(access(delassus_operator)
                    .m_sum_compliance_damping_inverse.matrix()
                    .diagonal()
                    .isConstant(mu, 0));
    }

    const Eigen::VectorXd rhs = Eigen::VectorXd::Random(delassus_operator.size());
    Eigen::VectorXd res(delassus_operator.size());

    delassus_operator.applyOnTheRight(rhs, res);

    // Eval J Minv Jt
    Data data_gt(model);
    auto Minv_gt = computeMinverse(model, data_gt, q_neutral);
    make_symmetric(Minv_gt);
    BOOST_CHECK(Minv_gt.isApprox(Minv_gt.transpose()));

    auto M_gt = crba(model, data_gt, q_neutral, Convention::WORLD);
    make_symmetric(M_gt);

    ConstraintDataVector constraint_datas_gt = createData(constraint_models);
    Eigen::MatrixXd constraints_jacobian_gt(delassus_operator.size(), model.nv);
    constraints_jacobian_gt.setZero();
    computeJointJacobians(model, data_gt, data.q_in);
    data_gt.q_in = data.q_in;
    calc(model, data_gt, constraint_models, constraint_datas_gt);
    getConstraintsJacobian(
      model, data_gt, constraint_models, constraint_datas_gt, constraints_jacobian_gt);

    const Eigen::MatrixXd delassus_dense_gt_undamped =
      constraints_jacobian_gt * Minv_gt * constraints_jacobian_gt.transpose();
    const Eigen::MatrixXd delassus_dense_gt =
      delassus_dense_gt_undamped + delassus_operator.getDamping().matrix();

    Eigen::VectorXd tau_constraints = Eigen::VectorXd::Zero(model.nv);
    evalConstraintJacobianTransposeMatrixProduct(
      model, data_gt, constraint_models, constraint_datas_gt, rhs, tau_constraints);
    const Eigen::VectorXd Jt_rhs_gt = constraints_jacobian_gt.transpose() * rhs;
    BOOST_CHECK(tau_constraints.isApprox(Jt_rhs_gt));

    std::vector<Force> fext_gt(size_t(model.njoints), Force::Zero());
    Data data_aba(model);
    mapConstraintForcesToJointForces(
      model, data_aba, constraint_models, constraint_datas_gt, rhs, fext_gt, LocalFrameTag());
    auto fext_gt_copy = fext_gt;

    aba(
      model, data_aba, q_neutral, Eigen::VectorXd::Zero(model.nv), 0 * tau_constraints, fext_gt,
      Convention::LOCAL);

    Eigen::VectorXd tau_constraints_ref = Eigen::VectorXd::Zero(model.nv);
    for (JointIndex joint_id = JointIndex(model.njoints) - 1; joint_id >= 1; --joint_id)
    {
      const auto parent_id = model.parents[joint_id];
      const auto & jmodel = model.joints[joint_id];
      const auto & jdata = data_aba.joints[joint_id];
      const auto joint_nv = jmodel.nv();
      const auto joint_idx_v = jmodel.idx_v();

      tau_constraints_ref.segment(joint_idx_v, joint_nv) =
        jdata.S().matrix().transpose() * fext_gt_copy[joint_id].toVector();

      fext_gt_copy[parent_id] += data_aba.liMi[joint_id].act(fext_gt_copy[joint_id]);
    }
    BOOST_CHECK(tau_constraints_ref.isApprox(Jt_rhs_gt));

    for (Model::JointIndex joint_id = 1; joint_id < Model::JointIndex(model.njoints); ++joint_id)
    {
      BOOST_CHECK(data.joints[joint_id].S().isApprox(data_aba.joints[joint_id].S()));
      BOOST_CHECK(data.joints[joint_id].U().isApprox(data_aba.joints[joint_id].U()));
      BOOST_CHECK(data.joints[joint_id].Dinv().isApprox(data_aba.joints[joint_id].Dinv()));
      BOOST_CHECK(data.joints[joint_id].UDinv().isApprox(data_aba.joints[joint_id].UDinv()));
      BOOST_CHECK(data.liMi[joint_id].isApprox(data_aba.liMi[joint_id]));
      BOOST_CHECK(data.Yaba[joint_id].isApprox(data_aba.Yaba[joint_id]));
    }
    BOOST_CHECK(delassus_operator.getInternalData().u.isApprox(data_aba.u));

    const Eigen::VectorXd Minv_Jt_rhs_gt = Minv_gt * Jt_rhs_gt;
    BOOST_CHECK(delassus_operator.getInternalData().ddq.isApprox(Minv_Jt_rhs_gt));

    const auto res_gt = (delassus_dense_gt * rhs).eval();
    BOOST_CHECK(res.isApprox(res_gt));

    // Multiple call and operator *
    {
      for (int i = 0; i < 100; ++i)
      {
        Eigen::VectorXd res(delassus_operator.size());
        delassus_operator.applyOnTheRight(rhs, res);
        BOOST_CHECK(res.isApprox(res_gt));

        const Eigen::VectorXd res2 = delassus_operator * rhs;
        BOOST_CHECK(res2 == res); // Should be exactly the same
        BOOST_CHECK(res2.isApprox(res_gt));
      }
    }
  } // End: Test operator *

  // Test second call consistency
  {
    //    Data data(model);
    //    std::reference_wrapper<Data> data_ref = data;
    DelassusOperatorRigidBodyReferenceWrapper delassus_operator(
      model_ref, data_ref, constraint_models_ref, constraint_datas_ref, damping_value);
    delassus_operator.updateDamping(mu_inv);
    delassus_operator.updateCompliance(0);
    delassus_operator.compute();

    Data data2(model);
    std::reference_wrapper<Data> data2_ref = data2;

    ConstraintDataVector constraint_datas2 = createData(constraint_models_ref.get());
    computeJointJacobians(model, data2, data.q_in);
    data2.q_in = data.q_in;
    calc(model, data2, constraint_models, constraint_datas2);
    std::reference_wrapper<ConstraintDataVector> constraint_datas2_ref = constraint_datas2;

    DelassusOperatorRigidBodyReferenceWrapper delassus_operator2(
      model_ref, data2_ref, constraint_models_ref, constraint_datas2_ref, damping_value);
    delassus_operator2.updateDamping(mu_inv);
    delassus_operator2.updateCompliance(0);
    delassus_operator2.compute();

    // Check consistency between a fresh and dirty data
    {
      BOOST_CHECK(!delassus_operator.isDirty());
      BOOST_CHECK(!delassus_operator2.isDirty());
      BOOST_CHECK(delassus_operator.getDamping() == delassus_operator2.getDamping());
      BOOST_CHECK(delassus_operator.getCompliance() == delassus_operator2.getCompliance());

      BOOST_CHECK(data.oMi == data2.oMi);
      BOOST_CHECK(data.liMi == data2.liMi);
      BOOST_CHECK(data.J == data2.J);
      BOOST_CHECK(data.Yaba == data2.Yaba);
      BOOST_CHECK(data.joint_elimination_order == data2.joint_elimination_order);
      BOOST_CHECK(data.constraints_supported_dim == data2.constraints_supported_dim);
      BOOST_CHECK(data.joint_neighbours == data2.joint_neighbours);
      BOOST_CHECK((data.joint_cross_coupling.keys() == data2.joint_cross_coupling.keys()).all());

      for (JointIndex joint_id = 1; joint_id < JointIndex(model.njoints); ++joint_id)
      {
        BOOST_CHECK(data.joints[joint_id].S() == data2.joints[joint_id].S());
        BOOST_CHECK(data.joints[joint_id].U() == data2.joints[joint_id].U());
        BOOST_CHECK(data.joints[joint_id].Dinv() == data2.joints[joint_id].Dinv());
        BOOST_CHECK(data.joints[joint_id].UDinv() == data2.joints[joint_id].UDinv());

        BOOST_CHECK(data.joints_augmented[joint_id].S() == data2.joints_augmented[joint_id].S());
        BOOST_CHECK(data.joints_augmented[joint_id].U() == data2.joints_augmented[joint_id].U());
        BOOST_CHECK(
          data.joints_augmented[joint_id].Dinv() == data2.joints_augmented[joint_id].Dinv());
        BOOST_CHECK(
          data.joints_augmented[joint_id].UDinv() == data2.joints_augmented[joint_id].UDinv());

        BOOST_CHECK(data.oYaba_augmented[joint_id] == data2.oYaba_augmented[joint_id]);
      }
    }
  }

  // Test solveInPlace
  {
    //    Data data(model);
    //    std::reference_wrapper<Data> data_ref = data;
    DelassusOperatorRigidBodyReferenceWrapper delassus_operator(
      model_ref, data_ref, constraint_models_ref, constraint_datas_ref, damping_value);
    delassus_operator.updateDamping(mu_inv);
    delassus_operator.updateCompliance(0);
    delassus_operator.compute();

    // Check data allocation
    {
      // typedef std::pair<JointIndex, JointIndex> JointPair;
      const auto & neighbours = data.joint_neighbours;
      const auto & joint_cross_coupling = data.joint_cross_coupling;
      // const auto & projected_joint_cross_coupling = data.projected_joint_cross_coupling;
      for (const auto joint_i : data.joint_elimination_order)
      {
        const auto & joint_i_neighbours = neighbours[joint_i];
        const auto parent_joint_i = model.parents[joint_i];
        for (size_t j = 0; j < joint_i_neighbours.size(); j++)
        {
          const JointIndex joint_j = joint_i_neighbours[j];

          if (joint_j != parent_joint_i)
          {
            BOOST_CHECK(
              joint_cross_coupling.exists({joint_j, parent_joint_i})
              || joint_cross_coupling.exists({parent_joint_i, joint_j}));
          }

          for (size_t k = j + 1; k < joint_i_neighbours.size(); ++k)
          {
            const JointIndex joint_k = joint_i_neighbours[k];
            BOOST_CHECK(joint_cross_coupling.exists({joint_k, joint_i}));
            BOOST_CHECK(joint_cross_coupling.exists({joint_j, joint_k}));
          }
        }
      }
    }
    Data data_crba(model);
    Eigen::MatrixXd M = crba(model, data_crba, q_neutral, Convention::WORLD);
    make_symmetric(M);

    auto constraint_datas_crba = createData(constraint_models);
    computeJointJacobians(model, data_crba, data.q_in);
    data_crba.q_in = data.q_in;
    calc(model, data_crba, constraint_models, constraint_datas_crba);
    const auto Jc =
      getConstraintsJacobian(model, data_crba, constraint_models, constraint_datas_crba);

    const Eigen::MatrixXd M_augmented = M + mu * Jc.transpose() * Jc;
    const Eigen::MatrixXd M_augmented_inv = M_augmented.inverse();

    const Eigen::VectorXd rhs = Eigen::VectorXd::Unit(model.nv, 0);
    Eigen::VectorXd res = rhs;

    const Eigen::VectorXd col_ref = M_augmented_inv * rhs;
    delassus_operator.getAugmentedMassMatrixOperator().solveInPlace(res);
    BOOST_CHECK(res.isApprox(col_ref, 1e-10));

    for (Eigen::Index col_id = 0; col_id < model.nv; ++col_id)
    {
      const Eigen::VectorXd rhs = Eigen::VectorXd::Unit(model.nv, col_id);
      const auto res_ref = (M_augmented_inv * rhs).eval();

      Eigen::VectorXd res = rhs;
      delassus_operator.getAugmentedMassMatrixOperator().solveInPlace(res);
      BOOST_CHECK(res.isApprox(res_ref, 1e-10));
    }

    // Test Delassus inverse
    const auto delassus_size = delassus_operator.size();
    const Eigen::MatrixXd M_inv = M.inverse();
    const Eigen::MatrixXd delassus_dense =
      Jc * M_inv * Jc.transpose()
      + mu_inv * Eigen::MatrixXd::Identity(delassus_size, delassus_size);
    const Eigen::MatrixXd delassus_dense_inv = delassus_dense.inverse();

    for (Eigen::Index col_id = 0; col_id < delassus_size; ++col_id)
    {
      const Eigen::VectorXd rhs = Eigen::VectorXd::Unit(delassus_size, col_id);
      const auto res_ref = (delassus_dense_inv * rhs).eval();

      Eigen::VectorXd res = rhs;
      delassus_operator.solveInPlace(res);
      BOOST_CHECK(res.isApprox(res_ref, 1e-10));
    }
  }

  // Test updateDamping
  {
    Data data(model);
    std::reference_wrapper<Data> data_ref = data;

    auto constraint_datas = createData(constraint_models);
    computeJointJacobians(model, data, q_neutral);
    data.q_in = q_neutral;
    calc(model, data, constraint_models, constraint_datas);
    std::reference_wrapper<ConstraintDataVector> constraint_datas_ref = constraint_datas;

    DelassusOperatorRigidBodyReferenceWrapper delassus_operator(
      model_ref, data_ref, constraint_models_ref, constraint_datas_ref, damping_value);
    delassus_operator.compute();

    Data data2(model);
    std::reference_wrapper<Data> data2_ref = data2;
    auto constraint_datas2 = createData(constraint_models);
    computeJointJacobians(model, data2, q_neutral);
    data2.q_in = q_neutral;
    calc(model, data, constraint_models, constraint_datas2);
    std::reference_wrapper<ConstraintDataVector> constraint_datas2_ref = constraint_datas2;

    const double new_damping_value = 1e-6;
    // const double new_mu = 1. / new_damping_value;
    DelassusOperatorRigidBodyReferenceWrapper delassus_operator2(
      model_ref, data2_ref, constraint_models_ref, constraint_datas2_ref, new_damping_value);
    BOOST_CHECK(delassus_operator2.isDirty());
    delassus_operator2.compute();
    BOOST_CHECK(!delassus_operator2.isDirty());

    BOOST_CHECK(!delassus_operator.isDirty());
    delassus_operator.updateDamping(new_damping_value);
    BOOST_CHECK(delassus_operator.isDirty());
    delassus_operator.compute(true);
    BOOST_CHECK(!delassus_operator.isDirty());

    BOOST_CHECK(delassus_operator.getDamping() == delassus_operator2.getDamping());
    BOOST_CHECK(delassus_operator.getCompliance() == delassus_operator2.getCompliance());

    BOOST_CHECK(data.J == data2.J);
    BOOST_CHECK(data.joint_elimination_order == data2.joint_elimination_order);
    for (JointIndex joint_id = 1; joint_id < JointIndex(model.njoints); ++joint_id)
    {
      BOOST_CHECK(data.oMi[joint_id] == data2.oMi[joint_id]);
      BOOST_CHECK(data.liMi[joint_id] == data2.liMi[joint_id]);

      BOOST_CHECK(data.Yaba[joint_id] == data2.Yaba[joint_id]);
      BOOST_CHECK(data.joints[joint_id].StU() == data2.joints[joint_id].StU());
      BOOST_CHECK(data.joints[joint_id].Dinv() == data2.joints[joint_id].Dinv());
      BOOST_CHECK(data.joints[joint_id].UDinv() == data2.joints[joint_id].UDinv());

      BOOST_CHECK(data.oYaba_augmented[joint_id] == data2.oYaba_augmented[joint_id]);
      BOOST_CHECK(data.joints_augmented[joint_id].StU() == data2.joints_augmented[joint_id].StU());
      BOOST_CHECK(
        data.joints_augmented[joint_id].Dinv() == data2.joints_augmented[joint_id].Dinv());
      BOOST_CHECK(
        data.joints_augmented[joint_id].UDinv() == data2.joints_augmented[joint_id].UDinv());
    }

    Data data_crba(model);
    Eigen::MatrixXd M = crba(model, data_crba, q_neutral, Convention::WORLD);
    make_symmetric(M);

    auto constraint_datas_crba = createData(constraint_models);
    computeJointJacobians(model, data_crba, q_neutral);
    data_crba.q_in = q_neutral;
    calc(model, data_crba, constraint_models, constraint_datas_crba);
    const auto Jc =
      getConstraintsJacobian(model, data_crba, constraint_models, constraint_datas_crba);

    const auto delassus_size = delassus_operator.size();
    const Eigen::MatrixXd M_inv = M.inverse();
    const Eigen::MatrixXd delassus_dense =
      Jc * M_inv * Jc.transpose()
      + new_damping_value * Eigen::MatrixXd::Identity(delassus_size, delassus_size);
    const Eigen::MatrixXd delassus_dense_inv = delassus_dense.inverse();

    for (Eigen::Index col_id = 0; col_id < delassus_size; ++col_id)
    {
      const Eigen::VectorXd rhs = Eigen::VectorXd::Unit(delassus_size, col_id);
      const auto res_ref = (delassus_dense_inv * rhs).eval();

      Eigen::VectorXd res = rhs;
      delassus_operator.solveInPlace(res);
      BOOST_CHECK(res.isApprox(res_ref, 1e-8));

      Eigen::VectorXd res2 = rhs;
      delassus_operator2.solveInPlace(res2);
      BOOST_CHECK(res2.isApprox(res_ref, 1e-8));

      BOOST_CHECK(res.isApprox(res2));
    }
  }

  // Compare with lcaba
  //  {
  //    Data data_lcaba(model);
  //    RigidConstraintModel rcm(
  //      CONTACT_6D, model, cm_RF_LOCAL.joint1_id, cm_RF_LOCAL.joint1_placement,
  //      cm_RF_LOCAL.joint2_id, cm_RF_LOCAL.joint2_placement,
  //      //                             cm_RF_LOCAL.joint2_id, cm_RF_LOCAL.joint2_placement,
  //      //                             cm_RF_LOCAL.joint1_id, cm_RF_LOCAL.joint1_placement,
  //      LOCAL);
  //
  //    std::vector<RigidConstraintModel> rcm_vector;
  //    rcm_vector.push_back(rcm);
  //    std::vector<RigidConstraintData> rcd_vector;
  //    rcd_vector.push_back(rcm.createData());
  //
  //    const auto & rcd = rcd_vector[0];
  //
  //    computeJointMinimalOrdering(model, data_lcaba, rcm_vector);
  //    ProximalSettings prox_settings(1e-14, min_damping_value, 1);
  //    lcaba(model, data_lcaba, q_neutral, v, tau, rcm_vector, rcd_vector, prox_settings);
  //
  //    BOOST_CHECK(data_lcaba.joint_elimination_order == data.joint_elimination_order);
  //
  //
  //
  //
  //    //    for(JointIndex joint_id = 1; joint_id < JointIndex(model.njoints); ++joint_id)
  //    //    {
  //    //      BOOST_CHECK(data.oMi[joint_id].isApprox(data_lcaba.oMi[joint_id]));
  //    //      BOOST_CHECK(data.liMi[joint_id].isApprox(data_lcaba.liMi[joint_id]));
  //    //      std::cout << "oYaba_aug[" << joint_id << "]:\n" << data.oYaba_augmented[joint_id] <<
  //    //      std::endl; std::cout << "oYaba_aug_lcaba[" << joint_id << "]:\n" <<
  //    //      data_lcaba.oYaba_augmented[joint_id] << std::endl;
  //    //    }
  //    //    std::cout << "elimination ordering: ";
  //    //    for(const auto val: data_lcaba.joint_elimination_order)
  //    //      std::cout << val << ", ";
  //    std::cout << std::endl;
  //    std::cout << "---------" << std::endl;
  //
  //    std::cout << "---------" << std::endl;
  //
  //    std::cout << "oYaba_aug[" << joint1_id << "]:\n"
  //              << data.oYaba_augmented[joint1_id] << std::endl;
  //    std::cout << "oYaba_aug_lcaba[" << joint1_id << "]:\n"
  //              << data_lcaba.oYaba_augmented[joint1_id] << std::endl;
  //
  //    std::cout << "---------" << std::endl;
  //
  //    std::cout << "oYaba_aug[" << joint2_id << "]:\n"
  //              << data.oYaba_augmented[joint2_id] << std::endl;
  //    std::cout << "oYaba_aug_lcaba[" << joint2_id << "]:\n"
  //              << data_lcaba.oYaba_augmented[joint2_id] << std::endl;
  //  }
}

BOOST_AUTO_TEST_CASE(general_test_point_contact_constraint_model)
{
  typedef PointContactConstraintModelTpl<double> ConstraintModel;
  typedef DelassusOperatorRigidBodySystemsTpl<
    double, 0, JointCollectionDefaultTpl, ConstraintModel, std::reference_wrapper>
    DelassusOperatorRigidBodyReferenceWrapper;
  typedef
    typename DelassusOperatorRigidBodyReferenceWrapper::ConstraintModelVector ConstraintModelVector;
  typedef
    typename DelassusOperatorRigidBodyReferenceWrapper::ConstraintDataVector ConstraintDataVector;

  Model model;
  std::reference_wrapper<Model> model_ref = model;
  buildModels::humanoidRandom(model, true);
  model.gravity.setZero();
  const Eigen::VectorXd q_neutral = neutral(model);
  const Eigen::VectorXd v = Eigen::VectorXd::Random(model.nv);
  const Eigen::VectorXd tau = Eigen::VectorXd::Random(model.nv);

  const std::string RF = "rleg6_joint";
  const std::string LF = "lleg6_joint";

  Data data(model), data_gt(model);
  std::reference_wrapper<Data> data_ref = data;
  //  const ConstraintModel cm_RF_LOCAL(model, model.getJointId(RF), SE3::Random());
  //  const ConstraintModel cm_RF_LOCAL(model, 0, SE3::Identity(), model.getJointId(RF),
  //  SE3::Random());

  ConstraintModelVector constraint_models;
  ConstraintDataVector constraint_datas;

  const ConstraintModel cm_LF_RF_LOCAL(
    model, model.getJointId(LF), SE3::Random(), model.getJointId(RF), SE3::Random());
  constraint_models.push_back(cm_LF_RF_LOCAL);
  constraint_datas.push_back(cm_LF_RF_LOCAL.createData());

  const ConstraintModel cm_LF_LOCAL(model, model.getJointId(LF), SE3::Random());
  constraint_models.push_back(cm_LF_LOCAL);
  constraint_datas.push_back(cm_LF_LOCAL.createData());

  const ConstraintModel cm_RF_LOCAL(model, model.getJointId(RF), SE3::Random());
  constraint_models.push_back(cm_RF_LOCAL);
  constraint_datas.push_back(cm_RF_LOCAL.createData());

  ConstraintDataVector constraint_datas_gt = constraint_datas;

  // Fill data and constraint data with necessary computations
  computeJointJacobians(model, data, q_neutral);
  data.q_in = q_neutral;
  calc(model, data, constraint_models, constraint_datas);
  //
  computeJointJacobians(model, data_gt, q_neutral);
  data_gt.q_in = q_neutral;
  calc(model, data_gt, constraint_models, constraint_datas_gt);

  std::reference_wrapper<ConstraintModelVector> constraint_models_ref = constraint_models;
  std::reference_wrapper<ConstraintDataVector> constraint_datas_ref = constraint_datas;

  const double min_damping_value = 1e-4;

  DelassusOperatorRigidBodyReferenceWrapper delassus_operator(
    model_ref, data_ref, constraint_models_ref, constraint_datas_ref, min_damping_value);
  // Eval J Minv Jt
  auto Minv_gt = computeMinverse(model, data_gt, q_neutral);
  make_symmetric(Minv_gt);
  BOOST_CHECK(Minv_gt.isApprox(Minv_gt.transpose()));

  auto M_gt = crba(model, data_gt, q_neutral);
  make_symmetric(M_gt);

  Eigen::MatrixXd constraints_jacobian_gt(delassus_operator.size(), model.nv);
  constraints_jacobian_gt.setZero();
  calc(model, data_gt, constraint_models, constraint_datas_gt);
  getConstraintsJacobian(
    model, data_gt, constraint_models, constraint_datas_gt, constraints_jacobian_gt);

  const Eigen::MatrixXd delassus_dense_gt_undamped =
    constraints_jacobian_gt * Minv_gt * constraints_jacobian_gt.transpose();
  const Eigen::MatrixXd delassus_dense_gt =
    delassus_dense_gt_undamped + delassus_operator.getDamping().matrix();

  // Test Jacobian transpose operator
  {
    const Eigen::VectorXd rhs = Eigen::VectorXd::Random(delassus_operator.size());

    computeJointJacobians(model, data, q_neutral);
    for (Model::JointIndex joint_id = 1; joint_id < Model::JointIndex(model.njoints); ++joint_id)
    {
      BOOST_CHECK(data.joints[joint_id].S().isApprox(data_gt.joints[joint_id].S()));
      BOOST_CHECK(data.liMi[joint_id].isApprox(data_gt.liMi[joint_id]));
      BOOST_CHECK(data.oMi[joint_id].isApprox(data_gt.oMi[joint_id]));
    }

    const Eigen::VectorXd Jt_rhs_gt = constraints_jacobian_gt.transpose() * rhs;
    Eigen::VectorXd Jt_rhs(model.nv);

    calc(model, data, constraint_models, constraint_datas);
    evalConstraintJacobianTransposeMatrixProduct(
      model, data, constraint_models, constraint_datas, rhs, Jt_rhs);

    BOOST_CHECK(Jt_rhs.isApprox(Jt_rhs_gt));
  };

  // Test operator *
  {
    const Eigen::VectorXd rhs = Eigen::VectorXd::Random(delassus_operator.size());
    Eigen::VectorXd res(delassus_operator.size());

    delassus_operator.compute();
    delassus_operator.applyOnTheRight(rhs, res);

    // Eval Jt*rhs vs internal computations. This test is useful to check intermediate computation.
    //    Eigen::VectorXd Jt_rhs(model.nv);
    //    evalConstraintJacobianTransposeMatrixProduct(model,data,constraint_models,constraint_datas,rhs,Jt_rhs);
    //    BOOST_CHECK(delassus_operator.getInternalData().u.isApprox(Jt_rhs));
    //
    //    std::cout << "delassus_operator.getInternalData().u: " <<
    //    delassus_operator.getInternalData().u.transpose() << std::endl; std::cout << "Jt_rhs: " <<
    //    Jt_rhs.transpose() << std::endl; const Eigen::VectorXd Jt_rhs_gt =
    //    constraints_jacobian_gt.transpose() * rhs;
    //    BOOST_CHECK(delassus_operator.getInternalData().u.isApprox(Jt_rhs_gt));
    //
    //    std::vector<Data::Force> joint_forces_gt(
    //      size_t(model.njoints), Data::Force::Zero());
    //    mapConstraintForcesToJointForces(
    //      model, data_gt, constraint_models, constraint_datas_gt, rhs, joint_forces_gt);

    Eigen::VectorXd tau_constraints = Eigen::VectorXd::Zero(model.nv);
    evalConstraintJacobianTransposeMatrixProduct(
      model, data_gt, constraint_models, constraint_datas_gt, rhs, tau_constraints);
    const Eigen::VectorXd Jt_rhs_gt = constraints_jacobian_gt.transpose() * rhs;
    BOOST_CHECK(tau_constraints.isApprox(Jt_rhs_gt));

    Data data_aba(model);
    aba(
      model, data_aba, q_neutral, Eigen::VectorXd::Zero(model.nv), tau_constraints,
      Convention::LOCAL);

    for (Model::JointIndex joint_id = 1; joint_id < Model::JointIndex(model.njoints); ++joint_id)
    {
      //      const InternalData & internal_data = delassus_operator.getInternalData();
      BOOST_CHECK(data.joints[joint_id].S().isApprox(data_aba.joints[joint_id].S()));
      BOOST_CHECK(data.liMi[joint_id].isApprox(data_aba.liMi[joint_id]));
      //      BOOST_CHECK(internal_data.oMi[joint_id].isApprox(data_aba.oMi[joint_id])); //
      //      minimal::ABA does not compute this quantity
      BOOST_CHECK(data.Yaba[joint_id].isApprox(data_aba.Yaba[joint_id]));
    }
    BOOST_CHECK(delassus_operator.getInternalData().u.isApprox(data_aba.u));

    //    std::cout << "delassus_operator.getInternalData().u: " <<
    //    delassus_operator.getInternalData().u.transpose() << std::endl; std::cout << "data_aba.u:
    //    "
    //    << data_aba.u.transpose() << std::endl;
    //
    const Eigen::VectorXd Minv_Jt_rhs_gt = Minv_gt * Jt_rhs_gt;
    //
    //    //    std::cout << "Minv_Jt_rhs: " << delassus_operator.getInternalData().ddq.transpose()
    //    <<
    //    //    std::endl; std::cout << "Minv_Jt_rhs_gt: " << Minv_Jt_rhs_gt.transpose() <<
    //    std::endl;
    //
    BOOST_CHECK(delassus_operator.getInternalData().ddq.isApprox(Minv_Jt_rhs_gt));
    //
    const auto res_gt = (delassus_dense_gt * rhs).eval();
    BOOST_CHECK(res.isApprox(res_gt));
    //
    //    //    std::cout << "res: " << res.transpose() << std::endl;
    //    //    std::cout << "res_gt: " << res_gt.transpose() << std::endl;
    //
    // Multiple call and operator *
    {
      for (int i = 0; i < 100; ++i)
      {
        Eigen::VectorXd res(delassus_operator.size());
        delassus_operator.applyOnTheRight(rhs, res);
        BOOST_CHECK(res.isApprox(res_gt));

        const Eigen::VectorXd res2 = delassus_operator * rhs;
        BOOST_CHECK(res2 == res); // Should be exactly the same
        BOOST_CHECK(res2.isApprox(res_gt));
      }
    }
  } // End: Test operator *

  // Update damping
  {
    const Eigen::VectorXd rhs = Eigen::VectorXd::Random(delassus_operator.size());
    const double mu = 1;
    delassus_operator.updateDamping(mu);
    Eigen::MatrixXd damping_mat =
      mu * Eigen::MatrixXd::Identity(delassus_operator.size(), delassus_operator.size());
    BOOST_CHECK(delassus_operator.getDamping().matrix().isApprox(damping_mat));

    BOOST_CHECK(delassus_operator.isDirty());

    Eigen::VectorXd res_damped(delassus_operator.size());
    delassus_operator.applyOnTheRight(rhs, res_damped);
    const auto res_gt_damped =
      ((delassus_dense_gt_undamped
        + mu * Eigen::MatrixXd::Identity(delassus_operator.size(), delassus_operator.size()))
       * rhs)
        .eval();
    BOOST_CHECK(res_damped.isApprox(res_gt_damped));
  }

  // Update compliance
  {
    const Eigen::VectorXd rhs = Eigen::VectorXd::Random(delassus_operator.size());
    const double compliance = 3e-2;
    const double mu = 1;
    delassus_operator.updateDamping(mu);
    delassus_operator.updateCompliance(compliance);
    BOOST_CHECK(delassus_operator.getCompliance().isApproxToConstant(compliance));

    BOOST_CHECK(delassus_operator.isDirty());

    Eigen::VectorXd res_damped(delassus_operator.size());
    delassus_operator.applyOnTheRight(rhs, res_damped);
    const auto res_gt_damped =
      ((delassus_dense_gt_undamped
        + compliance * Eigen::MatrixXd::Identity(delassus_operator.size(), delassus_operator.size())
        + mu * Eigen::MatrixXd::Identity(delassus_operator.size(), delassus_operator.size()))
       * rhs)
        .eval();
    BOOST_CHECK(res_damped.isApprox(res_gt_damped));
  }

  // Test solveInPlace
  {
    //      const Eigen::VectorXd rhs = Eigen::VectorXd::Random(delassus_operator.size());
    const Eigen::VectorXd rhs = Eigen::VectorXd::Unit(model.nv, 0);
    Eigen::VectorXd res = rhs;

    const double mu_inv = min_damping_value;
    const double mu = 1. / mu_inv;

    Data data(model);
    std::reference_wrapper<Data> data_ref = data;
    computeJointJacobians(model, data, q_neutral);
    data.q_in = q_neutral;
    calc(model, data, constraint_models, constraint_datas);

    DelassusOperatorRigidBodyReferenceWrapper delassus_operator(
      model_ref, data_ref, constraint_models_ref, constraint_datas_ref, min_damping_value);
    delassus_operator.updateDamping(mu_inv);
    delassus_operator.updateCompliance(0);
    delassus_operator.compute();
    delassus_operator.getAugmentedMassMatrixOperator().solveInPlace(res);

    // Check joint_elimination_order
    const auto & joint_elimination_order = data.joint_elimination_order;
    for (auto it = joint_elimination_order.begin(); it != joint_elimination_order.end(); ++it)
    {
      const auto joint_id = *it;
      const auto & joint_support = model.supports[joint_id];
      for (const auto supporting_joint : joint_support)
      {
        bool is_after = std::find(it, joint_elimination_order.end(), supporting_joint)
                        != joint_elimination_order.end();
        BOOST_CHECK(is_after || supporting_joint == 0);
      }
    }

    Data data_crba(model);
    Eigen::MatrixXd M = crba(model, data_crba, q_neutral, Convention::WORLD);
    make_symmetric(M);

    auto constraint_datas_crba = createData(constraint_models);
    computeJointJacobians(model, data_crba, q_neutral);
    data_crba.q_in = q_neutral;
    calc(model, data, constraint_models, constraint_datas_crba);
    const auto Jc =
      getConstraintsJacobian(model, data_crba, constraint_models, constraint_datas_crba);

    const Eigen::MatrixXd M_augmented = M + mu * Jc.transpose() * Jc;
    const Eigen::MatrixXd M_augmented_inv = M_augmented.inverse();
    const Eigen::VectorXd col_ref = M_augmented_inv * rhs;

    BOOST_CHECK(res.isApprox(col_ref, 1e-10));

    for (Eigen::Index col_id = 0; col_id < model.nv; ++col_id)
    {
      const Eigen::VectorXd rhs = Eigen::VectorXd::Unit(model.nv, col_id);
      const auto res_ref = (M_augmented_inv * rhs).eval();

      Eigen::VectorXd res = rhs;
      delassus_operator.getAugmentedMassMatrixOperator().solveInPlace(res);
      BOOST_CHECK(res.isApprox(res_ref, 1e-10));
    }

    // Test Delassus inverse
    const auto delassus_size = delassus_operator.size();
    const Eigen::MatrixXd M_inv = M.inverse();
    const Eigen::MatrixXd delassus_dense =
      Jc * M_inv * Jc.transpose()
      + mu_inv * Eigen::MatrixXd::Identity(delassus_size, delassus_size);
    const Eigen::MatrixXd delassus_dense_inv = delassus_dense.inverse();

    for (Eigen::Index col_id = 0; col_id < delassus_size; ++col_id)
    {
      const Eigen::VectorXd rhs = Eigen::VectorXd::Unit(delassus_size, col_id);
      const auto res_ref = (delassus_dense_inv * rhs).eval();

      Eigen::VectorXd res = rhs;
      delassus_operator.solveInPlace(res);
      BOOST_CHECK(res.isApprox(res_ref, 1e-10));
    }

    //
    //    const Eigen::VectorXd res_gt = delassus_dense_gt.llt().solve(rhs);
    //
    //    BOOST_CHECK(res.isApprox(res_gt));
    //    std::cout << "res:\n" << res.transpose() << std::endl;
    //    std::cout << "res_gt:\n" << res_gt.transpose() << std::endl;
    //
    //    // Check accuracy
    //
    //    const double min_damping_value_sqrt = math::sqrt(min_damping_value);
    //    const double min_damping_value_sqrt_inv = 1. / min_damping_value_sqrt;
    //    const Eigen::MatrixXd scaled_matrix =
    //      Eigen::MatrixXd::Identity(model.nv, model.nv) * min_damping_value_sqrt;
    //    const Eigen::MatrixXd scaled_matrix_inv =
    //      Eigen::MatrixXd::Identity(delassus_operator.size(), delassus_operator.size())
    //      * min_damping_value_sqrt_inv;
    //    const Eigen::MatrixXd M_gt_scaled = scaled_matrix * M_gt * scaled_matrix;
    //    std::cout << "M_gt_scaled:\n" << M_gt_scaled << std::endl;
    //    std::cout << "M_gt:\n" << M_gt << std::endl;
    //    const Eigen::MatrixXd M_gt_scaled_plus_Jt_J =
    //      M_gt_scaled + constraints_jacobian_gt.transpose() * constraints_jacobian_gt;
    //    const Eigen::MatrixXd M_gt_scaled_plus_Jt_J_inv = M_gt_scaled_plus_Jt_J.inverse();
    //    const Eigen::MatrixXd damped_delassus_inverse_woodbury =
    //      1. / min_damping_value
    //        * Eigen::MatrixXd::Identity(delassus_operator.size(), delassus_operator.size())
    //      - scaled_matrix_inv
    //          * (constraints_jacobian_gt * M_gt_scaled_plus_Jt_J *
    //          constraints_jacobian_gt.transpose())
    //              .eval()
    //          * scaled_matrix_inv;
    //
    //    const Eigen::VectorXd res_gt_woodbury = damped_delassus_inverse_woodbury * rhs;
    //
    //    std::cout << "res: " << res.transpose() << std::endl;
    //    std::cout << "res_gt: " << res_gt.transpose() << std::endl;
    //    std::cout << "res_gt_woodbury: " << res_gt_woodbury.transpose() << std::endl;
    //    std::cout << "res - res_gt: " << (res - res_gt).norm() << std::endl;
  }
}

template<
  template<typename> class Holder,
  typename Model,
  typename Data,
  typename ConstraintModelVector,
  typename ConstraintDataVector,
  typename GeneralizedConfigurationVector,
  typename Scalar>
void test_apply_on_the_right(
  const Holder<Model> & model_ref,
  const Holder<Data> & data_ref,
  const Holder<ConstraintModelVector> & constraint_models_ref,
  const Holder<ConstraintDataVector> & constraint_datas_ref,
  const Eigen::MatrixBase<GeneralizedConfigurationVector> & q_neutral,
  const Scalar damping_value)
{
  typedef typename ConstraintModelVector::value_type ConstraintModel;
  typedef DelassusOperatorRigidBodySystemsTpl<
    double, 0, JointCollectionDefaultTpl, ConstraintModel, std::reference_wrapper>
    DelassusOperatorRigidBodyReferenceWrapper;

  const Model & model = model_ref;
  Data & data = data_ref;
  const ConstraintModelVector & constraint_models = constraint_models_ref;
  ConstraintDataVector & constraint_datas = constraint_datas_ref;

  // Necessary to update data oMi, lMi and J for internal rigid body computations
  computeJointJacobians(model, data, q_neutral);
  // Necessary to keep constraint datas up to date
  data.q_in = q_neutral;
  calc(model, data, constraint_models, constraint_datas);

  int max_csize = 0, csize = 0;
  for (size_t k = 0; k < constraint_models.size(); ++k)
  {
    const auto & constraint_model = constraint_models[k];
    max_csize += constraint_model.residualSize(MaximalSelection());
    csize += constraint_model.residualSize();
  }
  BOOST_CHECK(csize <= max_csize);

  DelassusOperatorRigidBodyReferenceWrapper delassus_operator(
    model_ref, data_ref, constraint_models_ref, constraint_datas_ref, damping_value);

  BOOST_CHECK(delassus_operator.size() == csize);

  delassus_operator.updateDamping(damping_value);
  delassus_operator.updateCompliance(0);
  delassus_operator.compute();
  BOOST_CHECK(delassus_operator.size() == csize);

  const Eigen::VectorXd rhs = Eigen::VectorXd::Random(delassus_operator.size());
  Eigen::VectorXd res(delassus_operator.size());

  delassus_operator.applyOnTheRight(rhs, res);

  // Eval J Minv Jt
  Data data_gt(model);
  auto Minv_gt = computeMinverse(model, data_gt, q_neutral);
  make_symmetric(Minv_gt);
  BOOST_CHECK(Minv_gt.isApprox(Minv_gt.transpose()));

  auto M_gt = crba(model, data_gt, q_neutral);
  make_symmetric(M_gt);

  ConstraintDataVector constraint_datas_gt = createData(constraint_models);
  computeJointJacobians(model, data_gt, q_neutral);
  data_gt.q_in = q_neutral;
  BOOST_CHECK(data_gt.q_in == data.q_in);
  calc(model, data_gt, constraint_models, constraint_datas_gt);
  int csize_gt = 0;
  for (std::size_t i = 0; i < constraint_models.size(); ++i)
  {
    const auto & cmodel = constraint_models[i];
    csize_gt += cmodel.residualSize();
  }
  BOOST_CHECK(csize_gt == csize);

  Eigen::MatrixXd constraints_jacobian_gt(delassus_operator.size(), model.nv);
  constraints_jacobian_gt.setZero();
  getConstraintsJacobian(
    model, data_gt, constraint_models, constraint_datas_gt, constraints_jacobian_gt);

  const Eigen::MatrixXd delassus_dense_gt_undamped =
    constraints_jacobian_gt * Minv_gt * constraints_jacobian_gt.transpose();
  const Eigen::MatrixXd delassus_dense_gt =
    delassus_dense_gt_undamped + delassus_operator.getDamping().matrix();

  Eigen::VectorXd tau_constraints = Eigen::VectorXd::Zero(model.nv);
  evalConstraintJacobianTransposeMatrixProduct(
    model, data_gt, constraint_models, constraint_datas_gt, rhs, tau_constraints);
  const Eigen::VectorXd Jt_rhs_gt = constraints_jacobian_gt.transpose() * rhs;
  BOOST_CHECK(tau_constraints.isApprox(Jt_rhs_gt));

  Data data_aba(model);
  aba(
    model, data_aba, q_neutral, Eigen::VectorXd::Zero(model.nv), tau_constraints,
    Convention::LOCAL);

  for (typename Model::JointIndex joint_id = 1;
       joint_id < typename Model::JointIndex(model.njoints); ++joint_id)
  {
    BOOST_CHECK(data.joints[joint_id].S().isApprox(data_aba.joints[joint_id].S()));
    BOOST_CHECK(data.liMi[joint_id].isApprox(data_aba.liMi[joint_id]));
    BOOST_CHECK(data.Yaba[joint_id].isApprox(data_aba.Yaba[joint_id]));
  }
  BOOST_CHECK(delassus_operator.getInternalData().u.isApprox(data_aba.u));

  const Eigen::VectorXd Minv_Jt_rhs_gt = Minv_gt * Jt_rhs_gt;
  BOOST_CHECK(delassus_operator.getInternalData().ddq.isApprox(Minv_Jt_rhs_gt));

  const auto res_gt = (delassus_dense_gt * rhs).eval();
  BOOST_CHECK(res.isApprox(res_gt));

  // Multiple call and operator *
  {
    for (int i = 0; i < 100; ++i)
    {
      Eigen::VectorXd res(delassus_operator.size());
      delassus_operator.applyOnTheRight(rhs, res);
      BOOST_CHECK(res.isApprox(res_gt));

      const Eigen::VectorXd res2 = delassus_operator * rhs;
      BOOST_CHECK(res2 == res); // Should be exactly the same
      BOOST_CHECK(res2.isApprox(res_gt));
    }
  }
}
template<
  template<typename> class Holder,
  typename Model,
  typename Data,
  typename ConstraintModelVector,
  typename ConstraintDataVector,
  typename GeneralizedConfigurationVector,
  typename Scalar>
void test_solve_in_place(
  const Holder<Model> & model_ref,
  const Holder<Data> & data_ref,
  const Holder<ConstraintModelVector> & constraint_models_ref,
  const Holder<ConstraintDataVector> & constraint_datas_ref,
  const Eigen::MatrixBase<GeneralizedConfigurationVector> & q_neutral,
  const Scalar damping_value)
{
  typedef typename ConstraintModelVector::value_type ConstraintModel;
  typedef DelassusOperatorRigidBodySystemsTpl<
    double, 0, JointCollectionDefaultTpl, ConstraintModel, std::reference_wrapper>
    DelassusOperatorRigidBodyReferenceWrapper;

  const Model & model = pinocchio::internal::helper::get_ref(model_ref);
  Data & data = pinocchio::internal::helper::get_ref(data_ref);
  const ConstraintModelVector & constraint_models =
    pinocchio::internal::helper::get_ref(constraint_models_ref);
  ConstraintDataVector & constraint_datas =
    pinocchio::internal::helper::get_ref(constraint_datas_ref);

  // Necessary to update data oMi, lMi and J for internal rigid body computations
  computeJointJacobians(model, data, q_neutral);
  // Necessary to keep constraint datas up to date
  data.q_in = q_neutral;
  calc(model, data, constraint_models, constraint_datas);

  DelassusOperatorRigidBodyReferenceWrapper delassus_operator(
    model_ref, data_ref, constraint_models_ref, constraint_datas_ref, damping_value);
  delassus_operator.updateDamping(damping_value);
  delassus_operator.updateCompliance(0);
  delassus_operator.compute();

  Data data_crba(model);
  Eigen::MatrixXd M = crba(model, data_crba, q_neutral, Convention::WORLD);
  make_symmetric(M);

  computeJointJacobians(model, data_crba, q_neutral);
  data_crba.q_in = q_neutral;
  BOOST_CHECK(data_crba.q_in == data.q_in);

  auto constraint_datas_crba = createData(constraint_models);
  calc(model, data_crba, constraint_models, constraint_datas_crba);
  const auto Jc =
    getConstraintsJacobian(model, data_crba, constraint_models, constraint_datas_crba);

  const Scalar mu = Scalar(1) / damping_value;
  const Eigen::MatrixXd muJcTJc = mu * Jc.transpose() * Jc;
  const Eigen::MatrixXd M_augmented = M + muJcTJc;
  const Eigen::MatrixXd M_augmented_inv = M_augmented.inverse();

  const Eigen::VectorXd rhs = Eigen::VectorXd::Unit(model.nv, 0);
  Eigen::VectorXd res = rhs;

  const Eigen::VectorXd col_ref = M_augmented_inv * rhs;
  delassus_operator.getAugmentedMassMatrixOperator().solveInPlace(res);
  BOOST_CHECK(res.isApprox(col_ref, 1e-10));

  for (Eigen::Index col_id = 0; col_id < model.nv; ++col_id)
  {
    const Eigen::VectorXd rhs = Eigen::VectorXd::Unit(model.nv, col_id);
    const auto res_ref = (M_augmented_inv * rhs).eval();

    Eigen::VectorXd res = rhs;
    delassus_operator.getAugmentedMassMatrixOperator().solveInPlace(res);
    BOOST_CHECK(res.isApprox(res_ref, 1e-10));
  }

  // Test Delassus inverse
  const auto delassus_size = delassus_operator.size();
  const Eigen::MatrixXd M_inv = M.inverse();
  const Eigen::MatrixXd delassus_dense =
    Jc * M_inv * Jc.transpose()
    + damping_value * Eigen::MatrixXd::Identity(delassus_size, delassus_size);
  const Eigen::MatrixXd delassus_dense_inv = delassus_dense.inverse();

  for (Eigen::Index col_id = 0; col_id < delassus_size; ++col_id)
  {
    const Eigen::VectorXd rhs = Eigen::VectorXd::Unit(delassus_size, col_id);
    const auto res_ref = (delassus_dense_inv * rhs).eval();

    Eigen::VectorXd res = rhs;
    delassus_operator.solveInPlace(res);
    BOOST_CHECK(res.isApprox(res_ref, 1e-10));
  }
}

BOOST_AUTO_TEST_CASE(general_test_joint_friction_constraint)
{
  typedef JointFrictionConstraintModelTpl<double> ConstraintModel;
  typedef DelassusOperatorRigidBodySystemsTpl<
    double, 0, JointCollectionDefaultTpl, ConstraintModel, std::reference_wrapper>
    DelassusOperatorRigidBodyReferenceWrapper;
  typedef
    typename DelassusOperatorRigidBodyReferenceWrapper::ConstraintModelVector ConstraintModelVector;
  typedef
    typename DelassusOperatorRigidBodyReferenceWrapper::ConstraintDataVector ConstraintDataVector;

  Model model;
  std::reference_wrapper<Model> model_ref = model;

  buildModels::humanoidRandom(model, true);

  const Eigen::VectorXd q_neutral = neutral(model);
  const Eigen::VectorXd v = Eigen::VectorXd::Random(model.nv);
  const Eigen::VectorXd tau = Eigen::VectorXd::Random(model.nv);

  Data data(model);
  std::reference_wrapper<Data> data_ref = data;

  ConstraintModelVector constraint_models;
  ConstraintDataVector constraint_datas;

  const std::string RF_name = "rleg6_joint";
  const JointIndex RF_id = model.getJointId(RF_name);

  const Model::IndexVector & RF_support = model.supports[RF_id];
  const Model::IndexVector active_joint_ids(RF_support.begin() + 1, RF_support.end());

  JointFrictionConstraintModel constraint_model(model, active_joint_ids);

  constraint_models.push_back(constraint_model);
  constraint_datas.push_back(constraint_model.createData());
  std::reference_wrapper<ConstraintModelVector> constraint_models_ref = constraint_models;
  std::reference_wrapper<ConstraintDataVector> constraint_datas_ref = constraint_datas;

  const double damping_value = 1e-4;

  // Test operator *
  {
    test_apply_on_the_right(
      model_ref, data_ref, constraint_models_ref, constraint_datas_ref, q_neutral, damping_value);
  } // End: Test operator *

  // Test solveInPlace
  {
    test_solve_in_place(
      model_ref, data_ref, constraint_models_ref, constraint_datas_ref, q_neutral, damping_value);
  }
}

BOOST_AUTO_TEST_CASE(general_test_joint_limit_constraint)
{
  typedef JointLimitConstraintModelTpl<double> ConstraintModel;
  typedef DelassusOperatorRigidBodySystemsTpl<
    double, 0, JointCollectionDefaultTpl, ConstraintModel, std::reference_wrapper>
    DelassusOperatorRigidBodyReferenceWrapper;
  typedef
    typename DelassusOperatorRigidBodyReferenceWrapper::ConstraintModelVector ConstraintModelVector;
  typedef
    typename DelassusOperatorRigidBodyReferenceWrapper::ConstraintDataVector ConstraintDataVector;

  Model model;
  std::reference_wrapper<Model> model_ref = model;

  buildModels::humanoidRandom(model, true);
  model.lowerPositionLimit.tail(model.nq - 7).fill(-1.);
  model.upperPositionLimit.tail(model.nq - 7).fill(+1.);

  const Eigen::VectorXd q_neutral = neutral(model);
  const Eigen::VectorXd v = Eigen::VectorXd::Random(model.nv);
  const Eigen::VectorXd tau = Eigen::VectorXd::Random(model.nv);

  Data data(model);
  std::reference_wrapper<Data> data_ref = data;

  ConstraintModelVector constraint_models;
  ConstraintDataVector constraint_datas;

  const JointIndex last_joint_id = Model::JointIndex(model.njoints);

  Model::IndexVector activable_joint_ids;
  for (Model::JointIndex i = 2 /* skip free_flyer*/; i < last_joint_id; ++i)
  {
    activable_joint_ids.push_back(i);

    const auto & jmodel = model.joints[i];
    const int nq = jmodel.nq();
    const auto has_configuration_limit = jmodel.hasConfigurationLimit();
    for (size_t k = 0; k < size_t(nq); ++k)
    {
      BOOST_CHECK(has_configuration_limit[k] == true);
    }
  }

  const Eigen::VectorXd q = model.lowerPositionLimit;
  data.q_in = q;
  {
    ConstraintModel constraint_model(model, activable_joint_ids);
    BOOST_CHECK(constraint_model.residualSize(MaximalSelection()) == 2 * (model.nv - 6));
    // when creating a joint limit constraint, all joint limits should be activated by default:
    BOOST_CHECK(constraint_model.residualSize() == 2 * (model.nv - 6));

    constraint_model.makeSelectionFilteredByLimitProximity(q);
    BOOST_CHECK(constraint_model.residualSize() == (model.nv - 6));

    constraint_models.push_back(constraint_model);
    constraint_datas.push_back(constraint_model.createData());
  }
  auto & constraint_model = constraint_models.back();

  // simple size check
  calc(model, data, constraint_models, constraint_datas);
  BOOST_CHECK(constraint_model.residualSize() == model.nv - 6);

  std::reference_wrapper<ConstraintModelVector> constraint_models_ref = constraint_models;
  std::reference_wrapper<ConstraintDataVector> constraint_datas_ref = constraint_datas;

  const double damping_value = 1e-4;

  // Test operator *
  {
    test_apply_on_the_right(
      model_ref, data_ref, constraint_models_ref, constraint_datas_ref, q_neutral, damping_value);
  } // End: Test operator *

  // Test solveInPlace
  {
    test_solve_in_place(
      model_ref, data_ref, constraint_models_ref, constraint_datas_ref, q_neutral, damping_value);
  }
}

BOOST_AUTO_TEST_CASE(general_test_constraint_generic)
{
  typedef JointFrictionConstraintModelTpl<double> JointFrictionConstraintModel;
  typedef DelassusOperatorRigidBodySystemsTpl<
    double, 0, JointCollectionDefaultTpl, ConstraintModel, std::reference_wrapper>
    DelassusOperatorRigidBodyReferenceWrapper;
  typedef
    typename DelassusOperatorRigidBodyReferenceWrapper::ConstraintModelVector ConstraintModelVector;
  typedef
    typename DelassusOperatorRigidBodyReferenceWrapper::ConstraintDataVector ConstraintDataVector;

  Model model;
  std::reference_wrapper<Model> model_ref = model;

  buildModels::humanoidRandom(model, true);

  const Eigen::VectorXd q_neutral = neutral(model);
  const Eigen::VectorXd v = Eigen::VectorXd::Random(model.nv);
  const Eigen::VectorXd tau = Eigen::VectorXd::Random(model.nv);

  Data data(model);
  std::reference_wrapper<Data> data_ref = data;

  ConstraintModelVector constraint_models;
  ConstraintDataVector constraint_datas;

  const std::string RF_name = "rleg6_joint";
  const JointIndex RF_id = model.getJointId(RF_name);

  const Model::IndexVector & RF_support = model.supports[RF_id];
  const Model::IndexVector active_joint_ids(RF_support.begin() + 1, RF_support.end());

  JointFrictionConstraintModel constraint_model(model, active_joint_ids);

  constraint_models.push_back(constraint_model);
  constraint_datas.push_back(constraint_model.createData());
  std::reference_wrapper<ConstraintModelVector> constraint_models_ref = constraint_models;
  std::reference_wrapper<ConstraintDataVector> constraint_datas_ref = constraint_datas;

  const double damping_value = 1e-4;

  // Test operator *
  {
    test_apply_on_the_right(
      model_ref, data_ref, constraint_models_ref, constraint_datas_ref, q_neutral, damping_value);
  } // End: Test operator *

  // Test solveInPlace
  {
    test_solve_in_place(
      model_ref, data_ref, constraint_models_ref, constraint_datas_ref, q_neutral, damping_value);
  }
}

BOOST_AUTO_TEST_CASE(general_test_no_constraints)
{
  typedef PointContactConstraintModelTpl<double> ConstraintModel;
  typedef DelassusOperatorRigidBodySystemsTpl<
    double, 0, JointCollectionDefaultTpl, ConstraintModel, std::reference_wrapper>
    DelassusOperatorRigidBodyReferenceWrapper;
  typedef
    typename DelassusOperatorRigidBodyReferenceWrapper::ConstraintModelVector ConstraintModelVector;
  typedef
    typename DelassusOperatorRigidBodyReferenceWrapper::ConstraintDataVector ConstraintDataVector;

  Model model;
  std::reference_wrapper<Model> model_ref = model;
  buildModels::humanoidRandom(model, true);
  model.gravity.setZero();

  const Eigen::VectorXd q_neutral = neutral(model);
  const Eigen::VectorXd v = Eigen::VectorXd::Random(model.nv);
  const Eigen::VectorXd tau = Eigen::VectorXd::Random(model.nv);

  Data data(model), data_gt(model);
  std::reference_wrapper<Data> data_ref = data;

  ConstraintModelVector constraint_models;
  ConstraintDataVector constraint_datas;

  ConstraintDataVector constraint_datas_gt = constraint_datas;

  computeJointJacobians(model, data, q_neutral);
  data.q_in = q_neutral;
  calc(model, data, constraint_models, constraint_datas);
  computeJointJacobians(model, data_gt, q_neutral);
  data_gt.q_in = q_neutral;
  calc(model, data_gt, constraint_models, constraint_datas_gt);

  std::reference_wrapper<ConstraintModelVector> constraint_models_ref = constraint_models;
  std::reference_wrapper<ConstraintDataVector> constraint_datas_ref = constraint_datas;

  const double min_damping_value = 1e-4;

  DelassusOperatorRigidBodyReferenceWrapper delassus_operator(
    model_ref, data_ref, constraint_models_ref, constraint_datas_ref, min_damping_value);

  // Test solveInPlace
  {
    const Eigen::Index col_id = 7;
    const Eigen::VectorXd rhs = Eigen::VectorXd::Unit(model.nv, col_id);
    Eigen::VectorXd res = rhs;

    const double mu_inv = min_damping_value;
    const double mu = 1. / mu_inv;

    delassus_operator.updateDamping(mu);
    delassus_operator.updateCompliance(0);
    delassus_operator.compute();

    delassus_operator.getAugmentedMassMatrixOperator().solveInPlace(res);

    Data data_crba(model);
    Eigen::MatrixXd M = crba(model, data_crba, q_neutral, Convention::WORLD);
    make_symmetric(M);
    const Eigen::MatrixXd M_inv = M.inverse();
    BOOST_CHECK(data.J.isApprox(data_crba.J));

    Data data_aba(model);
    const auto res_ref =
      aba(model, data_aba, q_neutral, Eigen::VectorXd::Zero(model.nv), rhs, Convention::WORLD);

    BOOST_CHECK(data.J.isApprox(data_aba.J));
    for (Model::JointIndex joint_id = 1; joint_id < Model::JointIndex(model.njoints); ++joint_id)
    {
      BOOST_CHECK(data.liMi[joint_id].isApprox(data_aba.liMi[joint_id]));
      BOOST_CHECK(data.oMi[joint_id].isApprox(data_aba.oMi[joint_id]));
      BOOST_CHECK(data.oYaba_augmented[joint_id].isApprox(data_aba.oYaba[joint_id]));

      BOOST_CHECK(data.joints_augmented[joint_id].U().isApprox(data_aba.joints[joint_id].U()));
      BOOST_CHECK(
        data.joints_augmented[joint_id].Dinv().isApprox(data_aba.joints[joint_id].Dinv()));
      BOOST_CHECK(
        data.joints_augmented[joint_id].UDinv().isApprox(data_aba.joints[joint_id].UDinv()));
    }

    BOOST_CHECK(res.isApprox(M_inv.col(col_id)));
    BOOST_CHECK(res.isApprox(res_ref));

    for (Eigen::Index col_id = 0; col_id < model.nv; ++col_id)
    {
      const Eigen::VectorXd rhs = Eigen::VectorXd::Unit(model.nv, col_id);
      const auto res_ref =
        aba(model, data_aba, q_neutral, Eigen::VectorXd::Zero(model.nv), rhs, Convention::WORLD);
      const auto res_ref2 = (M_inv * rhs).eval();
      BOOST_CHECK(res_ref.isApprox(res_ref2));

      Eigen::VectorXd res = rhs;
      delassus_operator.getAugmentedMassMatrixOperator().solveInPlace(res);
      BOOST_CHECK(res.isApprox(res_ref));
      BOOST_CHECK(res.isApprox(res_ref2));
    }

    // Check joint_elimination_order
    const auto & joint_elimination_order = data.joint_elimination_order;
    for (auto it = joint_elimination_order.begin(); it != joint_elimination_order.end(); ++it)
    {
      const auto joint_id = *it;
      const auto & joint_support = model.supports[joint_id];
      for (const auto support_joint : joint_support)
      {
        bool is_after = std::find(it, joint_elimination_order.end(), support_joint)
                        != joint_elimination_order.end();
        BOOST_CHECK(is_after || support_joint == 0);
      }
    }
  }
}

BOOST_AUTO_TEST_CASE(test_copy)
{
  typedef JointFrictionConstraintModelTpl<double> ConstraintModel;
  typedef DelassusOperatorRigidBodySystemsTpl<
    double, 0, JointCollectionDefaultTpl, ConstraintModel, std::reference_wrapper>
    DelassusOperator;
  typedef typename DelassusOperator::ConstraintModelVector ConstraintModelVector;
  typedef typename DelassusOperator::ConstraintDataVector ConstraintDataVector;

  Model model;
  buildModels::manipulator(model);
  model.lowerDryFrictionLimit.setConstant(-1.0);
  model.upperDryFrictionLimit.setConstant(1.0);
  Data data(model);

  ConstraintModelVector constraint_models;
  ConstraintDataVector constraint_datas;
  JointFrictionConstraintModel::JointIndexVector active_joints;
  for (size_t i = 1; i < model.joints.size(); ++i)
    active_joints.push_back(model.joints[i].id());
  JointFrictionConstraintModel joints_friction(model, active_joints);
  constraint_models.push_back(joints_friction);
  constraint_datas.push_back(joints_friction.createData());

  const Eigen::VectorXd q = neutral(model);
  computeJointJacobians(model, data, q);
  data.q_in = q;
  calc(model, data, constraint_models, constraint_datas);

  const double compliance_value = 1e-2;
  DelassusOperator delassus(
    pinocchio::internal::helper::make_ref(model), pinocchio::internal::helper::make_ref(data),
    pinocchio::internal::helper::make_ref(constraint_models),
    pinocchio::internal::helper::make_ref(constraint_datas));
  delassus.updateCompliance(compliance_value);
  delassus.compute();

  // copy constructor: maps must point to the copy's own storage
  DelassusOperator delassus_copy(delassus);
  BOOST_CHECK(access(delassus_copy).m_compliance.data() != access(delassus).m_compliance.data());
  BOOST_CHECK(access(delassus_copy).m_compliance.isApprox(access(delassus).m_compliance));

  // copy constructor: independence — modify original, copy unchanged
  const double new_compliance_value = 5e-3;
  delassus.updateCompliance(new_compliance_value);
  BOOST_CHECK(access(delassus_copy).m_compliance.isConstant(compliance_value, 0));

  // copy assignment: maps must point to the assigned object's own storage
  DelassusOperator delassus_assigned(
    pinocchio::internal::helper::make_ref(model), pinocchio::internal::helper::make_ref(data),
    pinocchio::internal::helper::make_ref(constraint_models),
    pinocchio::internal::helper::make_ref(constraint_datas));
  delassus_assigned = delassus;
  BOOST_CHECK(
    access(delassus_assigned).m_compliance.data() != access(delassus).m_compliance.data());
  BOOST_CHECK(access(delassus_assigned).m_compliance.isApprox(access(delassus).m_compliance));

  // copy assignment: independence — modify original, assigned copy unchanged
  delassus.updateCompliance(compliance_value);
  BOOST_CHECK(access(delassus_assigned).m_compliance.isConstant(new_compliance_value, 0));
}
