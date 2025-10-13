//
// Copyright (c) 2025 INRIA
//

#include "pinocchio/math/fwd.hpp"
#include "pinocchio/multibody/joint/joints.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/aba.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/compute-all-terms.hpp"

#include <boost/test/unit_test.hpp>
#include <iostream>

using namespace pinocchio;

template<typename D>
void addJointAndBody(
  Model & model,
  const JointModelBase<D> & jmodel,
  const Model::JointIndex parent_id,
  const SE3 & joint_placement,
  const std::string & joint_name,
  const Inertia & Y)
{
  Model::JointIndex idx;

  idx = model.addJoint(parent_id, jmodel, joint_placement, joint_name);
  model.appendBodyToJoint(idx, Y);
}

BOOST_AUTO_TEST_SUITE(JointSpline)

BOOST_AUTO_TEST_CASE(vsPrismatic)
{
  using namespace pinocchio;
  typedef SE3::Vector3 Vector3;
  typedef SE3::Matrix3 Matrix3;

  Motion expected_v_J(Motion::Zero());
  Motion expected_c_J(Motion::Zero());

  SE3 expected_configuration(SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0., 0., 0.2)));

  PINOCCHIO_ALIGNED_STD_VECTOR(SE3) ctrlFrames;
  ctrlFrames.push_back(SE3::Identity());
  ctrlFrames.push_back(SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0., 0., 1.)));

  JointModelSpline jmodel(ctrlFrames, 1);
  JointDataSpline jdata = jmodel.createData();

  jmodel.setIndexes(0, 0, 0);

  Eigen::VectorXd q(Eigen::VectorXd::Zero(1));

  // -------
  q << 0.2;

  jmodel.calc(jdata, q);

  BOOST_CHECK(expected_configuration.rotation().isApprox(jdata.M.rotation(), 1e-12));
  BOOST_CHECK(expected_configuration.translation().isApprox(jdata.M.translation(), 1e-12));

  // -------
  Eigen::VectorXd q_dot(Eigen::VectorXd::Zero(1));
  q << 0.3;
  q_dot << 0.4;

  jmodel.calc(jdata, q, q_dot);

  expected_configuration.translation() << 0, 0, 0.3;
  expected_v_J.linear() << 0., 0., 0.4;

  BOOST_CHECK(expected_configuration.rotation().isApprox(jdata.M.rotation(), 1e-12));
  BOOST_CHECK(expected_configuration.translation().isApprox(jdata.M.translation(), 1e-12));
  BOOST_CHECK(expected_v_J.toVector().isApprox(((Motion)jdata.v).toVector(), 1e-12));
  BOOST_CHECK(expected_c_J.isApprox((Motion)jdata.c, 1e-12));
}

BOOST_AUTO_TEST_CASE(vsFiniteDiff)
{
  using namespace pinocchio;
  typedef SE3::Vector3 Vector3;
  typedef SE3::Matrix3 Matrix3;

  typedef typename JointModelSpline::ConfigVector_t CV;
  typedef typename JointModelSpline::TangentVector_t TV;
  typedef typename LieGroup<JointModelSpline>::type LieGroupType;

  PINOCCHIO_ALIGNED_STD_VECTOR(SE3) ctrlFrames;
  ctrlFrames.push_back(SE3::Identity());
  ctrlFrames.push_back(SE3::Random());
  ctrlFrames.push_back(SE3::Random());
  ctrlFrames.push_back(SE3::Random());

  JointModelSpline jmodel(ctrlFrames);
  JointDataSpline jdata = jmodel.createData();

  jmodel.setIndexes(0, 0, 0);

  double eps = 1e-8;
  CV q_ref(1);
  q_ref[0] = 0.6;
  CV q(q_ref);

  const Eigen::DenseIndex nv = jdata.S.nv();
  TV q_dot(nv);
  TV q_dot_ref(nv);
  q_dot.setZero();

  q_dot[0] = eps;
  q_dot_ref[0] = 0.3;

  q = LieGroupType().integrate(q_ref, q_dot);

  {
    // Check S
    jmodel.calc(jdata, q_ref);
    SE3 M_ref(jdata.M);
    Eigen::Matrix<double, 6, JointModelSpline::NV> S(6, JointModelSpline::NV),
      S_ref(jdata.S.matrix());

    jmodel.calc(jdata, q);
    SE3 M_ = jdata.M;

    S.col(0) = log6(M_ref.inverse() * M_).toVector();
    S.col(0) /= eps;

    BOOST_CHECK(S.isApprox(S_ref, eps * 1e1));
  }
  // Check bias
  {
    jmodel.calc(jdata, q_ref, q_dot_ref);
    const Motion & c_ref = jdata.c;
    Eigen::Matrix<double, 6, JointModelSpline::NV> S_ref(jdata.S.matrix());

    jmodel.calc(jdata, q);
    Eigen::Matrix<double, 6, JointModelSpline::NV> S_(jdata.S.matrix());

    Motion dSdq_fd((S_ - S_ref) / eps);
    Motion c_fd = dSdq_fd * q_dot_ref[0];

    BOOST_CHECK(c_ref.isApprox(c_fd, eps * 1e1));
  }
}

BOOST_AUTO_TEST_SUITE_END()
