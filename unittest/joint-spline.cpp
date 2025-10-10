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
  std::cout << ctrlFrames.size() << std::endl;

  JointModelSpline jmodel(ctrlFrames, 1);
  JointDataSpline jdata = jmodel.createData();

  jmodel.setIndexes(0, 0, 0);

  Eigen::VectorXd q(Eigen::VectorXd::Zero(1));

  // -------
  q << 0.2;

  jmodel.calc(jdata, q);

  BOOST_CHECK(expected_configuration.rotation().isApprox(jdata.M.rotation(), 1e-12));
  BOOST_CHECK(expected_configuration.translation().isApprox(jdata.M.translation(), 1e-12));

  Eigen::VectorXd q_dot(Eigen::VectorXd::Zero(1));

  // -------
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

// BOOST_AUTO_TEST_CASE(vsSphericalXYZ)
// {

//   using namespace pinocchio;
//   typedef SE3::Vector3 Vector3;
//   typedef SE3::Matrix3 Matrix3;

//   Inertia inertia(1., Vector3(0.5, 0., 0.0), Matrix3::Identity());

//   PINOCCHIO_ALIGNED_STD_VECTOR(SE3) ctrlFrames;
//   ctrlFrames.push_back(SE3::Identity());

//   Eigen::Matrix3d R1;
//   R1 << 0.866025, 0, 0.5,
//         0,        1, 0       ,
//        -0.5, 0, 0.866025;
//   // R1 << 0.770151, -0.219024,  0.599079,
//   //      0.420735,  0.880347, -0.219024,
//   //     -0.479426,  0.420735,  0.770151;
//   ctrlFrames.push_back(SE3(R1, Eigen::Vector3d::Zero()));

//   // 0.5
//   Eigen::Matrix3d R2;

//   R2 << 0.707107, 0, 0.707107,
//         0,        1, 0       ,
//        -0.707107, 0, 0.707107;
//   // R1 << 0.770151, -0.219024,  0.599079,
//   //      0.420735,  0.880347, -0.219024,
//   //     -0.479426,  0.420735,  0.770151;
//   ctrlFrames.push_back(SE3(R2, Eigen::Vector3d::Zero()));

//   Eigen::Matrix3d R3;

//   R3 << 0.5, 0, 0.866025,
//         0,        1, 0       ,
//        -0.866025, 0, 0.5;
//   // R1 << 0.770151, -0.219024,  0.599079,
//   //      0.420735,  0.880347, -0.219024,
//   //     -0.479426,  0.420735,  0.770151;
//   ctrlFrames.push_back(SE3(R3, Eigen::Vector3d::Zero()));

//   // 1
//   Eigen::Matrix3d R4;
//   R4 << 0, 0, 1, 0, 1, 0, -1, 0, 0;
//   // R2 <<  0.291927, -0.072075,  0.953721,
//   //       0.454649,  0.88775,  -0.072075,
//   //      -0.841471,  0.454649,  0.291927;
//   ctrlFrames.push_back(SE3(R4, Eigen::Vector3d::Zero()));

//   Model modelSpline;
//   addJointAndBody(modelSpline, JointModelSpline(ctrlFrames), 0, SE3::Identity(), "spline_joint",
//   inertia); Data dataSpline(modelSpline);

//   Model modelSph;
//   addJointAndBody(modelSph, JointModelSphericalZYX(), 0, SE3::Identity(), "sph_joint", inertia);
//   Data dataSph(modelSph);

//   Eigen::VectorXd qSpline(Eigen::VectorXd::Zero(1));
//   qSpline << 0.5;

//   Eigen::Vector3d qSph;
//   qSph << 0, M_PI / 3, 0;

//   forwardKinematics(modelSpline, dataSpline, qSpline);
//   forwardKinematics(modelSph, dataSph, qSph);

//   BOOST_CHECK(dataSpline.oMi[1].isApprox(dataSph.oMi[1]));
//   std::cout << dataSpline.oMi[1] << std::endl;
//   std::cout << dataSph.oMi[1] << std::endl;
// }

BOOST_AUTO_TEST_SUITE_END()
