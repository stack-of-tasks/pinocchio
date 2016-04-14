//
// Copyright (c) 2015 CNRS
// Copyright (c) 2015 Wandercraft, 86 rue de Paris 91400 Orsay, France.
//
// This file is part of Pinocchio
// Pinocchio is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// Pinocchio is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// General Lesser Public License for more details. You should have
// received a copy of the GNU Lesser General Public License along with
// Pinocchio If not, see
// <http://www.gnu.org/licenses/>.

#include <iostream>

#include "pinocchio/math/fwd.hpp"
#include "pinocchio/spatial/force.hpp"
#include "pinocchio/spatial/motion.hpp"
#include "pinocchio/spatial/se3.hpp"
#include "pinocchio/spatial/inertia.hpp"
#include "pinocchio/multibody/joint/joint-revolute.hpp"
#include "pinocchio/multibody/joint/joint-revolute-unaligned.hpp"
#include "pinocchio/multibody/joint/joint-spherical.hpp"
#include "pinocchio/multibody/joint/joint-spherical-ZYX.hpp"
#include "pinocchio/multibody/joint/joint-prismatic.hpp"
#include "pinocchio/multibody/joint/joint-prismatic-unaligned.hpp"
#include "pinocchio/multibody/joint/joint-translation.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/aba.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/compute-all-terms.hpp"

#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE JointsTest
#include <boost/test/unit_test.hpp>

//#define VERBOSE

template <typename JoinData_t>
void printOutJointData (
#ifdef VERBOSE
  const Eigen::VectorXd & q,
  const Eigen::VectorXd & q_dot,
  const JoinData_t & joint_data
#else
  const Eigen::VectorXd & ,
  const Eigen::VectorXd & ,
  const JoinData_t & 
#endif
                        )
{
  using namespace std;
  using namespace se3;

#ifdef VERBOSE
  cout << "q: " << q.transpose () << endl;
  cout << "q_dot: " << q_dot.transpose () << endl;
  cout << "Joint configuration:" << endl << joint_data.M << endl;
  cout << "v_J:\n" << (Motion) joint_data.v << endl;
  cout << "c_J:\n" << (Motion) joint_data.c << endl;
#endif
}


BOOST_AUTO_TEST_SUITE (JointRevoluteUnaligned)

BOOST_AUTO_TEST_CASE (vsRX)
{
  using namespace se3;
  typedef Eigen::Matrix <double, 3, 1> Vector3;
  typedef Eigen::Matrix <double, 6, 1> Vector6;
  typedef Eigen::Matrix <double, 3, 3> Matrix3;

  Eigen::Vector3d axis;
  axis << 1.0, 0.0, 0.0;

  Model modelRX, modelRevoluteUnaligned;

  Inertia inertia (1., Vector3 (0.5, 0., 0.0), Matrix3::Identity ());
  SE3 pos(1); pos.translation() = SE3::Linear_t(1.,0.,0.);

  JointModelRevoluteUnaligned joint_model_RU(axis);
  modelRX.addBody (0, JointModelRX (), pos, inertia, "rx");
  modelRevoluteUnaligned.addBody(0, joint_model_RU ,pos, inertia, "revolute-unaligne");

  Data dataRX(modelRX);
  Data dataRevoluteUnaligned(modelRevoluteUnaligned);


  Eigen::VectorXd q = Eigen::VectorXd::Ones (modelRX.nq);
  Eigen::VectorXd v = Eigen::VectorXd::Ones (modelRX.nv);
  Eigen::VectorXd tauRX = Eigen::VectorXd::Ones (modelRX.nv);       Eigen::VectorXd tauRevoluteUnaligned = Eigen::VectorXd::Ones (modelRevoluteUnaligned.nv);
  Eigen::VectorXd aRX = Eigen::VectorXd::Ones (modelRX.nv);         Eigen::VectorXd aRevoluteUnaligned(aRX);
  


  forwardKinematics(modelRX, dataRX, q, v);
  forwardKinematics(modelRevoluteUnaligned, dataRevoluteUnaligned, q, v);

  computeAllTerms(modelRX, dataRX, q, v);
  computeAllTerms(modelRevoluteUnaligned, dataRevoluteUnaligned, q, v);

  BOOST_CHECK(dataRevoluteUnaligned.oMi[1].isApprox(dataRX.oMi[1]));
  BOOST_CHECK(dataRevoluteUnaligned.liMi[1].isApprox(dataRX.liMi[1]));
  BOOST_CHECK(dataRevoluteUnaligned.Ycrb[1].matrix().isApprox(dataRX.Ycrb[1].matrix()));
  BOOST_CHECK(dataRevoluteUnaligned.f[1].toVector().isApprox(dataRX.f[1].toVector()));
  
  BOOST_CHECK(dataRevoluteUnaligned.nle.isApprox(dataRX.nle));
  BOOST_CHECK(dataRevoluteUnaligned.com[0].isApprox(dataRX.com[0]));



  // InverseDynamics == rnea
  tauRX = rnea(modelRX, dataRX, q, v, aRX);
  tauRevoluteUnaligned = rnea(modelRevoluteUnaligned, dataRevoluteUnaligned, q, v, aRevoluteUnaligned);

  BOOST_CHECK(tauRX.isApprox(tauRevoluteUnaligned));

  // ForwardDynamics == aba
  Eigen::VectorXd aAbaRX = aba(modelRX,dataRX, q, v, tauRX);
  Eigen::VectorXd aAbaRevoluteUnaligned = aba(modelRevoluteUnaligned,dataRevoluteUnaligned, q, v, tauRevoluteUnaligned);


  BOOST_CHECK(aAbaRX.isApprox(aAbaRevoluteUnaligned));

  // crba
  crba(modelRX, dataRX,q);
  crba(modelRevoluteUnaligned, dataRevoluteUnaligned, q);

  BOOST_CHECK(dataRX.M.isApprox(dataRevoluteUnaligned.M));
   
  // Jacobian
  Eigen::Matrix<double, 6, Eigen::Dynamic> jacobianRX;jacobianRX.resize(6,1); jacobianRX.setZero();
  Eigen::Matrix<double, 6, Eigen::Dynamic> jacobianRevoluteUnaligned;jacobianRevoluteUnaligned.resize(6,1);jacobianRevoluteUnaligned.setZero();
  computeJacobians(modelRX, dataRX, q);
  computeJacobians(modelRevoluteUnaligned, dataRevoluteUnaligned, q);
  getJacobian<true>(modelRX, dataRX, 1, jacobianRX);
  getJacobian<true>(modelRevoluteUnaligned, dataRevoluteUnaligned, 1, jacobianRevoluteUnaligned);


  BOOST_CHECK(jacobianRX.isApprox(jacobianRevoluteUnaligned));


}
BOOST_AUTO_TEST_SUITE_END ()

BOOST_AUTO_TEST_SUITE (JointPrismaticUnaligned)

BOOST_AUTO_TEST_CASE (vsPX)
{
  using namespace se3;
  typedef Eigen::Matrix <double, 3, 1> Vector3;
  typedef Eigen::Matrix <double, 6, 1> Vector6;
  typedef Eigen::Matrix <double, 3, 3> Matrix3;

  Eigen::Vector3d axis;
  axis << 1.0, 0.0, 0.0;

  Model modelPX, modelPrismaticUnaligned;

  Inertia inertia (1., Vector3 (0.5, 0., 0.0), Matrix3::Identity ());
  SE3 pos(1); pos.translation() = SE3::Linear_t(1.,0.,0.);

  JointModelPrismaticUnaligned joint_model_PU(axis);
  modelPX.addBody (0, JointModelPX (), pos, inertia, "px");
  modelPrismaticUnaligned.addBody(0, joint_model_PU ,pos, inertia, "prismatic-unaligne");

  Data dataPX(modelPX);
  Data dataPrismaticUnaligned(modelPrismaticUnaligned);


  Eigen::VectorXd q = Eigen::VectorXd::Ones (modelPX.nq);
  Eigen::VectorXd v = Eigen::VectorXd::Ones (modelPX.nv);
  Eigen::VectorXd tauPX = Eigen::VectorXd::Ones (modelPX.nv);       Eigen::VectorXd tauPrismaticUnaligned = Eigen::VectorXd::Ones (modelPrismaticUnaligned.nv);
  Eigen::VectorXd aPX = Eigen::VectorXd::Ones (modelPX.nv);         Eigen::VectorXd aPrismaticUnaligned(aPX);
  


  forwardKinematics(modelPX, dataPX, q, v);
  forwardKinematics(modelPrismaticUnaligned, dataPrismaticUnaligned, q, v);

  computeAllTerms(modelPX, dataPX, q, v);
  computeAllTerms(modelPrismaticUnaligned, dataPrismaticUnaligned, q, v);

  BOOST_CHECK(dataPrismaticUnaligned.oMi[1].isApprox(dataPX.oMi[1]));
  BOOST_CHECK(dataPrismaticUnaligned.liMi[1].isApprox(dataPX.liMi[1]));
  BOOST_CHECK(dataPrismaticUnaligned.Ycrb[1].matrix().isApprox(dataPX.Ycrb[1].matrix()));
  BOOST_CHECK(dataPrismaticUnaligned.f[1].toVector().isApprox(dataPX.f[1].toVector()));
  
  BOOST_CHECK(dataPrismaticUnaligned.nle.isApprox(dataPX.nle));
  BOOST_CHECK(dataPrismaticUnaligned.com[0].isApprox(dataPX.com[0]));



  // InverseDynamics == rnea
  tauPX = rnea(modelPX, dataPX, q, v, aPX);
  tauPrismaticUnaligned = rnea(modelPrismaticUnaligned, dataPrismaticUnaligned, q, v, aPrismaticUnaligned);

  BOOST_CHECK(tauPX.isApprox(tauPrismaticUnaligned));

  // ForwardDynamics == aba
  Eigen::VectorXd aAbaPX = aba(modelPX,dataPX, q, v, tauPX);
  Eigen::VectorXd aAbaPrismaticUnaligned = aba(modelPrismaticUnaligned,dataPrismaticUnaligned, q, v, tauPrismaticUnaligned);


  BOOST_CHECK(aAbaPX.isApprox(aAbaPrismaticUnaligned));

  // crba
  crba(modelPX, dataPX,q);
  crba(modelPrismaticUnaligned, dataPrismaticUnaligned, q);

  BOOST_CHECK(dataPX.M.isApprox(dataPrismaticUnaligned.M));
   
  // Jacobian
  Eigen::Matrix<double, 6, Eigen::Dynamic> jacobianPX;jacobianPX.resize(6,1); jacobianPX.setZero();
  Eigen::Matrix<double, 6, Eigen::Dynamic> jacobianPrismaticUnaligned;jacobianPrismaticUnaligned.resize(6,1);jacobianPrismaticUnaligned.setZero();
  computeJacobians(modelPX, dataPX, q);
  computeJacobians(modelPrismaticUnaligned, dataPrismaticUnaligned, q);
  getJacobian<true>(modelPX, dataPX, 1, jacobianPX);
  getJacobian<true>(modelPrismaticUnaligned, dataPrismaticUnaligned, 1, jacobianPrismaticUnaligned);


  BOOST_CHECK(jacobianPX.isApprox(jacobianPrismaticUnaligned));


}
BOOST_AUTO_TEST_SUITE_END ()


BOOST_AUTO_TEST_SUITE ( JointSphericalZYX )

BOOST_AUTO_TEST_CASE ( test_kinematics )
{
  using namespace se3;

  typedef Motion::Vector3 Vector3;

  Motion expected_v_J (Motion::Zero ());
  Motion expected_c_J (Motion::Zero ());

  SE3 expected_configuration (SE3::Identity ());

  JointDataSphericalZYX joint_data;
  JointModelSphericalZYX joint_model;

  joint_model.setIndexes (0, 0, 0);

  Vector3 q (Vector3::Zero());
  Vector3 q_dot (Vector3::Zero());

  // -------
  q = Vector3 (0., 0., 0.);
  q_dot = Vector3 (0., 0., 0.);

  joint_model.calc (joint_data, q, q_dot);

  printOutJointData <JointDataSphericalZYX> (q, q_dot, joint_data);

  BOOST_CHECK (expected_configuration.rotation ().isApprox(joint_data.M.rotation(), 1e-12));
  BOOST_CHECK (expected_configuration.translation ().isApprox(joint_data.M.translation (), 1e-12));
  BOOST_CHECK (expected_v_J.toVector ().isApprox(((Motion) joint_data.v).toVector(), 1e-12));
  BOOST_CHECK (expected_c_J.toVector ().isApprox(((Motion) joint_data.c).toVector(), 1e-12));

  // -------
  q = Vector3 (1., 0., 0.);
  q_dot = Vector3 (1., 0., 0.);

  joint_model.calc (joint_data, q, q_dot);

  printOutJointData <JointDataSphericalZYX> (q, q_dot, joint_data);

  expected_configuration.rotation ().transpose () <<
  0.54030230586814,  0.8414709848079,               -0,
  -0.8414709848079, 0.54030230586814,                0,
  0,                0,               1;

  expected_v_J.angular () << 0., 0., 1.;

  BOOST_CHECK (expected_configuration.rotation ().isApprox(joint_data.M.rotation(), 1e-12));
  BOOST_CHECK (expected_configuration.translation ().isApprox(joint_data.M.translation (), 1e-12));
  BOOST_CHECK (expected_v_J.toVector ().isApprox(((Motion) joint_data.v).toVector(), 1e-12));
  BOOST_CHECK (expected_c_J.toVector ().isApprox(((Motion) joint_data.c).toVector(), 1e-12));

  // -------
  q = Vector3 (0., 1., 0.);
  q_dot = Vector3 (0., 1., 0.);

  joint_model.calc (joint_data, q, q_dot);

  printOutJointData <JointDataSphericalZYX> (q, q_dot, joint_data);

  expected_configuration.rotation ().transpose () <<
  0.54030230586814,                0, -0.8414709848079,
  0,                1,                0,
  0.8414709848079,                0, 0.54030230586814;

  expected_v_J.angular () << 0., 1., 0.;

  BOOST_CHECK (expected_configuration.rotation ().isApprox(joint_data.M.rotation(), 1e-12));
  BOOST_CHECK (expected_configuration.translation ().isApprox(joint_data.M.translation (), 1e-12));
  BOOST_CHECK (expected_v_J.toVector ().isApprox(((Motion) joint_data.v).toVector(), 1e-12));
  BOOST_CHECK (expected_c_J.toVector ().isApprox(((Motion) joint_data.c).toVector(), 1e-12));

  // -------
  q = Vector3 (0., 0., 1.);
  q_dot = Vector3 (0., 0., 1.);

  joint_model.calc (joint_data, q, q_dot);

  printOutJointData <JointDataSphericalZYX> (q, q_dot, joint_data);

  expected_configuration.rotation ().transpose () <<
  1,                0,               -0,
  0, 0.54030230586814,  0.8414709848079,
  0, -0.8414709848079, 0.54030230586814;

  expected_v_J.angular () << 1., 0., 0.;

  BOOST_CHECK (expected_configuration.rotation ().isApprox(joint_data.M.rotation(), 1e-12));
  BOOST_CHECK (expected_configuration.translation ().isApprox(joint_data.M.translation (), 1e-12));
  BOOST_CHECK (expected_v_J.toVector ().isApprox(((Motion) joint_data.v).toVector(), 1e-12));
  BOOST_CHECK (expected_c_J.toVector ().isApprox(((Motion) joint_data.c).toVector(), 1e-12));

  // -------
  q = Vector3 (1., 1., 1.);
  q_dot = Vector3 (1., 1., 1.);

  joint_model.calc (joint_data, q, q_dot);

  printOutJointData <JointDataSphericalZYX> (q, q_dot, joint_data);

  expected_configuration.rotation ().transpose () <<
  0.29192658172643,   0.45464871341284,   -0.8414709848079,
  -0.072075012795695,   0.88774981831738,   0.45464871341284,
  0.95372116649051, -0.072075012795695,   0.29192658172643;

  expected_v_J.angular () << 0.1585290151921,  0.99495101928098, -0.54954440308147;
  expected_c_J.angular () << -0.54030230586814,   -1.257617821355,  -1.4495997326938;

  BOOST_CHECK (expected_configuration.rotation ().isApprox(joint_data.M.rotation(), 1e-12));
  BOOST_CHECK (expected_configuration.translation ().isApprox(joint_data.M.translation (), 1e-12));
  BOOST_CHECK (expected_v_J.toVector ().isApprox(((Motion) joint_data.v).toVector(), 1e-12));
  BOOST_CHECK (expected_c_J.toVector ().isApprox(((Motion) joint_data.c).toVector(), 1e-12));

  // -------
  q = Vector3 (1., 1.5, 1.9);
  q_dot = Vector3 (2., 3., 1.);

  joint_model.calc (joint_data, q, q_dot);

  printOutJointData <JointDataSphericalZYX> (q, q_dot, joint_data);

  expected_configuration.rotation ().transpose () <<
  0.03821947317172,  0.059523302749877,  -0.99749498660405,
  0.78204612603915,   0.61961526601658,   0.06693862014091,
  0.62204752922718,   -0.7826454488138, -0.022868599288288;

  expected_v_J.angular () << -0.99498997320811, -0.83599146030869,  -2.8846374616388;
  expected_c_J.angular () << -0.42442321000622,  -8.5482150213859,   2.7708697933151;

  BOOST_CHECK (expected_configuration.rotation ().isApprox(joint_data.M.rotation(), 1e-12));
  BOOST_CHECK (expected_configuration.translation ().isApprox(joint_data.M.translation (), 1e-12));
  BOOST_CHECK (expected_v_J.toVector ().isApprox(((Motion) joint_data.v).toVector(), 1e-12));
  BOOST_CHECK (expected_c_J.toVector ().isApprox(((Motion) joint_data.c).toVector(), 1e-12));
}

BOOST_AUTO_TEST_CASE ( test_rnea )
{
  using namespace se3;
  typedef Eigen::Matrix <double, 3, 1> Vector3;
  typedef Eigen::Matrix <double, 3, 3> Matrix3;

  Model model;
  Inertia inertia (1., Vector3 (0.5, 0., 0.0), Matrix3::Identity ());

  model.addBody (model.getBodyId("universe"), JointModelSphericalZYX (), SE3::Identity (), inertia, "root");

  Data data (model);

  Eigen::VectorXd q = Eigen::VectorXd::Zero (model.nq);
  Eigen::VectorXd v = Eigen::VectorXd::Zero (model.nv);
  Eigen::VectorXd a = Eigen::VectorXd::Zero (model.nv);

  rnea (model, data, q, v, a);
  Vector3 tau_expected (0., -4.905, 0.);

  BOOST_CHECK (tau_expected.isApprox(data.tau, 1e-14));

  q = Eigen::VectorXd::Ones (model.nq);
  v = Eigen::VectorXd::Ones (model.nv);
  a = Eigen::VectorXd::Ones (model.nv);

  rnea (model, data, q, v, a);
  tau_expected << -0.53611600195085, -0.74621832606188, -0.38177329067604;

  BOOST_CHECK (tau_expected.isApprox(data.tau, 1e-12));

  q << 3, 2, 1;
  v = Eigen::VectorXd::Ones (model.nv);
  a = Eigen::VectorXd::Ones (model.nv);

  rnea (model, data, q, v, a);
  tau_expected << 0.73934458094049,  2.7804530848031, 0.50684940972146;

  BOOST_CHECK (tau_expected.isApprox(data.tau, 1e-12));
}

BOOST_AUTO_TEST_CASE ( test_crba )
{
  using namespace se3;
  using namespace std;
  typedef Eigen::Matrix <double, 3, 1> Vector3;
  typedef Eigen::Matrix <double, 3, 3> Matrix3;

  Model model;
  Inertia inertia (1., Vector3 (0.5, 0., 0.0), Matrix3::Identity ());

  model.addBody (model.getBodyId("universe"), JointModelSphericalZYX (), SE3::Identity (), inertia, "root");

  Data data (model);

  Eigen::VectorXd q (Eigen::VectorXd::Zero (model.nq));
  Eigen::MatrixXd M_expected (model.nv,model.nv);

  crba (model, data, q);
  M_expected <<
  1.25,    0,    0,
  0, 1.25,    0,
  0,    0,    1;

  BOOST_CHECK (M_expected.isApprox(data.M, 1e-14));

  q = Eigen::VectorXd::Ones (model.nq);

  crba (model, data, q);
  M_expected <<
  1.0729816454316, -5.5511151231258e-17,     -0.8414709848079,
  -5.5511151231258e-17,                 1.25,                    0,
  -0.8414709848079,                    0,                    1;

  BOOST_CHECK (M_expected.isApprox(data.M, 1e-12));

  q << 3, 2, 1;

  crba (model, data, q);
  M_expected <<
  1.043294547392, 2.7755575615629e-17,   -0.90929742682568,
  0,                1.25,                   0,
  -0.90929742682568,                   0,                  1;

  BOOST_CHECK (M_expected.isApprox(data.M, 1e-10));
}

BOOST_AUTO_TEST_SUITE_END ()


BOOST_AUTO_TEST_SUITE ( JointPrismatic )

BOOST_AUTO_TEST_CASE ( test_kinematics )
{
  using namespace se3;


  Motion expected_v_J (Motion::Zero ());
  Motion expected_c_J (Motion::Zero ());

  SE3 expected_configuration (SE3::Identity ());

  JointDataPX joint_data;
  JointModelPX joint_model;

  joint_model.setIndexes (0, 0, 0);

  Eigen::VectorXd q (Eigen::VectorXd::Zero (1));
  Eigen::VectorXd q_dot (Eigen::VectorXd::Zero (1));

  // -------
  q << 0. ;
  q_dot << 0.;

  joint_model.calc (joint_data, q, q_dot);

  printOutJointData <JointDataPX> (q, q_dot, joint_data);

  BOOST_CHECK (expected_configuration.rotation ().isApprox(joint_data.M.rotation(), 1e-12));
  BOOST_CHECK (expected_configuration.translation ().isApprox(joint_data.M.translation (), 1e-12));
  BOOST_CHECK (expected_v_J.toVector ().isApprox(((Motion) joint_data.v).toVector(), 1e-12));
  BOOST_CHECK (expected_c_J.toVector ().isApprox(((Motion) joint_data.c).toVector(), 1e-12));

  // -------
  q << 1.;
  q_dot << 1.;


  joint_model.calc (joint_data, q, q_dot);

  printOutJointData <JointDataPX> (q, q_dot, joint_data);

  expected_configuration.translation () << 1, 0, 0;

  expected_v_J.linear () << 1., 0., 0.;

  BOOST_CHECK (expected_configuration.rotation ().isApprox(joint_data.M.rotation(), 1e-12));
  BOOST_CHECK (expected_configuration.translation ().isApprox(joint_data.M.translation (), 1e-12));
  BOOST_CHECK (expected_v_J.toVector ().isApprox(((Motion) joint_data.v).toVector(), 1e-12));
  BOOST_CHECK (expected_c_J.toVector ().isApprox(((Motion) joint_data.c).toVector(), 1e-12));
}

BOOST_AUTO_TEST_CASE ( test_rnea )
{
  using namespace se3;
  typedef Eigen::Matrix <double, 3, 1> Vector3;
  typedef Eigen::Matrix <double, 3, 3> Matrix3;

  Model model;
  Inertia inertia (1., Vector3 (0.5, 0., 0.0), Matrix3::Identity ());

  model.addBody (model.getBodyId("universe"), JointModelPX(), SE3::Identity (), inertia, "root");

  Data data (model);

  Eigen::VectorXd q (Eigen::VectorXd::Zero (model.nq));
  Eigen::VectorXd v (Eigen::VectorXd::Zero (model.nv));
  Eigen::VectorXd a (Eigen::VectorXd::Zero (model.nv));

  rnea (model, data, q, v, a);

  Eigen::VectorXd tau_expected (Eigen::VectorXd::Zero (model.nq));
  tau_expected  << 0;

  BOOST_CHECK (tau_expected.isApprox(data.tau, 1e-14));

  // -----
  q = Eigen::VectorXd::Ones (model.nq);
  v = Eigen::VectorXd::Ones (model.nv);
  a = Eigen::VectorXd::Ones (model.nv);

  rnea (model, data, q, v, a);
  tau_expected << 1;

  BOOST_CHECK (tau_expected.isApprox(data.tau, 1e-12));

  q << 3;
  v = Eigen::VectorXd::Ones (model.nv);
  a = Eigen::VectorXd::Ones (model.nv);

  rnea (model, data, q, v, a);
  tau_expected << 1;

  BOOST_CHECK (tau_expected.isApprox(data.tau, 1e-12));
}

BOOST_AUTO_TEST_CASE ( test_crba )
{
  using namespace se3;
  using namespace std;
  typedef Eigen::Matrix <double, 3, 1> Vector3;
  typedef Eigen::Matrix <double, 3, 3> Matrix3;

  Model model;
  Inertia inertia (1., Vector3 (0.5, 0., 0.0), Matrix3::Identity ());

  model.addBody (model.getBodyId("universe"), JointModelPX (), SE3::Identity (), inertia, "root");

  Data data (model);

  Eigen::VectorXd q (Eigen::VectorXd::Zero (model.nq));
  Eigen::MatrixXd M_expected (model.nv,model.nv);

  crba (model, data, q);
  M_expected << 1.0;

  BOOST_CHECK (M_expected.isApprox(data.M, 1e-14));

  q = Eigen::VectorXd::Ones (model.nq);

  crba (model, data, q);

  BOOST_CHECK (M_expected.isApprox(data.M, 1e-12));

  q << 3;

  crba (model, data, q);
  
  BOOST_CHECK (M_expected.isApprox(data.M, 1e-10));
}

BOOST_AUTO_TEST_SUITE_END ()

BOOST_AUTO_TEST_SUITE ( JointTranslation )

BOOST_AUTO_TEST_CASE ( test_kinematics )
{
  using namespace se3;

  typedef Motion::Vector3 Vector3;

  Motion expected_v_J (Motion::Zero ());
  Motion expected_c_J (Motion::Zero ());

  SE3 expected_configuration (SE3::Identity ());

  JointDataTranslation joint_data;
  JointModelTranslation joint_model;

  joint_model.setIndexes (0, 0, 0);

  Vector3 q (Vector3::Zero());
  Vector3 q_dot (Vector3::Zero());

  // -------
  q = Vector3 (0., 0., 0.);
  q_dot = Vector3 (0., 0., 0.);

  joint_model.calc (joint_data, q, q_dot);

  printOutJointData <JointDataTranslation> (q, q_dot, joint_data);

  BOOST_CHECK (expected_configuration.rotation ().isApprox(joint_data.M.rotation(), 1e-12));
  BOOST_CHECK (expected_configuration.translation ().isApprox(joint_data.M.translation (), 1e-12));
  BOOST_CHECK (expected_v_J.toVector ().isApprox(((Motion) joint_data.v).toVector(), 1e-12));
  BOOST_CHECK (expected_c_J.toVector ().isApprox(((Motion) joint_data.c).toVector(), 1e-12));

  // -------
  q = Vector3 (1., 0., 0.);
  q_dot = Vector3 (1., 0., 0.);

  joint_model.calc (joint_data, q, q_dot);

  printOutJointData <JointDataTranslation> (q, q_dot, joint_data);

  expected_configuration.translation () << 1, 0, 0;

  expected_v_J.linear () << 1., 0., 0.;

  BOOST_CHECK (expected_configuration.rotation ().isApprox(joint_data.M.rotation(), 1e-12));
  BOOST_CHECK (expected_configuration.translation ().isApprox(joint_data.M.translation (), 1e-12));
  BOOST_CHECK (expected_v_J.toVector ().isApprox(((Motion) joint_data.v).toVector(), 1e-12));
  BOOST_CHECK (expected_c_J.toVector ().isApprox(((Motion) joint_data.c).toVector(), 1e-12));

  // -------
  q = Vector3 (0., 1., 0.);
  q_dot = Vector3 (0., 1., 0.);

  joint_model.calc (joint_data, q, q_dot);

  printOutJointData <JointDataTranslation> (q, q_dot, joint_data);

  expected_configuration.translation () << 0, 1., 0;

  expected_v_J.linear () << 0., 1., 0.;

  BOOST_CHECK (expected_configuration.rotation ().isApprox(joint_data.M.rotation(), 1e-12));
  BOOST_CHECK (expected_configuration.translation ().isApprox(joint_data.M.translation (), 1e-12));
  BOOST_CHECK (expected_v_J.toVector ().isApprox(((Motion) joint_data.v).toVector(), 1e-12));
  BOOST_CHECK (expected_c_J.toVector ().isApprox(((Motion) joint_data.c).toVector(), 1e-12));

  // -------
  q = Vector3 (0., 0., 1.);
  q_dot = Vector3 (0., 0., 1.);

  joint_model.calc (joint_data, q, q_dot);

  printOutJointData <JointDataTranslation> (q, q_dot, joint_data);

  expected_configuration.translation () << 0, 0, 1;

  expected_v_J.linear () << 0., 0., 1.;

  BOOST_CHECK (expected_configuration.rotation ().isApprox(joint_data.M.rotation(), 1e-12));
  BOOST_CHECK (expected_configuration.translation ().isApprox(joint_data.M.translation (), 1e-12));
  BOOST_CHECK (expected_v_J.toVector ().isApprox(((Motion) joint_data.v).toVector(), 1e-12));
  BOOST_CHECK (expected_c_J.toVector ().isApprox(((Motion) joint_data.c).toVector(), 1e-12));

  // -------
  q = Vector3 (1., 1., 1.);
  q_dot = Vector3 (1., 1., 1.);

  joint_model.calc (joint_data, q, q_dot);

  printOutJointData <JointDataTranslation> (q, q_dot, joint_data);

  expected_configuration.translation () << 1., 1., 1.;
  expected_v_J.linear () << 1., 1., 1.;

  BOOST_CHECK (expected_configuration.rotation ().isApprox(joint_data.M.rotation(), 1e-12));
  BOOST_CHECK (expected_configuration.translation ().isApprox(joint_data.M.translation (), 1e-12));
  BOOST_CHECK (expected_v_J.toVector ().isApprox(((Motion) joint_data.v).toVector(), 1e-12));
  BOOST_CHECK (expected_c_J.toVector ().isApprox(((Motion) joint_data.c).toVector(), 1e-12));

  // -------
  q = Vector3 (1., 1.5, 1.9);
  q_dot = Vector3 (2., 3., 1.);

  joint_model.calc (joint_data, q, q_dot);

  printOutJointData <JointDataTranslation> (q, q_dot, joint_data);

  expected_configuration.translation () = q;
  expected_v_J.linear () = q_dot;

  BOOST_CHECK (expected_configuration.rotation ().isApprox(joint_data.M.rotation(), 1e-12));
  BOOST_CHECK (expected_configuration.translation ().isApprox(joint_data.M.translation (), 1e-12));
  BOOST_CHECK (expected_v_J.toVector ().isApprox(((Motion) joint_data.v).toVector(), 1e-12));
  BOOST_CHECK (expected_c_J.toVector ().isApprox(((Motion) joint_data.c).toVector(), 1e-12));
}

BOOST_AUTO_TEST_CASE ( test_rnea )
{
  using namespace se3;
  typedef Eigen::Matrix <double, 3, 1> Vector3;
  typedef Eigen::Matrix <double, 3, 3> Matrix3;

  Model model;
  Inertia inertia (1., Vector3 (0.5, 0., 0.0), Matrix3::Identity ());

  model.addBody (model.getBodyId("universe"), JointModelTranslation (), SE3::Identity (), inertia, "root");

  Data data (model);

  Eigen::VectorXd q (Eigen::VectorXd::Zero (model.nq));
  Eigen::VectorXd v (Eigen::VectorXd::Zero (model.nv));
  Eigen::VectorXd a (Eigen::VectorXd::Zero (model.nv));

  rnea (model, data, q, v, a);
  Vector3 tau_expected (Vector3::Zero ());

  tau_expected  << 0,    0, 9.81;

  BOOST_CHECK (tau_expected.isApprox(data.tau, 1e-14));

  q = Eigen::VectorXd::Ones (model.nq);
  v = Eigen::VectorXd::Ones (model.nv);
  a = Eigen::VectorXd::Ones (model.nv);

  rnea (model, data, q, v, a);
  tau_expected << 1,     1, 10.81;

  BOOST_CHECK (tau_expected.isApprox(data.tau, 1e-12));

  q << 3, 2, 1;
  v = Eigen::VectorXd::Ones (model.nv);
  a = Eigen::VectorXd::Ones (model.nv);

  rnea (model, data, q, v, a);
  tau_expected << 1,     1, 10.81;

  BOOST_CHECK (tau_expected.isApprox(data.tau, 1e-12));
}

BOOST_AUTO_TEST_CASE ( test_crba )
{
  using namespace se3;
  using namespace std;
  typedef Eigen::Matrix <double, 3, 1> Vector3;
  typedef Eigen::Matrix <double, 3, 3> Matrix3;

  Model model;
  Inertia inertia (1., Vector3 (0.5, 0., 0.0), Matrix3::Identity ());

  model.addBody (model.getBodyId("universe"), JointModelTranslation (), SE3::Identity (), inertia, "root");

  Data data (model);

  Eigen::VectorXd q (Eigen::VectorXd::Zero (model.nq));
  Eigen::MatrixXd M_expected (model.nv,model.nv);

  crba (model, data, q);
  M_expected = Matrix3::Identity ();

  BOOST_CHECK (M_expected.isApprox(data.M, 1e-14));

  q = Eigen::VectorXd::Ones (model.nq);

  crba (model, data, q);

  BOOST_CHECK (M_expected.isApprox(data.M, 1e-12));

  q << 3, 2, 1;
  
  crba (model, data, q);
  
  BOOST_CHECK (M_expected.isApprox(data.M, 1e-10));
}

BOOST_AUTO_TEST_SUITE_END ()


BOOST_AUTO_TEST_SUITE ( JointSpherical )

BOOST_AUTO_TEST_CASE ( test_kinematics )
{
  using namespace se3;

  typedef Motion::Vector3 Vector3;
  typedef Eigen::Matrix <double, 4, 1> Vector4;

  Motion expected_v_J (Motion::Zero ());
  Motion expected_c_J (Motion::Zero ());

  SE3 expected_configuration (SE3::Identity ());

  JointDataSpherical joint_data;
  JointModelSpherical joint_model;

  joint_model.setIndexes (0, 0, 0);

  Vector4 q (Vector4::Zero());
  Vector3 q_dot (Vector3::Zero());

  // -------
  q = Vector4 (0., 0, 0., 1.); q.normalize();
  q_dot = Vector3 (0., 0., 0.);

  joint_model.calc (joint_data, q, q_dot);

  printOutJointData <JointDataSpherical> (q, q_dot, joint_data);

  BOOST_CHECK (expected_configuration.rotation ().isApprox(joint_data.M.rotation(), 1e-12));
  BOOST_CHECK (expected_configuration.translation ().isApprox(joint_data.M.translation (), 1e-12));
  BOOST_CHECK (expected_v_J.toVector ().isApprox(((Motion) joint_data.v).toVector(), 1e-12));
  BOOST_CHECK (expected_c_J.toVector ().isApprox(((Motion) joint_data.c).toVector(), 1e-12));

  // -------
  q = Vector4 (1., 0, 0., 1.); q.normalize();
  q_dot = Vector3 (1., 0., 0.);

  joint_model.calc (joint_data, q, q_dot);

  printOutJointData <JointDataSpherical> (q, q_dot, joint_data);

  expected_configuration.rotation ().transpose () <<
  1,                   0,                   0,
  0, 2.2204460492503e-16,                   1,
  0,                  -1, 2.2204460492503e-16;

  expected_v_J.angular () << 1., 0., 0.;

  BOOST_CHECK (expected_configuration.rotation ().isApprox(joint_data.M.rotation(), 1e-12));
  BOOST_CHECK (expected_configuration.translation ().isApprox(joint_data.M.translation (), 1e-12));
  BOOST_CHECK (expected_v_J.toVector ().isApprox(((Motion) joint_data.v).toVector(), 1e-12));
  BOOST_CHECK (expected_c_J.toVector ().isApprox(((Motion) joint_data.c).toVector(), 1e-12));

  // -------
  q = Vector4 (0., 1., 0., 1.); q.normalize();
  q_dot = Vector3 (0., 1., 0.);

  joint_model.calc (joint_data, q, q_dot);

  printOutJointData <JointDataSpherical> (q, q_dot, joint_data);

  expected_configuration.rotation ().transpose () <<
  2.2204460492503e-16,                   0,                  -1,
  0,                   1,                   0,
  1,                   0, 2.2204460492503e-16;

  expected_v_J.angular () << 0., 1., 0.;

  BOOST_CHECK (expected_configuration.rotation ().isApprox(joint_data.M.rotation(), 1e-12));
  BOOST_CHECK (expected_configuration.translation ().isApprox(joint_data.M.translation (), 1e-12));
  BOOST_CHECK (expected_v_J.toVector ().isApprox(((Motion) joint_data.v).toVector(), 1e-12));
  BOOST_CHECK (expected_c_J.toVector ().isApprox(((Motion) joint_data.c).toVector(), 1e-12));

  // -------
  q = Vector4 (0., 0, 1., 1.); q.normalize();
  q_dot = Vector3 (0., 0., 1.);

  joint_model.calc (joint_data, q, q_dot);

  printOutJointData <JointDataSpherical> (q, q_dot, joint_data);

  expected_configuration.rotation ().transpose () <<
  2.2204460492503e-16,                   1,                   0,
  -1, 2.2204460492503e-16,                   0,
  0,                   0,                   1;

  expected_v_J.angular () << 0., 0., 1.;

  BOOST_CHECK (expected_configuration.rotation ().isApprox(joint_data.M.rotation(), 1e-12));
  BOOST_CHECK (expected_configuration.translation ().isApprox(joint_data.M.translation (), 1e-12));
  BOOST_CHECK (expected_v_J.toVector ().isApprox(((Motion) joint_data.v).toVector(), 1e-12));
  BOOST_CHECK (expected_c_J.toVector ().isApprox(((Motion) joint_data.c).toVector(), 1e-12));

  // -------
  q = Vector4 (1., 1., 1., 1.); q.normalize();
  q_dot = Vector3 (1., 1., 1.);

  joint_model.calc (joint_data, q, q_dot);

  printOutJointData <JointDataSpherical> (q, q_dot, joint_data);

  expected_configuration.rotation ().transpose () <<
  0, 1, 0,
  0, 0, 1,
  1, 0, 0;

  expected_v_J.angular () << 1., 1., 1.;

  BOOST_CHECK (expected_configuration.rotation ().isApprox(joint_data.M.rotation(), 1e-12));
  BOOST_CHECK (expected_configuration.translation ().isApprox(joint_data.M.translation (), 1e-12));
  BOOST_CHECK (expected_v_J.toVector ().isApprox(((Motion) joint_data.v).toVector(), 1e-12));
  BOOST_CHECK (expected_c_J.toVector ().isApprox(((Motion) joint_data.c).toVector(), 1e-12));

  // -------
  q = Vector4 (1., 1.5, 1.9, 1.); q.normalize();
  q_dot = Vector3 (2., 3., 1.);

  joint_model.calc (joint_data, q, q_dot);

  printOutJointData <JointDataSpherical> (q, q_dot, joint_data);

  expected_configuration.rotation ().transpose () <<
  -0.4910941475827,  0.86513994910942,  0.10178117048346,
  -0.10178117048346, -0.17302798982188,  0.97964376590331,
  0.86513994910942,  0.47073791348601,  0.17302798982188;

  expected_v_J.angular () = q_dot;

  BOOST_CHECK (expected_configuration.rotation ().isApprox(joint_data.M.rotation(), 1e-12));
  BOOST_CHECK (expected_configuration.translation ().isApprox(joint_data.M.translation (), 1e-12));
  BOOST_CHECK (expected_v_J.toVector ().isApprox(((Motion) joint_data.v).toVector(), 1e-12));
  BOOST_CHECK (expected_c_J.toVector ().isApprox(((Motion) joint_data.c).toVector(), 1e-12));
}

BOOST_AUTO_TEST_CASE ( test_rnea )
{
  using namespace se3;
  typedef Eigen::Matrix <double, 3, 1> Vector3;
  typedef Eigen::Matrix <double, 3, 3> Matrix3;

  Model model;
  Inertia inertia (1., Vector3 (0.5, 0., 0.0), Matrix3::Identity ());

  model.addBody (model.getBodyId("universe"), JointModelSpherical (), SE3::Identity (), inertia, "root");

  Data data (model);

  Eigen::VectorXd q (Eigen::VectorXd::Zero (model.nq));
  Eigen::VectorXd v (Eigen::VectorXd::Zero (model.nv));
  Eigen::VectorXd a (Eigen::VectorXd::Zero (model.nv));

  rnea (model, data, q, v, a);
  Vector3 tau_expected (Vector3::Zero ());

  tau_expected  <<  0, -4.905,      0;

  BOOST_CHECK (tau_expected.isApprox(data.tau, 1e-14));

  q = Eigen::VectorXd::Ones (model.nq); q.normalize ();
  v = Eigen::VectorXd::Ones (model.nv);
  a = Eigen::VectorXd::Ones (model.nv);

  rnea (model, data, q, v, a);
  tau_expected << 1,     1, 6.405;

  BOOST_CHECK (tau_expected.isApprox(data.tau, 1e-12));

  q << 3, 2, 1, 1; q.normalize ();
  v = Eigen::VectorXd::Ones (model.nv);
  a = Eigen::VectorXd::Ones (model.nv);

  rnea (model, data, q, v, a);
  tau_expected << 1, 4.597,  4.77;

  BOOST_CHECK (tau_expected.isApprox(data.tau, 1e-12));
}

BOOST_AUTO_TEST_CASE ( test_crba )
{
  using namespace se3;
  using namespace std;
  typedef Eigen::Matrix <double, 3, 1> Vector3;
  typedef Eigen::Matrix <double, 3, 3> Matrix3;

  Model model;
  Inertia inertia (1., Vector3 (0.5, 0., 0.0), Matrix3::Identity ());

  model.addBody (model.getBodyId("universe"), JointModelSpherical (), SE3::Identity (), inertia, "root");

  Data data (model);

  Eigen::VectorXd q (Eigen::VectorXd::Zero (model.nq)); q(3) = 1.; q.normalize();
  Eigen::MatrixXd M_expected (model.nv,model.nv);

  crba (model, data, q);
  M_expected = Matrix3::Identity ();
  M_expected(1,1) = 1.25; M_expected(2,2) = 1.25;

  BOOST_CHECK (M_expected.isApprox(data.M, 1e-14));

  q = Eigen::VectorXd::Ones (model.nq); q.normalize();

  crba (model, data, q);

  BOOST_CHECK (M_expected.isApprox(data.M, 1e-12));
  q << 3, 2, 1, 1; q.normalize();
  
  crba (model, data, q);
  
  BOOST_CHECK (M_expected.isApprox(data.M, 1e-10));
}

BOOST_AUTO_TEST_SUITE_END ()

BOOST_AUTO_TEST_SUITE ( caseJointFixed )

BOOST_AUTO_TEST_CASE ( test_merge_body )
{
  using namespace se3;
  typedef Eigen::Matrix <double, 3, 1> Vector3;
  typedef Eigen::Matrix <double, 3, 3> Matrix3;

  Model model;
  Inertia inertiaRoot (1., Vector3 (0.5, 0., 0.0), Matrix3::Identity ());
  //Inertia inertiaFixedBodyAtCom (1., Vector3 (0.5, 0., 0.0), Matrix3::Identity ());
  Inertia inertiaFixedBodyAtJoint (1., Vector3 (0.75, 0., 0.0), Matrix3::Identity ());
  SE3 liMi(Matrix3::Identity(),Vector3(1.0, 1.0, 0.0));
  //SE3 liMi(Matrix3::Identity(),Vector3d::Zero());

  model.addBody (model.getBodyId("universe"), JointModelRX (), SE3::Identity (), inertiaRoot,
                 "root_joint", "root_body");
  model.mergeFixedBody(model.getBodyId("root_body"), liMi, inertiaFixedBodyAtJoint);

  Inertia mergedInertia(model.inertias[(size_t)(model.getBodyId("root_body"))]);

  double expected_mass=2;
  Eigen::Vector3d expected_com(Eigen::Vector3d::Zero());expected_com << 1.125, 0.5, 0.;
  Eigen::Matrix3d expectedBodyInertia; expectedBodyInertia << 2.5,    -0.625,   0.,
                                                              -0.625, 2.78125,  0.,
                                                              0.,     0.,       3.28125;

  BOOST_CHECK_EQUAL(mergedInertia.mass(), expected_mass);
  BOOST_CHECK (mergedInertia.lever().isApprox(expected_com, 1e-12));
  BOOST_CHECK (mergedInertia.inertia().matrix().isApprox(expectedBodyInertia, 1e-12));
  
}

BOOST_AUTO_TEST_SUITE_END ()

BOOST_AUTO_TEST_SUITE ( JointDense )

BOOST_AUTO_TEST_CASE ( toJointModelDense )
{
  using namespace se3;


  JointModelRX jmodel;
  jmodel.setIndexes (2, 0, 0);

  JointModelDense<JointModelBase<JointModelRX>::NQ, JointModelBase<JointModelRX>::NV> jmd(jmodel.id(), jmodel.idx_q(), jmodel.idx_v());
  JointModelDense<JointModelBase<JointModelRX>::NQ, JointModelBase<JointModelRX>::NV> jmd2 = jmodel.toDense();
  (void)jmd; (void)jmd2;

  BOOST_CHECK_MESSAGE(jmd.idx_q() == jmodel.idx_q() , "The comparison of the joint index in configuration space failed");
  BOOST_CHECK_MESSAGE(jmd.idx_q() == jmd2.idx_q() , "The comparison of the joint index in  configuration space failed");

  BOOST_CHECK_MESSAGE(jmd.idx_v() == jmodel.idx_v() , "The comparison of the joint index in velocity space failed");
  BOOST_CHECK_MESSAGE(jmd.idx_v() == jmd2.idx_v() , "The comparison of the joint index in  velocity space failed");

  BOOST_CHECK_MESSAGE(jmd.id() == jmodel.id() , "The comparison of the joint index in model's kinematic tree failed");
  BOOST_CHECK_MESSAGE(jmd.id() == jmd2.id() , "The comparison of the joint index in model's kinematic tree failed");

}

BOOST_AUTO_TEST_CASE ( toJointDataDense )
{
  using namespace se3;

  JointModelRX jmodel;
  jmodel.setIndexes (2, 0, 0);

  JointDataRX jdata = jmodel.createData();

  JointDataDense< JointDataBase<JointModelRX::JointData>::NQ,
                  JointDataBase<JointModelRX::JointData>::NV
                  > jdd = jdata.toDense();

  BOOST_CHECK(ConstraintXd(jdata.S).matrix().isApprox(jdd.S.matrix()));

}
BOOST_AUTO_TEST_SUITE_END ()

BOOST_AUTO_TEST_SUITE (JointPlanar)

BOOST_AUTO_TEST_CASE (vsFreeFlyer)
{
  using namespace se3;
  typedef Eigen::Matrix <double, 3, 1> Vector3;
  typedef Eigen::Matrix <double, 6, 1> Vector6;
  typedef Eigen::Matrix <double, 7, 1> VectorFF;
  typedef Eigen::Matrix <double, 3, 3> Matrix3;

  Model modelPlanar, modelFreeflyer;

  Inertia inertia (1., Vector3 (0.5, 0., 0.0), Matrix3::Identity ());
  SE3 pos(1); pos.translation() = SE3::Linear_t(1.,0.,0.);


  modelPlanar.addBody (0, JointModelPlanar (), pos, inertia, "planar");
  modelFreeflyer.addBody(0, JointModelFreeFlyer(),pos, inertia, "ff");

  Data dataPlanar(modelPlanar);
  Data dataFreeFlyer(modelFreeflyer);


  Eigen::VectorXd q = Eigen::VectorXd::Ones (modelPlanar.nq);q[2] = PI /2;  VectorFF qff; qff << 1, 1, 0, 0, 0, sqrt(2)/2, sqrt(2)/2 ; 
  Eigen::VectorXd v = Eigen::VectorXd::Ones (modelPlanar.nv);               Vector6 vff; vff << 1, 1, 0, 0, 0, 1;
  Eigen::VectorXd tauPlanar = Eigen::VectorXd::Ones (modelPlanar.nv);       Eigen::VectorXd tauff = Eigen::VectorXd::Ones (modelFreeflyer.nv);
  Eigen::VectorXd aPlanar = Eigen::VectorXd::Ones (modelPlanar.nv);         Eigen::VectorXd aff(vff);
  


  forwardKinematics(modelPlanar, dataPlanar, q, v);
  forwardKinematics(modelFreeflyer, dataFreeFlyer, qff, vff);

  computeAllTerms(modelPlanar, dataPlanar, q, v);
  computeAllTerms(modelFreeflyer, dataFreeFlyer, qff, vff);

  BOOST_CHECK(dataFreeFlyer.oMi[1].isApprox(dataPlanar.oMi[1]));
  BOOST_CHECK(dataFreeFlyer.liMi[1].isApprox(dataPlanar.liMi[1]));
  BOOST_CHECK(dataFreeFlyer.Ycrb[1].matrix().isApprox(dataPlanar.Ycrb[1].matrix()));
  BOOST_CHECK(dataFreeFlyer.f[1].toVector().isApprox(dataPlanar.f[1].toVector()));
  
  Eigen::VectorXd nle_expected_ff(3); nle_expected_ff << dataFreeFlyer.nle[0],
                                                         dataFreeFlyer.nle[1],
                                                         dataFreeFlyer.nle[5]
                                                         ;
  BOOST_CHECK(nle_expected_ff.isApprox(dataPlanar.nle));
  BOOST_CHECK(dataFreeFlyer.com[0].isApprox(dataPlanar.com[0]));



  // InverseDynamics == rnea
  tauPlanar = rnea(modelPlanar, dataPlanar, q, v, aPlanar);
  tauff = rnea(modelFreeflyer, dataFreeFlyer, qff, vff, aff);

  Vector3 tau_expected; tau_expected << tauff(0), tauff(1), tauff(5);
  BOOST_CHECK(tauPlanar.isApprox(tau_expected));

  // ForwardDynamics == aba
  Eigen::VectorXd aAbaPlanar = aba(modelPlanar,dataPlanar, q, v, tauPlanar);
  Eigen::VectorXd aAbaFreeFlyer = aba(modelFreeflyer,dataFreeFlyer, qff, vff, tauff);
  Vector3 a_expected; a_expected << aAbaFreeFlyer[0],
                                    aAbaFreeFlyer[1],
                                    aAbaFreeFlyer[5]
                                    ;
  BOOST_CHECK(aAbaPlanar.isApprox(a_expected));

  // crba
  crba(modelPlanar, dataPlanar,q);
  crba(modelFreeflyer, dataFreeFlyer, qff);

  Eigen::Matrix<double, 3, 3> M_expected;
  M_expected.block<2,2>(0,0) = dataFreeFlyer.M.block<2,2>(0,0);
  M_expected.block<1,2>(2,0) = dataFreeFlyer.M.block<1,2>(5,0);
  M_expected.block<2,1>(0,2) = dataFreeFlyer.M.col(5).head<2>();
  M_expected.block<1,1>(2,2) = dataFreeFlyer.M.col(5).tail<1>();

  BOOST_CHECK(dataPlanar.M.isApprox(M_expected));
   
  // Jacobian
  Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian_planar;jacobian_planar.resize(6,3); jacobian_planar.setZero();
  Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian_ff;jacobian_ff.resize(6,6);jacobian_ff.setZero();
  computeJacobians(modelPlanar, dataPlanar, q);
  computeJacobians(modelFreeflyer, dataFreeFlyer, qff);
  getJacobian<true>(modelPlanar, dataPlanar, 1, jacobian_planar);
  getJacobian<true>(modelFreeflyer, dataFreeFlyer, 1, jacobian_ff);


  Eigen::Matrix<double, 6, 3> jacobian_expected; jacobian_expected << jacobian_ff.col(0),
                                                                      jacobian_ff.col(1),
                                                                      jacobian_ff.col(5)
                                                                      ;

  BOOST_CHECK(jacobian_planar.isApprox(jacobian_expected));


}
BOOST_AUTO_TEST_SUITE_END ()
