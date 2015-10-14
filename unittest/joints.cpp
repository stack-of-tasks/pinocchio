//
// Copyright (c) 2015 CNRS
// Copyright (c) 2015 Wandercraft
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

#include "pinocchio/spatial/force.hpp"
#include "pinocchio/spatial/motion.hpp"
#include "pinocchio/spatial/se3.hpp"
#include "pinocchio/spatial/inertia.hpp"
#include "pinocchio/multibody/joint/joint-revolute.hpp"
#include "pinocchio/multibody/joint/joint-revolute-unaligned.hpp"
#include "pinocchio/multibody/joint/joint-spherical.hpp"
#include "pinocchio/multibody/joint/joint-spherical-ZYX.hpp"
#include "pinocchio/multibody/joint/joint-prismatic.hpp"
#include "pinocchio/multibody/joint/joint-translation.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/crba.hpp"

#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE JointsTest
#include <boost/test/unit_test.hpp>
#include "pinocchio/tools/matrix-comparison.hpp"

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

  is_matrix_absolutely_closed (expected_configuration.rotation (), joint_data.M.rotation());
  is_matrix_absolutely_closed (expected_configuration.translation (), joint_data.M.translation ());
  is_matrix_absolutely_closed (expected_v_J.toVector (), ((Motion) joint_data.v).toVector());
  is_matrix_absolutely_closed (expected_c_J.toVector (), ((Motion) joint_data.c).toVector());

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

  is_matrix_absolutely_closed (expected_configuration.rotation (), joint_data.M.rotation(), 1e-12);
  is_matrix_absolutely_closed (expected_configuration.translation (), joint_data.M.translation (), 1e-12);
  is_matrix_absolutely_closed (expected_v_J.toVector (), ((Motion) joint_data.v).toVector(), 1e-12);
  is_matrix_absolutely_closed (expected_c_J.toVector (), ((Motion) joint_data.c).toVector(), 1e-12);

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

  is_matrix_absolutely_closed (expected_configuration.rotation (), joint_data.M.rotation(), 1e-12);
  is_matrix_absolutely_closed (expected_configuration.translation (), joint_data.M.translation (), 1e-12);
  is_matrix_absolutely_closed (expected_v_J.toVector (), ((Motion) joint_data.v).toVector(), 1e-12);
  is_matrix_absolutely_closed (expected_c_J.toVector (), ((Motion) joint_data.c).toVector(), 1e-12);

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

  is_matrix_absolutely_closed (expected_configuration.rotation (), joint_data.M.rotation(), 1e-12);
  is_matrix_absolutely_closed (expected_configuration.translation (), joint_data.M.translation (), 1e-12);
  is_matrix_absolutely_closed (expected_v_J.toVector (), ((Motion) joint_data.v).toVector(), 1e-12);
  is_matrix_absolutely_closed (expected_c_J.toVector (), ((Motion) joint_data.c).toVector(), 1e-12);

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

  is_matrix_absolutely_closed (expected_configuration.rotation (), joint_data.M.rotation(), 1e-10);
  is_matrix_absolutely_closed (expected_configuration.translation (), joint_data.M.translation (), 1e-10);
  is_matrix_absolutely_closed (expected_v_J.toVector (), ((Motion) joint_data.v).toVector(), 1e-10);
  is_matrix_absolutely_closed (expected_c_J.toVector (), ((Motion) joint_data.c).toVector(), 1e-10);

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

  is_matrix_absolutely_closed (expected_configuration.rotation (), joint_data.M.rotation(), 1e-10);
  is_matrix_absolutely_closed (expected_configuration.translation (), joint_data.M.translation (), 1e-10);
  is_matrix_absolutely_closed (expected_v_J.toVector (), ((Motion) joint_data.v).toVector(), 1e-10);
  is_matrix_absolutely_closed (expected_c_J.toVector (), ((Motion) joint_data.c).toVector(), 1e-10);
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

  is_matrix_absolutely_closed (tau_expected, data.tau, 1e-14);

  q = Eigen::VectorXd::Ones (model.nq);
  v = Eigen::VectorXd::Ones (model.nv);
  a = Eigen::VectorXd::Ones (model.nv);

  rnea (model, data, q, v, a);
  tau_expected << -0.53611600195085, -0.74621832606188, -0.38177329067604;

  is_matrix_absolutely_closed (tau_expected, data.tau, 1e-12);

  q << 3, 2, 1;
  v = Eigen::VectorXd::Ones (model.nv);
  a = Eigen::VectorXd::Ones (model.nv);

  rnea (model, data, q, v, a);
  tau_expected << 0.73934458094049,  2.7804530848031, 0.50684940972146;

  is_matrix_absolutely_closed (tau_expected, data.tau, 1e-12);
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

  is_matrix_absolutely_closed (M_expected, data.M, 1e-14);

  q = Eigen::VectorXd::Ones (model.nq);

  crba (model, data, q);
  M_expected <<
  1.0729816454316, -5.5511151231258e-17,     -0.8414709848079,
  -5.5511151231258e-17,                 1.25,                    0,
  -0.8414709848079,                    0,                    1;

  is_matrix_absolutely_closed (M_expected, data.M, 1e-12);

  q << 3, 2, 1;

  crba (model, data, q);
  M_expected <<
  1.043294547392, 2.7755575615629e-17,   -0.90929742682568,
  0,                1.25,                   0,
  -0.90929742682568,                   0,                  1;

  is_matrix_absolutely_closed (M_expected, data.M, 1e-10);
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

  is_matrix_absolutely_closed (expected_configuration.rotation (), joint_data.M.rotation());
  is_matrix_absolutely_closed (expected_configuration.translation (), joint_data.M.translation ());
  is_matrix_absolutely_closed (expected_v_J.toVector (), ((Motion) joint_data.v).toVector());
  is_matrix_absolutely_closed (expected_c_J.toVector (), ((Motion) joint_data.c).toVector());

  // -------
  q << 1.;
  q_dot << 1.;


  joint_model.calc (joint_data, q, q_dot);

  printOutJointData <JointDataPX> (q, q_dot, joint_data);

  expected_configuration.translation () << 1, 0, 0;

  expected_v_J.linear () << 1., 0., 0.;

  is_matrix_absolutely_closed (expected_configuration.rotation (), joint_data.M.rotation(), 1e-12);
  is_matrix_absolutely_closed (expected_configuration.translation (), joint_data.M.translation (), 1e-12);
  is_matrix_absolutely_closed (expected_v_J.toVector (), ((Motion) joint_data.v).toVector(), 1e-12);
  is_matrix_absolutely_closed (expected_c_J.toVector (), ((Motion) joint_data.c).toVector(), 1e-12);
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

  is_matrix_absolutely_closed (tau_expected, data.tau, 1e-14);

  // -----
  q = Eigen::VectorXd::Ones (model.nq);
  v = Eigen::VectorXd::Ones (model.nv);
  a = Eigen::VectorXd::Ones (model.nv);

  rnea (model, data, q, v, a);
  tau_expected << 1;

  is_matrix_absolutely_closed (tau_expected, data.tau, 1e-12);

  q << 3;
  v = Eigen::VectorXd::Ones (model.nv);
  a = Eigen::VectorXd::Ones (model.nv);

  rnea (model, data, q, v, a);
  tau_expected << 1;

  is_matrix_absolutely_closed (tau_expected, data.tau, 1e-12);
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

  is_matrix_absolutely_closed (M_expected, data.M, 1e-14);

  q = Eigen::VectorXd::Ones (model.nq);

  crba (model, data, q);

  is_matrix_absolutely_closed (M_expected, data.M, 1e-12);

  q << 3;

  crba (model, data, q);
  
  is_matrix_absolutely_closed (M_expected, data.M, 1e-10);
}

BOOST_AUTO_TEST_SUITE_END ()

BOOST_AUTO_TEST_SUITE ( JointTranslation )

BOOST_AUTO_TEST_CASE ( test_kinematics )
{
  using namespace se3;

  typedef Motion::Vector3 Vector3;
  typedef Motion::Vector6 Vector6;

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

  is_matrix_absolutely_closed (expected_configuration.rotation (), joint_data.M.rotation());
  is_matrix_absolutely_closed (expected_configuration.translation (), joint_data.M.translation ());
  is_matrix_absolutely_closed (expected_v_J.toVector (), ((Motion) joint_data.v).toVector());
  is_matrix_absolutely_closed (expected_c_J.toVector (), ((Motion) joint_data.c).toVector());

  // -------
  q = Vector3 (1., 0., 0.);
  q_dot = Vector3 (1., 0., 0.);

  joint_model.calc (joint_data, q, q_dot);

  printOutJointData <JointDataTranslation> (q, q_dot, joint_data);

  expected_configuration.translation () << 1, 0, 0;

  expected_v_J.linear () << 1., 0., 0.;

  is_matrix_absolutely_closed (expected_configuration.rotation (), joint_data.M.rotation(), 1e-12);
  is_matrix_absolutely_closed (expected_configuration.translation (), joint_data.M.translation (), 1e-12);
  is_matrix_absolutely_closed (expected_v_J.toVector (), ((Motion) joint_data.v).toVector(), 1e-12);
  is_matrix_absolutely_closed (expected_c_J.toVector (), ((Motion) joint_data.c).toVector(), 1e-12);

  // -------
  q = Vector3 (0., 1., 0.);
  q_dot = Vector3 (0., 1., 0.);

  joint_model.calc (joint_data, q, q_dot);

  printOutJointData <JointDataTranslation> (q, q_dot, joint_data);

  expected_configuration.translation () << 0, 1., 0;

  expected_v_J.linear () << 0., 1., 0.;

  is_matrix_absolutely_closed (expected_configuration.rotation (), joint_data.M.rotation(), 1e-12);
  is_matrix_absolutely_closed (expected_configuration.translation (), joint_data.M.translation (), 1e-12);
  is_matrix_absolutely_closed (expected_v_J.toVector (), ((Motion) joint_data.v).toVector(), 1e-12);
  is_matrix_absolutely_closed (expected_c_J.toVector (), ((Motion) joint_data.c).toVector(), 1e-12);

  // -------
  q = Vector3 (0., 0., 1.);
  q_dot = Vector3 (0., 0., 1.);

  joint_model.calc (joint_data, q, q_dot);

  printOutJointData <JointDataTranslation> (q, q_dot, joint_data);

  expected_configuration.translation () << 0, 0, 1;

  expected_v_J.linear () << 0., 0., 1.;

  is_matrix_absolutely_closed (expected_configuration.rotation (), joint_data.M.rotation(), 1e-12);
  is_matrix_absolutely_closed (expected_configuration.translation (), joint_data.M.translation (), 1e-12);
  is_matrix_absolutely_closed (expected_v_J.toVector (), ((Motion) joint_data.v).toVector(), 1e-12);
  is_matrix_absolutely_closed (expected_c_J.toVector (), ((Motion) joint_data.c).toVector(), 1e-12);

  // -------
  q = Vector3 (1., 1., 1.);
  q_dot = Vector3 (1., 1., 1.);

  joint_model.calc (joint_data, q, q_dot);

  printOutJointData <JointDataTranslation> (q, q_dot, joint_data);

  expected_configuration.translation () << 1., 1., 1.;
  expected_v_J.linear () << 1., 1., 1.;

  is_matrix_absolutely_closed (expected_configuration.rotation (), joint_data.M.rotation(), 1e-10);
  is_matrix_absolutely_closed (expected_configuration.translation (), joint_data.M.translation (), 1e-10);
  is_matrix_absolutely_closed (expected_v_J.toVector (), ((Motion) joint_data.v).toVector(), 1e-10);
  is_matrix_absolutely_closed (expected_c_J.toVector (), ((Motion) joint_data.c).toVector(), 1e-10);

  // -------
  q = Vector3 (1., 1.5, 1.9);
  q_dot = Vector3 (2., 3., 1.);

  joint_model.calc (joint_data, q, q_dot);

  printOutJointData <JointDataTranslation> (q, q_dot, joint_data);

  expected_configuration.translation () = q;
  expected_v_J.linear () = q_dot;

  is_matrix_absolutely_closed (expected_configuration.rotation (), joint_data.M.rotation(), 1e-10);
  is_matrix_absolutely_closed (expected_configuration.translation (), joint_data.M.translation (), 1e-10);
  is_matrix_absolutely_closed (expected_v_J.toVector (), ((Motion) joint_data.v).toVector(), 1e-10);
  is_matrix_absolutely_closed (expected_c_J.toVector (), ((Motion) joint_data.c).toVector(), 1e-10);
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

  is_matrix_absolutely_closed (tau_expected, data.tau, 1e-14);

  q = Eigen::VectorXd::Ones (model.nq);
  v = Eigen::VectorXd::Ones (model.nv);
  a = Eigen::VectorXd::Ones (model.nv);

  rnea (model, data, q, v, a);
  tau_expected << 1,     1, 10.81;

  is_matrix_absolutely_closed (tau_expected, data.tau, 1e-12);

  q << 3, 2, 1;
  v = Eigen::VectorXd::Ones (model.nv);
  a = Eigen::VectorXd::Ones (model.nv);

  rnea (model, data, q, v, a);
  tau_expected << 1,     1, 10.81;

  is_matrix_absolutely_closed (tau_expected, data.tau, 1e-12);
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

  is_matrix_absolutely_closed (M_expected, data.M, 1e-14);

  q = Eigen::VectorXd::Ones (model.nq);

  crba (model, data, q);

  is_matrix_absolutely_closed (M_expected, data.M, 1e-12);

  q << 3, 2, 1;
  
  crba (model, data, q);
  
  is_matrix_absolutely_closed (M_expected, data.M, 1e-10);
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

  is_matrix_absolutely_closed (expected_configuration.rotation (), joint_data.M.rotation());
  is_matrix_absolutely_closed (expected_configuration.translation (), joint_data.M.translation ());
  is_matrix_absolutely_closed (expected_v_J.toVector (), ((Motion) joint_data.v).toVector());
  is_matrix_absolutely_closed (expected_c_J.toVector (), ((Motion) joint_data.c).toVector());

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

  is_matrix_absolutely_closed (expected_configuration.rotation (), joint_data.M.rotation(), 1e-12);
  is_matrix_absolutely_closed (expected_configuration.translation (), joint_data.M.translation (), 1e-12);
  is_matrix_absolutely_closed (expected_v_J.toVector (), ((Motion) joint_data.v).toVector(), 1e-12);
  is_matrix_absolutely_closed (expected_c_J.toVector (), ((Motion) joint_data.c).toVector(), 1e-12);

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

  is_matrix_absolutely_closed (expected_configuration.rotation (), joint_data.M.rotation(), 1e-12);
  is_matrix_absolutely_closed (expected_configuration.translation (), joint_data.M.translation (), 1e-12);
  is_matrix_absolutely_closed (expected_v_J.toVector (), ((Motion) joint_data.v).toVector(), 1e-12);
  is_matrix_absolutely_closed (expected_c_J.toVector (), ((Motion) joint_data.c).toVector(), 1e-12);

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

  is_matrix_absolutely_closed (expected_configuration.rotation (), joint_data.M.rotation(), 1e-12);
  is_matrix_absolutely_closed (expected_configuration.translation (), joint_data.M.translation (), 1e-12);
  is_matrix_absolutely_closed (expected_v_J.toVector (), ((Motion) joint_data.v).toVector(), 1e-12);
  is_matrix_absolutely_closed (expected_c_J.toVector (), ((Motion) joint_data.c).toVector(), 1e-12);

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

  is_matrix_absolutely_closed (expected_configuration.rotation (), joint_data.M.rotation(), 1e-10);
  is_matrix_absolutely_closed (expected_configuration.translation (), joint_data.M.translation (), 1e-10);
  is_matrix_absolutely_closed (expected_v_J.toVector (), ((Motion) joint_data.v).toVector(), 1e-10);
  is_matrix_absolutely_closed (expected_c_J.toVector (), ((Motion) joint_data.c).toVector(), 1e-10);

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

  is_matrix_absolutely_closed (expected_configuration.rotation (), joint_data.M.rotation(), 1e-10);
  is_matrix_absolutely_closed (expected_configuration.translation (), joint_data.M.translation (), 1e-10);
  is_matrix_absolutely_closed (expected_v_J.toVector (), ((Motion) joint_data.v).toVector(), 1e-10);
  is_matrix_absolutely_closed (expected_c_J.toVector (), ((Motion) joint_data.c).toVector(), 1e-10);
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

  is_matrix_absolutely_closed (tau_expected, data.tau, 1e-14);

  q = Eigen::VectorXd::Ones (model.nq); q.normalize ();
  v = Eigen::VectorXd::Ones (model.nv);
  a = Eigen::VectorXd::Ones (model.nv);

  rnea (model, data, q, v, a);
  tau_expected << 1,     1, 6.405;

  is_matrix_absolutely_closed (tau_expected, data.tau, 1e-12);

  q << 3, 2, 1, 1; q.normalize ();
  v = Eigen::VectorXd::Ones (model.nv);
  a = Eigen::VectorXd::Ones (model.nv);

  rnea (model, data, q, v, a);
  tau_expected << 1, 4.597,  4.77;

  is_matrix_absolutely_closed (tau_expected, data.tau, 1e-12);
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

  is_matrix_absolutely_closed (M_expected, data.M, 1e-14);

  q = Eigen::VectorXd::Ones (model.nq); q.normalize();

  crba (model, data, q);

  is_matrix_absolutely_closed (M_expected, data.M, 1e-12);
  q << 3, 2, 1, 1; q.normalize();
  
  crba (model, data, q);
  
  is_matrix_absolutely_closed (M_expected, data.M, 1e-10);
}

BOOST_AUTO_TEST_SUITE_END ()


BOOST_AUTO_TEST_SUITE ( JointRevoluteUnaligned )

BOOST_AUTO_TEST_CASE ( test_kinematics )
{
  using namespace se3;


  Eigen::Vector3d axis;
  axis << 1.0, 0.0, 0.0;

  JointModelRevoluteUnaligned joint_model_RU(axis);
  JointDataRevoluteUnaligned joint_data_RU(axis);

  JointModelRX joint_model_RX;
  JointDataRX joint_data_RX;

  joint_model_RU.setIndexes (0, 0, 0);
  joint_model_RX.setIndexes (0, 0, 0);

  Eigen::VectorXd q (Eigen::VectorXd::Zero (1));
  Eigen::VectorXd q_dot (Eigen::VectorXd::Zero (1));

  // -------
  q << 0.;
  q_dot << 0.;

  joint_model_RU.calc (joint_data_RU, q, q_dot);
  joint_model_RX.calc (joint_data_RX, q, q_dot);

  printOutJointData <JointDataRevoluteUnaligned> (q, q_dot, joint_data_RU);

  is_matrix_absolutely_closed (joint_data_RU.M.rotation(), joint_data_RX.M.rotation());
  is_matrix_absolutely_closed (joint_data_RU.M.translation (), joint_data_RX.M.translation ());
  is_matrix_absolutely_closed (((Motion) joint_data_RU.v).toVector(), ((Motion) joint_data_RX.v).toVector());
  is_matrix_absolutely_closed (((Motion) joint_data_RU.c).toVector(), ((Motion) joint_data_RX.c).toVector());

}

BOOST_AUTO_TEST_CASE ( test_rnea )
{
  using namespace se3;
  typedef Eigen::Matrix <double, 3, 1> Vector3;
  typedef Eigen::Matrix <double, 3, 3> Matrix3;

  Model modelRX;
  Model modelRU;
  Inertia inertia (1., Vector3 (0.5, 0., 0.0), Matrix3::Identity ());

  Eigen::Vector3d axis;
  axis << 1.0, 0.0, 0.0;

  modelRX.addBody (modelRX.getBodyId("universe"), JointModelRX (), SE3::Identity (), inertia, "root");
  modelRU.addBody (modelRU.getBodyId("universe"), JointModelRevoluteUnaligned (axis), SE3::Identity (), inertia, "root");

  Data dataRX (modelRX);
  Data dataRU (modelRU);

  BOOST_CHECK_EQUAL(modelRU.nq,modelRX.nq);
  BOOST_CHECK_EQUAL(modelRU.nv,modelRX.nv);
  

  Eigen::VectorXd q (Eigen::VectorXd::Zero (modelRU.nq));
  Eigen::VectorXd v (Eigen::VectorXd::Zero (modelRU.nv));
  Eigen::VectorXd a (Eigen::VectorXd::Zero (modelRU.nv));

  rnea (modelRX, dataRX, q, v, a);
  rnea (modelRU, dataRU, q, v, a);

  is_matrix_absolutely_closed (dataRX.tau, dataRU.tau, 1e-14);

  q = Eigen::VectorXd::Ones (modelRU.nq); //q.normalize ();
  v = Eigen::VectorXd::Ones (modelRU.nv);
  a = Eigen::VectorXd::Ones (modelRU.nv);

  rnea (modelRX, dataRX, q, v, a);
  rnea (modelRU, dataRU, q, v, a);

  is_matrix_absolutely_closed (dataRX.tau, dataRU.tau, 1e-12);

  q << 3.;
  v = Eigen::VectorXd::Ones (modelRU.nv);
  a = Eigen::VectorXd::Ones (modelRU.nv);

  rnea (modelRX, dataRX, q, v, a);
  rnea (modelRU, dataRU, q, v, a);

  is_matrix_absolutely_closed (dataRX.tau, dataRU.tau, 1e-12);
}

BOOST_AUTO_TEST_CASE ( test_crba )
{
  using namespace se3;
  using namespace std;
  typedef Eigen::Matrix <double, 3, 1> Vector3;
  typedef Eigen::Matrix <double, 3, 3> Matrix3;

  Model modelRX;
  Model modelRU;
  Inertia inertia (1., Vector3 (0.5, 0., 0.0), Matrix3::Identity ());

  Eigen::Vector3d axis;
  axis << 1.0, 0.0, 0.0;

  modelRX.addBody (modelRX.getBodyId("universe"), JointModelRX (), SE3::Identity (), inertia, "root");
  modelRU.addBody (modelRU.getBodyId("universe"), JointModelRevoluteUnaligned (axis), SE3::Identity (), inertia, "root");

  Data dataRX (modelRX);
  Data dataRU (modelRU);

  BOOST_CHECK_EQUAL(modelRU.nq,modelRX.nq);
  BOOST_CHECK_EQUAL(modelRU.nv,modelRX.nv);


  Eigen::VectorXd q (Eigen::VectorXd::Zero (modelRU.nq));

  crba (modelRX, dataRX, q);
  crba (modelRU, dataRU, q);

  is_matrix_absolutely_closed (dataRX.M, dataRU.M, 1e-14);

  // ----
  q = Eigen::VectorXd::Ones (modelRU.nq);

  crba (modelRX, dataRX, q);
  crba (modelRU, dataRU, q);

  is_matrix_absolutely_closed (dataRX.M, dataRU.M, 1e-14);

  // ----
  q << 3;
  
  crba (modelRX, dataRX, q);
  crba (modelRU, dataRU, q);

  is_matrix_absolutely_closed (dataRX.M, dataRU.M, 1e-14);
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
  is_matrix_absolutely_closed (mergedInertia.lever(), expected_com);
  is_matrix_absolutely_closed (mergedInertia.inertia().matrix(), expectedBodyInertia);
  
  exit(0);
}

BOOST_AUTO_TEST_SUITE_END ()

BOOST_AUTO_TEST_SUITE ( JointDense )

BOOST_AUTO_TEST_CASE ( toJointModelDense )
{
  using namespace se3;


  JointModelRX jmodel;
  jmodel.setIndexes (2, 0, 0);

  JointModelDense<JointModelBase<JointModelRX>::NQ, JointModelBase<JointModelRX>::NV> jmd(jmodel.id(), jmodel.idx_q(), jmodel.idx_v(), jmodel.lowerPosLimit(),
                          jmodel.upperPosLimit(), jmodel.maxEffortLimit(), jmodel.maxVelocityLimit());
  JointModelDense<JointModelBase<JointModelRX>::NQ, JointModelBase<JointModelRX>::NV> jmd2 = jmodel.toDense();
  (void)jmd; (void)jmd2;

  assert(jmd.idx_q() == jmodel.idx_q() && "The comparison of the joint index in configuration space failed");
  assert(jmd.idx_q() == jmd2.idx_q() && "The comparison of the joint index in  configuration space failed");

  assert(jmd.idx_v() == jmodel.idx_v() && "The comparison of the joint index in velocity space failed");
  assert(jmd.idx_v() == jmd2.idx_v() && "The comparison of the joint index in  velocity space failed");

  assert(jmd.id() == jmodel.id() && "The comparison of the joint index in model's kinematic tree failed");
  assert(jmd.id() == jmd2.id() && "The comparison of the joint index in model's kinematic tree failed");

}

BOOST_AUTO_TEST_CASE ( toJointDataDense )
{
  using namespace se3;

  JointModelRX jmodel;
  jmodel.setIndexes (2, 0, 0);

  JointDataRX jdata;

  JointDataDense< JointDataBase<JointModelRX::JointData>::NQ,
                  JointDataBase<JointModelRX::JointData>::NV
                                                                      > jdd = jdata.toDense();

  assert(jdata.S.nv() == jdd.S.nv() && "");

}
BOOST_AUTO_TEST_SUITE_END ()
