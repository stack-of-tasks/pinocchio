#include <iostream>

#include "pinocchio/spatial/force.hpp"
#include "pinocchio/spatial/motion.hpp"
#include "pinocchio/spatial/se3.hpp"
#include "pinocchio/spatial/inertia.hpp"
#include "pinocchio/multibody/joint/joint-revolute.hpp"
#include "pinocchio/multibody/joint/joint-prismatic.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/crba.hpp"

#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE JointsTest
#include <boost/test/unit_test.hpp>
#include <boost/test/floating_point_comparison.hpp>

//#define VERBOSE

template <typename JoinData_t>
void printOutJointData (const Eigen::VectorXd & q,
                        const Eigen::VectorXd & q_dot,
                        const JoinData_t & joint_data
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

inline void is_matrix_closed (const Eigen::MatrixXd & M1,
                              const Eigen::MatrixXd & M2,
                              double tolerance = std::numeric_limits <Eigen::MatrixXd::Scalar>::epsilon ()
                              )
{
  BOOST_REQUIRE_EQUAL (M1.rows (), M2.rows ());
  BOOST_REQUIRE_EQUAL (M1.cols (), M2.cols ());

  for (Eigen::MatrixXd::Index i = 0; i < M1.rows (); i++)
  {
    for (Eigen::MatrixXd::Index j = 0; j < M1.cols (); j++)
    {
      BOOST_CHECK_CLOSE (M1 (i,j), M2 (i,j), tolerance);
    }
  }
}


BOOST_AUTO_TEST_SUITE ( JointSphericalZYX )

BOOST_AUTO_TEST_CASE ( test_kinematics )
{
  // using namespace se3;

  // typedef Motion::Vector3 Vector3;
  // typedef Motion::Vector6 Vector6;

  // Motion expected_v_J (Motion::Zero ());
  // Motion expected_c_J (Motion::Zero ());

  // SE3 expected_configuration (SE3::Identity ());

  // JointDataSphericalZYX joint_data;
  // JointModelSphericalZYX joint_model;

  // joint_model.setIndexes (0, 0, 0);

  // Vector3 q (Vector3::Zero());
  // Vector3 q_dot (Vector3::Zero());

  // // -------
  // q = Vector3 (0., 0., 0.);
  // q_dot = Vector3 (0., 0., 0.);

  // joint_model.calc (joint_data, q, q_dot);

  // printOutJointData <JointDataSphericalZYX> (q, q_dot, joint_data);

  // is_matrix_closed (expected_configuration.rotation (), joint_data.M.rotation());
  // is_matrix_closed (expected_configuration.translation (), joint_data.M.translation ());
  // is_matrix_closed (expected_v_J.toVector (), ((Motion) joint_data.v).toVector());
  // is_matrix_closed (expected_c_J.toVector (), ((Motion) joint_data.c).toVector());

  // // -------
  // q = Vector3 (1., 0., 0.);
  // q_dot = Vector3 (1., 0., 0.);

  // joint_model.calc (joint_data, q, q_dot);

  // printOutJointData <JointDataSphericalZYX> (q, q_dot, joint_data);

  // expected_configuration.rotation ().transpose () <<
  // 0.54030230586814,  0.8414709848079,               -0,
  // -0.8414709848079, 0.54030230586814,                0,
  // 0,                0,               1;

  // expected_v_J.angular () << 0., 0., 1.;

  // is_matrix_closed (expected_configuration.rotation (), joint_data.M.rotation(), 1e-12);
  // is_matrix_closed (expected_configuration.translation (), joint_data.M.translation (), 1e-12);
  // is_matrix_closed (expected_v_J.toVector (), ((Motion) joint_data.v).toVector(), 1e-12);
  // is_matrix_closed (expected_c_J.toVector (), ((Motion) joint_data.c).toVector(), 1e-12);

  // // -------
  // q = Vector3 (0., 1., 0.);
  // q_dot = Vector3 (0., 1., 0.);

  // joint_model.calc (joint_data, q, q_dot);

  // printOutJointData <JointDataSphericalZYX> (q, q_dot, joint_data);

  // expected_configuration.rotation ().transpose () <<
  // 0.54030230586814,                0, -0.8414709848079,
  // 0,                1,                0,
  // 0.8414709848079,                0, 0.54030230586814;

  // expected_v_J.angular () << 0., 1., 0.;

  // is_matrix_closed (expected_configuration.rotation (), joint_data.M.rotation(), 1e-12);
  // is_matrix_closed (expected_configuration.translation (), joint_data.M.translation (), 1e-12);
  // is_matrix_closed (expected_v_J.toVector (), ((Motion) joint_data.v).toVector(), 1e-12);
  // is_matrix_closed (expected_c_J.toVector (), ((Motion) joint_data.c).toVector(), 1e-12);

  // // -------
  // q = Vector3 (0., 0., 1.);
  // q_dot = Vector3 (0., 0., 1.);

  // joint_model.calc (joint_data, q, q_dot);

  // printOutJointData <JointDataSphericalZYX> (q, q_dot, joint_data);

  // expected_configuration.rotation ().transpose () <<
  // 1,                0,               -0,
  // 0, 0.54030230586814,  0.8414709848079,
  // 0, -0.8414709848079, 0.54030230586814;

  // expected_v_J.angular () << 1., 0., 0.;

  // is_matrix_closed (expected_configuration.rotation (), joint_data.M.rotation(), 1e-12);
  // is_matrix_closed (expected_configuration.translation (), joint_data.M.translation (), 1e-12);
  // is_matrix_closed (expected_v_J.toVector (), ((Motion) joint_data.v).toVector(), 1e-12);
  // is_matrix_closed (expected_c_J.toVector (), ((Motion) joint_data.c).toVector(), 1e-12);

  // // -------
  // q = Vector3 (1., 1., 1.);
  // q_dot = Vector3 (1., 1., 1.);

  // joint_model.calc (joint_data, q, q_dot);

  // printOutJointData <JointDataSphericalZYX> (q, q_dot, joint_data);

  // expected_configuration.rotation ().transpose () <<
  // 0.29192658172643,   0.45464871341284,   -0.8414709848079,
  // -0.072075012795695,   0.88774981831738,   0.45464871341284,
  // 0.95372116649051, -0.072075012795695,   0.29192658172643;

  // expected_v_J.angular () << 0.1585290151921,  0.99495101928098, -0.54954440308147;
  // expected_c_J.angular () << -0.54030230586814,   -1.257617821355,  -1.4495997326938;

  // is_matrix_closed (expected_configuration.rotation (), joint_data.M.rotation(), 1e-10);
  // is_matrix_closed (expected_configuration.translation (), joint_data.M.translation (), 1e-10);
  // is_matrix_closed (expected_v_J.toVector (), ((Motion) joint_data.v).toVector(), 1e-10);
  // is_matrix_closed (expected_c_J.toVector (), ((Motion) joint_data.c).toVector(), 1e-10);

  // // -------
  // q = Vector3 (1., 1.5, 1.9);
  // q_dot = Vector3 (2., 3., 1.);

  // joint_model.calc (joint_data, q, q_dot);

  // printOutJointData <JointDataSphericalZYX> (q, q_dot, joint_data);

  // expected_configuration.rotation ().transpose () <<
  // 0.03821947317172,  0.059523302749877,  -0.99749498660405,
  // 0.78204612603915,   0.61961526601658,   0.06693862014091,
  // 0.62204752922718,   -0.7826454488138, -0.022868599288288;

  // expected_v_J.angular () << -0.99498997320811, -0.83599146030869,  -2.8846374616388;
  // expected_c_J.angular () << -0.42442321000622,  -8.5482150213859,   2.7708697933151;

  // is_matrix_closed (expected_configuration.rotation (), joint_data.M.rotation(), 1e-10);
  // is_matrix_closed (expected_configuration.translation (), joint_data.M.translation (), 1e-10);
  // is_matrix_closed (expected_v_J.toVector (), ((Motion) joint_data.v).toVector(), 1e-10);
  // is_matrix_closed (expected_c_J.toVector (), ((Motion) joint_data.c).toVector(), 1e-10);
}

BOOST_AUTO_TEST_CASE ( test_rnea )
{
  // using namespace se3;
  // typedef Eigen::Matrix <double, 3, 1> Vector3;
  // typedef Eigen::Matrix <double, 3, 3> Matrix3;

  // Model model;
  // Inertia inertia (1., Vector3 (0.5, 0., 0.0), Matrix3::Identity ());

  // model.addBody (model.getBodyId("universe"), JointModelSphericalZYX (), SE3::Identity (), inertia, "root");

  // Data data (model);

  // Eigen::VectorXd q = Eigen::VectorXd::Zero (model.nq);
  // Eigen::VectorXd v = Eigen::VectorXd::Zero (model.nv);
  // Eigen::VectorXd a = Eigen::VectorXd::Zero (model.nv);

  // rnea (model, data, q, v, a);
  // Vector3 tau_expected (0., -4.905, 0.);

  // is_matrix_closed (tau_expected, data.tau, 1e-14);

  // q = Eigen::VectorXd::Ones (model.nq);
  // v = Eigen::VectorXd::Ones (model.nv);
  // a = Eigen::VectorXd::Ones (model.nv);

  // rnea (model, data, q, v, a);
  // tau_expected << -0.53611600195085, -0.74621832606188, -0.38177329067604;

  // is_matrix_closed (tau_expected, data.tau, 1e-12);

  // q << 3, 2, 1;
  // v = Eigen::VectorXd::Ones (model.nv);
  // a = Eigen::VectorXd::Ones (model.nv);

  // rnea (model, data, q, v, a);
  // tau_expected << 0.73934458094049,  2.7804530848031, 0.50684940972146;

  // is_matrix_closed (tau_expected, data.tau, 1e-12);

}

BOOST_AUTO_TEST_CASE ( test_crba )
{
  // using namespace se3;
  // using namespace std;
  // typedef Eigen::Matrix <double, 3, 1> Vector3;
  // typedef Eigen::Matrix <double, 3, 3> Matrix3;

  // Model model;
  // Inertia inertia (1., Vector3 (0.5, 0., 0.0), Matrix3::Identity ());

  // model.addBody (model.getBodyId("universe"), JointModelSphericalZYX (), SE3::Identity (), inertia, "root");

  // Data data (model);

  // Eigen::VectorXd q (Eigen::VectorXd::Zero (model.nq));
  // Eigen::MatrixXd M_expected (model.nv,model.nv);

  // crba (model, data, q);
  // M_expected <<
  // 1.25,    0,    0,
  // 0, 1.25,    0,
  // 0,    0,    1;

  // is_matrix_closed (M_expected, data.M, 1e-14);

  // q = Eigen::VectorXd::Ones (model.nq);

  // crba (model, data, q);
  // M_expected <<
  // 1.0729816454316, -5.5511151231258e-17,     -0.8414709848079,
  // -5.5511151231258e-17,                 1.25,                    0,
  // -0.8414709848079,                    0,                    1;

  // is_matrix_closed (M_expected, data.M, 1e-12);

  // q << 3, 2, 1;

  // crba (model, data, q);
  // M_expected <<
  // 1.043294547392, 2.7755575615629e-17,   -0.90929742682568,
  // 0,                1.25,                   0,
  // -0.90929742682568,                   0,                  1;

  // is_matrix_closed (M_expected, data.M, 1e-10);
}

BOOST_AUTO_TEST_SUITE_END ()


BOOST_AUTO_TEST_SUITE ( JointPrismatic )

BOOST_AUTO_TEST_CASE ( test_kinematics )
{
  using namespace se3;

  typedef Motion::Vector3 Vector3;
  typedef Motion::Vector6 Vector6;

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

  is_matrix_closed (expected_configuration.rotation (), joint_data.M.rotation());
  is_matrix_closed (expected_configuration.translation (), joint_data.M.translation ());
  is_matrix_closed (expected_v_J.toVector (), ((Motion) joint_data.v).toVector());
  is_matrix_closed (expected_c_J.toVector (), ((Motion) joint_data.c).toVector());

  // -------
  q << 1.;
  q_dot << 1.;


  joint_model.calc (joint_data, q, q_dot);

  printOutJointData <JointDataPX> (q, q_dot, joint_data);

  expected_configuration.translation () << 1, 0, 0;

  expected_v_J.linear () << 1., 0., 0.;

  is_matrix_closed (expected_configuration.rotation (), joint_data.M.rotation(), 1e-12);
  is_matrix_closed (expected_configuration.translation (), joint_data.M.translation (), 1e-12);
  is_matrix_closed (expected_v_J.toVector (), ((Motion) joint_data.v).toVector(), 1e-12);
  is_matrix_closed (expected_c_J.toVector (), ((Motion) joint_data.c).toVector(), 1e-12);

}

BOOST_AUTO_TEST_CASE ( test_rnea )
{
  using namespace se3;
  typedef Eigen::VectorXd VectorXd;
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

  is_matrix_closed (tau_expected, data.tau, 1e-14);

  // -----
  q = Eigen::VectorXd::Ones (model.nq);
  v = Eigen::VectorXd::Ones (model.nv);
  a = Eigen::VectorXd::Ones (model.nv);

  rnea (model, data, q, v, a);
  tau_expected << 1;

  is_matrix_closed (tau_expected, data.tau, 1e-12);

  q << 3;
  v = Eigen::VectorXd::Ones (model.nv);
  a = Eigen::VectorXd::Ones (model.nv);

  rnea (model, data, q, v, a);
  tau_expected << 1;

  is_matrix_closed (tau_expected, data.tau, 1e-12);
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

  is_matrix_closed (M_expected, data.M, 1e-14);

  q = Eigen::VectorXd::Ones (model.nq);

  crba (model, data, q);

  is_matrix_closed (M_expected, data.M, 1e-12);

  q << 3;

  crba (model, data, q);
  
  is_matrix_closed (M_expected, data.M, 1e-10);
}

BOOST_AUTO_TEST_SUITE_END ()
