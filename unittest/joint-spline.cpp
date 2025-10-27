//
// Copyright (c) 2025 INRIA
//

#include "pinocchio/math/fwd.hpp"
#include "pinocchio/multibody/joint/joints.hpp"
#include "pinocchio/algorithm/splines.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/aba.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/compute-all-terms.hpp"

#include <boost/test/unit_test.hpp>
#include <iostream>

// Pour calc:
// Tu peux faire le fitting de trajectoire en Python:
// Multibody complexe sur N contrôl points extrait de configuration random
// Stocker la vitesse de la trajectoire généré pour comparer à S*v
// Comment faire la correspondance q, v spline avec celui du modèle initial ?
// Si tu mes N fois le même contrôle point (N étant l'ordre de la BSpline), tu devrais forcer le
// passage à un point Ta spline devient une polyligne simple à tester

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

/// @brief Get a predefined  spline joint trajectory.
/// This was taken from the model opensim gait10dof18mus.osim, and correspond the knee joint.
void getTrajectory(std::vector<SE3> & ctrlFrames)
{
  Eigen::Matrix3d rotation;
  Eigen::Vector3d translation;
  rotation << -0.500004, 0, -0.866023, 0, 1, 0, 0.866023, 0, -0.500004;
  translation << -0.0032, -0.4226, 0;
  ctrlFrames.push_back(pinocchio::SE3(rotation, translation));

  rotation << -0.173649, 0, -0.984808, 0, 1, 0, 0.984808, 0, -0.173649;
  translation << 0.00179, -0.416947, 0;
  ctrlFrames.push_back(pinocchio::SE3(rotation, translation));

  rotation << 0.173652, 0, -0.984807, 0, 1, 0, 0.984807, 0, 0.173652;
  translation << 0.00411, -0.411057, 0;
  ctrlFrames.push_back(pinocchio::SE3(rotation, translation));

  rotation << 0.342021, 0, -0.939692, 0, 1, 0, 0.939692, 0, 0.342021;
  translation << 0.00438827, -0.4082, 0;
  ctrlFrames.push_back(pinocchio::SE3(rotation, translation));

  rotation << 0.499998, 0, -0.866027, 0, 1, 0, 0.866027, 0, 0.499998;
  translation << 0.0041, -0.405495, 0;
  ctrlFrames.push_back(pinocchio::SE3(rotation, translation));

  rotation << 0.766044, 0, -0.642788, 0, 1, 0, 0.642788, 0, 0.766044;
  translation << 0.00212, -0.400825, 0;
  ctrlFrames.push_back(pinocchio::SE3(rotation, translation));

  rotation << 0.866025, 0, -0.5, 0, 1, 0, 0.5, 0, 0.866025;
  translation << 0.000757726, -0.399, 0;
  ctrlFrames.push_back(pinocchio::SE3(rotation, translation));

  rotation << 0.939693, 0, -0.34202, 0, 1, 0, 0.34202, 0, 0.939693;
  translation << -0.001, -0.3976, 0;
  ctrlFrames.push_back(pinocchio::SE3(rotation, translation));

  rotation << 0.984808, 0, -0.173648, 0, 1, 0, 0.173648, 0, 0.984808;
  translation << -0.0031, -0.3966, 0;
  ctrlFrames.push_back(pinocchio::SE3(rotation, translation));

  rotation << 0.987363, 0, 0.158478, 0, 1, 0, -0.158478, 0, 0.987363;
  translation << -0.00513719, -0.395264, 0;
  ctrlFrames.push_back(pinocchio::SE3(rotation, translation));

  rotation << 0.980591, 0, 0.196066, 0, 1, 0, -0.196066, 0, 0.980591;
  translation << -0.005227, -0.395149, 0;
  ctrlFrames.push_back(pinocchio::SE3(rotation, translation));

  rotation << 0.94362, 0, 0.33103, 0, 1, 0, -0.33103, 0, 0.94362;
  translation << -0.005435, -0.394792, 0;
  ctrlFrames.push_back(pinocchio::SE3(rotation, translation));

  rotation << 0.882249, 0, 0.470783, 0, 1, 0, -0.470783, 0, 0.882249;
  translation << -0.005574, -0.394507, 0;
  ctrlFrames.push_back(pinocchio::SE3(rotation, translation));

  rotation << 0.0493163, 0, 0.998783, 0, 1, 0, -0.998783, 0, 0.0493163;
  translation << -0.005435, -0.394812, 0;
  ctrlFrames.push_back(pinocchio::SE3(rotation, translation));

  rotation << -0.500004, 0, 0.866023, 0, 1, 0, -0.866023, 0, -0.500004;
  translation << -0.00525, -0.396, 0;
  ctrlFrames.push_back(pinocchio::SE3(rotation, translation));
}

BOOST_AUTO_TEST_SUITE(JointSpline)

/// @brief Test on the knot vector generation
BOOST_AUTO_TEST_CASE(makeKnots)
{
  int degree = 3;

  std::vector<SE3> ctrlFrames;
  ctrlFrames.push_back(SE3::Identity());
  ctrlFrames.push_back(SE3::Random());
  ctrlFrames.push_back(SE3::Random());

  BOOST_CHECK_THROW(JointModelSpline(ctrlFrames, degree), std::invalid_argument);

  ctrlFrames.push_back(SE3::Random());

  JointModelSpline jmodel(ctrlFrames, degree);

  // Check size
  BOOST_CHECK(jmodel.knots.size() == (degree + ctrlFrames.size() + 1));

  // Check Values
  Eigen::VectorXd knots_expected(degree + ctrlFrames.size() + 1);
  knots_expected << 0., 0., 0., 0., 1., 1., 1., 1.;
  BOOST_CHECK(jmodel.knots == knots_expected);
}

/// @brief Test to make sure the relative motions are correct
BOOST_AUTO_TEST_CASE(relativeMotions)
{
  int degree = 3;

  std::vector<SE3> ctrlFrames;
  ctrlFrames.push_back(SE3::Identity());
  ctrlFrames.push_back(SE3::Random());
  ctrlFrames.push_back(SE3::Random());
  ctrlFrames.push_back(SE3::Random());

  JointModelSpline jmodel(ctrlFrames, degree);

  // Check size
  BOOST_CHECK(jmodel.relativeMotions.size() == (ctrlFrames.size() - 1));

  // check values
  for (size_t i = 0; i < ctrlFrames.size() - 1; i++)
    BOOST_CHECK(jmodel.relativeMotions[i].isApprox(
      pinocchio::log6(ctrlFrames[i].inverse() * ctrlFrames[i + 1])));
}

/// @brief Test on the basisSpline function and its first derivative
BOOST_AUTO_TEST_CASE(basisSplineFunctions)
{
  int degree = 3;

  std::vector<SE3> ctrlFrames;
  ctrlFrames.push_back(SE3::Identity());
  ctrlFrames.push_back(SE3::Random());
  ctrlFrames.push_back(SE3::Random());
  ctrlFrames.push_back(SE3::Random());

  JointModelSpline jmodel(ctrlFrames, degree);
  jmodel.setIndexes(0, 0, 0);
  JointDataSpline jdata = jmodel.createData();

  Eigen::VectorXd q(1);
  q << 0;
  jmodel.calc(jdata, q);

  Eigen::VectorXd bSpline_expected(4);
  bSpline_expected << 1, 0, 0, 0;
  BOOST_CHECK(bSpline_expected.isApprox(jdata.N));

  q << 1;
  jmodel.calc(jdata, q);
  bSpline_expected << 0, 0, 0, 1;
  BOOST_CHECK(bSpline_expected.isApprox(jdata.N));

  // Check the integral
  q << 0.3;
  jmodel.calc(jdata, q);
  BOOST_CHECK_CLOSE(jdata.N.sum(), 1, 1e-8);

  // Check first derivative
  for (size_t i = 0; i < jmodel.nbCtrlFrames; i++)
  {
    double den1 = (jmodel.knots[i + degree] - jmodel.knots[i]);
    double left = 0;
    if (den1 > Eigen::NumTraits<double>::dummy_precision())
      left = degree / den1 * jmodel.bsplineBasis(i, degree - 1, q[0]);

    double den2 = (jmodel.knots[i + degree + 1] - jmodel.knots[i + 1]);
    double right = 0;
    if (den2 > Eigen::NumTraits<double>::dummy_precision())
      right = degree / den2 * jmodel.bsplineBasis(i + 1, degree - 1, q[0]);

    BOOST_CHECK_CLOSE(left - right, jdata.N_der[i], 1e-5);
  }
}

/// @brief Test the spanning function
BOOST_AUTO_TEST_CASE(findSpan)
{
  int degree = 2;

  std::vector<SE3> ctrlFrames;
  ctrlFrames.push_back(SE3::Identity());
  for (int k = 0; k < 10; k++)
    ctrlFrames.push_back(SE3::Random());

  JointModelSpline jmodel(ctrlFrames, degree);

  Eigen::VectorXd q(1);
  q << 0.5;
  SpanIndexes indexes =
    pinocchio::FindSpan<double, 0>::run(q, degree, ctrlFrames.size(), jmodel.knots);

  BOOST_CHECK(indexes.start_idx == 5);
  BOOST_CHECK(indexes.end_idx == 8);

  q[0] = 1;
  indexes = pinocchio::FindSpan<double, 0>::run(q, degree, ctrlFrames.size(), jmodel.knots);

  BOOST_CHECK(indexes.start_idx == ctrlFrames.size() - (degree + 1));
  BOOST_CHECK(indexes.end_idx == ctrlFrames.size());
}

/// @brief Comparing a simple spline joint with a PZ
/// Make sure pose and joint subspace are the same
BOOST_AUTO_TEST_CASE(vsPrismaticZ)
{
  using namespace pinocchio;
  typedef SE3::Vector3 Vector3;
  typedef SE3::Matrix3 Matrix3;

  // Spline Joint
  std::vector<SE3> ctrlFrames;
  ctrlFrames.push_back(SE3::Identity());
  ctrlFrames.push_back(SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0., 0., 1.)));

  JointModelSpline jmodel(ctrlFrames, 1);
  JointDataSpline jdata = jmodel.createData();
  jmodel.setIndexes(0, 0, 0);

  // Prismatic joint
  JointModelPZ jmodelPz;
  JointDataPZ jdataPz = jmodelPz.createData();
  jmodelPz.setIndexes(0, 0, 0);

  Eigen::VectorXd q(Eigen::VectorXd::Zero(1));

  // -------
  q << 0.2;

  jmodel.calc(jdata, q);
  jmodelPz.calc(jdataPz, q);

  BOOST_CHECK(jdata.M.isApprox(jdataPz.M, 1e-12));
  BOOST_CHECK(jdata.S.matrix().isApprox(jdataPz.S.matrix(), 1e-12));

  // -------
  Eigen::VectorXd q_dot(Eigen::VectorXd::Zero(1));
  q << 0.3;
  q_dot << 0.4;

  jmodel.calc(jdata, q, q_dot);
  jmodelPz.calc(jdataPz, q, q_dot);

  BOOST_CHECK(jdata.M.isApprox(jdataPz.M, 1e-12));
  BOOST_CHECK(jdata.S.matrix().isApprox(jdataPz.S.matrix(), 1e-12));
  BOOST_CHECK(jdata.v.isApprox(jdataPz.v, 1e-12));
  BOOST_CHECK(jdata.c.isApprox(jdataPz.c, 1e-12));
}

/// @brief Comparing a simple spline joint with a RX
/// Make sure pose and joint subspace are the same
BOOST_AUTO_TEST_CASE(vsRevoluteX)
{
  using namespace pinocchio;
  typedef SE3::Vector3 Vector3;
  typedef SE3::Matrix3 Matrix3;

  // Spline Joint
  Eigen::Matrix3d rotation;
  std::vector<SE3> ctrlFrames;
  Eigen::AngleAxisd Rx(1, Eigen::Vector3d::UnitX());

  ctrlFrames.push_back(SE3::Identity());
  ctrlFrames.push_back(SE3(Rx.toRotationMatrix(), Eigen::Vector3d(0., 0., 0.)));

  JointModelSpline jmodel(ctrlFrames, 1);
  JointDataSpline jdata = jmodel.createData();
  jmodel.setIndexes(0, 0, 0);

  // Prismatic joint
  JointModelRX jmodelRx;
  JointDataRX jdataRx = jmodelRx.createData();
  jmodelRx.setIndexes(0, 0, 0);

  Eigen::VectorXd q(Eigen::VectorXd::Zero(1));

  // -------
  q << 0.2;

  jmodel.calc(jdata, q);
  jmodelRx.calc(jdataRx, q);

  BOOST_CHECK(jdata.M.isApprox(jdataRx.M, 1e-12));
  BOOST_CHECK(jdata.S.matrix().isApprox(jdataRx.S.matrix(), 1e-12));

  // -------
  Eigen::VectorXd q_dot(Eigen::VectorXd::Zero(1));
  q << 0.3;
  q_dot << 0.4;

  jmodel.calc(jdata, q, q_dot);
  jmodelRx.calc(jdataRx, q, q_dot);

  BOOST_CHECK(jdata.M.isApprox(jdataRx.M, 1e-12));
  BOOST_CHECK(jdata.S.matrix().isApprox(jdataRx.S.matrix(), 1e-12));
  BOOST_CHECK(jdata.v.isApprox(jdataRx.v, 1e-12));
  BOOST_CHECK(jdata.c.isApprox(jdataRx.c, 1e-12));
}

/// @brief Test out rnea vs aba
BOOST_AUTO_TEST_CASE(abaVSrnea)
{
  typedef SE3::Vector3 Vector3;
  typedef SE3::Matrix3 Matrix3;

  Model modelSpline;

  Inertia inertia(1., Vector3(0., 0., 0.0), Matrix3::Identity());

  std::vector<SE3> ctrlFrames;
  getTrajectory(ctrlFrames);
  addJointAndBody(
    modelSpline, JointModelSpline(ctrlFrames), 0, SE3::Identity(), "kneeSpline", inertia);
  Data dataSplineRnea(modelSpline);
  Data dataSplineAba(modelSpline);

  Eigen::VectorXd q(modelSpline.nq);
  pinocchio::randomConfiguration(
    modelSpline, Eigen::VectorXd::Zero(1), Eigen::VectorXd::Ones(1), q);
  Eigen::VectorXd vq(Eigen::VectorXd::Random(modelSpline.nv));
  Eigen::VectorXd aq(Eigen::VectorXd::Random(modelSpline.nv));
  Eigen::VectorXd tauRnea(1);
  Eigen::VectorXd aAba(1);

  tauRnea = pinocchio::rnea(modelSpline, dataSplineRnea, q, vq, aq);
  aAba = pinocchio::aba(modelSpline, dataSplineAba, q, vq, tauRnea);

  BOOST_CHECK(aq.isApprox(aAba));
}

/// @brief Test S and bias c computation via finite differences
BOOST_AUTO_TEST_CASE(vsFiniteDifference)
{
  using namespace pinocchio;

  typedef typename JointModelSpline::ConfigVector_t CV;
  typedef typename JointModelSpline::TangentVector_t TV;
  typedef typename LieGroup<JointModelSpline>::type LieGroupType;

  std::vector<SE3> ctrlFrames;
  getTrajectory(ctrlFrames);

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
  // Check S
  {
    jmodel.calc(jdata, q_ref);
    SE3 M_ref(jdata.M);
    Eigen::Matrix<double, 6, JointModelSpline::NV> S(6, JointModelSpline::NV),
      S_ref(jdata.S.matrix());

    jmodel.calc(jdata, q);
    SE3 M_ = jdata.M;

    S.col(0) = log6(M_ref.inverse() * M_).toVector();
    S.col(0) /= eps;

    BOOST_CHECK(S.isApprox(S_ref, 1e-6));
  }
  // Check bias
  {
    jmodel.calc(jdata, q_ref, q_dot_ref);
    const Motion & c_ref = jdata.c;
    Eigen::Matrix<double, 6, JointModelSpline::NV> S_ref(jdata.S.matrix());

    jmodel.calc(jdata, q);
    Eigen::Matrix<double, 6, JointModelSpline::NV> S_(jdata.S.matrix());

    Motion dSdq_fd((S_ - S_ref) / eps);
    Motion c_fd = dSdq_fd * q_dot_ref[0] * q_dot_ref[0];

    BOOST_CHECK(c_ref.isApprox(c_fd, 1e-6));

    std::cout << c_fd << std::endl;
    std::cout << c_ref << std::endl;
  }
}

BOOST_AUTO_TEST_SUITE_END()
