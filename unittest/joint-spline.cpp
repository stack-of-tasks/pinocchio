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

BOOST_AUTO_TEST_SUITE(JointSpline)

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

BOOST_AUTO_TEST_CASE(expectedMotion)
{
  using namespace pinocchio;
  typedef SE3::Vector3 Vector3;
  typedef SE3::Matrix3 Matrix3;

  Motion expected_v_J(Motion::Zero());
  Motion expected_c_J(Motion::Zero());

  SE3 expected_configuration(SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0., 0., 0.2)));

  std::vector<SE3> ctrlFrames;
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

BOOST_AUTO_TEST_CASE(vsFiniteDiff)
{
  using namespace pinocchio;
  typedef SE3::Vector3 Vector3;
  typedef SE3::Matrix3 Matrix3;

  typedef typename JointModelSpline::ConfigVector_t CV;
  typedef typename JointModelSpline::TangentVector_t TV;
  typedef typename LieGroup<JointModelSpline>::type LieGroupType;

  std::vector<SE3> ctrlFrames;
  ctrlFrames.push_back(SE3::Identity());
  ctrlFrames.push_back(SE3::Random());
  ctrlFrames.push_back(SE3::Random());
  ctrlFrames.push_back(SE3::Random());

  JointModelSpline jmodel(ctrlFrames);
  JointDataSpline jdata = jmodel.createData();

  jmodel.setIndexes(0, 0, 0);

  double eps = 1e-8;
  CV q_ref = LieGroupType().randomConfiguration(CV::Zero(), CV::Ones());

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
  // {
  //   jmodel.calc(jdata, q_ref, q_dot_ref);
  //   const Motion & c_ref = jdata.c;
  //   Eigen::Matrix<double, 6, JointModelSpline::NV> S_ref(jdata.S.matrix());

  //   jmodel.calc(jdata, q);
  //   Eigen::Matrix<double, 6, JointModelSpline::NV> S_(jdata.S.matrix());

  //   Motion dSdq_fd((S_ - S_ref) / eps);
  //   Motion c_fd = dSdq_fd * q_dot_ref[0];

  //   BOOST_CHECK(c_ref.isApprox(c_fd, eps * 1e1));
  // }
}

/// @brief Test out rnea vs aba
/// Control frames were taken from an opensim model and represent the movement of the knee joint
BOOST_AUTO_TEST_CASE(abaVSrnea)
{
  typedef SE3::Vector3 Vector3;
  typedef SE3::Matrix3 Matrix3;

  Model modelSpline;

  Inertia inertia(1., Vector3(0., 0., 0.0), Matrix3::Identity());

  std::vector<SE3> ctrlFrames;
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

BOOST_AUTO_TEST_SUITE_END()
