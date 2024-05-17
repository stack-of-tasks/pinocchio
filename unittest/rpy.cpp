//
// Copyright (c) 2019-2020 INRIA
//

#include <pinocchio/math/rpy.hpp>
#include <pinocchio/math/quaternion.hpp>
#include <pinocchio/spatial/skew.hpp>

#include <boost/variant.hpp> // to avoid C99 warnings

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

BOOST_AUTO_TEST_CASE(test_rpyToMatrix)
{
  double r = static_cast<double>(rand()) / (static_cast<double>(RAND_MAX / (2 * M_PI))) - M_PI;
  double p = static_cast<double>(rand()) / (static_cast<double>(RAND_MAX / M_PI)) - (M_PI / 2);
  double y = static_cast<double>(rand()) / (static_cast<double>(RAND_MAX / (2 * M_PI))) - M_PI;

  Eigen::Matrix3d R = pinocchio::rpy::rpyToMatrix(r, p, y);

  Eigen::Matrix3d Raa =
    (Eigen::AngleAxisd(y, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(p, Eigen::Vector3d::UnitY())
     * Eigen::AngleAxisd(r, Eigen::Vector3d::UnitX()))
      .toRotationMatrix();

  BOOST_CHECK(R.isApprox(Raa));

  Eigen::Vector3d v;
  v << r, p, y;

  Eigen::Matrix3d Rv = pinocchio::rpy::rpyToMatrix(v);

  BOOST_CHECK(Rv.isApprox(Raa));
  BOOST_CHECK(Rv.isApprox(R));
}

BOOST_AUTO_TEST_CASE(test_matrixToRpy)
{
#ifdef NDEBUG
  const int n = 1e5;
#else
  const int n = 1e2;
#endif
  for (int k = 0; k < n; ++k)
  {
    Eigen::Quaterniond quat;
    pinocchio::quaternion::uniformRandom(quat);
    const Eigen::Matrix3d R = quat.toRotationMatrix();

    const Eigen::Vector3d v = pinocchio::rpy::matrixToRpy(R);
    Eigen::Matrix3d Rprime = pinocchio::rpy::rpyToMatrix(v);

    BOOST_CHECK(Rprime.isApprox(R));
    BOOST_CHECK(-M_PI <= v[0] && v[0] <= M_PI);
    BOOST_CHECK(-M_PI / 2 <= v[1] && v[1] <= M_PI / 2);
    BOOST_CHECK(-M_PI <= v[2] && v[2] <= M_PI);
  }

#ifdef NDEBUG
  const int n2 = 1e3;
#else
  const int n2 = 1e2;
#endif

  // Test singular case theta = pi/2
  for (int k = 0; k < n2; ++k)
  {
    double r = static_cast<double>(rand()) / (static_cast<double>(RAND_MAX / (2 * M_PI))) - M_PI;
    double y = static_cast<double>(rand()) / (static_cast<double>(RAND_MAX / (2 * M_PI))) - M_PI;
    Eigen::Matrix3d Rp;
    Rp << 0.0, 0.0, 1.0, 0.0, 1.0, 0.0, -1.0, 0.0, 0.0;
    const Eigen::Matrix3d R = Eigen::AngleAxisd(y, Eigen::Vector3d::UnitZ()).toRotationMatrix() * Rp
                              * Eigen::AngleAxisd(r, Eigen::Vector3d::UnitX()).toRotationMatrix();

    const Eigen::Vector3d v = pinocchio::rpy::matrixToRpy(R);
    Eigen::Matrix3d Rprime = pinocchio::rpy::rpyToMatrix(v);

    BOOST_CHECK(Rprime.isApprox(R));
    BOOST_CHECK(-M_PI <= v[0] && v[0] <= M_PI);
    BOOST_CHECK(-M_PI / 2 <= v[1] && v[1] <= M_PI / 2);
    BOOST_CHECK(-M_PI <= v[2] && v[2] <= M_PI);
  }

  // Test singular case theta = -pi/2
  for (int k = 0; k < n2; ++k)
  {
    double r = static_cast<double>(rand()) / (static_cast<double>(RAND_MAX / (2 * M_PI))) - M_PI;
    double y = static_cast<double>(rand()) / (static_cast<double>(RAND_MAX / (2 * M_PI))) - M_PI;
    Eigen::Matrix3d Rp;
    Rp << 0.0, 0.0, -1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 0.0;
    const Eigen::Matrix3d R = Eigen::AngleAxisd(y, Eigen::Vector3d::UnitZ()).toRotationMatrix() * Rp
                              * Eigen::AngleAxisd(r, Eigen::Vector3d::UnitX()).toRotationMatrix();

    const Eigen::Vector3d v = pinocchio::rpy::matrixToRpy(R);
    Eigen::Matrix3d Rprime = pinocchio::rpy::rpyToMatrix(v);

    BOOST_CHECK(Rprime.isApprox(R));
    BOOST_CHECK(-M_PI <= v[0] && v[0] <= M_PI);
    BOOST_CHECK(-M_PI / 2 <= v[1] && v[1] <= M_PI / 2);
    BOOST_CHECK(-M_PI <= v[2] && v[2] <= M_PI);
  }
}

BOOST_AUTO_TEST_CASE(test_computeRpyJacobian)
{
  // Check identity at zero
  Eigen::Vector3d rpy(Eigen::Vector3d::Zero());
  Eigen::Matrix3d j0 = pinocchio::rpy::computeRpyJacobian(rpy);
  BOOST_CHECK(j0.isIdentity());
  Eigen::Matrix3d jL = pinocchio::rpy::computeRpyJacobian(rpy, pinocchio::LOCAL);
  BOOST_CHECK(jL.isIdentity());
  Eigen::Matrix3d jW = pinocchio::rpy::computeRpyJacobian(rpy, pinocchio::WORLD);
  BOOST_CHECK(jW.isIdentity());
  Eigen::Matrix3d jA = pinocchio::rpy::computeRpyJacobian(rpy, pinocchio::LOCAL_WORLD_ALIGNED);
  BOOST_CHECK(jA.isIdentity());

  // Check correct identities between different versions
  double r = static_cast<double>(rand()) / (static_cast<double>(RAND_MAX / (2 * M_PI))) - M_PI;
  double p = static_cast<double>(rand()) / (static_cast<double>(RAND_MAX / M_PI)) - (M_PI / 2);
  double y = static_cast<double>(rand()) / (static_cast<double>(RAND_MAX / (2 * M_PI))) - M_PI;
  rpy = Eigen::Vector3d(r, p, y);
  Eigen::Matrix3d R = pinocchio::rpy::rpyToMatrix(rpy);
  j0 = pinocchio::rpy::computeRpyJacobian(rpy);
  jL = pinocchio::rpy::computeRpyJacobian(rpy, pinocchio::LOCAL);
  jW = pinocchio::rpy::computeRpyJacobian(rpy, pinocchio::WORLD);
  jA = pinocchio::rpy::computeRpyJacobian(rpy, pinocchio::LOCAL_WORLD_ALIGNED);
  BOOST_CHECK(j0 == jL);
  BOOST_CHECK(jW == jA);
  BOOST_CHECK(jW.isApprox(R * jL));

  // Check against analytical formulas
  Eigen::Vector3d jL0Expected = Eigen::Vector3d::UnitX();
  Eigen::Vector3d jL1Expected =
    Eigen::AngleAxisd(r, Eigen::Vector3d::UnitX()).toRotationMatrix().transpose().col(1);
  Eigen::Vector3d jL2Expected = (Eigen::AngleAxisd(p, Eigen::Vector3d::UnitY())
                                 * Eigen::AngleAxisd(r, Eigen::Vector3d::UnitX()))
                                  .toRotationMatrix()
                                  .transpose()
                                  .col(2);
  BOOST_CHECK(jL.col(0).isApprox(jL0Expected));
  BOOST_CHECK(jL.col(1).isApprox(jL1Expected));
  BOOST_CHECK(jL.col(2).isApprox(jL2Expected));

  Eigen::Vector3d jW0Expected = (Eigen::AngleAxisd(y, Eigen::Vector3d::UnitZ())
                                 * Eigen::AngleAxisd(p, Eigen::Vector3d::UnitY()))
                                  .toRotationMatrix()
                                  .col(0);
  Eigen::Vector3d jW1Expected =
    Eigen::AngleAxisd(y, Eigen::Vector3d::UnitZ()).toRotationMatrix().col(1);
  Eigen::Vector3d jW2Expected = Eigen::Vector3d::UnitZ();
  BOOST_CHECK(jW.col(0).isApprox(jW0Expected));
  BOOST_CHECK(jW.col(1).isApprox(jW1Expected));
  BOOST_CHECK(jW.col(2).isApprox(jW2Expected));

  // Check against finite differences
  Eigen::Vector3d rpydot = Eigen::Vector3d::Random();
  double const eps = 1e-7;
  double const tol = 1e-5;

  Eigen::Matrix3d dRdr = (pinocchio::rpy::rpyToMatrix(r + eps, p, y) - R) / eps;
  Eigen::Matrix3d dRdp = (pinocchio::rpy::rpyToMatrix(r, p + eps, y) - R) / eps;
  Eigen::Matrix3d dRdy = (pinocchio::rpy::rpyToMatrix(r, p, y + eps) - R) / eps;
  Eigen::Matrix3d Rdot = dRdr * rpydot[0] + dRdp * rpydot[1] + dRdy * rpydot[2];

  Eigen::Vector3d omegaL = jL * rpydot;
  BOOST_CHECK(Rdot.isApprox(R * pinocchio::skew(omegaL), tol));

  Eigen::Vector3d omegaW = jW * rpydot;
  BOOST_CHECK(Rdot.isApprox(pinocchio::skew(omegaW) * R, tol));
}

BOOST_AUTO_TEST_CASE(test_computeRpyJacobianInverse)
{
  // Check correct identities between different versions
  double r = static_cast<double>(rand()) / (static_cast<double>(RAND_MAX / (2 * M_PI))) - M_PI;
  double p = static_cast<double>(rand()) / (static_cast<double>(RAND_MAX / M_PI)) - (M_PI / 2);
  p *= 0.999; // ensure we are not too close to a singularity
  double y = static_cast<double>(rand()) / (static_cast<double>(RAND_MAX / (2 * M_PI))) - M_PI;
  Eigen::Vector3d rpy(r, p, y);

  Eigen::Matrix3d j0 = pinocchio::rpy::computeRpyJacobian(rpy);
  Eigen::Matrix3d j0inv = pinocchio::rpy::computeRpyJacobianInverse(rpy);
  BOOST_CHECK(j0inv.isApprox(j0.inverse()));

  Eigen::Matrix3d jL = pinocchio::rpy::computeRpyJacobian(rpy, pinocchio::LOCAL);
  Eigen::Matrix3d jLinv = pinocchio::rpy::computeRpyJacobianInverse(rpy, pinocchio::LOCAL);
  BOOST_CHECK(jLinv.isApprox(jL.inverse()));

  Eigen::Matrix3d jW = pinocchio::rpy::computeRpyJacobian(rpy, pinocchio::WORLD);
  Eigen::Matrix3d jWinv = pinocchio::rpy::computeRpyJacobianInverse(rpy, pinocchio::WORLD);
  BOOST_CHECK(jWinv.isApprox(jW.inverse()));

  Eigen::Matrix3d jA = pinocchio::rpy::computeRpyJacobian(rpy, pinocchio::LOCAL_WORLD_ALIGNED);
  Eigen::Matrix3d jAinv =
    pinocchio::rpy::computeRpyJacobianInverse(rpy, pinocchio::LOCAL_WORLD_ALIGNED);
  BOOST_CHECK(jAinv.isApprox(jA.inverse()));
}

BOOST_AUTO_TEST_CASE(test_computeRpyJacobianTimeDerivative)
{
  // Check zero at zero velocity
  double r = static_cast<double>(rand()) / (static_cast<double>(RAND_MAX / (2 * M_PI))) - M_PI;
  double p = static_cast<double>(rand()) / (static_cast<double>(RAND_MAX / M_PI)) - (M_PI / 2);
  double y = static_cast<double>(rand()) / (static_cast<double>(RAND_MAX / (2 * M_PI))) - M_PI;
  Eigen::Vector3d rpy(r, p, y);
  Eigen::Vector3d rpydot(Eigen::Vector3d::Zero());
  Eigen::Matrix3d dj0 = pinocchio::rpy::computeRpyJacobianTimeDerivative(rpy, rpydot);
  BOOST_CHECK(dj0.isZero());
  Eigen::Matrix3d djL =
    pinocchio::rpy::computeRpyJacobianTimeDerivative(rpy, rpydot, pinocchio::LOCAL);
  BOOST_CHECK(djL.isZero());
  Eigen::Matrix3d djW =
    pinocchio::rpy::computeRpyJacobianTimeDerivative(rpy, rpydot, pinocchio::WORLD);
  BOOST_CHECK(djW.isZero());
  Eigen::Matrix3d djA =
    pinocchio::rpy::computeRpyJacobianTimeDerivative(rpy, rpydot, pinocchio::LOCAL_WORLD_ALIGNED);
  BOOST_CHECK(djA.isZero());

  // Check correct identities between different versions
  rpydot = Eigen::Vector3d::Random();
  dj0 = pinocchio::rpy::computeRpyJacobianTimeDerivative(rpy, rpydot);
  djL = pinocchio::rpy::computeRpyJacobianTimeDerivative(rpy, rpydot, pinocchio::LOCAL);
  djW = pinocchio::rpy::computeRpyJacobianTimeDerivative(rpy, rpydot, pinocchio::WORLD);
  djA =
    pinocchio::rpy::computeRpyJacobianTimeDerivative(rpy, rpydot, pinocchio::LOCAL_WORLD_ALIGNED);
  BOOST_CHECK(dj0 == djL);
  BOOST_CHECK(djW == djA);

  Eigen::Matrix3d R = pinocchio::rpy::rpyToMatrix(rpy);
  Eigen::Matrix3d jL = pinocchio::rpy::computeRpyJacobian(rpy, pinocchio::LOCAL);
  Eigen::Matrix3d jW = pinocchio::rpy::computeRpyJacobian(rpy, pinocchio::WORLD);
  Eigen::Vector3d omegaL = jL * rpydot;
  Eigen::Vector3d omegaW = jW * rpydot;
  BOOST_CHECK(omegaW.isApprox(R * omegaL));
  BOOST_CHECK(djW.isApprox(pinocchio::skew(omegaW) * R * jL + R * djL));
  BOOST_CHECK(djW.isApprox(R * pinocchio::skew(omegaL) * jL + R * djL));

  // Check against finite differences
  double const eps = 1e-7;
  double const tol = 1e-5;
  Eigen::Vector3d rpyEps = rpy;

  rpyEps[0] += eps;
  Eigen::Matrix3d djLdr = (pinocchio::rpy::computeRpyJacobian(rpyEps, pinocchio::LOCAL) - jL) / eps;
  rpyEps[0] = rpy[0];
  rpyEps[1] += eps;
  Eigen::Matrix3d djLdp = (pinocchio::rpy::computeRpyJacobian(rpyEps, pinocchio::LOCAL) - jL) / eps;
  rpyEps[1] = rpy[1];
  rpyEps[2] += eps;
  Eigen::Matrix3d djLdy = (pinocchio::rpy::computeRpyJacobian(rpyEps, pinocchio::LOCAL) - jL) / eps;
  rpyEps[2] = rpy[2];
  Eigen::Matrix3d djLf = djLdr * rpydot[0] + djLdp * rpydot[1] + djLdy * rpydot[2];
  BOOST_CHECK(djL.isApprox(djLf, tol));

  rpyEps[0] += eps;
  Eigen::Matrix3d djWdr = (pinocchio::rpy::computeRpyJacobian(rpyEps, pinocchio::WORLD) - jW) / eps;
  rpyEps[0] = rpy[0];
  rpyEps[1] += eps;
  Eigen::Matrix3d djWdp = (pinocchio::rpy::computeRpyJacobian(rpyEps, pinocchio::WORLD) - jW) / eps;
  rpyEps[1] = rpy[1];
  rpyEps[2] += eps;
  Eigen::Matrix3d djWdy = (pinocchio::rpy::computeRpyJacobian(rpyEps, pinocchio::WORLD) - jW) / eps;
  rpyEps[2] = rpy[2];
  Eigen::Matrix3d djWf = djWdr * rpydot[0] + djWdp * rpydot[1] + djWdy * rpydot[2];
  BOOST_CHECK(djW.isApprox(djWf, tol));
}

BOOST_AUTO_TEST_SUITE_END()
