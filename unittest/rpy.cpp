//
// Copyright (c) 2019 INRIA
//

#include <pinocchio/math/rpy.hpp>

#include <boost/variant.hpp> // to avoid C99 warnings

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

BOOST_AUTO_TEST_CASE(test_rpyToMatrix)
{
  double r = static_cast <double> (rand()) / (static_cast <double> (RAND_MAX/(2*M_PI))) - M_PI;
  double p = static_cast <double> (rand()) / (static_cast <double> (RAND_MAX/M_PI)) - (M_PI/2);
  double y = static_cast <double> (rand()) / (static_cast <double> (RAND_MAX/(2*M_PI))) - M_PI;

  Eigen::Matrix3d R = pinocchio::rpy::rpyToMatrix(r, p, y);

  Eigen::Matrix3d R_Eig = (Eigen::AngleAxisd(y, Eigen::Vector3d::UnitZ())
                           * Eigen::AngleAxisd(p, Eigen::Vector3d::UnitY())
                           * Eigen::AngleAxisd(r, Eigen::Vector3d::UnitX())
                          ).toRotationMatrix();

  BOOST_CHECK(R.isApprox(R_Eig));

  Eigen::Vector3d v;
  v << r, p, y;

  Eigen::Matrix3d Rv = pinocchio::rpy::rpyToMatrix(v);

  BOOST_CHECK(Rv.isApprox(R_Eig));
}

BOOST_AUTO_TEST_CASE(test_matrixToRpy)
{
  double r = static_cast <double> (rand()) / (static_cast <double> (RAND_MAX/(2*M_PI))) - M_PI;
  double p = static_cast <double> (rand()) / (static_cast <double> (RAND_MAX/M_PI)) - (M_PI/2);
  double y = static_cast <double> (rand()) / (static_cast <double> (RAND_MAX/(2*M_PI))) - M_PI;

  Eigen::Matrix3d R = pinocchio::rpy::rpyToMatrix(r, p, y);

  Eigen::Vector3d v = pinocchio::rpy::matrixToRpy(R);

  BOOST_CHECK_CLOSE(r,v[0],1e-12);
  BOOST_CHECK_CLOSE(p,v[1],1e-12);
  BOOST_CHECK_CLOSE(y,v[2],1e-12);
}

BOOST_AUTO_TEST_SUITE_END()

