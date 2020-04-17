//
// Copyright (c) 2019-2020 INRIA
//

#include <pinocchio/math/rpy.hpp>
#include <pinocchio/math/quaternion.hpp>

#include <boost/variant.hpp> // to avoid C99 warnings

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

BOOST_AUTO_TEST_CASE(test_rpyToMatrix)
{
  const int n = 1e5;
  for(int k = 0; k < n ; ++k)
  {
    double r = static_cast <double> (rand()) / (static_cast <double> (RAND_MAX/(2*M_PI))) - M_PI;
    double p = static_cast <double> (rand()) / (static_cast <double> (RAND_MAX/M_PI)) - (M_PI/2);
    double y = static_cast <double> (rand()) / (static_cast <double> (RAND_MAX/(2*M_PI))) - M_PI;
    
    Eigen::Matrix3d R = pinocchio::rpy::rpyToMatrix(r, p, y);
    
    Eigen::Matrix3d Raa = (Eigen::AngleAxisd(y, Eigen::Vector3d::UnitZ())
                             * Eigen::AngleAxisd(p, Eigen::Vector3d::UnitY())
                             * Eigen::AngleAxisd(r, Eigen::Vector3d::UnitX())
                             ).toRotationMatrix();
    
    BOOST_CHECK(R.isApprox(Raa));
    
    Eigen::Vector3d v;
    v << r, p, y;
    
    Eigen::Matrix3d Rv = pinocchio::rpy::rpyToMatrix(v);
    
    BOOST_CHECK(Rv.isApprox(Raa));
    BOOST_CHECK(Rv.isApprox(R));
  }
}

BOOST_AUTO_TEST_CASE(test_matrixToRpy)
{
  const int n = 1e6;
  for(int k = 0; k < n ; ++k)
  {
    Eigen::Quaterniond quat;
    pinocchio::quaternion::uniformRandom(quat);
    const Eigen::Matrix3d R = quat.toRotationMatrix();
    
    const Eigen::Vector3d v = pinocchio::rpy::matrixToRpy(R);
    Eigen::Matrix3d Rprime = pinocchio::rpy::rpyToMatrix(v);
    
    BOOST_CHECK(Rprime.isApprox(R));
  }

  for(int k = 0; k < n ; ++k)
  {
    double r = static_cast <double> (rand()) / (static_cast <double> (RAND_MAX/(2*M_PI))) - M_PI;
    double p = static_cast <double> (rand()) / (static_cast <double> (RAND_MAX/M_PI)) - (M_PI/2);
    double y = static_cast <double> (rand()) / (static_cast <double> (RAND_MAX/(2*M_PI))) - M_PI;
    Eigen::Vector3d rpy;
    rpy << r, p, y;

    const Eigen::Matrix3d R = pinocchio::rpy::rpyToMatrix(rpy);
    const Eigen::Vector3d rpy2 = pinocchio::rpy::matrixToRpy(R);

    BOOST_CHECK(rpy.isApprox(rpy2, 1e-6));
  }
}

BOOST_AUTO_TEST_SUITE_END()
