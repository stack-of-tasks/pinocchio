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
#ifdef NDEBUG
  const int n = 1e5;
#else
  const int n = 1e2;
#endif
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
  #ifdef NDEBUG
    const int n = 1e5;
  #else
    const int n = 1e2;
  #endif
  for(int k = 0; k < n ; ++k)
  {
    Eigen::Quaterniond quat;
    pinocchio::quaternion::uniformRandom(quat);
    const Eigen::Matrix3d R = quat.toRotationMatrix();

    const Eigen::Vector3d v = pinocchio::rpy::matrixToRpy(R);
    Eigen::Matrix3d Rprime = pinocchio::rpy::rpyToMatrix(v);

    BOOST_CHECK(Rprime.isApprox(R));
    BOOST_CHECK(-M_PI <= v[0] && v[0] <= M_PI);
    BOOST_CHECK(-M_PI/2 <= v[1] && v[1] <= M_PI/2);
    BOOST_CHECK(-M_PI <= v[2] && v[2] <= M_PI);
  }

#ifdef NDEBUG
  const int n2 = 1e3;
#else
  const int n2 = 1e2;
#endif

  // Test singular case theta = pi/2
  for(int k = 0; k < n2 ; ++k)
  {
    double r = static_cast <double> (rand()) / (static_cast <double> (RAND_MAX/(2*M_PI))) - M_PI;
    double y = static_cast <double> (rand()) / (static_cast <double> (RAND_MAX/(2*M_PI))) - M_PI;
    Eigen::Matrix3d Rp;
    Rp <<  0.0, 0.0, 1.0,
           0.0, 1.0, 0.0,
          -1.0, 0.0, 0.0;
    const Eigen::Matrix3d R = Eigen::AngleAxisd(y, Eigen::Vector3d::UnitZ()).toRotationMatrix()
                            * Rp
                            * Eigen::AngleAxisd(r, Eigen::Vector3d::UnitX()).toRotationMatrix();

    const Eigen::Vector3d v = pinocchio::rpy::matrixToRpy(R);
    Eigen::Matrix3d Rprime = pinocchio::rpy::rpyToMatrix(v);

    BOOST_CHECK(Rprime.isApprox(R));
    BOOST_CHECK(-M_PI <= v[0] && v[0] <= M_PI);
    BOOST_CHECK(-M_PI/2 <= v[1] && v[1] <= M_PI/2);
    BOOST_CHECK(-M_PI <= v[2] && v[2] <= M_PI);
  }

  // Test singular case theta = -pi/2
  for(int k = 0; k < n2 ; ++k)
  {
    double r = static_cast <double> (rand()) / (static_cast <double> (RAND_MAX/(2*M_PI))) - M_PI;
    double y = static_cast <double> (rand()) / (static_cast <double> (RAND_MAX/(2*M_PI))) - M_PI;
    Eigen::Matrix3d Rp;
    Rp << 0.0, 0.0, -1.0,
          0.0, 1.0,  0.0,
          1.0, 0.0,  0.0;
    const Eigen::Matrix3d R = Eigen::AngleAxisd(y, Eigen::Vector3d::UnitZ()).toRotationMatrix()
                            * Rp
                            * Eigen::AngleAxisd(r, Eigen::Vector3d::UnitX()).toRotationMatrix();

    const Eigen::Vector3d v = pinocchio::rpy::matrixToRpy(R);
    Eigen::Matrix3d Rprime = pinocchio::rpy::rpyToMatrix(v);

    BOOST_CHECK(Rprime.isApprox(R));
    BOOST_CHECK(-M_PI <= v[0] && v[0] <= M_PI);
    BOOST_CHECK(-M_PI/2 <= v[1] && v[1] <= M_PI/2);
    BOOST_CHECK(-M_PI <= v[2] && v[2] <= M_PI);
  }
}

BOOST_AUTO_TEST_SUITE_END()
