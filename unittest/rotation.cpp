//
// Copyright (c) 2019 INRIA
//

#include <pinocchio/math/rotation.hpp>
#include <pinocchio/math/sincos.hpp>
#include <Eigen/Geometry>

#include <boost/variant.hpp> // to avoid C99 warnings

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

BOOST_AUTO_TEST_CASE(test_toRotationMatrix)
{
  using namespace pinocchio;
  const int max_tests = 1e5;
  for(int k = 0; k < max_tests; ++k)
  {
    const double theta = std::rand();
    double cos_value, sin_value;
    
    pinocchio::SINCOS(theta,&sin_value,&cos_value);
    Eigen::Vector3d axis(Eigen::Vector3d::Random().normalized());

    Eigen::Matrix3d rot; toRotationMatrix(axis,cos_value,sin_value,rot);
    Eigen::Matrix3d rot_ref = Eigen::AngleAxisd(theta,axis).toRotationMatrix();
    
    BOOST_CHECK(rot.isApprox(rot_ref));
  }
}

BOOST_AUTO_TEST_SUITE_END()


