//
// Copyright (c) 2019-2020 INRIA
//

#include <iostream>

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

BOOST_AUTO_TEST_CASE(test_orthogonal_projection)
{
  using namespace pinocchio;
  const int max_tests = 1e5;
  
  typedef Eigen::Vector4d Vector4;
  typedef Eigen::Quaterniond Quaternion;
  typedef Eigen::Matrix3d Matrix3;
  
  for(int k = 0; k < max_tests; ++k)
  {
    const Vector4 vec4 = Vector4::Random();
    const Quaternion quat(vec4.normalized());
    
    const Matrix3 rot = quat.toRotationMatrix();
    
    Matrix3 rot_proj = orthogonalProjection(rot);
    BOOST_CHECK(rot.isApprox(rot_proj));
  }
  
  for(int k = 0; k < max_tests; ++k)
  {
    const Matrix3 mat = Matrix3::Random();
    
    Matrix3 rot_proj = orthogonalProjection(mat);
    BOOST_CHECK((rot_proj.transpose()*rot_proj).isIdentity());
    BOOST_CHECK(fabs(rot_proj.determinant() - 1.) <= Eigen::NumTraits<double>::dummy_precision());
  }
}

BOOST_AUTO_TEST_SUITE_END()


