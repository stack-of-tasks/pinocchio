//
// Copyright (c) 2019-2020 INRIA CNRS
//

#include <pinocchio/math/quaternion.hpp>
#include <pinocchio/spatial/se3.hpp>

#include <boost/variant.hpp> // to avoid C99 warnings

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

BOOST_AUTO_TEST_CASE(test_assignQuaternion)
{
  using namespace pinocchio;
  const int max_tests = 1e5;
  for(int k = 0; k < max_tests; ++k)
  {
    const SE3 M(SE3::Random());
    SE3::Quaternion quat_ref(M.rotation());
    
    SE3::Quaternion quat;
    quaternion::assignQuaternion(quat,M.rotation());
    
    BOOST_CHECK(quat.coeffs().isApprox(quat_ref.coeffs()));
  }
}

BOOST_AUTO_TEST_CASE(test_uniformRandom)
{
  srand(0);

  using namespace pinocchio;
  Eigen::Quaternion<double> q;

  for (int i = 0; i < (1 << 10); ++i) {
    quaternion::uniformRandom(q);
    BOOST_CHECK_MESSAGE((q.coeffs().array().abs() <= 1).all(),
        "Quaternion coeffs out of bounds: " << i << ' ' << q.coeffs().transpose());
  }
}

BOOST_AUTO_TEST_CASE(test_isNormalized)
{
  srand(0);

  using namespace pinocchio;
  typedef Eigen::Quaternion<double> Quaternion;
  typedef Quaternion::Coefficients Vector4;
  
#ifdef NDEBUG
  const int max_test = 1e6;
#else
  const int max_test = 1e2;
#endif
  for(int i = 0; i < max_test; ++i)
  {
    Quaternion q;
    q.coeffs() = Vector4::Random() + Vector4::Constant(2);
    BOOST_CHECK(!quaternion::isNormalized(q));
    
    q.normalize();
    BOOST_CHECK(quaternion::isNormalized(q));
  }
  
  // Specific check for the Zero vector
  BOOST_CHECK(!quaternion::isNormalized(Quaternion(Vector4::Zero())));
}

BOOST_AUTO_TEST_SUITE_END()

