//
// Copyright (c) 2025 INRIA
//

#define BOOST_TEST_MODULE reference

#include "pinocchio/utils/reference.hpp"

#include <boost/utility/binary.hpp>

#include <boost/test/unit_test.hpp>

using namespace pinocchio;

BOOST_AUTO_TEST_CASE(test_get_ref)
{
  using namespace ::pinocchio::internal::helper;

  {
    double v = 10;
    BOOST_CHECK(&v == &get_ref(v));

    std::reference_wrapper<double> v_ref = v;
    BOOST_CHECK(&v == &get_ref(v_ref));

    std::shared_ptr<double> v_ptr = std::make_shared<double>(v);
    BOOST_CHECK(v_ptr.get() != &get_ref(v_ref));
    BOOST_CHECK(v_ptr.get() == &get_ref(v_ptr));

    const std::shared_ptr<double> v_const_ptr = std::make_shared<double>(v);
    BOOST_CHECK(v_const_ptr.get() != &get_ref(v_ref));
    BOOST_CHECK(v_const_ptr.get() == &get_ref(v_const_ptr));

    std::unique_ptr<double> v_uptr = std::make_unique<double>(v);
    BOOST_CHECK(v_uptr.get() != &get_ref(v_ref));
    BOOST_CHECK(v_uptr.get() == &get_ref(v_uptr));

    const std::unique_ptr<double> v_const_uptr = std::make_unique<double>(v);
    BOOST_CHECK(v_const_uptr.get() != &get_ref(v_ref));
    BOOST_CHECK(v_const_uptr.get() == &get_ref(v_const_uptr));
  }

  {
    const double const_v = 10;
    BOOST_CHECK(&const_v == &get_ref(const_v));
    double non_const_v = const_v;
    BOOST_CHECK(&non_const_v == &get_ref(non_const_v));

    std::reference_wrapper<const double> const_v_ref = const_v;
    BOOST_CHECK(&const_v == &get_ref(const_v_ref));

    const std::reference_wrapper<double> const_ref_wrapper_v = non_const_v;
    BOOST_CHECK(&non_const_v == &get_ref(const_ref_wrapper_v));

    std::shared_ptr<const double> const_v_ptr = std::make_shared<const double>(const_v);
    BOOST_CHECK(const_v_ptr.get() != &get_ref(const_v_ref));
    BOOST_CHECK(const_v_ptr.get() == &get_ref(const_v_ptr));

    const std::shared_ptr<const double> const_v_const_ptr = std::make_shared<const double>(const_v);
    BOOST_CHECK(const_v_const_ptr.get() != &get_ref(const_v_ref));
    BOOST_CHECK(const_v_const_ptr.get() == &get_ref(const_v_const_ptr));

    std::unique_ptr<const double> const_v_uptr = std::make_unique<const double>(const_v);
    BOOST_CHECK(const_v_uptr.get() != &get_ref(const_v_ref));
    BOOST_CHECK(const_v_uptr.get() == &get_ref(const_v_uptr));

    const std::unique_ptr<const double> const_v_const_uptr =
      std::make_unique<const double>(const_v);
    BOOST_CHECK(const_v_const_uptr.get() != &get_ref(const_v_ref));
    BOOST_CHECK(const_v_const_uptr.get() == &get_ref(const_v_const_uptr));
  }
}
