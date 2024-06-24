//
// Copyright (c) 2021 CNRS INRIA
//

#include "pinocchio/macros.hpp"

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

using namespace pinocchio;

std::string expected_msg;

bool check_exception_msg(const std::exception & exception)
{
  BOOST_CHECK_EQUAL(expected_msg, exception.what());
  return expected_msg == exception.what();
}

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

void function_1(std::vector<int> v, size_t size)
{
  PINOCCHIO_CHECK_ARGUMENT_SIZE(v.size(), size);
  //"size of input vector should be " << size)
}
void function_2(std::vector<int> v, size_t size)
{
  PINOCCHIO_CHECK_ARGUMENT_SIZE(v.size(), size, "custom message " << "with stream");
}

BOOST_AUTO_TEST_CASE(test_check_arguments)
{
  expected_msg = "wrong argument size: expected 2, got 3\n"
                 "hint: v.size() is different from size\n";
  BOOST_CHECK_EXCEPTION(
    function_1(std::vector<int>(3), 2), std::invalid_argument, check_exception_msg);
  expected_msg = "wrong argument size: expected 2, got 3\n"
                 "hint: custom message with stream\n";
  BOOST_CHECK_EXCEPTION(
    function_2(std::vector<int>(3), 2), std::invalid_argument, check_exception_msg);
}

BOOST_AUTO_TEST_SUITE_END()
