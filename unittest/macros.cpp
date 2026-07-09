//
// Copyright (c) 2021-2024 CNRS INRIA
//

#define BOOST_TEST_MODULE macros

#include "pinocchio/macros.hpp"

#include <boost/utility/binary.hpp>

#include <boost/test/unit_test.hpp>

std::string expected_msg;

bool check_exception_msg(const std::exception & exception)
{
  BOOST_CHECK_EQUAL(expected_msg, exception.what());
  return expected_msg == exception.what();
}

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
  {
    BOOST_CHECK_THROW(function_1(std::vector<int>(3), 2), std::invalid_argument);
    try
    {
      function_1(std::vector<int>(3), 2);
    }
    catch (std::invalid_argument & e)
    {
      const std::string message(e.what());

      expected_msg = "wrong argument size: expected 2, got 3\n"
                     "hint: v.size() is different from size\n";
      BOOST_CHECK(message.find(expected_msg) != std::string::npos);
    }
  }

  {
    BOOST_CHECK_THROW(function_2(std::vector<int>(3), 2), std::invalid_argument);
    try
    {
      function_2(std::vector<int>(3), 2);
    }
    catch (std::invalid_argument & e)
    {
      const std::string message(e.what());

      expected_msg = "wrong argument size: expected 2, got 3\n"
                     "hint: custom message with stream\n";
      BOOST_CHECK(message.find(expected_msg) != std::string::npos);
    }
  }
}
