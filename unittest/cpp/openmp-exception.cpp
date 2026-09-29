//
// Copyright (c) 2024 INRIA
//

#define BOOST_TEST_MODULE openmp_exception

#include <sstream>

#include "pinocchio/utils/openmp.hpp"

#include <boost/utility/binary.hpp>

#include <boost/test/unit_test.hpp>

using namespace pinocchio;

template<typename T>
void throw_if_equal_values(const T value, const T ref_value)
{
  if (value == ref_value)
  {
    std::stringstream message;
    message << value << " is equal to " << ref_value;
    throw std::logic_error(message.str());
  }
}

template<typename... Parameters>
void run_parallel_loop(const int n, OpenMPException & openmp_exception, Parameters... params)
{
#pragma omp parallel for
  for (int i = 0; i < n; i++)
  {
    if (openmp_exception.hasThrown())
      continue;
    openmp_exception.run(&throw_if_equal_values<int>, i, params...);
  }

  openmp_exception.rethrowException();
}

BOOST_AUTO_TEST_CASE(test_openmp_exception_catch)
{

  const int num_threads = omp_get_num_threads();
  omp_set_num_threads(num_threads);
  {
    OpenMPException openmp_exception;
    try
    {
      run_parallel_loop(10000, openmp_exception, 20);
    }
    catch (...)
    {
    }
  }

  {
    OpenMPException openmp_exception;
    BOOST_CHECK_THROW(run_parallel_loop(10000, openmp_exception, 20), std::logic_error);
  }
  {
    OpenMPException openmp_exception;
    BOOST_CHECK_NO_THROW(run_parallel_loop(10000, openmp_exception, 10001));
  }
}
