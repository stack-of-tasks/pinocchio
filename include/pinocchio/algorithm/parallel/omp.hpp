//
// Copyright (c) 2022 INRIA
//

#ifndef __pinocchio_algorithm_parallel_omp_hpp__
#define __pinocchio_algorithm_parallel_omp_hpp__

#include <omp.h>

namespace pinocchio
{
  inline void set_default_omp_options(const size_t num_threads = (size_t)omp_get_max_threads())
  {
    omp_set_num_threads((int)num_threads);
    omp_set_dynamic(0);
  }
} // namespace pinocchio

#endif // ifndef __pinocchio_algorithm_parallel_omp_hpp__
