//
// Copyright (c) 2021 INRIA
//

#ifndef __pinocchio_utils_openmp_hpp__
#define __pinocchio_utils_openmp_hpp__

#include <cstdlib>

namespace pinocchio
{

  /// \brief Returns the number of thread defined by the environment variable OMP_NUM_THREADS.
  ///        If this variable is not defined, this simply returns the default value 1.
  ///
  inline int getOpenMPNumThreadsEnv()
  {
    int num_threads = 1;
    
    if(const char* env_p = std::getenv("OMP_NUM_THREADS"))
      num_threads = atoi(env_p);

    return num_threads;
  }
}

#endif // ifndef __pinocchio_utils_openmp_hpp__
