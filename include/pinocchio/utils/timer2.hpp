//
// Copyright (c) 2021 LAAS-CNRS
//

#ifndef __pinocchio_utils_timer2_hpp__
#define __pinocchio_utils_timer2_hpp__

#include <ctime>

namespace pinocchio
{

  class Timer
  {
  public:
    Timer()
    {
      clock_gettime(CLOCK_MONOTONIC, &start_);
    }

    inline void reset()
    {
      clock_gettime(CLOCK_MONOTONIC, &start_);
    }

    inline double get_duration()
    {
      clock_gettime(CLOCK_MONOTONIC, &finish_);
      duration_ = static_cast<double>(finish_.tv_sec - start_.tv_sec) * 1000000;
      duration_ += static_cast<double>(finish_.tv_nsec - start_.tv_nsec) / 1000;
      return duration_ / 1000.;
    }

    inline double get_us_duration()
    {
      clock_gettime(CLOCK_MONOTONIC, &finish_);
      duration_ = static_cast<double>(finish_.tv_sec - start_.tv_sec) * 1000000;
      duration_ += static_cast<double>(finish_.tv_nsec - start_.tv_nsec) / 1000;
      return duration_;
    }

  private:
    struct timespec start_;
    struct timespec finish_;
    double duration_;
  };
} // namespace pinocchio

#endif
