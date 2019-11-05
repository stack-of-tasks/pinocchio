//
// Copyright (c) 2015-2019 CNRS INRIA
//

#ifndef __pinocchio_utils_timer_hpp__
#define __pinocchio_utils_timer_hpp__

#ifdef WIN32
#include <Windows.h>
#include <stdint.h> // portable: uint64_t   MSVC: __int64

int gettimeofday(struct timeval* tp, struct timezone* tzp)
{
  // Note: some broken versions only have 8 trailing zero's, the correct epoch has 9 trailing zero's
  // This magic number is the number of 100 nanosecond intervals since January 1, 1601 (UTC)
  // until 00:00:00 January 1, 1970
  static const uint64_t EPOCH = ((uint64_t)116444736000000000ULL);

  SYSTEMTIME  system_time;
  FILETIME    file_time;
  uint64_t    time;

  GetSystemTime(&system_time);
  SystemTimeToFileTime(&system_time, &file_time);
  time = ((uint64_t)file_time.dwLowDateTime);
  time += ((uint64_t)file_time.dwHighDateTime) << 32;

  tp->tv_sec = (long)((time - EPOCH) / 10000000L);
  tp->tv_usec = (long)(system_time.wMilliseconds * 1000);
  return 0;
}
#else
#include <sys/time.h>
#endif
#include <iostream>
#include <stack>

#define SMOOTH(s) for(size_t _smooth=0;_smooth<s;++_smooth) 

/* Return the time spent in secs. */
inline double operator-(const struct timeval & t1,const struct timeval & t0)
{
  /* TODO: double check the double conversion from long (on 64x). */
  return double(t1.tv_sec - t0.tv_sec)+1e-6*double(t1.tv_usec - t0.tv_usec);
}

struct PinocchioTicToc
{
  enum Unit { S = 1, MS = 1000, US = 1000000, NS = 1000000000 };
  Unit DEFAULT_UNIT;
  
  static std::string unitName(Unit u)
  {
    switch(u) { case S: return "s"; case MS: return "ms"; case US: return "us"; case NS: return "ns"; }
    return "";
  }
  
  std::stack<struct timeval> stack;
  mutable struct timeval t0;
  
  PinocchioTicToc( Unit def = MS ) : DEFAULT_UNIT(def) {}
  
  inline void tic() {
    stack.push(t0);
    gettimeofday(&(stack.top()),NULL);
  }
  
  inline double toc() { return toc(DEFAULT_UNIT); };
  
  inline double toc(const Unit factor)
  {
    gettimeofday(&t0,NULL);
    double dt = (t0-stack.top())*factor;
    stack.pop();
    return dt;
  }
  
  inline void toc(std::ostream & os, double SMOOTH=1)
  {
    os << toc(DEFAULT_UNIT)/SMOOTH << " " << unitName(DEFAULT_UNIT) << std::endl;
  }
};

#endif // ifndef __pinocchio_utils_timer_hpp__
