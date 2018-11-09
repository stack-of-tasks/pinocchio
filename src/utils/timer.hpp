//
// Copyright (c) 2015,2018 CNRS
//

#include <sys/time.h>
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

