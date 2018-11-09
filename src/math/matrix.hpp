//
// Copyright (c) 2016 CNRS
//

#ifndef __math_matrix_hpp__
#define __math_matrix_hpp__

#include <Eigen/Dense>

namespace pinocchio
{

  template<typename Derived>
  inline bool hasNaN(const Eigen::DenseBase<Derived> & m) 
  {
    return !((m.derived().array()==m.derived().array()).all());
  }


}
#endif //#ifndef __math_matrix_hpp__
