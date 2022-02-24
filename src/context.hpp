//
// Copyright (c) 2021 CNRS INRIA
//

#ifndef __pinocchio_context_hpp__
#define __pinocchio_context_hpp__

#include <Eigen/Core>

namespace pinocchio {
  namespace context {
    typedef PINOCCHIO_SCALAR_TYPE Scalar;
    enum { Options = 0 };
    typedef Eigen::Matrix<Scalar,Eigen::Dynamic,1,Options> VectorXs;
    typedef Eigen::Matrix<Scalar, 6, Eigen::Dynamic, Options> Matrix6xs;
    
  } //namespace context
} //namespace pinocchio

#endif // #ifndef __pinocchio_context_hpp__
