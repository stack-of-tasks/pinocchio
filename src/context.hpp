//
// Copyright (c) 2021 CNRS INRIA
//

#ifndef __pinocchio_context_hpp__
#define __pinocchio_context_hpp__

#include <Eigen/Dense>

// no pinocchio structure / classes imported here

namespace pinocchio {
  namespace context {
    typedef PINOCCHIO_SCALAR_TYPE Scalar;
    enum { Options = 0 };
    typedef Eigen::Matrix<Scalar,Eigen::Dymamic,1,Options> VectorXs;
    
  } //namespace context
} //namespace pinocchio

#endif // #ifndef __pinocchio_context_hpp__
