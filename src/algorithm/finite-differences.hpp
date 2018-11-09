//
// Copyright (c) 2016,2018 CNRS
//

#ifndef __pinocchio_finite_differences_hpp__
#define __pinocchio_finite_differences_hpp__

#include "pinocchio/multibody/model.hpp"

namespace pinocchio {
  
  ///
  /// \brief Computes the finite difference increments for each degree of freedom according to the current joint configuration.
  ///
  /// \tparam JointCollection Collection of Joint types.
  ///
  /// \input[in] model The model of the kinematic tree.
  ///
  /// \returns The finite difference increments for each degree of freedom.
  ///
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  inline typename ModelTpl<Scalar,Options,JointCollectionTpl>::TangentVectorType
  finiteDifferenceIncrement(const ModelTpl<Scalar,Options,JointCollectionTpl> & model);
}

/* --- Details -------------------------------------------------------------------- */
#include "pinocchio/algorithm/finite-differences.hxx"

#endif // ifndef __pinocchio_finite_differences_hpp__
