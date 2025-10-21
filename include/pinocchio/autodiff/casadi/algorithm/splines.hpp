//
// Copyright (c) 2025 INRIA
//

#ifndef __pinocchio_autodiff_casadi_splines_hpp__
#define __pinocchio_autodiff_casadi_splines_hpp__

#include "pinocchio/algorithm/splines.hpp"


namespace pinocchio
{
  // Fwd Declare
  struct SpanIndexes;

  template<typename Scalar, int Options>
  struct FindSpan;

  template<int Options>
  struct FindSpan<::casadi::SX, Options>
  {
    template<typename ConfigVector, typename KnotsVector>
    static SpanIndexes run(
      const Eigen::MatrixBase<ConfigVector> & /*q*/,
      const int /*degree*/,
      const int nbCtrlFrames,
      const Eigen::MatrixBase<KnotsVector> & /*knots*/)
    {
      return {0, static_cast<size_t>(nbCtrlFrames)};
    }
  };
} // namespace pinocchio

#endif // __pinocchio_autodiff_casadi_splines_hpp__