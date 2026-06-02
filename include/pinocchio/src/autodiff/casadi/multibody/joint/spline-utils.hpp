//
// Copyright (c) 2025 INRIA
//

#pragma once

// IWYU pragma: private, include "pinocchio/autodiff/casadi.hpp"

#ifdef PINOCCHIO_LSP
  #undef PINOCCHIO_LSP
  #include "pinocchio/autodiff/casadi.hpp"
#endif // PINOCCHIO_LSP

namespace pinocchio
{
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
