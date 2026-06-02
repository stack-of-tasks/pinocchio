//
// Copyright (c) 2025 INRIA
// Copyright (c) 2026 ISIR
//

#pragma once

// IWYU pragma: private, include "pinocchio/autodiff/casadi.hpp"

#ifdef PINOCCHIO_LSP
  #undef PINOCCHIO_LSP
  #include "pinocchio/multibody/joint.hpp"
#endif // PINOCCHIO_LSP

namespace pinocchio
{
  /// @brief Helper structure defining a range of indices.
  /// @details This struct identifies the subset of control frames in a spline that are active
  /// (i.e., have non-zero basis functions) for a specific spline parameter value.
  /// Using this local support property allows for efficient computation of the joint
  /// transformation, S, and bias c.
  struct SpanIndexes
  {
    size_t start_idx;
    size_t end_idx;
  };

  /// @brief Algorithm to locate the span for a given B-spline parameter, q.
  /// @details This struct implements a binary search (FindSpan) to determine which knot span
  /// a given parameter value falls into. In B-spline curves, a parameter value $u$ implies that
  /// only $(p+1)$ control points affect the curve at that location (where $p$ is the degree).
  template<typename Scalar, int Options>
  struct FindSpan
  {
    template<typename ConfigVector, typename KnotsVector>
    static SpanIndexes run(
      const Eigen::MatrixBase<ConfigVector> & q,
      const size_t degree,
      const size_t nbCtrlFrames,
      const Eigen::MatrixBase<KnotsVector> & knots)
    {
      // Edge case: if q is at or beyond the end of the spline parameterization
      if (q[0] >= knots(knots.size() - 1))
        return {nbCtrlFrames - (degree + 1), nbCtrlFrames};

      size_t low = degree;
      size_t high = nbCtrlFrames;
      size_t mid;

      while (low < high)
      {
        mid = low + (high - low) / 2;
        if (q[0] < knots[mid])
          high = mid;
        else
          low = mid + 1;
      }

      return {low - (degree + 1), low};
    }
  };
} // namespace pinocchio
