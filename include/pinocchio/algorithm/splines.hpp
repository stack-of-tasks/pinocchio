//
// Copyright (c) 2025 INRIA
//

#ifndef __pinocchio_algorithm_splines_hpp__
#define __pinocchio_algorithm_splines_hpp__

namespace pinocchio
{
    struct SpanIndexes
  {
    size_t start_idx;
    size_t end_idx;
  };

  // template<typename Scalar, int Options>
  // struct FindSpan
  // {
  //   template<typename ConfigVector, typename KnotsVector>
  //   static SpanIndexes run(
  //     const Eigen::MatrixBase<ConfigVector> & /*q*/,
  //     const int /*degree*/,
  //     const int nbCtrlFrames,
  //     const Eigen::MatrixBase<KnotsVector> & /*knots*/)
  //   {
  //     return {0, static_cast<size_t>(nbCtrlFrames)};
  //   }
  // };

  template<typename Scalar, int Options>
  struct FindSpan
  {
    template<typename ConfigVector, typename KnotsVector>
    static SpanIndexes run(
      const Eigen::MatrixBase<ConfigVector> & q,
      const int degree,
      const int nbCtrlFrames,
      const Eigen::MatrixBase<KnotsVector> & knots)
    {
      // Edge case: if q is at or beyond the end of the spline parameterization
      if (q[0] >= 1.0)
        return {static_cast<size_t>(nbCtrlFrames - (degree + 1)), static_cast<size_t>(nbCtrlFrames)};

      int low = degree;
      int high = nbCtrlFrames;
      int mid;

      while (low < high)
      {
        mid = low + (high - low) / 2;
        if (q[0] < knots[mid])
          high = mid;
        else
          low = mid + 1;
      }

      return {static_cast<size_t>(low - (degree + 1)), static_cast<size_t>(low)};
    }
  };
} // namespace pinocchio

#endif // __pinocchio_algorithm_splines_hpp__