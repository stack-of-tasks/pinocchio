//
// Copyright (c) 2025 INRIA
// Copyright (c) 2026 ISIR
//

#pragma once

// IWYU pragma: private, include "pinocchio/multibody/joint.hpp

#ifdef PINOCCHIO_LSP
  #undef PINOCCHIO_LSP
  #include "pinocchio/multibody/joint.hpp"
#endif // PINOCCHIO_LSP

namespace pinocchio
{
  namespace internal
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

        if(q[0] <= knots[0])
          return {0, degree + 1};

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

      template<typename Scalar>
      Scalar bsplineBasis(
        size_t i, size_t k, const Scalar x, const Eigen::Matrix<Scalar, Eigen::Dynamic, 1> & knots)
      {
        if (k == 0)
        {
          // clang-format off
        // if(knots[i] <= x && x <= knots[i + 1])
        //  return 1;
        // else
        //  return 0;
          // clang-format on
          return if_then_else(
            LE, knots[i], x, if_then_else(LE, x, knots[i + 1], Scalar(1), Scalar(0)),
            Scalar(0));
        }

        // Calculate the left term
        // clang-format off
      // if(den1 > dummy_precision)
      //  left = (x - knots[i]) / den1 * bsplineBasis(i, k - 1, x)
      // else
      //  left = 0
        // clang-format on
        const Scalar den1 = knots[i + k] - knots[i];
        const Scalar left = if_then_else(
          GT, den1, Eigen::NumTraits<Scalar>::dummy_precision(),
          (x - knots[i]) / den1 * bsplineBasis(i, k - 1, x, knots), Scalar(0));

        // Calculate the right term
        // clang-format off
      // if(den2 > dummy_precision)
      //  right = (knots[i + k + 1] - x) / den2 * bsplineBasis(i + 1, k - 1, x)
      // else
      //  right = 0
        // clang-format on
        const Scalar den2 = knots[i + k + 1] - knots[i + 1];
        const Scalar right = if_then_else(
          GT, den2, Eigen::NumTraits<Scalar>::dummy_precision(),
          (knots[i + k + 1] - x) / den2 * bsplineBasis(i + 1, k - 1, x, knots), Scalar(0));

        return left + right;
      }

      template<typename Scalar>
      Scalar bsplineBasisDerivative(
        size_t i, size_t k, const Scalar x, const Eigen::Matrix<Scalar, Eigen::Dynamic, 1> & knots)
      {
        if (k == 0)
        {
          return Scalar(0);
        }
        const Scalar k_scalar = static_cast<Scalar>(k);

        // Calculate the first term of the derivative
        // clang-format off
      // if(den1 > dummy_precision)
      //  term1 = (k_scalar / den1) * bsplineBasis(i, k - 1, x)
      // else
      //  term1 = 0
        // clang-format on
        const Scalar den1 = knots[i + k] - knots[i];
        const Scalar term1 = if_then_else(
          GT, den1, Eigen::NumTraits<Scalar>::dummy_precision(),
          (k_scalar / den1) * bsplineBasis(i, k - 1, x, knots), Scalar(0));

        // Calculate the second term of the derivative
        // clang-format off
      // if(den2 > dummy_precision)
      //  term2 = (k_scalar / den2) * bsplineBasis(i + 1, k - 1, x)
      // else
      //  term2 = 0
        // clang-format on
        const Scalar den2 = knots[i + k + 1] - knots[i + 1];
        const Scalar term2 = if_then_else(
          GT, den2, Eigen::NumTraits<Scalar>::dummy_precision(),
          (k_scalar / den2) * bsplineBasis(i + 1, k - 1, x, knots), Scalar(0));

        return term1 - term2;
      }

      template<typename Scalar>
      Scalar bsplineBasisDerivative2(
        size_t i, size_t k, const Scalar x, const Eigen::Matrix<Scalar, Eigen::Dynamic, 1> & knots)
      {
        if (k < 2)
        {
          return Scalar(0);
        }

        const Scalar k_scalar = static_cast<Scalar>(k);

        // Calculate the first term
        // clang-format off
      // if(den1 > dummy_precision)
      //  term1 = (k_scalar / den1) * bsplineBasisDerivative(i, k - 1, x)
      // else
      //  term1 = 0
        // clang-format on
        const Scalar den1 = knots[i + k] - knots[i];
        const Scalar term1 = if_then_else(
          GT, den1, Eigen::NumTraits<Scalar>::dummy_precision(),
          (k_scalar / den1) * bsplineBasisDerivative(i, k - 1, x, knots), Scalar(0));

        // Calculate the second term
        // clang-format off
      // if(den2 > dummy_precision)
      //  term2 = (k_scalar / den2) * bsplineBasisDerivative(i + 1, k - 1, x)
      // else
      //  term2 = 0
        // clang-format on
        const Scalar den2 = knots[i + k + 1] - knots[i + 1];
        const Scalar term2 = if_then_else(
          GT, den2, Eigen::NumTraits<Scalar>::dummy_precision(),
          (k_scalar / den2) * bsplineBasisDerivative(i + 1, k - 1, x, knots), Scalar(0));

        return term1 - term2;
      }

      template<typename Scalar>
      Eigen::Matrix<Scalar, Eigen::Dynamic, 1>
      generateOpenUniformKnots(const Scalar min_q, const Scalar max_q, size_t nCtrl, size_t degree)
      {
        using Vector = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;

        const size_t n_knots = nCtrl + degree + 1;

        Vector knots;
        knots.resize(n_knots);

        const Scalar range = max_q - min_q;

        knots.head(degree + 1).setConstant(min_q);
        const Scalar nInner = static_cast<Scalar>(nCtrl - degree - 1);
        const Scalar denominator = static_cast<Scalar>(nInner + 1);

        for (size_t i = degree + 1; i < nCtrl; i++)
          knots[i] = min_q + range * static_cast<Scalar>(i - degree) / denominator;

        knots.tail(degree + 1).setConstant(max_q);
        return knots;
      }

      template<typename Scalar>
      Eigen::Matrix<Scalar, Eigen::Dynamic, 1>
      generateUniformKnots(const Scalar min_q, const Scalar max_q, size_t nCtrl, size_t degree)
      {
        using Vector = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;

        const size_t n_knots = nCtrl + degree + 1;
        Vector knots;
        knots.resize(n_knots);

        const Scalar step = (max_q - min_q) / static_cast<Scalar>(n_knots - 1);

        for (size_t i = 0; i < n_knots; ++i)
          knots[i] = min_q + step * static_cast<Scalar>(i);

        return knots;
      }
    } // namespace internal

  } // namespace pinocchio
