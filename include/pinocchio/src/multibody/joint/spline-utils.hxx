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
        // TODO is it useful ?
        if (q[0] >= knots(knots.size() - 1))
        {
          return {nbCtrlFrames - 1, nbCtrlFrames};
        }

        if (q[0] <= knots[0])
        {
          return {0, 1};
        }

        // TODO we can probably do better with std::lower
        // and std::upper bounds.
        size_t order = degree + 1;
        // Search first control point knot range containing q.
        size_t low = 0;
        for (std::size_t i = 0; i < nbCtrlFrames; ++i)
        {
          if (
            knots[static_cast<Eigen::Index>(i)] <= q[0]
            && q[0] < knots[static_cast<Eigen::Index>(i + order)])
          {
            low = i;
            break;
          }
        }

        size_t high = low;
        // Search last control point knot range containing q.
        // If we are at the end of the range high == low.
        for (std::size_t i = low + 1; i < nbCtrlFrames; ++i)
        {
          if (!(knots[static_cast<Eigen::Index>(i)] <= q[0]
                && q[0] < knots[static_cast<Eigen::Index>(i + order)]))
          {
            break;
          }
          high = i;
        }
        return {low, high + 1};
      }
    };

    template<typename Scalar>
    Scalar bsplineBasis(
      size_t i, size_t k, const Scalar x, const Eigen::Matrix<Scalar, Eigen::Dynamic, 1> & knots)
    {
      if (k == 0)
      {
        // clang-format off
        // if(knots[i] <= x && x < knots[i + 1])
        //  return 1;
        // else
        //  return 0;
        // clang-format on
        Scalar is_in_standard_range = if_then_else(
          LE, knots[static_cast<Eigen::Index>(i)], x,
          if_then_else(LT, x, knots[static_cast<Eigen::Index>(i + 1)], Scalar(1), Scalar(0)),
          Scalar(0));

        // clang-format off
        // if(x == knots.back() && x == knots[i + 1])
        //  return 1;
        // else
        //  return 0;
        // clang-format on
        Scalar is_at_final_range = if_then_else(
          EQ, x, knots[static_cast<Eigen::Index>(knots.size() - 1)],
          if_then_else(EQ, x, knots[static_cast<Eigen::Index>(i + 1)], Scalar(1), Scalar(0)),
          Scalar(0));

        return is_in_standard_range + is_at_final_range;
      }

      // Calculate the left term
      // clang-format off
      // if(den1 > dummy_precision)
      //  left = (x - knots[i]) / den1 * bsplineBasis(i, k - 1, x)
      // else
      //  left = 0
      // clang-format on
      const Scalar den1(
        knots[static_cast<Eigen::Index>(i + k)] - knots[static_cast<Eigen::Index>(i)]);
      const Scalar left = if_then_else(
        GT, den1, Eigen::NumTraits<Scalar>::dummy_precision(),
        (x - knots[static_cast<Eigen::Index>(i)]) / den1 * bsplineBasis(i, k - 1, x, knots),
        Scalar(0));

      // Calculate the right term
      // clang-format off
      // if(den2 > dummy_precision)
      //  right = (knots[i + k + 1] - x) / den2 * bsplineBasis(i + 1, k - 1, x)
      // else
      //  right = 0
      // clang-format on
      const Scalar den2(
        knots[static_cast<Eigen::Index>(i + k + 1)] - knots[static_cast<Eigen::Index>(i + 1)]);
      const Scalar right = if_then_else(
        GT, den2, Eigen::NumTraits<Scalar>::dummy_precision(),
        (knots[static_cast<Eigen::Index>(i + k + 1)] - x) / den2
          * bsplineBasis(i + 1, k - 1, x, knots),
        Scalar(0));

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
      const Scalar k_scalar(static_cast<int>(k));

      // Calculate the first term of the derivative
      // clang-format off
      // if(den1 > dummy_precision)
      //  term1 = (k_scalar / den1) * bsplineBasis(i, k - 1, x)
      // else
      //  term1 = 0
      // clang-format on
      const Scalar den1(
        knots[static_cast<Eigen::Index>(i + k)] - knots[static_cast<Eigen::Index>(i)]);
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
      const Scalar den2(
        knots[static_cast<Eigen::Index>(i + k + 1)] - knots[static_cast<Eigen::Index>(i + 1)]);
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

      const Scalar k_scalar(static_cast<int>(k));

      // Calculate the first term
      // clang-format off
      // if(den1 > dummy_precision)
      //  term1 = (k_scalar / den1) * bsplineBasisDerivative(i, k - 1, x)
      // else
      //  term1 = 0
      // clang-format on
      const Scalar den1(
        knots[static_cast<Eigen::Index>(i + k)] - knots[static_cast<Eigen::Index>(i)]);
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
      const Scalar den2(
        knots[static_cast<Eigen::Index>(i + k + 1)] - knots[static_cast<Eigen::Index>(i + 1)]);
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
      knots.resize(static_cast<Eigen::Index>(n_knots));

      const Scalar range = max_q - min_q;

      knots.head(degree + 1).setConstant(min_q);
      const Scalar nInner = static_cast<Scalar>(nCtrl - degree - 1);
      const Scalar denominator = static_cast<Scalar>(nInner + 1);

      for (size_t i = degree + 1; i < nCtrl; i++)
        knots[static_cast<Eigen::Index>(i)] =
          min_q + range * static_cast<Scalar>(i - degree) / denominator;

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
      knots.resize(static_cast<Eigen::Index>(n_knots));

      const Scalar step = (max_q - min_q) / static_cast<Scalar>(n_knots - 1);

      for (size_t i = 0; i < n_knots; ++i)
        knots[static_cast<Eigen::Index>(i)] = min_q + step * static_cast<Scalar>(i);

      return knots;
    }

    /** De Boor algorithm implementation.
     * \p start_i Knot vector index that contains x
     * \p degree Curve degree
     * \p x Value to evaluate
     * \p knots Knot vector of size m
     * \p control_points control points of size m - degree - 1
     * \p workspace of size degree + 1
     * \return BSpline value at x
     */
    template<typename Scalar>
    Scalar deBoor(
      size_t start_i,
      size_t degree,
      Scalar x,
      const Eigen::Matrix<Scalar, Eigen::Dynamic, 1> & knots,
      const Eigen::Matrix<Scalar, Eigen::Dynamic, 1> & control_points,
      Eigen::Matrix<Scalar, Eigen::Dynamic, 1> & workspace)
    {
      assert(workspace.size() == static_cast<int>(degree) + 1);
      assert(knots.size() == control_points.size() + static_cast<int>(degree) + 1);
      assert(degree <= start_i);
      assert(start_i < knots.size() - 1 - degree);
      assert(knots[degree] <= x);
      assert(x <= knots[knots.size() - 1 - degree]);

      for (int i = 0; i < static_cast<int>(degree) + 1; ++i)
      {
        workspace[i] = control_points[i + start_i - degree];
      }
      for (int r = 1; r < static_cast<int>(degree) + 1; ++r)
      {
        const int current_degree = static_cast<int>(degree) - r;
        for (int i = static_cast<int>(degree); i > r - 1; --i)
        {
          const int current_knot = i + static_cast<int>(start_i) - static_cast<int>(degree);
          const int last_knot = current_knot + current_degree + 1;
          Scalar alpha = (x - knots[current_knot]) / (knots[last_knot] - knots[current_knot]);
          workspace[i] = (1. - alpha) * workspace[i - 1] + alpha * workspace[i];
        }
      }
      return workspace[degree];
    }

    /** De Boor algorithm modification to compute cumulative basis.
     * \p start_i Knot vector index that contains x
     * \p degree Curve degree
     * \p x Value to evaluate
     * \p ignore Ignore first ignore basis when computing the sum
     * \p knots Knot vector of size m
     * \p workspace of size (degree + 1, degree + 1)
     * \return BSpline value at x
     */
    template<typename Scalar>
    Scalar deBoorCumBasisSparse(
      size_t start_i,
      size_t degree,
      Scalar x,
      size_t ignore,
      const Eigen::Matrix<Scalar, Eigen::Dynamic, 1> & knots,
      Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> & workspace)
    {
      // TODO linearize
      assert(workspace.rows() == static_cast<int>(degree) + 1);
      assert(workspace.cols() == static_cast<int>(degree) + 1);
      assert(degree <= start_i);
      assert(start_i < knots.size() - 1 - degree);
      assert(knots[degree] <= x);
      assert(x <= knots[knots.size() - 1 - degree]);

      workspace.setZero();
      workspace.row(0).setOnes();
      if (ignore > 0)
      {
        workspace(0, ignore - 1) = Scalar(0.);
      }
      int first_pass_start_knot =
        static_cast<int>(start_i) - static_cast<int>(degree) + static_cast<int>(ignore);
      assert(first_pass_start_knot >= 0);
      // Evaluate basis function with only non zero right coefficient
      for (int r = 0; r < static_cast<int>(ignore); ++r)
      {
        int current_degree = static_cast<int>(degree) - (r + 1);
        assert(current_degree >= 0);
        int last_knot = first_pass_start_knot + current_degree + 1;
        assert(last_knot >= 0);
        Scalar alpha =
          (x - knots[first_pass_start_knot]) / (knots[last_knot] - knots[first_pass_start_knot]);
        workspace(r + 1, ignore - (r + 1)) = alpha * workspace(r, ignore - r);
      }

      // Iterate over all degree
      for (int r = 0; r < static_cast<int>(degree); ++r)
      {
        int current_degree = static_cast<int>(degree) - (r + 1);
        int start = std::max(0, static_cast<int>(ignore) - r);
        for (int j = start; j < current_degree + 1; ++j)
        {
          int current_knot = static_cast<int>(start_i) - current_degree + j;
          assert(current_knot >= 0);
          int last_knot = current_knot + current_degree + 1;
          assert(last_knot >= 0);
          Scalar alpha = (x - knots[current_knot]) / (knots[last_knot] - knots[current_knot]);
          workspace(r + 1, j) = (1.0 - alpha) * workspace(r, j) + alpha * workspace(r, j + 1);
        }
      }
      return workspace(workspace.rows() - 1, 0);
    }

    /** De Boor algorithm modification to compute all basis involved to compute one
     * point of the curve.
     * \param degree Curve degree
     * \param knots Knot vector at least of size \p degree + 1.
     * \param root_basis Degree 0 basis function index where knot vector contains \p x.
     * This is the only degree 0 basis function != 0.
     * \param x Value to evaluate
     * \param basis of size (degree + 1, degree + 1).
     * Each element i, j of this array will hold a basis function N_{i,j} where i and j
     * are respectively the basis function index and degree.
     */
    template<typename Scalar>
    void deBoorBasis(
      int degree,
      const Eigen::Matrix<Scalar, Eigen::Dynamic, 1> & knots,
      int root_basis,
      Scalar x,
      Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> & basis)
    {
      assert(degree >= 0);
      assert(basis.rows() == static_cast<int>(degree) + 1);
      assert(basis.cols() == static_cast<int>(degree) + 1);
      assert(knots.size() > degree + 1);
      assert(degree <= root_basis);
      assert(root_basis < knots.size() - 1 - degree);
      assert(knots[degree] <= x);
      assert(x <= knots[knots.size() - 1 - degree]);

      // Compute left most and right most basis functions
      basis(0, 0) = Scalar(1);
      for (int previous_degree = 0; previous_degree < degree; ++previous_degree)
      {
        const int current_degree = previous_degree + 1;
        const int left_most_basis = root_basis - current_degree;
        const int left_most_basis_start_knot = left_most_basis + 1;
        const int left_most_basis_end_knot = left_most_basis_start_knot + current_degree;
        const Scalar left_most_basis_alpha =
          (knots[left_most_basis_end_knot] - x)
          / (knots[left_most_basis_end_knot] - knots[left_most_basis_start_knot]);
        basis(current_degree, 0) = left_most_basis_alpha * basis(previous_degree, 0);

        const int right_most_basis = root_basis;
        const int right_most_basis_start_knot = right_most_basis;
        const int right_most_basis_end_knot = right_most_basis_start_knot + current_degree;
        const Scalar right_most_basis_alpha =
          (x - knots[right_most_basis_start_knot])
          / (knots[right_most_basis_end_knot] - knots[right_most_basis_start_knot]);
        basis(current_degree, current_degree) =
          (right_most_basis_alpha * basis(previous_degree, previous_degree));
      }

      // Compute central basis functions
      for (int previous_degree = 1; previous_degree < degree; ++previous_degree)
      {
        const int current_degree = previous_degree + 1;
        const int left_most_basis = root_basis - current_degree;
        const int basis_numbers = current_degree + 1;
        for (int i = 1; i < basis_numbers - 1; ++i)
        {
          const int current_basis = left_most_basis + i;
          const int left_side_start_knot = current_basis;
          const int left_side_end_knot = current_basis + current_degree;
          const Scalar left_side_alpha =
            (x - knots[left_side_start_knot])
            / (knots[left_side_end_knot] - knots[left_side_start_knot]);

          const int right_side_start_knot = left_side_start_knot + 1;
          const int right_side_end_knot = left_side_end_knot + 1;
          const Scalar right_side_alpha =
            (knots[right_side_end_knot] - x)
            / (knots[right_side_end_knot] - knots[right_side_start_knot]);

          basis(current_degree, i) = left_side_alpha * basis(previous_degree, i - 1)
                                     + right_side_alpha * basis(previous_degree, i);
        }
      }
    }

  } // namespace internal

} // namespace pinocchio
