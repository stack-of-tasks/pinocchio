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
    /// @brief Define a Knot span [start_idx; end_idx[.
    /// @details This struct identifies the subset of control frames in a spline that are active
    /// (i.e., have non-zero basis functions) for a specific spline parameter value.
    struct SpanIndexes
    {
      size_t start_idx;
      size_t end_idx;
    };

    /// @brief Compute SpanIndexes for a knot vector and a parameter q.
    template<typename Scalar>
    struct FindSpan
    {
      template<typename KnotsVector>
      static SpanIndexes
      run(const Scalar q, const int degree, const Eigen::MatrixBase<KnotsVector> & knots)
      {
        assert(degree >= 0);
        assert(knots.size() > degree + 1);
        assert(knots[degree] <= q);
        assert(q <= knots[knots.size() - 1 - degree]);

        int low = degree;
        for (; low < knots.size() - 1 - degree; ++low)
        {
          if (knots[low] <= q && q < knots[low + 1])
          {
            return {static_cast<size_t>(low), static_cast<size_t>(low + 1)};
          }
        }

        return {static_cast<size_t>(low - 1), static_cast<size_t>(low)};
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

    /** De Boor algorithm modification to compute all basis involved to compute one
     * point of the curve.
     * \param degree Curve degree.
     * \param knots Knot vector at least of size \p degree + 1.
     * \param root_basis Degree 0 basis function index where knot vector contains \p x.
     * This is the only degree 0 basis function != 0.
     * \param q Value to evaluate.
     * \param basis of size (degree + 1, degree + 1).
     * Each element i, j of this array will hold a basis function N_{i,j} where i and j
     * are respectively the basis function index and degree.
     */
    template<typename Scalar>
    void deBoorBasis(
      int degree,
      const Eigen::Matrix<Scalar, Eigen::Dynamic, 1> & knots,
      int root_basis,
      Scalar q,
      Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> & basis)
    {
      assert(degree >= 0);
      assert(basis.rows() >= static_cast<int>(degree) + 1);
      assert(basis.cols() >= static_cast<int>(degree) + 1);
      assert(knots.size() > degree + 1);
      assert(degree <= root_basis);
      assert(root_basis < knots.size() - 1 - degree);
      assert(knots[root_basis] <= q);
      assert(q <= knots[root_basis + 1]);

      /*
       * N_{i,j} is a basis function where i and j are respectively the index and the degree.
       * Here N_{i,2} computation scheme:
       *
       *  N_{0,2}       N_{1,2}       N_{2,2}
       *   (1,3)\  (1,3)/ (2,4)\ (2,4)/
       *         N_{1,1}       N_{2,1}
       *        /  (2,3)\ (2,3)/     \
       *  N_{1,0}       N_{2,0}       N_{3,0}
       * [u1, u2[      [u2, u3[      [u3, u4[
       *
       * When u is in [u2, u3[ range N_{1,0} and N_{3,0} basis function value to 0.
       * This simplify the computation scheme by allowing to only compute
       * N_{1,1}, N_{2,1}, N_{0,2}, N_{1,2} and N_{2,2}.
       * N_{1,1}, N_{0,2}, N_{2,1} and N_{2,2} can be computed only with one alpha value.
       * We do it in the first pass where we compute left most and right most basis functions.
       * Only N_{1,2} will need both alpha to be computed. We do that in the second pass.
       */

      // Compute left most and right most basis functions (first pass).
      basis(0, 0) = Scalar(1);
      for (int previous_degree = 0; previous_degree < degree; ++previous_degree)
      {
        const int current_degree = previous_degree + 1;
        const int left_most_basis = root_basis - current_degree;
        const int left_most_basis_start_knot = left_most_basis + 1;
        const int left_most_basis_end_knot = left_most_basis_start_knot + current_degree;
        const Scalar left_most_basis_alpha =
          (knots[left_most_basis_end_knot] - q)
          / (knots[left_most_basis_end_knot] - knots[left_most_basis_start_knot]);
        basis(current_degree, 0) = left_most_basis_alpha * basis(previous_degree, 0);

        const int right_most_basis = root_basis;
        const int right_most_basis_start_knot = right_most_basis;
        const int right_most_basis_end_knot = right_most_basis_start_knot + current_degree;
        const Scalar right_most_basis_alpha =
          (q - knots[right_most_basis_start_knot])
          / (knots[right_most_basis_end_knot] - knots[right_most_basis_start_knot]);
        basis(current_degree, current_degree) =
          (right_most_basis_alpha * basis(previous_degree, previous_degree));
      }

      // Compute central basis functions (second pass).
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
            (q - knots[left_side_start_knot])
            / (knots[left_side_end_knot] - knots[left_side_start_knot]);

          const int right_side_start_knot = left_side_start_knot + 1;
          const int right_side_end_knot = left_side_end_knot + 1;
          const Scalar right_side_alpha =
            (knots[right_side_end_knot] - q)
            / (knots[right_side_end_knot] - knots[right_side_start_knot]);

          basis(current_degree, i) = left_side_alpha * basis(previous_degree, i - 1)
                                     + right_side_alpha * basis(previous_degree, i);
        }
      }
    }

    /** Return basis function value N_{index,degree} from basis matrix computed by \p deBoorBasis.
     * \param root_basis Argument provided to \p deBoorBasis function.
     * \param basis Basis matrix computed by \p deBoorBasis.
     * \param index Index of the basis function.
     * \param degree Degree of the basis function.
     */
    template<typename Scalar>
    const Scalar & getAbsoluteBasis(
      int root_basis,
      const Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> & basis,
      int index,
      int degree)
    {
      assert(0 <= root_basis);
      assert(0 <= index);
      assert(0 <= degree);
      assert(degree < basis.rows());

      const int offset = root_basis - degree;
      assert(offset <= index);

      return basis(degree, index - offset);
    }

    /** Compute cumulative basis first derivative for N_{index,degree}.
     * \param root_basis Argument provided to \p deBoorBasis function.
     * \param knots Knot vector at least of size \p degree + 1.
     * \param basis Basis matrix computed by \p deBoorBasis.
     * \param index Index of the basis function.
     * \param degree Degree of the basis function.
     */
    template<typename Scalar>
    Scalar cumulativeBasisDerivative(
      int root_basis,
      const Eigen::Matrix<Scalar, Eigen::Dynamic, 1> & knots,
      const Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> & basis,
      int index,
      int degree)
    {
      assert(0 <= index);
      assert(0 <= degree);
      assert(index + degree < knots.size());

      const Scalar alpha = degree / (knots[index + degree] - knots[index]);
      return alpha * getAbsoluteBasis(root_basis, basis, index, degree - 1);
    }

    /** Compute cumulative basis second derivative for N_{index,degree}.
     * \param root_basis Argument provided to \p deBoorBasis function.
     * \param knots Knot vector at least of size \p degree + 1.
     * \param basis Basis matrix computed by \p deBoorBasis.
     * \param index Index of the basis function.
     * \param degree Degree of the basis function.
     */
    template<typename Scalar>
    Scalar cumulativeBasisDerivative2(
      int root_basis,
      const Eigen::Matrix<Scalar, Eigen::Dynamic, 1> & knots,
      const Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> & basis,
      int index,
      int degree)
    {
      assert(0 <= index);
      assert(0 <= degree);
      assert(index + degree + 1 < knots.size());

      const int derivative_degree = degree - 2;
      const Scalar index_knot_diff = knots[index + degree] - knots[index];
      Scalar phi_ddot_i_sum = Scalar(0);
      // basis only contains non zero basis function.
      // This condition prevent get an out of bound basis function on the left.
      if (index >= root_basis - derivative_degree)
      {
        phi_ddot_i_sum =
          getAbsoluteBasis(root_basis, basis, index, derivative_degree) / index_knot_diff;
      }
      // basis only contains non zero basis function.
      // This condition prevent get an out of bound basis function on the right.
      if (index + 1 < root_basis + 1)
      {
        const Scalar right_side_den = knots[index + degree + 1] - knots[index + 1];
        phi_ddot_i_sum -=
          getAbsoluteBasis(root_basis, basis, index + 1, derivative_degree) / right_side_den;
      }
      return ((degree * degree) / index_knot_diff) * phi_ddot_i_sum;
    }

  } // namespace internal

} // namespace pinocchio
