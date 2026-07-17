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
      assert(basis.rows() >= degree + 1);
      assert(basis.cols() >= degree + 1);
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

    /** Return num / den if den != 0 and 0 in the other case.
     * This function help support Casadi scalar type that doesn't support
     * comparison.
     */
    template<typename Scalar>
    Scalar safeAlpha(Scalar num, Scalar den)
    {
      // clang-format off
      // if(den > dummy_precision)
      //  return (num / den)
      // else
      //  return 0
      // clang-format on
      return if_then_else(
        GT, den, Eigen::NumTraits<Scalar>::dummy_precision(), (num / den), Scalar(0));
    }

    /** De Boor algorithm modification to compute all basis in the spline
     * valid span ([knots[degree], knots[m - degree - 1]].
     * This function will compute a lot of zeros and it's implemented for
     * Casadi scalar support. Use deBoorBasis instead.
     * \param degree Curve degree.
     * \param knots Knot vector of size m (at least of size \p degree + 1)
     * \param q Value to evaluate.
     * \param basis of size (degree + 1, m - degree - 1).
     * Each element i, j of this array will hold a basis function N_{i,j} where i and j
     * are respectively the basis function index and degree.
     */
    template<typename Scalar>
    void deBoorFullBasis(
      int degree,
      const Eigen::Matrix<Scalar, Eigen::Dynamic, 1> & knots,
      Scalar q,
      Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> & basis)
    {
      assert(degree >= 0);
      assert(basis.rows() >= degree + 1);
      // Number of max degree basis functions
      assert(basis.cols() >= knots.size() - degree - 1);
      assert(knots.size() > degree + 1);
      basis.setZero();
      const int first_degree0_basis = degree;
      const int last_degree0_basis = knots.size() - degree - 2;
      const int nb_degree0_basis = last_degree0_basis + 1 - first_degree0_basis;

      // Compute degree 0 basis function values
      for (int i = 0; i < nb_degree0_basis; ++i)
      {
        int current_basis = first_degree0_basis + i;
        // clang-format off
        // if(knots[i] <= x && x < knots[i + 1])
        //  return 1;
        // else
        //  return 0;
        // clang-format on
        Scalar is_in_standard_range = if_then_else(
          LE, knots[current_basis], q,
          if_then_else(LT, q, knots[current_basis + 1], Scalar(1), Scalar(0)), Scalar(0));

        // clang-format off
        // if(x == knots.back() && x == knots[i + 1])
        //  return 1;
        // else
        //  return 0;
        // clang-format on
        Scalar is_at_final_range = if_then_else(
          EQ, q, knots[knots.size() - degree - 1],
          if_then_else(EQ, q, knots[current_basis + 1], Scalar(1), Scalar(0)), Scalar(0));

        basis(0, i) = is_in_standard_range + is_at_final_range;
      }

      // Compute left most and right most basis functions (first pass).
      for (int previous_degree = 0; previous_degree < degree; ++previous_degree)
      {
        const int current_degree = previous_degree + 1;
        const int basis_numbers = nb_degree0_basis + current_degree;
        const int left_most_basis = first_degree0_basis - current_degree;
        const int left_most_basis_start_knot = left_most_basis + 1;
        const int left_most_basis_end_knot = left_most_basis_start_knot + current_degree;
        const Scalar left_most_basis_alpha_num = knots[left_most_basis_end_knot] - q;
        const Scalar left_most_basis_alpha_den =
          knots[left_most_basis_end_knot] - knots[left_most_basis_start_knot];
        basis(current_degree, 0) = safeAlpha(left_most_basis_alpha_num, left_most_basis_alpha_den)
                                   * basis(previous_degree, 0);

        const int right_most_basis = last_degree0_basis;
        const int right_most_basis_start_knot = right_most_basis;
        const int right_most_basis_end_knot = right_most_basis_start_knot + current_degree;
        const Scalar right_most_basis_alpha_num = q - knots[right_most_basis_start_knot];
        const Scalar right_most_basis_alpha_den =
          knots[right_most_basis_end_knot] - knots[right_most_basis_start_knot];
        basis(current_degree, basis_numbers - 1) =
          safeAlpha(right_most_basis_alpha_num, right_most_basis_alpha_den)
          * basis(previous_degree, basis_numbers - 2);
      }

      // Compute central basis functions (second pass).
      for (int previous_degree = 0; previous_degree < degree; ++previous_degree)
      {
        const int current_degree = previous_degree + 1;
        const int left_most_basis = first_degree0_basis - current_degree;
        const int basis_numbers = nb_degree0_basis + current_degree;
        for (int i = 1; i < basis_numbers - 1; ++i)
        {
          const int current_basis = left_most_basis + i;
          const int left_side_start_knot = current_basis;
          const int left_side_end_knot = current_basis + current_degree;
          const Scalar left_side_alpha_num = (q - knots[left_side_start_knot]);
          const Scalar left_side_alpha_den =
            (knots[left_side_end_knot] - knots[left_side_start_knot]);

          const int right_side_start_knot = left_side_start_knot + 1;
          const int right_side_end_knot = left_side_end_knot + 1;
          const Scalar right_side_alpha_num = (knots[right_side_end_knot] - q);
          const Scalar right_side_alpha_den =
            (knots[right_side_end_knot] - knots[right_side_start_knot]);

          basis(current_degree, i) =
            safeAlpha(left_side_alpha_num, left_side_alpha_den) * basis(previous_degree, i - 1)
            + safeAlpha(right_side_alpha_num, right_side_alpha_den) * basis(previous_degree, i);
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

      const int derivative1_degree = degree - 1;
      const int derivative2_degree = degree - 2;
      const Scalar derivative1_den = knots[index + degree] - knots[index];
      Scalar phi_ddot_i_sum = Scalar(0);
      // basis only contains non zero basis function.
      // This condition prevent get an out of bound basis function on the left.
      if (index >= root_basis - derivative2_degree)
      {
        const Scalar left_side_den = knots[index + derivative1_degree] - knots[index];
        phi_ddot_i_sum =
          getAbsoluteBasis(root_basis, basis, index, derivative2_degree) / left_side_den;
      }
      // basis only contains non zero basis function.
      // This condition prevent get an out of bound basis function on the right.
      if (index + 1 < root_basis + 1)
      {
        const Scalar right_side_den = knots[index + derivative1_degree + 1] - knots[index + 1];
        phi_ddot_i_sum -=
          getAbsoluteBasis(root_basis, basis, index + 1, derivative2_degree) / right_side_den;
      }
      return ((degree * derivative1_degree) / derivative1_den) * phi_ddot_i_sum;
    }

  } // namespace internal

} // namespace pinocchio
