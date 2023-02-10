//
// Copyright (c) 2022 INRIA
//

#ifndef __pinocchio_algorithm_pgs_solver_hpp__
#define __pinocchio_algorithm_pgs_solver_hpp__

#include "pinocchio/math/fwd.hpp"
#include "pinocchio/math/comparison-operators.hpp"

namespace pinocchio
{

  template<typename _Scalar>
  struct ContactSolverBaseTpl
  {
    typedef _Scalar Scalar;

    explicit ContactSolverBaseTpl(const int problem_size)
    : problem_size(problem_size)
    , max_it(1000), it(0)
    , absolute_precision(Scalar(1e-6)), relative_precision(Scalar(1e-6))
    , absolute_residual(Scalar(-1)), relative_residual(Scalar(-1))
    {}

    /// \brief Returns the size of the problem
    int getProblemSize() const { return problem_size; }

    /// \brief Get the number of iterations achieved by the solver.
    int getIterationCount() const { return it; }

    /// \brief Set the maximum number of iterations.
    void setMaxIterations(const int max_it)
    {
      PINOCCHIO_CHECK_INPUT_ARGUMENT(max_it > 0,"max_it should be greater than 0.");
      this->max_it = max_it;
    }
    /// \brief Get the maximum number of iterations allowed.
    int getMaxIterations() const { return max_it; }

    /// \brief Set the absolute precision for the problem.
    void setAbsolutePrecision(const Scalar absolute_precision)
    {
      PINOCCHIO_CHECK_INPUT_ARGUMENT(absolute_precision >= Scalar(0),"absolute_precision should be positive.");
      this->absolute_precision = absolute_precision;
    }
    /// \brief Get the absolute precision requested.
    Scalar getAbsolutePrecision() const { return absolute_precision; }

    /// \brief Set the relative precision for the problem.
    void setRelativePrecision(const Scalar relative_precision)
    {
      PINOCCHIO_CHECK_INPUT_ARGUMENT(relative_precision >= Scalar(0),"relative_precision should be positive.");
      this->relative_precision = relative_precision;
    }
    /// \brief Get the relative precision requested.
    Scalar getRelativePrecision() const { return relative_precision; }

    /// \brief Returns the value of the absolute residual value corresponding to the contact complementary conditions.
    Scalar getAbsoluteConvergenceResidual() const { return absolute_residual; }
    /// \brief Returns the value of the relative residual value corresponding to the difference between two successive iterates (infinity norms).
    Scalar getRelativeConvergenceResidual() const { return relative_residual; }

  protected:

    /// \brief Size of the problem
    int problem_size;
    /// \brief Maximum number of iterations.
    int max_it;
    /// \brief Number of iterations needed to achieve convergence.
    int it;
    /// \brief Desired absolute precision.
    Scalar absolute_precision;
    /// \brief Desired relative precision.
    Scalar relative_precision;
    /// \brief Absolule convergence residual value.
    Scalar absolute_residual;
    /// \brief Relative convergence residual value
    Scalar relative_residual;

  }; // struct ContactSolverBaseTpl

  /// \brief Projected Gauss Siedel solver
  template<typename _Scalar>
  struct PGSContactSolverTpl : ContactSolverBaseTpl<_Scalar>
  {
    typedef _Scalar Scalar;
    typedef ContactSolverBaseTpl<Scalar> Base;
    typedef Eigen::Matrix<Scalar,Eigen::Dynamic,1> VectorXs;

    explicit PGSContactSolverTpl(const int problem_size)
    : Base(problem_size)
    , x_previous(problem_size)
    {}

    ///
    /// \brief Solve the constrained conic problem composed of problem data (G,g,cones) and starting from the initial guess.
    ///
    /// \param[in] G Symmetric PSD matrix representing the Delassus of the contact problem.
    /// \param[in] g Free contact acceleration or velicity associted with the contact problem.
    /// \param[in] cones Vector of conic constraints.
    /// \param[in,out] x Initial guess and output solution of the problem
    /// \param[in] over_relax Over relaxation value
    ///
    /// \returns True if the problem has converged.
    template<typename MatrixLike, typename VectorLike, typename ConstraintAllocator, typename VectorLikeOut>
    bool solve(const Eigen::MatrixBase<MatrixLike> & G, const Eigen::MatrixBase<VectorLike> & g,
               const std::vector<CoulombFrictionConeTpl<Scalar>,ConstraintAllocator> & cones,
               const Eigen::DenseBase<VectorLikeOut> & x,
               const Scalar over_relax = Scalar(1));

  protected:

    /// \brief Previous temporary value of the optimum.
    VectorXs x_previous;

  }; // struct PGSContactSolverTpl
}

#include "pinocchio/algorithm/pgs-solver.hxx"

#endif // ifndef __pinocchio_algorithm_pgs_solver_hpp__
