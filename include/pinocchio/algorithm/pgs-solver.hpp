//
// Copyright (c) 2022-2024 INRIA
//

#ifndef __pinocchio_algorithm_pgs_solver_hpp__
#define __pinocchio_algorithm_pgs_solver_hpp__

#include "pinocchio/algorithm/constraints/fwd.hpp"
#include "pinocchio/algorithm/contact-solver-base.hpp"

namespace pinocchio
{

  /// \brief Projected Gauss Siedel solver
  template<typename _Scalar>
  struct PGSContactSolverTpl : ContactSolverBaseTpl<_Scalar>
  {
    typedef _Scalar Scalar;
    typedef ContactSolverBaseTpl<Scalar> Base;
    typedef Eigen::Matrix<Scalar, Eigen::Dynamic, 1> VectorXs;

    explicit PGSContactSolverTpl(const int problem_size)
    : Base(problem_size)
    , x(problem_size)
    , x_previous(problem_size)
    {
    }

    ///
    /// \brief Solve the constrained conic problem composed of problem data (G,g,cones) and starting
    /// from the initial guess.
    ///
    /// \param[in] G Symmetric PSD matrix representing the Delassus of the contact problem.
    /// \param[in] g Free contact acceleration or velicity associted with the contact problem.
    /// \param[in] cones Vector of conic constraints.
    /// \param[in,out] x Initial guess and output solution of the problem
    /// \param[in] over_relax Over relaxation value
    ///
    /// \returns True if the problem has converged.
    template<
      typename MatrixLike,
      typename VectorLike,
      typename ConstraintAllocator,
      typename VectorLikeOut>
    bool solve(
      const MatrixLike & G,
      const Eigen::MatrixBase<VectorLike> & g,
      const std::vector<CoulombFrictionConeTpl<Scalar>, ConstraintAllocator> & cones,
      const Eigen::DenseBase<VectorLikeOut> & x,
      const Scalar over_relax = Scalar(1));

  protected:
    /// \brief Previous temporary value of the optimum.
    VectorXs x, x_previous;
#ifdef PINOCCHIO_WITH_HPP_FCL
    using Base::timer;
#endif // PINOCCHIO_WITH_HPP_FCL

  }; // struct PGSContactSolverTpl
} // namespace pinocchio

#include "pinocchio/algorithm/pgs-solver.hxx"

#endif // ifndef __pinocchio_algorithm_pgs_solver_hpp__
