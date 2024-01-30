//
// Copyright (c) 2022-2024 INRIA
//

#ifndef __pinocchio_algorithm_admm_solver_hpp__
#define __pinocchio_algorithm_admm_solver_hpp__

#include "pinocchio/algorithm/constraints/fwd.hpp"
#include "pinocchio/math/fwd.hpp"
#include "pinocchio/math/comparison-operators.hpp"

#include "pinocchio/algorithm/contact-solver-base.hpp"
#include "pinocchio/algorithm/delassus-operator-base.hpp"

namespace pinocchio
{
  template<typename _Scalar>
  struct ADMMContactSolverTpl
  : ContactSolverBaseTpl<_Scalar>
  {
    typedef _Scalar Scalar;
    typedef ContactSolverBaseTpl<_Scalar> Base;
    typedef Eigen::Matrix<Scalar,Eigen::Dynamic,1> VectorXs;

    using Base::problem_size;

    explicit ADMMContactSolverTpl(const int problem_dim)
    : Base(problem_dim)
    , rho_power(Scalar(0.2))
    , ratio_primal_dual(Scalar(10))
    , x_(problem_dim)
    , y_(problem_dim)
    , y_previous(problem_dim)
    , z_(problem_dim)
    , s_(VectorXs::Zero(problem_dim))
    , rhs(problem_dim)
    , primal_feasibility_vector(problem_dim)
    , dual_feasibility_vector(problem_dim)
    {}

    /// \brief Set the power associated to the problem conditionning.
    void setRhoPower(const Scalar rho_power)
    {
      this->rho_power = rho_power;
    }
    /// \brief Get the power associated to the problem conditionning.
    Scalar getRhoPower() const { return rho_power; }

    /// \brief Set the primal/dual ratio.
    void setRatioPrimalDual(const Scalar ratio_primal_dual)
    {
      PINOCCHIO_CHECK_INPUT_ARGUMENT(ratio_primal_dual > 0.,"The ratio primal/dual should be positive strictly");
      this->ratio_primal_dual = ratio_primal_dual;
    }
    /// \brief Get the primal/dual ratio.
    Scalar getRatioPrimalDual() const { return ratio_primal_dual; }

    /// \returns the number of updates of the Cholesky factorization due to rho updates.
    int getCholeskyUpdateCount() const { return cholesky_update_count; }

    ///
    /// \brief Solve the constrained conic problem composed of problem data (G,g,cones) and starting from the initial guess.
    ///
    /// \param[in] G Symmetric PSD matrix representing the Delassus of the contact problem.
    /// \param[in] g Free contact acceleration or velicity associted with the contact problem.
    /// \param[in] cones Vector of conic constraints.
    /// \param[in,out] x Initial guess and output solution of the problem
    /// \param[in] mu_prox Proximal smoothing value associated to the algorithm.
    /// \param[in] R Proximal regularization value associated to the compliant contacts (corresponds to the lowest non-zero).
    /// \param[in] tau Over relaxation value
    ///
    /// \returns True if the problem has converged.
    template<typename DelassusDerived, typename VectorLike, typename ConstraintAllocator, typename VectorLikeOut, typename VectorLikeR>
    bool solve(DelassusOperatorBase<DelassusDerived> & delassus, const Eigen::MatrixBase<VectorLike> & g,
               const std::vector<CoulombFrictionConeTpl<Scalar>,ConstraintAllocator> & cones,
               const Eigen::DenseBase<VectorLikeOut> & x,
               const Scalar mu_prox,
               const Eigen::MatrixBase<VectorLikeR> & R,
               const Scalar tau = Scalar(0.99));


    ///
    /// \brief Solve the constrained conic problem composed of problem data (G,g,cones) and starting from the initial guess.
    ///
    /// \param[in] G Symmetric PSD matrix representing the Delassus of the contact problem.
    /// \param[in] g Free contact acceleration or velicity associted with the contact problem.
    /// \param[in] cones Vector of conic constraints.
    /// \param[in,out] x Initial guess and output solution of the problem
    /// \param[in] mu_prox Proximal smoothing value associated to the algorithm.
    /// \param[in] tau Over relaxation value
    ///
    /// \returns True if the problem has converged.
    template<typename DelassusDerived, typename VectorLike, typename ConstraintAllocator, typename VectorLikeOut, typename VectorLikeR>
    bool solve(DelassusOperatorBase<DelassusDerived> & delassus, const Eigen::MatrixBase<VectorLike> & g,
               const std::vector<CoulombFrictionConeTpl<Scalar>,ConstraintAllocator> & cones,
               const Eigen::DenseBase<VectorLikeOut> & x,
               const Scalar mu_prox,
               const Scalar tau = Scalar(0.99))
    {
      return solve(delassus.derived(),g.derived(),cones,x.const_cast_derived(),mu_prox,VectorXs::Zero(problem_size),tau);
    }

    /// \returns the primal solution of the problem
    const VectorXs & getPrimalSolution() const { return y_; }
    /// \returns the dual solution of the problem
    const VectorXs & getDualSolution() const { return z_; }
    /// \returns the complementarity shift
    const VectorXs & getComplementarityShift() const { return s_; }

  protected:

    /// \brief Power value associated to rho. This quantity will be automatically updated.
    Scalar rho_power;
    /// \brief Ratio primal/dual
    Scalar ratio_primal_dual;

    /// \brief Primal variables (corresponds to the contact forces)
    VectorXs x_, y_;
    /// \brief Previous value of y.
    VectorXs y_previous;
    /// \brief Dual varible of the ADMM (corresponds to the contact velocity or acceleration).
    VectorXs z_;
    /// \brief De Saxé shift
    VectorXs s_;

    VectorXs rhs, primal_feasibility_vector, dual_feasibility_vector;

    int cholesky_update_count;

  }; // struct ADMMContactSolverTpl
}

#include "pinocchio/algorithm/admm-solver.hxx"

#endif // ifndef __pinocchio_algorithm_admm_solver_hpp__
