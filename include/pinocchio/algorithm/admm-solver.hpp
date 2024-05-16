//
// Copyright (c) 2022-2024 INRIA
//

#ifndef __pinocchio_algorithm_admm_solver_hpp__
#define __pinocchio_algorithm_admm_solver_hpp__

#include "pinocchio/algorithm/constraints/fwd.hpp"
#include "pinocchio/math/fwd.hpp"
#include "pinocchio/math/comparison-operators.hpp"
#include "pinocchio/math/eigenvalues.hpp"

#include "pinocchio/algorithm/contact-solver-base.hpp"
#include "pinocchio/algorithm/delassus-operator-base.hpp"

#include <boost/optional.hpp>

namespace pinocchio
{
  template<typename _Scalar>
  struct PINOCCHIO_UNSUPPORTED_MESSAGE("The API will change towards more flexibility")
    ADMMContactSolverTpl : ContactSolverBaseTpl<_Scalar>
  {
    typedef _Scalar Scalar;
    typedef ContactSolverBaseTpl<_Scalar> Base;
    typedef Eigen::Matrix<Scalar, Eigen::Dynamic, 1> VectorXs;
    typedef const Eigen::Ref<const VectorXs> ConstRefVectorXs;
    typedef Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> MatrixXs;
    typedef PowerIterationAlgoTpl<VectorXs> PowerIterationAlgo;

    using Base::problem_size;

    //    struct SolverParameters
    //    {
    //      explicit SolverParameters(const int problem_dim)
    //      : rho_power(Scalar(0.2))
    //      , ratio_primal_dual(Scalar(10))
    //      , mu_prox
    //      {
    //
    //      }
    //
    //      /// \brief Rho solver ADMM
    //      boost::optional<Scalar> rho;
    //      /// \brief Power value associated to rho. This quantity will be automatically updated.
    //      Scalar rho_power;
    //      /// \brief Ratio primal/dual
    //      Scalar ratio_primal_dual;
    //      /// \brief Proximal value
    //      Scalar mu_prox;
    //
    //      /// \brief Largest eigenvalue
    //      boost::optional<Scalar> L_value;
    //      /// \brief Largest eigenvector
    //      boost::optional<VectorXs> L_vector;
    //    };
    //
    struct SolverStats
    {
      explicit SolverStats(const int max_it)
      : it(0)
      , cholesky_update_count(0)
      {
        primal_feasibility.reserve(size_t(max_it));
        dual_feasibility.reserve(size_t(max_it));
        dual_feasibility_ncp.reserve(size_t(max_it));
        complementarity.reserve(size_t(max_it));
        rho.reserve(size_t(max_it));
      }

      void reset()
      {
        primal_feasibility.clear();
        dual_feasibility.clear();
        complementarity.clear();
        dual_feasibility_ncp.clear();
        rho.clear();
        it = 0;
        cholesky_update_count = 0;
      }

      size_t size() const
      {
        return primal_feasibility.size();
      }

      ///  \brief Number of total iterations.
      int it;

      ///  \brief Number of Cholesky updates.
      int cholesky_update_count;

      /// \brief History of primal feasibility values.
      std::vector<Scalar> primal_feasibility;

      /// \brief History of dual feasibility values.
      std::vector<Scalar> dual_feasibility;
      std::vector<Scalar> dual_feasibility_ncp;

      /// \brief History of complementarity values.
      std::vector<Scalar> complementarity;

      /// \brief History of rho values.
      std::vector<Scalar> rho;
    };
    //
    //    struct SolverResults
    //    {
    //      explicit SolverResults(const int problem_dim, const int max_it)
    //      : L_vector(problem_dim)
    //
    //      /// \brief Largest eigenvalue
    //      Scalar L_value;
    //      /// \brief Largest eigenvector
    //      VectorXs L_vector;
    //
    //      SolverStats stats;
    //    };

    explicit ADMMContactSolverTpl(
      int problem_dim,
      Scalar mu_prox = Scalar(1e-6),
      Scalar tau = Scalar(0.5),
      Scalar rho_power = Scalar(0.2),
      Scalar rho_power_factor = Scalar(0.05),
      Scalar ratio_primal_dual = Scalar(10),
      int max_it_largest_eigen_value_solver = 20)
    : Base(problem_dim)
    , is_initialized(false)
    , mu_prox(mu_prox)
    , tau(tau)
    , rho(Scalar(-1))
    , rho_power(rho_power)
    , rho_power_factor(rho_power_factor)
    , ratio_primal_dual(ratio_primal_dual)
    , max_it_largest_eigen_value_solver(max_it_largest_eigen_value_solver)
    , power_iteration_algo(problem_dim)
    , x_(VectorXs::Zero(problem_dim))
    , y_(VectorXs::Zero(problem_dim))
    , x_previous(VectorXs::Zero(problem_dim))
    , y_previous(VectorXs::Zero(problem_dim))
    , z_previous(VectorXs::Zero(problem_dim))
    , z_(VectorXs::Zero(problem_dim))
    , s_(VectorXs::Zero(problem_dim))
    , rhs(problem_dim)
    , primal_feasibility_vector(VectorXs::Zero(problem_dim))
    , dual_feasibility_vector(VectorXs::Zero(problem_dim))
    , stats(Base::max_it)
    {
      power_iteration_algo.max_it = max_it_largest_eigen_value_solver;
    }

    /// \brief Set the ADMM penalty value.
    void setRho(const Scalar rho)
    {
      this->rho = rho;
    }
    /// \brief Get the ADMM penalty value.
    Scalar getRho() const
    {
      return rho;
    }

    /// \brief Set the power associated to the problem conditionning.
    void setRhoPower(const Scalar rho_power)
    {
      this->rho_power = rho_power;
    }
    /// \brief Get the power associated to the problem conditionning.
    Scalar getRhoPower() const
    {
      return rho_power;
    }

    /// \brief Set the power factor associated to the problem conditionning.
    void setRhoPowerFactor(const Scalar rho_power_factor)
    {
      this->rho_power_factor = rho_power_factor;
    }
    /// \brief Get the power factor  associated to the problem conditionning.
    Scalar getRhoPowerFactor() const
    {
      return rho_power_factor;
    }

    /// \brief Set the tau linear scaling factor.
    void setTau(const Scalar tau)
    {
      this->tau = tau;
    }
    /// \brief Get the tau linear scaling factor.
    Scalar getTau() const
    {
      return tau;
    }

    /// \brief Set the proximal value.
    void setProximalValue(const Scalar mu)
    {
      this->mu_prox = mu;
    }
    /// \brief Get the proximal value.
    Scalar getProximalValue() const
    {
      return mu_prox;
    }

    /// \brief Set the primal/dual ratio.
    void setRatioPrimalDual(const Scalar ratio_primal_dual)
    {
      PINOCCHIO_CHECK_INPUT_ARGUMENT(
        ratio_primal_dual > 0., "The ratio primal/dual should be positive strictly");
      this->ratio_primal_dual = ratio_primal_dual;
    }
    /// \brief Get the primal/dual ratio.
    Scalar getRatioPrimalDual() const
    {
      return ratio_primal_dual;
    }

    ///  \returns the number of updates of the Cholesky factorization due to rho updates.
    int getCholeskyUpdateCount() const
    {
      return cholesky_update_count;
    }

    ///
    /// \brief Solve the constrained conic problem composed of problem data (G,g,cones) and starting
    /// from the initial guess.
    ///
    /// \param[in] G Symmetric PSD matrix representing the Delassus of the contact problem.
    /// \param[in] g Free contact acceleration or velicity associted with the contact problem.
    /// \param[in] cones Vector of conic constraints.
    /// \param[in,out] x Initial guess and output solution of the problem
    /// \param[in] mu_prox Proximal smoothing value associated to the algorithm.
    /// \param[in] R Proximal regularization value associated to the compliant contacts (corresponds
    /// to the lowest non-zero). \param[in] tau Over relaxation value
    ///
    /// \returns True if the problem has converged.
    template<
      typename DelassusDerived,
      typename VectorLike,
      typename ConstraintAllocator,
      typename VectorLikeR>
    bool solve(
      DelassusOperatorBase<DelassusDerived> & delassus,
      const Eigen::MatrixBase<VectorLike> & g,
      const std::vector<CoulombFrictionConeTpl<Scalar>, ConstraintAllocator> & cones,
      const Eigen::MatrixBase<VectorLikeR> & R,
      const boost::optional<ConstRefVectorXs> primal_guess = boost::none,
      const boost::optional<ConstRefVectorXs> dual_guess = boost::none,
      bool compute_largest_eigen_values = true,
      bool stat_record = false);

    ///
    /// \brief Solve the constrained conic problem composed of problem data (G,g,cones) and starting
    /// from the initial guess.
    ///
    /// \param[in] G Symmetric PSD matrix representing the Delassus of the contact problem.
    /// \param[in] g Free contact acceleration or velicity associted with the contact problem.
    /// \param[in] cones Vector of conic constraints.
    /// \param[in,out] x Initial guess and output solution of the problem
    /// \param[in] mu_prox Proximal smoothing value associated to the algorithm.
    /// \param[in] tau Over relaxation value
    ///
    /// \returns True if the problem has converged.
    template<
      typename DelassusDerived,
      typename VectorLike,
      typename ConstraintAllocator,
      typename VectorLikeOut>
    bool solve(
      DelassusOperatorBase<DelassusDerived> & delassus,
      const Eigen::MatrixBase<VectorLike> & g,
      const std::vector<CoulombFrictionConeTpl<Scalar>, ConstraintAllocator> & cones,
      const Eigen::DenseBase<VectorLikeOut> & x)
    {
      return solve(
        delassus.derived(), g.derived(), cones, x.const_cast_derived(),
        VectorXs::Zero(problem_size));
    }

    /// \returns the primal solution of the problem
    const VectorXs & getPrimalSolution() const
    {
      return y_;
    }
    /// \returns the dual solution of the problem
    const VectorXs & getDualSolution() const
    {
      return z_;
    }
    /// \returns the complementarity shift
    const VectorXs & getComplementarityShift() const
    {
      return s_;
    }

    /// \brief Compute the penalty ADMM value from the current largest and lowest eigenvalues and
    /// the scaling spectral factor.
    static inline Scalar computeRho(const Scalar L, const Scalar m, const Scalar rho_power)
    {
      const Scalar cond = L / m;
      const Scalar rho = math::sqrt(L * m) * math::pow(cond, rho_power);
      return rho;
    }

    /// \brief Compute the  scaling spectral factor of the ADMM penalty term from the current
    /// largest and lowest eigenvalues and the ADMM penalty term.
    static inline Scalar computeRhoPower(const Scalar L, const Scalar m, const Scalar rho)
    {
      const Scalar cond = L / m;
      const Scalar sqtr_L_m = math::sqrt(L * m);
      const Scalar rho_power = math::log(rho / sqtr_L_m) / math::log(cond);
      return rho_power;
    }

    PowerIterationAlgo & getPowerIterationAlgo()
    {
      return power_iteration_algo;
    }

    SolverStats & getStats()
    {
      return stats;
    }

  protected:
    bool is_initialized;

    /// \brief proximal value
    Scalar mu_prox;

    /// \brief Linear scaling of the ADMM penalty term
    Scalar tau;

    /// \brief Penalty term associated to the ADMM.
    Scalar rho;
    /// \brief Power value associated to rho. This quantity will be automatically updated.
    Scalar rho_power;
    /// \brief Update factor for the primal/dual update of rho.
    Scalar rho_power_factor;
    ///  \brief Ratio primal/dual
    Scalar ratio_primal_dual;

    /// \brief Maximum number of iterarions called for the power iteration algorithm
    int max_it_largest_eigen_value_solver;

    /// \brief Power iteration algo.
    PowerIterationAlgo power_iteration_algo;

    /// \brief Primal variables (corresponds to the contact forces)
    VectorXs x_, y_;
    /// \brief Previous value of y.
    VectorXs x_previous, y_previous, z_previous;
    /// \brief Dual varible of the ADMM (corresponds to the contact velocity or acceleration).
    VectorXs z_;
    /// \brief De Saxé shift
    VectorXs s_;

    VectorXs rhs, primal_feasibility_vector, dual_feasibility_vector;

    int cholesky_update_count;

    /// \brief Stats of the solver
    SolverStats stats;

#ifdef PINOCCHIO_WITH_HPP_FCL
    using Base::timer;
#endif // PINOCCHIO_WITH_HPP_FCL
  }; // struct ADMMContactSolverTpl
} // namespace pinocchio

#include "pinocchio/algorithm/admm-solver.hxx"

#endif // ifndef __pinocchio_algorithm_admm_solver_hpp__
