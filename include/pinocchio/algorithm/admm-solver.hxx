//
// Copyright (c) 2022-2024 INRIA
//

#ifndef __pinocchio_algorithm_admm_solver_hxx__
#define __pinocchio_algorithm_admm_solver_hxx__

#include <limits>

#include "pinocchio/algorithm/contact-solver-utils.hpp"
#include "pinocchio/algorithm/constraints/coulomb-friction-cone.hpp"

namespace pinocchio
{

  template<typename _Scalar>
  template<
    typename DelassusDerived,
    typename VectorLike,
    typename ConstraintAllocator,
    typename VectorLikeR>
  bool ADMMContactSolverTpl<_Scalar>::solve(
    DelassusOperatorBase<DelassusDerived> & _delassus,
    const Eigen::MatrixBase<VectorLike> & g,
    const std::vector<CoulombFrictionConeTpl<Scalar>, ConstraintAllocator> & cones,
    const Eigen::MatrixBase<VectorLikeR> & R,
    const boost::optional<ConstRefVectorXs> primal_guess,
    const boost::optional<ConstRefVectorXs> dual_guess,
    bool compute_largest_eigen_values,
    bool stat_record)

  {
    using namespace internal;

    DelassusDerived & delassus = _delassus.derived();

    const Scalar mu_R = R.minCoeff();
    PINOCCHIO_CHECK_INPUT_ARGUMENT(tau <= Scalar(1) && tau > Scalar(0), "tau should lie in ]0,1].");
    PINOCCHIO_CHECK_INPUT_ARGUMENT(mu_prox >= 0, "mu_prox should be positive.");
    PINOCCHIO_CHECK_INPUT_ARGUMENT(mu_R >= Scalar(0), "R should be a positive vector.");
    PINOCCHIO_CHECK_ARGUMENT_SIZE(R.size(), problem_size);
    //    PINOCCHIO_CHECK_INPUT_ARGUMENT(math::max(R.maxCoeff(),mu_prox) >= 0,"mu_prox and mu_R
    //    cannot be both equal to zero.");

    if (compute_largest_eigen_values)
    {
      //      const Scalar L = delassus.computeLargestEigenValue(20); // Largest eigen_value
      //      estimate.
      power_iteration_algo.run(delassus);
    }
    const Scalar L = power_iteration_algo.largest_eigen_value;
    //    const Scalar L = delassus.computeLargestEigenValue(20);
    const Scalar m = mu_prox + mu_R;
    const Scalar cond = L / m;
    const Scalar rho_increment = std::pow(cond, rho_power_factor);

    Scalar complementarity,
      proximal_metric, // proximal metric between two successive iterates.
      primal_feasibility, dual_feasibility_ncp, dual_feasibility;

    //    std::cout << std::setprecision(12);

    Scalar rho;
    rho = computeRho(L, m, rho_power);
    //    if(!is_initialized)
    //    {
    //      rho = computeRho(L,m,rho_power);
    //    }
    //    else
    //    {
    //      rho = this->rho;
    //    }
    //    rho = computeRho(L,m,rho_power);

    //    std::cout << "L: " << L << std::endl;
    //    std::cout << "m: " << m << std::endl;
    //    std::cout << "prox_value: " << prox_value << std::endl;

    PINOCCHIO_EIGEN_MALLOC_NOT_ALLOWED();

    // Update the cholesky decomposition
    Scalar prox_value = mu_prox + tau * rho;
    rhs = R + VectorXs::Constant(this->problem_size, prox_value);
    delassus.updateDamping(rhs);
    cholesky_update_count = 1;

    // Initial update of the variables
    // Init x
    if (primal_guess)
    {
      x_ = primal_guess.get();
      PINOCCHIO_CHECK_ARGUMENT_SIZE(x_.size(), problem_size);
    }
    else if (!is_initialized)
    {
      x_.setZero();
    }
    else
    {
      x_ = y_; // takes the current value stored in the solver
    }

    // Init y
    computeConeProjection(cones, x_, y_);

    // Init z
    if (dual_guess)
    {
      z_ = dual_guess.get();
      PINOCCHIO_CHECK_ARGUMENT_SIZE(z_.size(), problem_size);
    }
    else if (!is_initialized)
    {
      delassus.applyOnTheRight(y_, z_); // z = G * y
      z_.noalias() += -prox_value * y_ + g;
      computeComplementarityShift(cones, z_, s_);
      z_ += s_; // Add De Saxé shift
      computeDualConeProjection(cones, z_, z_);
    }
    else
    {
      // Keep z from the previous iteration
    }

    //    std::cout << "x_: " << x_.transpose() << std::endl;
    //    std::cout << "y_: " << y_.transpose() << std::endl;
    //    std::cout << "z_: " << z_.transpose() << std::endl;

    if (stat_record)
    {
      stats.reset();

      // Compute initial problem primal and dual feasibility
      primal_feasibility_vector = x_ - y_;
      primal_feasibility = primal_feasibility_vector.template lpNorm<Eigen::Infinity>();
    }

    is_initialized = true;

    // End of Initialization phase

    bool abs_prec_reached = false, rel_prec_reached = false;

    Scalar y_previous_norm_inf = y_.template lpNorm<Eigen::Infinity>();
    int it = 1;
//    Scalar res = 0;
#ifdef PINOCCHIO_WITH_HPP_FCL
    timer.start();
#endif // PINOCCHIO_WITH_HPP_FCL
    for (; it <= Base::max_it; ++it)
    {
      //      std::cout << "---" << std::endl;
      //      std::cout << "it: " << it << std::endl;
      //      std::cout << "tau*rho: " << tau*rho << std::endl;

      x_previous = x_;
      y_previous = y_;
      z_previous = z_;
      complementarity = Scalar(0);

      // s-update
      computeComplementarityShift(cones, z_, s_);

      //      std::cout << "s_: " << s_.transpose() << std::endl;

      //      std::cout << "x_: " << x_.transpose() << std::endl;

      // z-update
      //      const Scalar alpha = 1.;
      //      z_ -= (tau*rho) * (x_ - y_);
      //      std::cout << "intermediate z_: " << z_.transpose() << std::endl;

      // x-update
      rhs = -(g + s_ - (rho * tau) * y_ - mu_prox * x_ - z_);
      const VectorXs rhs_copy = rhs;
      //      x_ = rhs;
      delassus.solveInPlace(rhs);
      //      VectorXs tmp = delassus * rhs - rhs_copy;
      //      res = math::max(res,tmp.template lpNorm<Eigen::Infinity>());
      //      std::cout << "residual = " << (delassus * rhs - x_).template lpNorm<Eigen::Infinity>()
      //      << std::endl;
      x_ = rhs;

      // y-update
      //      rhs *= alpha;
      //      rhs += (1-alpha)*y_previous;
      rhs = x_;
      rhs -= z_ / (tau * rho);
      computeConeProjection(cones, rhs, y_);
      //      std::cout << "y_: " << y_.transpose() << std::endl;

      // z-update
      z_ -= (tau * rho) * (x_ - y_);
      //      const Scalar gamma = Scalar(it) / Scalar(it + 300);

      //      z_ += gamma * (z_ - z_previous).eval();
      //      x_ += gamma * (x_ - x_previous).eval();
      //      computeConeProjection(cones, y_, y_);

      //      z_ -= (tau*rho) * (x_ * alpha + (1-alpha)*y_previous - y_);
      //      std::cout << "z_: " << z_.transpose() << std::endl;
      //      computeDualConeProjection(cones, z_, z_);

      // check termination criteria
      primal_feasibility_vector = x_ - y_;
      //      delassus.applyOnTheRight(x_,dual_feasibility_vector);
      //      dual_feasibility_vector.noalias() += g + s_ - prox_value * x_ - z_;

      {
        VectorXs & dy = rhs;
        dy = y_ - y_previous;
        proximal_metric = dy.template lpNorm<Eigen::Infinity>();
        dual_feasibility_vector.noalias() = (tau * rho) * dy;
      }

      {
        VectorXs & dx = rhs;
        dx = x_ - x_previous;
        dual_feasibility_vector.noalias() += mu_prox * dx;
      }

      //      delassus.applyOnTheRight(x_,dual_feasibility_vector);
      //      dual_feasibility_vector.noalias() += g;
      //      computeComplementarityShift(cones, z_, s_);
      //      dual_feasibility_vector.noalias() += s_ - prox_value * x_ - z_;

      primal_feasibility = primal_feasibility_vector.template lpNorm<Eigen::Infinity>();
      dual_feasibility = dual_feasibility_vector.template lpNorm<Eigen::Infinity>();
      complementarity = computeConicComplementarity(cones, z_, y_);
      //      complementarity = z_.dot(y_)/cones.size();

      if (stat_record)
      {
        VectorXs tmp(rhs);
        delassus.applyOnTheRight(y_, rhs);
        rhs.noalias() += g - prox_value * y_;
        computeComplementarityShift(cones, rhs, tmp);
        rhs.noalias() += tmp;

        internal::computeDualConeProjection(cones, rhs, tmp);
        tmp -= rhs;

        dual_feasibility_ncp = tmp.template lpNorm<Eigen::Infinity>();

        stats.primal_feasibility.push_back(primal_feasibility);
        stats.dual_feasibility.push_back(dual_feasibility);
        stats.dual_feasibility_ncp.push_back(dual_feasibility_ncp);
        stats.complementarity.push_back(complementarity);
        stats.rho.push_back(rho);
      }

      //      std::cout << "primal_feasibility: " << primal_feasibility << std::endl;
      //      std::cout << "dual_feasibility: " << dual_feasibility << std::endl;
      //      std::cout << "complementarity: " << complementarity << std::endl;

      // Checking stopping residual
      if (
        check_expression_if_real<Scalar, false>(complementarity <= this->absolute_precision)
        && check_expression_if_real<Scalar, false>(dual_feasibility <= this->absolute_precision)
        && check_expression_if_real<Scalar, false>(primal_feasibility <= this->absolute_precision))
        abs_prec_reached = true;
      else
        abs_prec_reached = false;

      const Scalar y_norm_inf = y_.template lpNorm<Eigen::Infinity>();
      if (check_expression_if_real<Scalar, false>(
            proximal_metric
            <= this->relative_precision * math::max(y_norm_inf, y_previous_norm_inf)))
        rel_prec_reached = true;
      else
        rel_prec_reached = false;

      //      if(abs_prec_reached || rel_prec_reached)
      if (abs_prec_reached)
        break;

      // Account for potential update of rho
      bool update_delassus_factorization = false;
      if (primal_feasibility > ratio_primal_dual * dual_feasibility)
      {
        rho *= rho_increment;
        //        rho *= math::pow(cond,rho_power_factor);
        //        rho_power += rho_power_factor;
        update_delassus_factorization = true;
      }
      else if (dual_feasibility > ratio_primal_dual * primal_feasibility)
      {
        rho /= rho_increment;
        //        rho *= math::pow(cond,-rho_power_factor);
        //        rho_power -= rho_power_factor;
        update_delassus_factorization = true;
      }

      if (update_delassus_factorization)
      {
        prox_value = mu_prox + tau * rho;
        rhs = R + VectorXs::Constant(this->problem_size, prox_value);
        delassus.updateDamping(rhs);
        cholesky_update_count++;
      }

      y_previous_norm_inf = y_norm_inf;
      //      std::cout << "rho_power: " << rho_power << std::endl;
      //      std::cout << "rho: " << rho << std::endl;
      //      std::cout << "---" << std::endl;
    }

    PINOCCHIO_EIGEN_MALLOC_ALLOWED();

    this->absolute_residual =
      math::max(primal_feasibility, math::max(complementarity, dual_feasibility));
    this->relative_residual = proximal_metric;
    this->it = it;
    //    std::cout << "max linalg res: " << res << std::endl;
    //    y_sol.const_cast_derived() = y_;

    // Save values
    this->rho_power = computeRhoPower(L, m, rho);
    this->rho = rho;

    if (stat_record)
    {
      stats.it = it;
      stats.cholesky_update_count = cholesky_update_count;
    }

#ifdef PINOCCHIO_WITH_HPP_FCL
    timer.stop();
#endif // PINOCCHIO_WITH_HPP_FCL

    //    if(abs_prec_reached || rel_prec_reached)
    if (abs_prec_reached)
      return true;

    return false;
  }
} // namespace pinocchio

#endif // ifndef __pinocchio_algorithm_admm_solver_hxx__
