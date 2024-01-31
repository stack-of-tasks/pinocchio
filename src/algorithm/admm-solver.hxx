//
// Copyright (c) 2022-2024 INRIA
//

#ifndef __pinocchio_algorithm_admm_solver_hxx__
#define __pinocchio_algorithm_admm_solver_hxx__

#include <limits>
#include <iomanip>
#include <iostream>

#include "pinocchio/algorithm/contact-solver-utils.hpp"
#include "pinocchio/algorithm/constraints/coulomb-friction-cone.hpp"

namespace pinocchio
{

  template<typename _Scalar>
  template<typename DelassusDerived, typename VectorLike, typename ConstraintAllocator, typename VectorLikeOut, typename VectorLikeR>
  bool ADMMContactSolverTpl<_Scalar>::solve(DelassusOperatorBase<DelassusDerived> & _delassus,
                                            const Eigen::MatrixBase<VectorLike> & g,
                                            const std::vector<CoulombFrictionConeTpl<Scalar>,ConstraintAllocator> & cones,
                                            const Eigen::DenseBase<VectorLikeOut> & y_sol,
                                            const Scalar mu_prox,
                                            const Eigen::MatrixBase<VectorLikeR> & R,
                                            const Scalar tau)

  {
    using namespace internal;

    DelassusDerived & delassus = _delassus.derived();

    PINOCCHIO_CHECK_INPUT_ARGUMENT(tau <= Scalar(1) && tau > Scalar(0),"tau should lie in ]0,1].");
    PINOCCHIO_CHECK_INPUT_ARGUMENT(mu_prox >= 0,"mu_prox should be positive.");
    PINOCCHIO_CHECK_INPUT_ARGUMENT((R.array() >= 0).all(),"mu_prox should be positive.");
//    PINOCCHIO_CHECK_INPUT_ARGUMENT(math::max(R.maxCoeff(),mu_prox) >= 0,"mu_prox and mu_R cannot be both equal to zero.");

    const Scalar mu_R = R.maxCoeff();

    const Scalar L = delassus.computeLargestEigenValue(20); // Largest eigen_value estimate.
    const Scalar m = math::max(mu_R,mu_prox);
    const Scalar cond = L / m;

//    std::cout << std::setprecision(12);

    Scalar rho = math::sqrt(m * L) * math::pow(cond,rho_power);
    Scalar prox_value = mu_prox + tau * rho;

//    std::cout << "L: " << L << std::endl;
//    std::cout << "m: " << m << std::endl;
//    std::cout << "prox_value: " << prox_value << std::endl;

    int it = 0;

    PINOCCHIO_EIGEN_MALLOC_NOT_ALLOWED();

    // Update the cholesky decomposition
    rhs = R + VectorXs::Constant(this->problem_size,prox_value);
    delassus.updateDamping(rhs);
    cholesky_update_count = 1;

    // Initial update of the variables
    x_ = y_sol;
    computeConeProjection(cones, x_, y_);
    delassus.applyOnTheRight(y_,z_); // z = G * y
    z_.noalias() += -prox_value * y_ + g;
    computeDualConeProjection(cones, z_, z_);
    //computeComplementarityShift(cones, z_, s_);

//    std::cout << "x_: " << x_.transpose() << std::endl;
//    std::cout << "y_: " << y_.transpose() << std::endl;
//    std::cout << "z_: " << z_.transpose() << std::endl;

    Scalar
    complementarity,
    proximal_metric, // proximal metric between two successive iterates.
    primal_feasibility,
    dual_feasibility;
    bool abs_prec_reached = false, rel_prec_reached = false;

    Scalar y_previous_norm_inf = y_.template lpNorm<Eigen::Infinity>();

    for(; it < Base::max_it; ++it)
    {
//      std::cout << "---" << std::endl;
//      std::cout << "it: " << it << std::endl;
//      std::cout << "tau*rho: " << tau*rho << std::endl;

      y_previous = y_;
      x_previous = x_;
      complementarity = Scalar(0);

      // s-update
      computeComplementarityShift(cones, z_, s_);

//      std::cout << "s_: " << s_.transpose() << std::endl;

      // x-update
      rhs = -(g + s_ - (rho*tau) * y_ - mu_prox * x_ - z_);
//      x_ = rhs;
      delassus.solveInPlace(rhs);
//      std::cout << "residual = " << (delassus * rhs - x_).template lpNorm<Eigen::Infinity>() << std::endl;
      x_ = rhs;
//      std::cout << "x_: " << x_.transpose() << std::endl;

      // z-update
//      z_ -= (tau*rho) * (x_ - y_);
//      std::cout << "intermediate z_: " << z_.transpose() << std::endl;

      // y-update
      rhs -= z_/(tau*rho);
      computeConeProjection(cones, rhs, y_);
//      std::cout << "y_: " << y_.transpose() << std::endl;

      // z-update
      z_ -= (tau*rho) * (x_ - y_);
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


      primal_feasibility = primal_feasibility_vector.template lpNorm<Eigen::Infinity>();
      dual_feasibility = dual_feasibility_vector.template lpNorm<Eigen::Infinity>();
//      complementarity = computeMaxConeComplementarity(cones,z_,y_);
      complementarity = z_.dot(y_)/cones.size();

//      std::cout << "primal_feasibility: " << primal_feasibility << std::endl;
//      std::cout << "dual_feasibility: " << dual_feasibility << std::endl;
//      std::cout << "complementarity: " << complementarity << std::endl;

      // Checking stopping residual
      if(   check_expression_if_real<Scalar,false>(complementarity <= this->absolute_precision)
         && check_expression_if_real<Scalar,false>(dual_feasibility <= this->absolute_precision)
         && check_expression_if_real<Scalar,false>(primal_feasibility <= this->absolute_precision))
        abs_prec_reached = true;
      else
        abs_prec_reached = false;

      const Scalar y_norm_inf = y_.template lpNorm<Eigen::Infinity>();
      if(check_expression_if_real<Scalar,false>(proximal_metric <= this->relative_precision * math::max(y_norm_inf,y_previous_norm_inf)))
        rel_prec_reached = true;
      else
        rel_prec_reached = false;

//      if(abs_prec_reached || rel_prec_reached)
      if(abs_prec_reached)
        break;

      // Account for potential update of rho
      const Scalar rho_power_factor = 0.05;
      bool update_delassus_factorization = false;
      if(primal_feasibility > ratio_primal_dual * dual_feasibility)
      {
        rho *= math::pow(cond,rho_power_factor);
        rho_power += rho_power_factor;
        update_delassus_factorization = true;
      }
      else if(dual_feasibility > ratio_primal_dual * primal_feasibility)
      {
        rho *= math::pow(cond,-rho_power_factor);
        rho_power -= rho_power_factor;
        update_delassus_factorization = true;
      }

      if(update_delassus_factorization)
      {
        prox_value = mu_prox + tau * rho;
        rhs = R + VectorXs::Constant(this->problem_size,prox_value);
        delassus.updateDamping(rhs);
        cholesky_update_count++;
      }

      y_previous_norm_inf = y_norm_inf;
//      std::cout << "rho_power: " << rho_power << std::endl;
//      std::cout << "rho: " << rho << std::endl;
//      std::cout << "---" << std::endl;
    }

    PINOCCHIO_EIGEN_MALLOC_ALLOWED();

    this->absolute_residual = math::max(primal_feasibility,math::max(complementarity,dual_feasibility));
    this->relative_residual = proximal_metric;
    this->it = it;
    y_sol.const_cast_derived() = y_;

//    if(abs_prec_reached || rel_prec_reached)
    if(abs_prec_reached)
      return true;

    return false;
  }
}

#endif // ifndef __pinocchio_algorithm_admm_solver_hxx__