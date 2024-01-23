//
// Copyright (c) 2022-2024 INRIA
//

#ifndef __pinocchio_algorithm_admm_solver_hxx__
#define __pinocchio_algorithm_admm_solver_hxx__

#include <limits>
#include <iomanip>
#include "pinocchio/algorithm/constraints/coulomb-friction-cone.hpp"

namespace pinocchio
{

namespace internal
{

/// \brief Project a vector x on the vector of cones.
template<typename Scalar, typename ConstraintAllocator, typename VectorLikeIn, typename VectorLikeOut>
void computeConeProjection(const std::vector<CoulombFrictionConeTpl<Scalar>,ConstraintAllocator> & cones,
                           const Eigen::DenseBase<VectorLikeIn> & x,
                           const Eigen::DenseBase<VectorLikeOut> & x_proj_)
{
  assert(x.size() == x_proj_.size());
  Eigen::DenseIndex index = 0;
  VectorLikeOut & x_proj = x_proj_.const_cast_derived();
  for(const auto & cone: cones)
  {
    x_proj.template segment<3>(index) = cone.project(x.template segment<3>(index));
    assert(cone.isInside(x_proj.template segment<3>(index),1e-12));
    index += 3;
  }
}

/// \brief Project a vector x on the dual of the cones contained in the vector of cones.
template<typename Scalar, typename ConstraintAllocator, typename VectorLikeIn, typename VectorLikeOut>
void computeDualConeProjection(const std::vector<CoulombFrictionConeTpl<Scalar>,ConstraintAllocator> & cones,
                               const Eigen::DenseBase<VectorLikeIn> & x,
                               const Eigen::DenseBase<VectorLikeOut> & x_proj_)
{
  assert(x.size() == x_proj_.size());
  Eigen::DenseIndex index = 0;
  VectorLikeOut & x_proj = x_proj_.const_cast_derived();
  for(const auto & cone: cones)
  {
    x_proj.template segment<3>(index) = cone.dual().project(x.template segment<3>(index));
    assert(cone.dual().isInside(x_proj.template segment<3>(index),1e-12));
    index += 3;
  }
}

template<typename Scalar, typename ConstraintAllocator, typename VectorLikeVelocity, typename VectorLikeForce>
Scalar computeMaxConeComplementarity(const std::vector<CoulombFrictionConeTpl<Scalar>,ConstraintAllocator> & cones,
                                     const Eigen::DenseBase<VectorLikeVelocity> & velocities,
                                     const Eigen::DenseBase<VectorLikeForce> & forces)
{
  assert(velocities.size() == forces.size());
  Eigen::DenseIndex index = 0;
  Scalar complementarity = 0;
  for(const auto & cone: cones)
  {
    const Scalar cone_complementarity = cone.computeContactComplementarity(velocities.template segment<3>(index),
                                                                           forces.template segment<3>(index));
    complementarity = math::max(complementarity, cone_complementarity);
    index += 3;
  }

  return complementarity;
}

template<typename Scalar, typename ConstraintAllocator, typename VectorLikeIn, typename VectorLikeOut>
void computeComplementarityShift(const std::vector<CoulombFrictionConeTpl<Scalar>,ConstraintAllocator> & cones,
                                 const Eigen::DenseBase<VectorLikeIn> & velocities,
                                 const Eigen::DenseBase<VectorLikeOut> & shift_)
{
  assert(velocities.size() == shift_.size());
  Eigen::DenseIndex index = 0;
  VectorLikeOut & shift = shift_.const_cast_derived();
  for(const auto & cone: cones)
  {
    shift.template segment<3>(index) = cone.computeNormalCorrection(velocities.template segment<3>(index));
    index += 3;
  }
}

}  // namespace internal

  template<typename _Scalar>
  template<typename DelassusExpression, typename VectorLike, typename ConstraintAllocator, typename VectorLikeOut, typename VectorLikeR>
  bool ADMMContactSolverTpl<_Scalar>::solve(DelassusExpression & delassus,
                                            const Eigen::MatrixBase<VectorLike> & g,
                                            const std::vector<CoulombFrictionConeTpl<Scalar>,ConstraintAllocator> & cones,
                                            const Eigen::DenseBase<VectorLikeOut> & y_sol,
                                            const Scalar mu_prox,
                                            const Eigen::MatrixBase<VectorLikeR> & R,
                                            const Scalar tau)

  {
    using namespace internal;

    PINOCCHIO_CHECK_INPUT_ARGUMENT(tau <= Scalar(1) && tau > Scalar(0),"tau should lie in ]0,1].");
    PINOCCHIO_CHECK_INPUT_ARGUMENT(mu_prox >= 0,"mu_prox should be positive.");
    PINOCCHIO_CHECK_INPUT_ARGUMENT((R.array() >= 0).all(),"mu_prox should be positive.");
//    PINOCCHIO_CHECK_INPUT_ARGUMENT(math::max(R.maxCoeff(),mu_prox) >= 0,"mu_prox and mu_R cannot be both equal to zero.");

    const Scalar mu_R = R.maxCoeff();

    const Scalar L = delassus.computeLargestEigenValue(5); // Largest eigen_value estimate.
    const Scalar m = math::max(mu_R,mu_prox);
    const Scalar cond = L / m;

    Scalar rho = math::sqrt(m * L) * math::pow(cond,rho_power);
    Scalar prox_value = mu_prox + tau * rho;

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
    computeComplementarityShift(cones, z_, s_);

    Scalar
    complementarity,
    proximal_metric, // proximal metric between two successive iterates.
    primal_feasibility,
    dual_feasibility;
    bool abs_prec_reached = false, rel_prec_reached = false;

    Scalar y_previous_norm_inf = y_.template lpNorm<Eigen::Infinity>();

    for(; it < Base::max_it; ++it)
    {
      y_previous = y_;
      complementarity = Scalar(0);

      // s-update
      computeComplementarityShift(cones, z_, s_);

      // x-update
      rhs = -(g + s_ - (rho*tau) * y_ - mu_prox * x_ - z_);
      delassus.solveInPlace(rhs);
      x_ = rhs;

      // z-update
      z_ -= (tau*rho) * (x_ - y_);

      // y-update
      rhs -= z_/(tau*rho);
      computeConeProjection(cones, rhs, y_);

      // z-update
      z_ -= (tau*rho) * (x_ - y_);
//      computeDualConeProjection(cones, z_, z_);

      // check termination criteria
      primal_feasibility_vector = x_ - y_;
      delassus.applyOnTheRight(x_,dual_feasibility_vector);
      dual_feasibility_vector.noalias() += g + s_ - prox_value * x_ - z_;

      primal_feasibility = primal_feasibility_vector.template lpNorm<Eigen::Infinity>();
      dual_feasibility = dual_feasibility_vector.template lpNorm<Eigen::Infinity>();
      complementarity = computeMaxConeComplementarity(cones,z_,y_);

      // Checking stopping residual
      if(   check_expression_if_real<Scalar,false>(complementarity <= this->absolute_precision)
         && check_expression_if_real<Scalar,false>(dual_feasibility <= this->absolute_precision))
        abs_prec_reached = true;
      else
        abs_prec_reached = false;

      proximal_metric = (y_ - y_previous).template lpNorm<Eigen::Infinity>();
      const Scalar y_norm_inf = y_.template lpNorm<Eigen::Infinity>();
      if(check_expression_if_real<Scalar,false>(proximal_metric <= this->relative_precision * math::max(y_norm_inf,y_previous_norm_inf)))
        rel_prec_reached = true;
      else
        rel_prec_reached = false;

      if(abs_prec_reached || rel_prec_reached)
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
    }

    PINOCCHIO_EIGEN_MALLOC_ALLOWED();

    this->absolute_residual = math::max(complementarity,dual_feasibility);
    this->relative_residual = proximal_metric;
    this->it = it;
    y_sol.const_cast_derived() = y_;

    if(abs_prec_reached || rel_prec_reached)
      return true;

    return false;
  }
}

#endif // ifndef __pinocchio_algorithm_admm_solver_hxx__
