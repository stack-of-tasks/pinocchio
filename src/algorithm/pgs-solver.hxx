//
// Copyright (c) 2022 INRIA
//

#ifndef __pinocchio_algorithm_pgs_solver_hxx__
#define __pinocchio_algorithm_pgs_solver_hxx__

#include <limits>

namespace pinocchio
{
  template<typename _Scalar>
  template<typename MatrixLike, typename VectorLike, typename ConstraintAllocator, typename VectorLikeOut>
  bool PGSUnilateralContactSolverTpl<_Scalar>::solve(const Eigen::MatrixBase<MatrixLike> & G, const Eigen::MatrixBase<VectorLike> & g,
                                                     const std::vector<CoulombFrictionConeTpl<Scalar>,ConstraintAllocator> & cones,
                                                     const Eigen::DenseBase<VectorLikeOut> & x_,
                                                     const Scalar over_relax)

  {
    typedef CoulombFrictionConeTpl<Scalar> CoulombFrictionCone;
    typedef Eigen::Matrix<Scalar,2,1> Vector2;
    typedef Eigen::Matrix<Scalar,3,1> Vector3;
    //typedef Eigen::Matrix<Scalar,6,1> Vector6;

    PINOCCHIO_CHECK_INPUT_ARGUMENT(over_relax < Scalar(2) && over_relax > Scalar(0),"over_relax should lie in ]0,2[.")

    const size_t nc = cones.size(); // num constraints
    VectorLikeOut & x = x_.const_cast_derived();

    int it = 0;
    Vector3 velocity; // tmp variable

    PINOCCHIO_EIGEN_MALLOC_NOT_ALLOWED();

    Scalar complementarity, proximal_metric;
    bool abs_prec_reached = false, rel_prec_reached = false;
    for(; it < max_it; ++it)
    {
      x_previous = x;
      complementarity = Scalar(0);
      for(size_t cone_id = 0; cone_id < nc; ++cone_id)
      {
        const Eigen::DenseIndex row_id = 3*cone_id;
        const CoulombFrictionCone & cone = cones[cone_id];

        const auto G_block = G.template block<3,3>(row_id, row_id);
        auto x_segment = x.template segment<3>(row_id);

        velocity.template head<3>().noalias() = G.template middleRows<3>(row_id) * x + g.template segment<3>(row_id);

        // Normal update
        Scalar & fz = x_segment.coeffRef(2);
        const Scalar fz_previous = fz;
        fz -= Scalar(over_relax/G_block(2,2)) * velocity[2];
        fz = math::max(Scalar(0), fz);

        // Account for the fz updated value
        velocity.template head<3>().noalias() += G_block.col(2) * (fz - fz_previous);

        // Tangential update
        const Scalar min_D_tangent = math::min(G_block(0,0),G_block(1,1));
        auto f_tangent = x_segment.template head<2>();
        const Vector2 f_tangent_previous = f_tangent;

        assert(min_D_tangent > 0 && "min_D_tangent is zero");
        f_tangent -= over_relax/min_D_tangent * velocity.template head<2>();
        const Scalar f_tangent_norm = f_tangent.norm();

        const Scalar mu_fz = cone.mu * fz;
        if(f_tangent_norm > mu_fz) // Project in the circle of radius mu_fz
          f_tangent *= mu_fz/f_tangent_norm;

        // Account for the f_tangent updated value
        velocity.template head<3>().noalias() = G_block.template leftCols<2>() * (f_tangent - f_tangent_previous);
        Scalar contact_complementarity = cone.computeContactComplementarity(velocity, x_segment);
        assert(contact_complementarity >= Scalar(0) && "contact_complementarity should be positive");
        complementarity = math::max(complementarity,contact_complementarity);
      }

      // Checking stopping residual
      if(check_expression_if_real<Scalar,false>(complementarity <= absolute_precision))
        abs_prec_reached = true;
      else
        abs_prec_reached = false;

      proximal_metric = (x - x_previous).template lpNorm<Eigen::Infinity>();
      if(check_expression_if_real<Scalar,false>(proximal_metric <= relative_precision))
        rel_prec_reached = true;
      else
        rel_prec_reached = false;

      if(abs_prec_reached || rel_prec_reached)
        break;
    }

    PINOCCHIO_EIGEN_MALLOC_ALLOWED();

    absolute_residual = complementarity;
    relative_residual = proximal_metric;
    this->it = it;

    if(abs_prec_reached || rel_prec_reached)
      return true;

    return false;
  }
}

#endif // ifndef __pinocchio_algorithm_pgs_solver_hxx__

