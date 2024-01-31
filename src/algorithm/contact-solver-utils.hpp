//
// Copyright (c) 2024 INRIA
//

#ifndef __pinocchio_algorithm_contact_solver_utils_hpp__
#define __pinocchio_algorithm_contact_solver_utils_hpp__

#include "pinocchio/math/fwd.hpp"
#include "pinocchio/math/comparison-operators.hpp"
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

} // namespace pinocchio

#endif // ifndef __pinocchio_algorithm_contact_solver_utils_hpp__
