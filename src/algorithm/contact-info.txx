//
// Copyright (c) 2023 CNRS INRIA
//

#ifndef __pinocchio_algorithm_contact_info_txx__
#define __pinocchio_algorithm_contact_info_txx__

namespace pinocchio {

  extern template struct BaumgarteCorrectorParametersTpl<context::Scalar>;
  extern template struct RigidConstraintModelTpl<context::Scalar, context::Options>;
  extern template struct RigidConstraintDataTpl<context::Scalar, context::Options>;

} // namespace pinocchio

#endif  // #ifndef __pinocchio_algorithm_contact_info_txx__
