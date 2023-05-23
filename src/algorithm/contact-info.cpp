//
// Copyright (c) 2023 CNRS INRIA
//

#include "pinocchio/algorithm/contact-info.hpp"

namespace pinocchio {

  template struct BaumgarteCorrectorParametersTpl<context::Scalar>;
  template struct RigidConstraintModelTpl<context::Scalar, context::Options>;
  template struct RigidConstraintDataTpl<context::Scalar, context::Options>;

} // namespace pinocchio
