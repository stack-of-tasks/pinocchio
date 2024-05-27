//
// Copyright (c) 2022 INRIA
//

#include "pinocchio/multibody/data.hpp"

namespace pinocchio
{

  template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI
  DataTpl<context::Scalar, context::Options, JointCollectionDefaultTpl>::DataTpl();

  template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI
  DataTpl<context::Scalar, context::Options, JointCollectionDefaultTpl>::DataTpl(
    const context::Model &);

} // namespace pinocchio
