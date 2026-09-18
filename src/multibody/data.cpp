//
// Copyright (c) 2022 INRIA
//

#include "pinocchio/src/context/template-instantiation.hxx"
#include "pinocchio/multibody.hpp"

namespace pinocchio
{

  template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI
  DataTpl<context::Scalar, context::Options, JointCollectionDefaultTpl>::DataTpl();

  template PINOCCHIO_EXPLICIT_INSTANTIATION_DEFINITION_DLLAPI
  DataTpl<context::Scalar, context::Options, JointCollectionDefaultTpl>::DataTpl(const Model &, const Allocation);

} // namespace pinocchio
