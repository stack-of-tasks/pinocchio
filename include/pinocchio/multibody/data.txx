//
// Copyright (c) 2022 INRIA
//

#ifndef __pinocchio_multibody_data_txx__
#define __pinocchio_multibody_data_txx__

#include "pinocchio/multibody/data.hpp"

namespace pinocchio
{

  extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI
  DataTpl<context::Scalar, context::Options, JointCollectionDefaultTpl>::DataTpl();

  extern template PINOCCHIO_EXPLICIT_INSTANTIATION_DECLARATION_DLLAPI
  DataTpl<context::Scalar, context::Options, JointCollectionDefaultTpl>::DataTpl(
    const context::Model &);

} // namespace pinocchio

#endif // ifndef __pinocchio_multibody_data_txx__
