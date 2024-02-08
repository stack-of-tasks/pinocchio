//
// Copyright (c) 2022 INRIA
//

#include "pinocchio/multibody/data.hpp"

namespace pinocchio {
  
  template DataTpl<context::Scalar, context::Options, JointCollectionDefaultTpl>::DataTpl();
  
  template DataTpl<context::Scalar, context::Options, JointCollectionDefaultTpl>::DataTpl(const context::Model &);

} // namespace pinocchio 
