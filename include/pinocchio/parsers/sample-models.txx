//
// Copyright (c) 2022 INRIA
//

#ifndef __pinocchio_parsers_sample_models_txx__
#define __pinocchio_parsers_sample_models_txx__

#include "pinocchio/parsers/sample-models.hpp"

namespace pinocchio {
  namespace buildModels {
    extern template PINOCCHIO_DLLAPI void manipulator
      <context::Scalar, context::Options, JointCollectionDefaultTpl>
    (context::Model &);

    extern template PINOCCHIO_DLLAPI void humanoid
      <context::Scalar, context::Options, JointCollectionDefaultTpl>
    (context::Model &, bool);

    extern template PINOCCHIO_DLLAPI void humanoidRandom
      <context::Scalar, context::Options, JointCollectionDefaultTpl>
    (context::Model &, bool);
  } // namespace buildModels
} // namespace pinocchio 

#endif // ifndef __pinocchio_parsers_sample_models_txx__
