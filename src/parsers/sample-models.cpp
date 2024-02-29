//
// Copyright (c) 2022 INRIA
//

#include "pinocchio/spatial/fwd.hpp"

#ifndef PINOCCHIO_SKIP_PARSERS_SAMPLE_MODELS

#include "pinocchio/parsers/sample-models.hpp"

namespace pinocchio {
  namespace buildModels {
    template void manipulator
      <context::Scalar, context::Options, JointCollectionDefaultTpl>
    (context::Model &);

    template void humanoid
      <context::Scalar, context::Options, JointCollectionDefaultTpl>
    (context::Model &, bool);

    template void humanoidRandom
      <context::Scalar, context::Options, JointCollectionDefaultTpl>
    (context::Model &, bool);
    
  } // namespace buildModels
} // namespace pinocchio 

#endif // PINOCCHIO_SKIP_PARSERS_SAMPLE_MODELS
