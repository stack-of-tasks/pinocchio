//
// Copyright (c) 2024-2025 INRIA
//

#pragma once

// IWYU pragma: private, include "pinocchio/constraints.hpp"

#ifdef PINOCCHIO_LSP
  #undef PINOCCHIO_LSP
  #include "pinocchio/constraints.hpp"
#endif // PINOCCHIO_LSP

namespace pinocchio
{
  template<typename _Scalar, int _Options>
  struct ConstraintCollectionDefaultTpl
  {
    typedef _Scalar Scalar;
    static constexpr int Options = _Options;

    typedef PointAnchorConstraintModelTpl<Scalar, Options> PointAnchorConstraintModel;
    typedef PointAnchorConstraintDataTpl<Scalar, Options> PointAnchorConstraintData;

    typedef PointContactConstraintModelTpl<Scalar, Options> PointContactConstraintModel;
    typedef PointContactConstraintDataTpl<Scalar, Options> PointContactConstraintData;

    typedef ConstantLengthConstraintModelTpl<Scalar, Options> ConstantLengthConstraintModel;
    typedef ConstantLengthConstraintDataTpl<Scalar, Options> ConstantLengthConstraintData;

    typedef JointFrictionConstraintModelTpl<Scalar, Options> JointFrictionConstraintModel;
    typedef JointFrictionConstraintDataTpl<Scalar, Options> JointFrictionConstraintData;

    typedef JointLimitConstraintModelTpl<Scalar, Options> JointLimitConstraintModel;
    typedef JointLimitConstraintDataTpl<Scalar, Options> JointLimitConstraintData;

    typedef FrameAnchorConstraintModelTpl<Scalar, Options> FrameAnchorConstraintModel;
    typedef FrameAnchorConstraintDataTpl<Scalar, Options> FrameAnchorConstraintData;

    typedef boost::variant<
      BlankConstraintModel,
      PointAnchorConstraintModel,
      PointContactConstraintModel,
      ConstantLengthConstraintModel,
      JointFrictionConstraintModel,
      JointLimitConstraintModel,
      FrameAnchorConstraintModel>
      ConstraintModelVariant;

    typedef boost::variant<
      BlankConstraintData,
      PointAnchorConstraintData,
      PointContactConstraintData,
      ConstantLengthConstraintData,
      JointFrictionConstraintData,
      JointLimitConstraintData,
      FrameAnchorConstraintData>
      ConstraintDataVariant;
  }; // struct ConstraintCollectionDefaultTpl

  typedef ConstraintCollectionDefault::ConstraintModelVariant ConstraintModelVariant;
  typedef ConstraintCollectionDefault::ConstraintDataVariant ConstraintDataVariant;

} // namespace pinocchio
