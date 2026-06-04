//
// Copyright (c) 2022-2024 INRIA
//

#pragma once

// IWYU pragma: private, include "pinocchio/constraints/fwd.hpp"

#ifdef PINOCCHIO_LSP
  #undef PINOCCHIO_LSP
  #include "pinocchio/constraints/fwd.hpp"
#endif // PINOCCHIO_LSP

namespace pinocchio
{
  template<typename Scalar>
  struct BaumgarteCorrectorParametersTpl;
  typedef BaumgarteCorrectorParametersTpl<context::Scalar> BaumgarteCorrectorParameters;

  /// \brief Non-constant constraints possess a current selection.
  /// For example, joints too far from their limits can be removed from the current
  /// selection, leaving only joints about to hit their limits to be considered by
  /// constraint algorithms.
  /// This selection is expected to be done manually and externally by the user.
  /// Algorithms only work with the current selection of a constraint.
  /// The user can still access the "maximal" selection, for example to manage physical
  /// parameters of the overall constraint.
  enum struct ConstraintSelectionType
  {
    CURRENT,
    MAXIMAL
  };

  template<ConstraintSelectionType val>
  struct ConstraintSelectionTag
  {
  };

  using CurrentSelection = ConstraintSelectionTag<ConstraintSelectionType::CURRENT>;
  using MaximalSelection = ConstraintSelectionTag<ConstraintSelectionType::MAXIMAL>;

  // Constraints
  template<typename Scalar, int Options = 0>
  struct RigidConstraintModelTpl;
  template<typename Scalar, int Options = 0>
  struct RigidConstraintDataTpl;

  template<typename Scalar, int Options = 0>
  struct JointFrictionConstraintModelTpl;
  typedef JointFrictionConstraintModelTpl<context::Scalar> JointFrictionConstraintModel;

  template<typename Scalar, int Options = 0>
  struct JointFrictionConstraintDataTpl;
  typedef JointFrictionConstraintDataTpl<context::Scalar> JointFrictionConstraintData;

  template<typename Scalar, int Options = 0>
  struct JointLimitConstraintModelTpl;
  typedef JointLimitConstraintModelTpl<context::Scalar> JointLimitConstraintModel;

  template<typename Scalar, int Options = 0>
  struct JointLimitConstraintDataTpl;
  typedef JointLimitConstraintDataTpl<context::Scalar> JointLimitConstraintData;

  template<typename Scalar, int Options = 0>
  struct PointAnchorConstraintModelTpl;
  typedef PointAnchorConstraintModelTpl<context::Scalar> PointAnchorConstraintModel;
  template<typename Scalar, int Options = 0>
  struct PointAnchorConstraintDataTpl;
  typedef PointAnchorConstraintDataTpl<context::Scalar> PointAnchorConstraintData;

  template<typename Scalar, int Options = 0>
  struct PointContactConstraintModelTpl;
  typedef PointContactConstraintModelTpl<context::Scalar> PointContactConstraintModel;
  template<typename Scalar, int Options = 0>
  struct PointContactConstraintDataTpl;
  typedef PointContactConstraintDataTpl<context::Scalar> PointContactConstraintData;

  template<typename Scalar, int Options = 0>
  struct FrameAnchorConstraintModelTpl;
  typedef FrameAnchorConstraintModelTpl<context::Scalar> FrameAnchorConstraintModel;
  template<typename Scalar, int Options = 0>
  struct FrameAnchorConstraintDataTpl;
  typedef FrameAnchorConstraintDataTpl<context::Scalar> FrameAnchorConstraintData;

  template<typename Scalar, int Options = 0>
  struct ConstraintCollectionDefaultTpl;

  typedef ConstraintCollectionDefaultTpl<context::Scalar, context::Options>
    ConstraintCollectionDefault;

  template<
    typename Scalar,
    int _Options,
    template<typename S, int O> class ConstraintCollectionTpl = ConstraintCollectionDefaultTpl>
  struct ConstraintModelTpl;
  typedef ConstraintModelTpl<context::Scalar, context::Options, ConstraintCollectionDefaultTpl>
    ConstraintModel;

  template<
    typename Scalar,
    int _Options,
    template<typename S, int O> class ConstraintCollectionTpl = ConstraintCollectionDefaultTpl>
  struct ConstraintDataTpl;
  typedef ConstraintDataTpl<context::Scalar, context::Options, ConstraintCollectionDefaultTpl>
    ConstraintData;

  // Sets
  template<typename Scalar, int Options = 0>
  struct BoxSetTpl;
  typedef BoxSetTpl<context::Scalar> BoxSet;

  // Cone sets
  template<typename Scalar, int Options = 0>
  struct FullSpaceConeTpl;
  typedef FullSpaceConeTpl<context::Scalar> FullSpaceCone;

  template<typename Scalar, int Options = 0>
  struct ZeroConeTpl;
  typedef ZeroConeTpl<context::Scalar> ZeroCone;

  template<typename Scalar>
  struct CoulombFrictionConeTpl;
  typedef CoulombFrictionConeTpl<context::Scalar> CoulombFrictionCone;

  template<typename Scalar>
  struct DualCoulombFrictionConeTpl;
  typedef DualCoulombFrictionConeTpl<context::Scalar> DualCoulombFrictionCone;

  template<typename Scalar>
  struct NonNegativeOrthantConeTpl;
  typedef NonNegativeOrthantConeTpl<context::Scalar> NonNegativeOrthantCone;

} // namespace pinocchio
