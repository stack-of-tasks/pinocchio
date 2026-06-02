//
// Copyright (c) 2016-2019 CNRS INRIA
//

#pragma once

// IWYU pragma: private, include "pinocchio/multibody/joint/fwd.hpp"

#ifdef PINOCCHIO_LSP
  #undef PINOCCHIO_LSP
  #include "pinocchio/multibody/joint/fwd.hpp"
#endif // PINOCCHIO_LSP

namespace pinocchio
{
  /// \internal
  inline constexpr int MAX_JOINT_NV = 6;
  /// \endinternal

  ///
  /// \brief Generate the label (X, Y or Z) of the axis relative to its index.
  ///
  /// \tparam axis Index of the axis (either 0 for X, 1 for Y and Z for 2).
  ///
  /// \returns a char containing the label of the axis.
  ///
  template<int axis>
  inline char axisLabel();

  template<>
  inline char axisLabel<0>()
  {
    return 'X';
  }
  template<>
  inline char axisLabel<1>()
  {
    return 'Y';
  }
  template<>
  inline char axisLabel<2>()
  {
    return 'Z';
  }

  struct JointModelVoid
  {
  };
  struct JointDataVoid
  {
  };

  template<typename Scalar, int Options, int axis>
  struct JointModelRevoluteTpl;
  template<typename Scalar, int Options, int axis>
  struct JointDataRevoluteTpl;

  template<typename Scalar, int Options = context::Options>
  struct JointModelRevoluteUnalignedTpl;
  typedef JointModelRevoluteUnalignedTpl<context::Scalar> JointModelRevoluteUnaligned;

  template<typename Scalar, int Options = context::Options>
  struct JointDataRevoluteUnalignedTpl;
  typedef JointDataRevoluteUnalignedTpl<context::Scalar> JointDataRevoluteUnaligned;

  template<typename Scalar, int Options = context::Options>
  struct JointModelRevoluteUnboundedUnalignedTpl;
  typedef JointModelRevoluteUnboundedUnalignedTpl<context::Scalar>
    JointModelRevoluteUnboundedUnaligned;
  template<typename Scalar, int Options = context::Options>
  struct JointDataRevoluteUnboundedUnalignedTpl;
  typedef JointDataRevoluteUnboundedUnalignedTpl<context::Scalar>
    JointDataRevoluteUnboundedUnaligned;

  template<typename Scalar, int Options, int axis>
  struct JointModelRevoluteUnboundedTpl;
  template<typename Scalar, int Options, int axis>
  struct JointDataRevoluteUnboundedTpl;

  template<typename Scalar, int Options, int axis>
  struct JointModelHelicalTpl;
  template<typename Scalar, int Options, int axis>
  struct JointDataHelicalTpl;

  template<typename Scalar, int Options = context::Options>
  struct JointModelHelicalUnalignedTpl;
  typedef JointModelHelicalUnalignedTpl<context::Scalar> JointModelHelicalUnaligned;

  template<typename Scalar, int Options = context::Options>
  struct JointDataHelicalUnalignedTpl;
  typedef JointDataHelicalUnalignedTpl<context::Scalar> JointDataHelicalUnaligned;

  template<typename Scalar, int Options = context::Options>
  struct JointModelSphericalTpl;
  typedef JointModelSphericalTpl<context::Scalar> JointModelSpherical;

  template<typename Scalar, int Options = context::Options>
  struct JointDataSphericalTpl;
  typedef JointDataSphericalTpl<context::Scalar> JointDataSpherical;

  template<typename Scalar, int Options = context::Options>
  struct JointModelSphericalZYXTpl;
  typedef JointModelSphericalZYXTpl<context::Scalar> JointModelSphericalZYX;

  template<typename Scalar, int Options = context::Options>
  struct JointDataSphericalZYXTpl;
  typedef JointDataSphericalZYXTpl<context::Scalar> JointDataSphericalZYX;

  template<typename Scalar, int Options = context::Options>
  struct JointModelEllipsoidTpl;
  typedef JointModelEllipsoidTpl<context::Scalar> JointModelEllipsoid;

  template<typename Scalar, int Options = context::Options>
  struct JointDataEllipsoidTpl;
  typedef JointDataEllipsoidTpl<context::Scalar> JointDataEllipsoid;

  template<typename Scalar, int Options, int axis>
  struct JointModelPrismaticTpl;
  template<typename Scalar, int Options, int axis>
  struct JointDataPrismaticTpl;

  template<typename Scalar, int Options = context::Options>
  struct JointModelPrismaticUnalignedTpl;
  typedef JointModelPrismaticUnalignedTpl<context::Scalar> JointModelPrismaticUnaligned;

  template<typename Scalar, int Options = context::Options>
  struct JointDataPrismaticUnalignedTpl;
  typedef JointDataPrismaticUnalignedTpl<context::Scalar> JointDataPrismaticUnaligned;

  template<typename Scalar, int Options = context::Options>
  struct JointModelUniversalTpl;
  typedef JointModelUniversalTpl<context::Scalar> JointModelUniversal;

  template<typename Scalar, int Options = context::Options>
  struct JointDataUniversalTpl;
  typedef JointDataUniversalTpl<context::Scalar> JointDataUniversal;

  template<typename Scalar, int Options = context::Options>
  struct JointModelFreeFlyerTpl;
  typedef JointModelFreeFlyerTpl<context::Scalar> JointModelFreeFlyer;

  template<typename Scalar, int Options = context::Options>
  struct JointDataFreeFlyerTpl;
  typedef JointDataFreeFlyerTpl<context::Scalar> JointDataFreeFlyer;

  template<typename Scalar, int Options = context::Options>
  struct JointModelPlanarTpl;
  typedef JointModelPlanarTpl<context::Scalar> JointModelPlanar;

  template<typename Scalar, int Options = context::Options>
  struct JointDataPlanarTpl;
  typedef JointDataPlanarTpl<context::Scalar> JointDataPlanar;

  template<typename Scalar, int Options = context::Options>
  struct JointModelTranslationTpl;
  typedef JointModelTranslationTpl<context::Scalar> JointModelTranslation;

  template<typename Scalar, int Options = context::Options>
  struct JointDataTranslationTpl;
  typedef JointDataTranslationTpl<context::Scalar> JointDataTranslation;

  template<typename Scalar, int Options = context::Options>
  struct JointModelSplineTpl;
  typedef JointModelSplineTpl<context::Scalar> JointModelSpline;

  template<typename Scalar, int Options = context::Options>
  struct JointDataSplineTpl;
  typedef JointDataSplineTpl<context::Scalar> JointDataSpline;

  template<typename Scalar, int Options = context::Options>
  struct JointCollectionDefaultTpl;
  typedef JointCollectionDefaultTpl<context::Scalar> JointCollectionDefault;

  template<
    typename Scalar,
    int Options = context::Options,
    template<typename S, int O> class JointCollectionTpl = JointCollectionDefaultTpl>
  struct JointModelCompositeTpl;
  typedef JointModelCompositeTpl<context::Scalar> JointModelComposite;

  template<
    typename Scalar,
    int Options = context::Options,
    template<typename S, int O> class JointCollectionTpl = JointCollectionDefaultTpl>
  struct JointDataCompositeTpl;
  typedef JointDataCompositeTpl<context::Scalar> JointDataComposite;

  template<
    typename Scalar,
    int Options = context::Options,
    template<typename S, int O> class JointCollectionTpl = JointCollectionDefaultTpl>
  struct JointModelMimicTpl;
  typedef JointModelMimicTpl<context::Scalar> JointModelMimic;

  template<
    typename Scalar,
    int Options = context::Options,
    template<typename S, int O> class JointCollectionTpl = JointCollectionDefaultTpl>
  struct JointDataMimicTpl;
  typedef JointDataMimicTpl<context::Scalar> JointDataMimic;

  template<
    typename Scalar,
    int Options = context::Options,
    template<typename S, int O> class JointCollectionTpl = JointCollectionDefaultTpl>
  struct JointModelTpl;
  typedef JointModelTpl<context::Scalar> JointModel;

  template<
    typename Scalar,
    int Options = context::Options,
    template<typename S, int O> class JointCollectionTpl = JointCollectionDefaultTpl>
  struct JointDataTpl;
  typedef JointDataTpl<context::Scalar> JointData;

  struct SpanIndexes;
  template<typename Scalar, int Options = context::Options>
  struct FindSpan;

} // namespace pinocchio
