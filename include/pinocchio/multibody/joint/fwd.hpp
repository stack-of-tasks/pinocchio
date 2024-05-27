//
// Copyright (c) 2016-2019 CNRS INRIA
//

#ifndef __pinocchio_multibody_joint_fwd_hpp__
#define __pinocchio_multibody_joint_fwd_hpp__

#include "pinocchio/fwd.hpp"

namespace pinocchio
{

  /// \internal
  enum
  {
    MAX_JOINT_NV = 6
  };
  /// \endinternal

  /**
   * \addtogroup pinocchio_joint
   * @{
   */

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
  struct JointModelTpl;
  typedef JointModelTpl<context::Scalar> JointModel;

  template<
    typename Scalar,
    int Options = context::Options,
    template<typename S, int O> class JointCollectionTpl = JointCollectionDefaultTpl>
  struct JointDataTpl;
  typedef JointDataTpl<context::Scalar> JointData;

  /**
   * @}
   */
  // end of group joint
} // namespace pinocchio

#include "pinocchio/multibody/fwd.hpp"

#endif // ifndef __pinocchio_multibody_joint_fwd_hpp__
