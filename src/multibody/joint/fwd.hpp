//
// Copyright (c) 2016-2019 CNRS INRIA
//

#ifndef __pinocchio_multibody_joint_fwd_hpp__
#define __pinocchio_multibody_joint_fwd_hpp__

#include "pinocchio/fwd.hpp"

namespace pinocchio
{

  /// \internal
  enum { MAX_JOINT_NV = 6 };
  /// \endinternal

  /**
   * \addtogroup pinocchio_joint
   * @{
   */
  
  struct JointModelVoid {};
  struct JointDataVoid {};

  template<typename Scalar, int Options, int axis> struct JointModelRevoluteTpl;
  template<typename Scalar, int Options, int axis> struct JointDataRevoluteTpl;

  template<typename Scalar, int Options = 0> struct JointModelRevoluteUnalignedTpl;
  typedef JointModelRevoluteUnalignedTpl<double> JointModelRevoluteUnaligned;
  
  template<typename Scalar, int Options = 0> struct JointDataRevoluteUnalignedTpl;
  typedef JointDataRevoluteUnalignedTpl<double> JointDataRevoluteUnaligned;

  template<typename Scalar, int Options = 0> struct JointModelRevoluteUnboundedUnalignedTpl;
  typedef JointModelRevoluteUnboundedUnalignedTpl<double> JointModelRevoluteUnboundedUnaligned;
  template<typename Scalar, int Options = 0> struct JointDataRevoluteUnboundedUnalignedTpl;
  typedef JointDataRevoluteUnboundedUnalignedTpl<double> JointDataRevoluteUnboundedUnaligned;
  
  template<typename Scalar, int Options, int axis> struct JointModelRevoluteUnboundedTpl;
  template<typename Scalar, int Options, int axis> struct JointDataRevoluteUnboundedTpl;

  template<typename Scalar, int Options = 0> struct JointModelSphericalTpl;
  typedef JointModelSphericalTpl<double> JointModelSpherical;
  
  template<typename Scalar, int Options = 0> struct JointDataSphericalTpl;
  typedef JointDataSphericalTpl<double> JointDataSpherical;

  template<typename Scalar, int Options = 0> struct JointModelSphericalZYXTpl;
  typedef JointModelSphericalZYXTpl<double> JointModelSphericalZYX;
  
  template<typename Scalar, int Options = 0> struct JointDataSphericalZYXTpl;
  typedef JointDataSphericalZYXTpl<double> JointDataSphericalZYX;

  template<typename Scalar, int Options, int axis> struct JointModelPrismaticTpl;
  template<typename Scalar, int Options, int axis> struct JointDataPrismaticTpl;

  template<typename Scalar, int Options = 0> struct JointModelPrismaticUnalignedTpl;
  typedef JointModelPrismaticUnalignedTpl<double> JointModelPrismaticUnaligned;

  template<typename Scalar, int Options = 0> struct JointDataPrismaticUnalignedTpl;
  typedef JointDataPrismaticUnalignedTpl<double> JointDataPrismaticUnaligned;

  template<typename Scalar, int Options = 0> struct JointModelFreeFlyerTpl;
  typedef JointModelFreeFlyerTpl<double> JointModelFreeFlyer;
  
  template<typename Scalar, int Options = 0> struct JointDataFreeFlyerTpl;
  typedef JointDataFreeFlyerTpl<double> JointDataFreeFlyer;

  template<typename Scalar, int Options = 0> struct JointModelPlanarTpl;
  typedef JointModelPlanarTpl<double> JointModelPlanar;
  
  template<typename Scalar, int Options = 0> struct JointDataPlanarTpl;
  typedef JointDataPlanarTpl<double> JointDataPlanar;

  template<typename Scalar, int Options = 0> struct JointModelTranslationTpl;
  typedef JointModelTranslationTpl<double> JointModelTranslation;
  
  template<typename Scalar, int Options = 0> struct JointDataTranslationTpl;
  typedef JointDataTranslationTpl<double> JointDataTranslation;

  template<typename Scalar, int Options = 0> struct JointCollectionDefaultTpl;
  typedef JointCollectionDefaultTpl<double> JointCollectionDefault;
  
  template<typename Scalar, int Options = 0, template<typename S, int O> class JointCollectionTpl = JointCollectionDefaultTpl>
  struct JointModelCompositeTpl;
  typedef JointModelCompositeTpl<double> JointModelComposite;
  
  template<typename Scalar, int Options = 0, template<typename S, int O> class JointCollectionTpl = JointCollectionDefaultTpl>
  struct JointDataCompositeTpl;
  typedef JointDataCompositeTpl<double> JointDataComposite;
  
  template<typename Scalar, int Options = 0, template<typename S, int O> class JointCollectionTpl = JointCollectionDefaultTpl>
  struct JointModelTpl;
  typedef JointModelTpl<double> JointModel;
  
  template<typename Scalar, int Options = 0, template<typename S, int O> class JointCollectionTpl = JointCollectionDefaultTpl>
  struct JointDataTpl;
  typedef JointDataTpl<double> JointData;
  
  /**
   * @}
   */
  // end of group joint
}

#include "pinocchio/multibody/fwd.hpp"

#endif // ifndef __pinocchio_multibody_joint_fwd_hpp__
