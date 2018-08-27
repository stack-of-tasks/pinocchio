//
// Copyright (c) 2016 CNRS
//
// This file is part of Pinocchio
// Pinocchio is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// Pinocchio is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// General Lesser Public License for more details. You should have
// received a copy of the GNU Lesser General Public License along with
// Pinocchio If not, see
// <http://www.gnu.org/licenses/>.

#ifndef __se3_joint_fwd_hpp__
#define __se3_joint_fwd_hpp__

namespace se3
{
  enum { MAX_JOINT_NV = 6 };

  template<typename Scalar, int Options, int axis> struct JointModelRevoluteTpl;
  template<typename Scalar, int Options, int axis> struct JointDataRevoluteTpl;

  template<typename Scalar, int Options = 0> struct JointModelRevoluteUnalignedTpl;
  typedef JointModelRevoluteUnalignedTpl<double> JointModelRevoluteUnaligned;
  
  template<typename Scalar, int Options = 0> struct JointDataRevoluteUnalignedTpl;
  typedef JointDataRevoluteUnalignedTpl<double> JointDataRevoluteUnaligned;

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

  template<typename Scalar, int Options, int axis> struct JointModelPrismatic;
  template<typename Scalar, int Options, int axis> struct JointDataPrismatic;

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

  struct JointModelComposite;
  struct JointDataComposite;
  
  struct JointModel;
  struct JointData;
  
}

#endif // ifndef __se3_joint_fwd_hpp__
