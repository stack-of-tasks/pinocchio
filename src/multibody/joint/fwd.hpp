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

  template<int axis> struct JointModelRevolute;
  template<int axis> struct JointDataRevolute;

  struct JointModelRevoluteUnaligned;
  struct JointDataRevoluteUnaligned;

  template<int axis> struct JointModelRevoluteUnbounded;
  template<int axis> struct JointDataRevoluteUnbounded;

  struct JointModelSpherical;
  struct JointDataSpherical;

  struct JointModelSphericalZYX;
  struct JointDataSphericalZYX;

  template<int axis> struct JointModelPrismatic;
  template<int axis> struct JointDataPrismatic;

  struct JointModelPrismaticUnaligned;
  struct JointDataPrismaticUnaligned;

  struct JointModelFreeFlyer;
  struct JointDataFreeFlyer;

  struct JointModelPlanar;
  struct JointDataPlanar;

  struct JointModelTranslation;
  struct JointDataTranslation;

  struct JointModelComposite;
  struct JointDataComposite;
  
  struct JointModel;
  struct JointData;
  
}

#endif // ifndef __se3_joint_fwd_hpp__
