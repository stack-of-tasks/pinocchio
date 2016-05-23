//
// Copyright (c) 2015 - 2016 CNRS
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

#ifndef __se3_joint_variant_hpp__
#define __se3_joint_variant_hpp__

#include "pinocchio/multibody/joint.hpp"
#include <Eigen/StdVector>
#include <boost/variant.hpp>

namespace se3
{
  typedef boost::variant< JointModelRX, JointModelRY, JointModelRZ, JointModelRevoluteUnaligned, JointModelSpherical, JointModelSphericalZYX, JointModelPX, JointModelPY, JointModelPZ, JointModelPrismaticUnaligned, JointModelFreeFlyer, JointModelPlanar, JointModelTranslation, JointModelDense<-1,-1> > JointModelVariant;
  typedef boost::variant< JointDataRX, JointDataRY, JointDataRZ, JointDataRevoluteUnaligned, JointDataSpherical, JointDataSphericalZYX, JointDataPX, JointDataPY, JointDataPZ, JointDataPrismaticUnaligned, JointDataFreeFlyer, JointDataPlanar, JointDataTranslation, JointDataDense<-1,-1> > JointDataVariant;

  typedef std::vector<JointModelVariant> JointModelVector;
  typedef std::vector<JointDataVariant> JointDataVector;


} // namespace se3

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(se3::JointModelVariant)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(se3::JointDataVariant)

#endif // ifndef __se3_joint_variant_hpp__
