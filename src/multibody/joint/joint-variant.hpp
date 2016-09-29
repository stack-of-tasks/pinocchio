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

#include "pinocchio/multibody/joint/fwd.hpp"
#include "pinocchio/multibody/joint/joint-free-flyer.hpp"
#include "pinocchio/multibody/joint/joint-planar.hpp"
#include "pinocchio/multibody/joint/joint-prismatic.hpp"
#include "pinocchio/multibody/joint/joint-prismatic-unaligned.hpp"
#include "pinocchio/multibody/joint/joint-revolute-unaligned.hpp"
#include "pinocchio/multibody/joint/joint-revolute.hpp"
#include "pinocchio/multibody/joint/joint-revolute-unbounded.hpp"
#include "pinocchio/multibody/joint/joint-spherical-ZYX.hpp"
#include "pinocchio/multibody/joint/joint-spherical.hpp"
#include "pinocchio/multibody/joint/joint-translation.hpp"

#include <boost/variant.hpp>
#include <boost/variant/recursive_wrapper.hpp>

namespace se3
{
  
  // The JointModelComposite contains several JointModel (which are JointModelVariant). Hence there is a circular
  // dependency between JointModelComposite and JointModelVariant that can be resolved with the use of boost::recursive_variant
  // For more details, see http://www.boost.org/doc/libs/1_58_0/doc/html/variant/tutorial.html#variant.tutorial.recursive 
  //
  // The same applies for JointDataComposite
  typedef boost::variant< JointModelRX, JointModelRY, JointModelRZ, JointModelRevoluteUnaligned, JointModelSpherical,
                          JointModelSphericalZYX, JointModelPX, JointModelPY, JointModelPZ,
                          JointModelPrismaticUnaligned, JointModelFreeFlyer, JointModelPlanar, JointModelTranslation,
                          JointModelRUBX, JointModelRUBY, JointModelRUBZ,
                          boost::recursive_wrapper<JointModelComposite> >JointModelVariant;
  typedef boost::variant< JointDataRX, JointDataRY, JointDataRZ, JointDataRevoluteUnaligned, JointDataSpherical,
                          JointDataSphericalZYX, JointDataPX, JointDataPY, JointDataPZ, JointDataPrismaticUnaligned,
                          JointDataFreeFlyer, JointDataPlanar, JointDataTranslation,
                          JointDataRUBX, JointDataRUBY, JointDataRUBZ,
                          boost::recursive_wrapper<JointDataComposite> > JointDataVariant;

} // namespace se3

#endif // ifndef __se3_joint_variant_hpp__
