//
// Copyright (c) 2018 CNRS
//
// This file is part of Pinocchio
// Pinocchio is free software: you can redistribute it
// and/or modify it under the terljMj of the GNU Lesser General Public
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

#ifndef __se3_joint_collection_hpp__
#define __se3_joint_collection_hpp__

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
  
  template<typename _Scalar, int _Options>
  struct JointCollectionDefaultTpl
  {
    typedef _Scalar Scalar;
    enum { Options = _Options };
    
    // Joint Revolute
    typedef JointModelRevoluteTpl<Scalar,Options,0> JointModelRX;
    typedef JointModelRevoluteTpl<Scalar,Options,1> JointModelRY;
    typedef JointModelRevoluteTpl<Scalar,Options,2> JointModelRZ;
    
    // Joint Revolute Unaligned
    typedef JointModelRevoluteUnalignedTpl<Scalar,Options> JointModelRevoluteUnaligned;
    
    // Joint Revolute UBounded
    typedef JointModelRevoluteUnboundedTpl<Scalar,Options,0> JointModelRUBX;
    typedef JointModelRevoluteUnboundedTpl<Scalar,Options,1> JointModelRUBY;
    typedef JointModelRevoluteUnboundedTpl<Scalar,Options,2> JointModelRUBZ;
    
    // Joint Prismatic
    typedef JointModelPrismaticTpl<Scalar,Options,0> JointModelPX;
    typedef JointModelPrismaticTpl<Scalar,Options,1> JointModelPY;
    typedef JointModelPrismaticTpl<Scalar,Options,2> JointModelPZ;
    
    // Joint Prismatic Unaligned
    typedef JointModelPrismaticUnalignedTpl<Scalar,Options> JointModelPrismaticUnaligned;
    
    // Joint Spherical
    typedef JointModelSphericalTpl<Scalar,Options> JointModelSpherical;
    
    // Joint Spherical ZYX
    typedef JointModelSphericalZYXTpl<Scalar,Options> JointModelSphericalZYX;
    
    // Joint Translation
    typedef JointModelTranslationTpl<Scalar,Options> JointModelTranslation;
    
    // Joint FreeFlyer
    typedef JointModelFreeFlyerTpl<Scalar,Options> JointModelFreeFlyer;
    
    // Joint Planar
    typedef JointModelPlanarTpl<Scalar,Options> JointModelPlanar;
    
    // Joint Composite
    typedef JointModelCompositeTpl<JointCollectionDefaultTpl> JointModelComposite;
    
    typedef boost::variant<
    JointModelRX, JointModelRY, JointModelRZ
    ,JointModelRevoluteUnaligned, JointModelSpherical,
    JointModelSphericalZYX, JointModelPX, JointModelPY, JointModelPZ,
    JointModelPrismaticUnaligned, JointModelFreeFlyer, JointModelPlanar, JointModelTranslation,
    JointModelRUBX, JointModelRUBY, JointModelRUBZ,
    boost::recursive_wrapper<JointModelComposite>
    > JointModelVariant;
   
    
    // Joint Revolute
    typedef JointDataRevoluteTpl<Scalar,Options,0> JointDataRX;
    typedef JointDataRevoluteTpl<Scalar,Options,1> JointDataRY;
    typedef JointDataRevoluteTpl<Scalar,Options,2> JointDataRZ;
    
    // Joint Revolute Unaligned
    typedef JointDataRevoluteUnalignedTpl<Scalar,Options> JointDataRevoluteUnaligned;
    
    // Joint Revolute UBounded
    typedef JointDataRevoluteUnboundedTpl<Scalar,Options,0> JointDataRUBX;
    typedef JointDataRevoluteUnboundedTpl<Scalar,Options,1> JointDataRUBY;
    typedef JointDataRevoluteUnboundedTpl<Scalar,Options,2> JointDataRUBZ;
    
    // Joint Prismatic
    typedef JointDataPrismaticTpl<Scalar,Options,0> JointDataPX;
    typedef JointDataPrismaticTpl<Scalar,Options,1> JointDataPY;
    typedef JointDataPrismaticTpl<Scalar,Options,2> JointDataPZ;
    
    // Joint Prismatic Unaligned
    typedef JointDataPrismaticUnalignedTpl<Scalar,Options> JointDataPrismaticUnaligned;
    
    // Joint Spherical
    typedef JointDataSphericalTpl<Scalar,Options> JointDataSpherical;
    
    // Joint Spherical ZYX
    typedef JointDataSphericalZYXTpl<Scalar,Options> JointDataSphericalZYX;
    
    // Joint Translation
    typedef JointDataTranslationTpl<Scalar,Options> JointDataTranslation;
    
    // Joint FreeFlyer
    typedef JointDataFreeFlyerTpl<Scalar,Options> JointDataFreeFlyer;
    
    // Joint Planar
    typedef JointDataPlanarTpl<Scalar,Options> JointDataPlanar;
    
    // Joint Composite
    typedef JointDataCompositeTpl<JointCollectionDefaultTpl> JointDataComposite;
    
    typedef boost::variant<
    JointDataRX, JointDataRY, JointDataRZ
    ,JointDataRevoluteUnaligned, JointDataSpherical,
    JointDataSphericalZYX, JointDataPX, JointDataPY, JointDataPZ,
    JointDataPrismaticUnaligned, JointDataFreeFlyer, JointDataPlanar, JointDataTranslation,
    JointDataRUBX, JointDataRUBY, JointDataRUBZ,
    boost::recursive_wrapper<JointDataComposite>
    > JointDataVariant;

  };
  
  typedef JointCollectionDefault::JointModelVariant JointModelVariant;
  typedef JointCollectionDefault::JointDataVariant JointDataVariant;
  
} // namespace se3

#endif // ifndef __se3_joint_collection_hpp__
