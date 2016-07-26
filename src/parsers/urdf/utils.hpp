//
// Copyright (c) 2015-2016 CNRS
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

#ifndef __se3_parsers_urdf_utils_hpp__
#define __se3_parsers_urdf_utils_hpp__

#include <urdf_model/model.h>

#include "pinocchio/spatial/se3.hpp"
#include "pinocchio/spatial/inertia.hpp"

namespace se3
{
  namespace urdf
  {

    ///
    /// \brief Convert URDF Inertial quantity to Spatial Inertia.
    ///
    /// \param[in] Y The input URDF Inertia.
    ///
    /// \return The converted Spatial Inertia se3::Inertia.
    ///
    inline Inertia convertFromUrdf (const ::urdf::Inertial & Y)
    {
      const ::urdf::Vector3 & p = Y.origin.position;
      const ::urdf::Rotation & q = Y.origin.rotation;
      
      const Eigen::Vector3d com(p.x,p.y,p.z);
      const Eigen::Matrix3d & R = Eigen::Quaterniond(q.w,q.x,q.y,q.z).matrix();
      
      Eigen::Matrix3d I; I <<
      Y.ixx,Y.ixy,Y.ixz,
      Y.ixy,Y.iyy,Y.iyz,
      Y.ixz,Y.iyz,Y.izz;
      return Inertia(Y.mass,com,R*I*R.transpose());
    }
    
    ///
    /// \brief Convert URDF Pose quantity to SE3.
    ///
    /// \param[in] M The input URDF Pose.
    ///
    /// \return The converted pose/transform se3::SE3.
    ///
    inline SE3 convertFromUrdf (const ::urdf::Pose & M)
    {
      const ::urdf::Vector3 & p = M.position;
      const ::urdf::Rotation & q = M.rotation;
      return SE3( Eigen::Quaterniond(q.w,q.x,q.y,q.z).matrix(), Eigen::Vector3d(p.x,p.y,p.z));
    }
    
    ///
    /// \brief The four possible cartesian types of an 3D axis.
    ///
    enum CartesianAxis { AXIS_X=0, AXIS_Y=1, AXIS_Z=2, AXIS_UNALIGNED };
    
    ///
    /// \brief Extract the cartesian property of a particular 3D axis.
    ///
    /// \param[in] axis The input URDF axis.
    ///
    /// \return The property of the particular axis se3::urdf::CartesianAxis.
    ///
    inline CartesianAxis extractCartesianAxis (const ::urdf::Vector3 & axis)
    {
      if( (axis.x==1.0)&&(axis.y==0.0)&&(axis.z==0.0) )
        return AXIS_X;
      else if( (axis.x==0.0)&&(axis.y==1.0)&&(axis.z==0.0) )
        return AXIS_Y;
      else if( (axis.x==0.0)&&(axis.y==0.0)&&(axis.z==1.0) )
        return AXIS_Z;
      else
        return AXIS_UNALIGNED;
    }

  } //urdf
} // se3
#endif // __se3_parsers_urdf_utils_hpp__
