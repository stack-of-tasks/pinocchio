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

#ifndef __se3_fcl_hxx__
#define __se3_fcl_hxx__


#include <iostream>


namespace se3
{

  inline std::ostream & operator << (std::ostream & os, const CollisionPair & X)
  {
    X.disp(os); return os;
  } 
  

#ifdef WITH_HPP_FCL  


  inline double DistanceResult::distance () const
  {
    return fcl_distance_result.min_distance;
  }

  inline Eigen::Vector3d DistanceResult::closestPointInner () const
  {
    return toVector3d(fcl_distance_result.nearest_points [0]);
  }

  inline Eigen::Vector3d DistanceResult::closestPointOuter () const
  {
    return toVector3d(fcl_distance_result.nearest_points [1]);
  }
    
  inline bool operator == (const fcl::CollisionObject & lhs, const fcl::CollisionObject & rhs)
  {
    return lhs.collisionGeometry() == rhs.collisionGeometry()
            && lhs.getAABB().min_ == rhs.getAABB().min_
            && lhs.getAABB().max_ == rhs.getAABB().max_;
  }
  
#endif // WITH_HPP_FCL

  
  inline bool operator==(const GeometryObject & lhs, const GeometryObject & rhs)
  {
    return ( lhs.name == rhs.name
            && lhs.parent == rhs.parent
            && lhs.collision_geometry == rhs.collision_geometry
            && lhs.placement == rhs.placement
            && lhs.mesh_path ==  rhs.mesh_path
            );
  }

  inline std::ostream & operator<< (std::ostream & os, const GeometryObject & geom_object)
  {
    os  << "Name: \t \n" << geom_object.name << "\n"
        << "Parent ID: \t \n" << geom_object.parent << "\n"
        // << "collision object: \t \n" << geom_object.collision_geometry << "\n"
        << "Position in parent frame: \t \n" << geom_object.placement << "\n"
        << "Absolute path to mesh file: \t \n" << geom_object.mesh_path << "\n"
        << std::endl;
    return os;
  }


} // namespace se3


#endif // ifndef __se3_fcl_hxx__
