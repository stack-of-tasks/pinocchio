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

#ifndef __se3_frame_hpp__
#define __se3_frame_hpp__

#include "pinocchio/spatial/fwd.hpp"
#include "pinocchio/spatial/se3.hpp"
#include "pinocchio/multibody/fwd.hpp"
#include "pinocchio/tools/string-generator.hpp"

#include <Eigen/StdVector>
#include <iostream>

namespace se3
{

  ///
  /// \brief A Plucker coordinate frame attached to a parent joint inside a kinematic tree
  ///
  struct Frame
  {
    typedef se3::JointIndex JointIndex;
      
    Frame() : name(random(8)), parent_id(), framePlacement() {} // needed by EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION
    
    ///
    /// \brief Default constructor of a Frame
    ///
    /// \param[in] name Name of the frame.
    /// \param[in] parent_id Index of the parent joint in the kinematic tree.
    /// \param[in] frame_placement Placement of the frame wrt the parent joint frame.
    ///
    Frame(const std::string & name, const JointIndex parent_id, const SE3 & frame_placement):
    name(name)
    , parent_id(parent_id)
    , framePlacement(frame_placement)
    {}
    
    ///
    /// \brief Compare the current Frame with another frame. Return true if all properties match.
    ///
    /// \param[in] other The frame to which the current frame is compared.
    ///
    bool operator == (const Frame & other) const
    {
      return name == other.name && parent_id == other.parent_id
      && framePlacement == other.framePlacement ;
    }
    
    /// \brief Name of the frame.
    std::string name;
    
    /// \brief Index of the parent joint.
    JointIndex parent_id;
    
    /// \brief Placement of the frame wrt the parent joint.
    SE3 framePlacement;
    
  }; // struct Frame

} // namespace se3

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(se3::Frame)

#endif // ifndef __se3_frame_hpp__
