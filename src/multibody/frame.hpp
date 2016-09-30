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
#include "pinocchio/container/aligned-vector.hpp"

#include <Eigen/StdVector>
#include <string>

namespace se3
{
  ///
  /// \brief Enum on the possible type of frame
  ///
  enum FrameType
  {
    OP_FRAME     = 0x1 << 0, // operational frame type
    JOINT        = 0x1 << 1, // joint frame type
    FIXED_JOINT  = 0x1 << 2, // fixed joint frame type
    BODY         = 0x1 << 3, // body frame type
    SENSOR       = 0x1 << 4 // sensor frame type
  };
  
  ///
  /// \brief A Plucker coordinate frame attached to a parent joint inside a kinematic tree
  ///
  struct Frame
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef se3::JointIndex JointIndex;
    
    ///
    /// \brief Default constructor of a frame.
    ///
    Frame() : name(randomStringGenerator(8)), parent(), placement(), type() {} // needed by std::vector
    
    ///
    /// \brief Builds a frame defined by its name, its joint parent id, its placement and its type.
    ///
    /// \param[in] name Name of the frame.
    /// \param[in] parent Index of the parent joint in the kinematic tree.
    /// \param[in] previousFrame Index of the parent frame in the kinematic tree.
    /// \param[in] placement Placement of the frame wrt the parent joint frame.
    /// \param[in] type The type of the frame, see the enum FrameType
    ///
    Frame(const std::string & name, const JointIndex parent, const FrameIndex previousFrame, const SE3 & frame_placement, const FrameType type)
    : name(name)
    , parent(parent)
    , previousFrame(previousFrame)
    , placement(frame_placement)
    , type(type)
    {}
    
    ///
    /// \returns true if *this and other matches and have the same parent, name and type.
    ///
    /// \param[in] other The frame to which the current frame is compared.
    ///
    bool operator == (const Frame & other) const
    {
      return name == other.name && parent == other.parent
      && previousFrame == other.previousFrame
      && placement == other.placement
      && type == other.type ;
    }
    
    /// \brief Name of the frame.
    std::string name;
    
    /// \brief Index of the parent joint.
    JointIndex parent;
    
    /// \brief Index of the previous frame.
    FrameIndex previousFrame;
    
    /// \brief Placement of the frame wrt the parent joint.
    SE3 placement;

    /// \brief Type of the frame
    FrameType type;

  }; // struct Frame

  inline std::ostream & operator << (std::ostream& os, const Frame & f)
  {
    os << "Frame name:" << f.name  << "paired to (parent joint/ previous frame)" << "(" <<f.parent << "/" << f.previousFrame << ")"<< std::endl;
    os << "with relative placement wrt parent joint:\n" << f.placement << std::endl;
    return os;
  }

} // namespace se3

#endif // ifndef __se3_frame_hpp__
