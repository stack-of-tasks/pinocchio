//
// Copyright (c) 2015-2016 CNRS
// Copyright (c) 2015 Wandercraft, 86 rue de Paris 91400 Orsay, France.
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

#ifndef __se3_urdf_hpp__
#define __se3_urdf_hpp__

#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>

#include <iostream>
#include <sstream>
#include <iomanip>
#include <boost/foreach.hpp>
#include "pinocchio/multibody/model.hpp"

#include <exception>
#include <limits>

namespace urdf
{
  typedef boost::shared_ptr<ModelInterface> ModelInterfacePtr;
  typedef boost::shared_ptr<const Joint> JointConstPtr;
  typedef boost::shared_ptr<const Link> LinkConstPtr;
  typedef boost::shared_ptr<Link> LinkPtr;
  typedef boost::shared_ptr<const Inertial> InertialConstPtr;
}

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
    inline Inertia convertFromUrdf (const ::urdf::Inertial & Y);

    ///
    /// \brief Convert URDF Pose quantity to SE3.
    ///
    /// \param[in] M The input URDF Pose.
    ///
    /// \return The converted pose/transform se3::SE3.
    ///
    inline SE3 convertFromUrdf (const ::urdf::Pose & M);

    ///
    /// \brief The four possible cartesian types of an 3D axis.
    ///
    enum AxisCartesian { AXIS_X, AXIS_Y, AXIS_Z, AXIS_UNALIGNED };

   
    ///
    /// \brief Extract the cartesian property of a particular 3D axis.
    ///
    /// \param[in] axis The input URDF axis.
    ///
    /// \return The property of the particular axis se3::urdf::AxisCartesian.
    ///
    inline AxisCartesian extractCartesianAxis (const ::urdf::Vector3 & axis);

    ///
    /// \brief Recursive procedure for reading the URDF tree.
    ///        The function returns an exception as soon as a necessary Inertia or Joint information are missing.
    ///
    /// \param[in] link The current URDF link.
    /// \param[in] model The model where the link must be added.
    /// \param[in] placementOffset The relative placement of the link relative to the closer non fixed joint in the tree.
    ///
    inline void parseTree (::urdf::LinkConstPtr link, 
                           Model & model,
                           const SE3 & placementOffset = SE3::Identity(),
                           bool verbose = false) throw (std::invalid_argument);


    ///
    /// \brief Parse a tree with a specific root joint linking the model to the environment.
    ///        The function returns an exception as soon as a necessary Inertia or Joint information are missing.
    ///
    /// \param[in] link The current URDF link.
    /// \param[in] model The model where the link must be added.
    /// \param[in] verbose Print parsing info.
    ///
    void parseRootTree (::urdf::LinkConstPtr link,
                        Model & model,
                        const bool verbose = false) throw (std::invalid_argument);
    
    
    ///
    /// \brief Parse a tree with a specific root joint linking the model to the environment.
    ///        The function returns an exception as soon as a necessary Inertia or Joint information are missing.
    ///
    /// \param[in] link The current URDF link.
    /// \param[in] model The model where the link must be added.
    /// \param[in] root_joint The specific root joint.
    /// \param[in] verbose Print parsing info.
    ///
    template <typename D>
    void parseRootTree (::urdf::LinkConstPtr link,
                        Model & model,
                        const JointModelBase<D> & root_joint,
                        const bool verbose = false) throw (std::invalid_argument);

    ///
    /// \brief Build the model from a URDF file with a particular joint as root of the model tree.
    ///
    /// \param[in] filemane The URDF complete file path.
    /// \param[in] root_joint The joint at the root of the model tree.
    /// \param[in] verbose Print parsing info.
    ///
    /// \return The se3::Model of the URDF file.
    ///
    template <typename D>
    Model buildModel (const std::string & filename,
                      const JointModelBase<D> & root_joint,
                      bool verbose = false) throw (std::invalid_argument);
          
    ///
    /// \brief Build the model from a URDF file with a fixed joint as root of the model tree.
    ///
    /// \param[in] filemane The URDF complete file path.
    /// \param[in] verbose Print parsing info.
    ///
    /// \return The se3::Model of the URDF file.
    ///
    inline Model buildModel (const std::string & filename,
                             const bool verbose = false) throw (std::invalid_argument);

  } // namespace urdf
} // namespace se3

/* --- Details -------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------- */
#include "pinocchio/multibody/parser/urdf.hxx"

#endif // ifndef __se3_urdf_hpp__
