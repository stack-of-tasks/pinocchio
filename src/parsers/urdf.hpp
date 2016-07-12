//
// Copyright (c) 2016 CNRS
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

#ifndef __se3_parsers_urdf_hpp__
#define __se3_parsers_urdf_hpp__

#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>

#include <iostream>
#include <sstream>
#include <iomanip>
#include <boost/foreach.hpp>
#include "pinocchio/multibody/model.hpp"

#include <exception>
#include <limits>

#ifdef WITH_HPP_FCL
#include <hpp/fcl/collision_object.h>
#include <hpp/fcl/collision.h>
#include <hpp/fcl/shape/geometric_shapes.h>
#include "pinocchio/multibody/geometry.hpp"
#endif

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

#ifdef WITH_HPP_FCL

    /**
     * @brief      Build The GeometryModel from a URDF file. Search for meshes
     *             in the directories specified by the user first and then in
     *             the environment variable ROS_PACKAGE_PATH
     *
     * @param[in]  model         The model of the robot, built with
     *                           urdf::buildModel
     * @param[in]  filename      The URDF complete (absolute) file path
     * @param[in]  package_dirs  A vector containing the different directories
     *                           where to search for models and meshes, typically 
     *                           obtained from calling se3::rosPaths()
     *
     *@param[in]   type          The type of objects that must be loaded ( can be VISUAL or COLLISION, or NONE)
     *
     * @return     The GeometryModel associated to the urdf file and the given Model.
     *
     */
    inline GeometryModel buildGeom(const Model & model,
                                   const std::string & filename,
                                   const std::vector<std::string> & package_dirs = std::vector<std::string> (),
                                   const GeometryType type = NONE) throw (std::invalid_argument);

#endif

  } // namespace urdf
} // namespace se3

/* --- Details -------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------- */
#include "pinocchio/parsers/urdf/model.hxx"
#ifdef WITH_HPP_FCL
#include "pinocchio/parsers/urdf/geometry.hxx"
#endif

#endif // ifndef __se3_parsers_urdf_hpp__
