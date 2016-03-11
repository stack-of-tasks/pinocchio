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

#ifndef __se3_urdf_geom_hpp__
#define __se3_urdf_geom_hpp__

#include <iostream>
#include <exception>
#include "pinocchio/multibody/model.hpp"

#include <hpp/fcl/collision_object.h>
#include <hpp/fcl/collision.h>
#include <hpp/fcl/shape/geometric_shapes.h>

#include "pinocchio/multibody/parser/urdf.hpp"

namespace se3
{
  namespace urdf
  {


    /**
     * @brief      Get a CollisionObject from an urdf link, searching
     *             for it in specified package directories
     *
     * @param[in]  link          The input urdf link
     * @param[in]  package_dirs  A vector containing the different directories where to search for packages
     *
     * @return     The mesh converted as a fcl::CollisionObject
     */
    inline fcl::CollisionObject retrieveCollisionGeometry (const ::urdf::LinkConstPtr & link, const std::vector < std::string > & package_dirs);

    
    /**
     * @brief      Recursive procedure for reading the URDF tree, looking for geometries
     *             This function fill the geometric model whith geometry objects retrieved from the URDF tree
     * 
     * @param[in]  link            The current URDF link
     * @param      model           The model to which is the GeometryModel associated
     * @param      model_geom      The GeometryModel where the Collision Objects must be added
     * @param[in]  package_dirs    A vector containing the different directories where to search for packages
     * @param[in]  rootJointAdded  If a root joint was added at the begining of the urdf kinematic chain by user when constructing the Model
     */
    inline void parseTreeForGeom( ::urdf::LinkConstPtr link,
                                const Model & model,
                                GeometryModel & model_geom,
                                const std::vector<std::string> & package_dirs) throw (std::invalid_argument);



    /**
     * @brief      Build The GeometryModel from a URDF file. Search for meshes
     *             in the directories specified by the user first and then in
     *             the environment variable ROS_PACKAGE_PATH
     *
     * @param      model         The model of the robot, built with
     *                           urdf::buildModel
     * @param[in]  filename      The URDF complete (absolute) file path
     * @param[in]  package_dirs  A vector containing the different directories
     *                           where to search for packages
     * @param[in]  root_joint    If we added a root joint to the Model in
     *                           addition to the urdf kinematic chain
     *
     * @return     The GeometryModel associated to the urdf file and the Model
     *             given
     */
    inline GeometryModel buildGeom(const Model & model,
                                  const std::string & filename,
                                  const std::vector<std::string> & package_dirs = std::vector<std::string>());

  } // namespace urdf
  
} // namespace se3


/* --- Details -------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------- */
#include "pinocchio/multibody/parser/urdf-with-geometry.hxx"

#endif // ifndef __se3_urdf_geom_hpp__

