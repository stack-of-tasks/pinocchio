//
// Copyright (c) 2015-2017 CNRS
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

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/deprecated.hh"
#include "pinocchio/multibody/geometry.hpp"

#include <urdf_model/model.h>

#include <string>
#include <exception>
#ifdef URDFDOM_USE_STD_SHARED_PTR
  #include <memory>
  #define URDF_SHARED_PTR(type) std::shared_ptr<type>
  #define URDF_WEAK_PTR(type) std::weak_ptr<type>
#else
  #include <boost/shared_ptr.hpp>
  #define URDF_SHARED_PTR(type) boost::shared_ptr<type>
  #define URDF_WEAK_PTR(type) boost::weak_ptr<type>
#endif

#ifndef URDFDOM_TYPEDEF_SHARED_PTR

#define URDF_TYPEDEF_CLASS_POINTER(Class) \
typedef URDF_SHARED_PTR(Class) Class##SharedPtr; \
typedef URDF_SHARED_PTR(const Class) Class##ConstSharedPtr; \
typedef URDF_WEAK_PTR(Class) Class##WeakPtr

namespace urdf
{
  URDF_TYPEDEF_CLASS_POINTER(Box);
  URDF_TYPEDEF_CLASS_POINTER(Collision);
  URDF_TYPEDEF_CLASS_POINTER(Cylinder);
  URDF_TYPEDEF_CLASS_POINTER(Geometry);
  URDF_TYPEDEF_CLASS_POINTER(Inertial);
  URDF_TYPEDEF_CLASS_POINTER(Joint);
  URDF_TYPEDEF_CLASS_POINTER(Link);
  URDF_TYPEDEF_CLASS_POINTER(Material);
  URDF_TYPEDEF_CLASS_POINTER(Mesh);
  URDF_TYPEDEF_CLASS_POINTER(ModelInterface);
  URDF_TYPEDEF_CLASS_POINTER(Sphere);
  URDF_TYPEDEF_CLASS_POINTER(Visual);
  
  template<class T, class U>
  URDF_SHARED_PTR(T) const_pointer_cast(URDF_SHARED_PTR(U) const & r)
  {
#ifdef URDFDOM_USE_STD_SHARED_PTR
    return std::const_pointer_cast<T>(r);
#else
    return boost::const_pointer_cast<T>(r);
#endif
  }
  
  template<class T, class U>
  URDF_SHARED_PTR(T) dynamic_pointer_cast(URDF_SHARED_PTR(U) const & r)
  {
#ifdef URDFDOM_USE_STD_SHARED_PTR
    return std::dynamic_pointer_cast<T>(r);
#else
    return boost::dynamic_pointer_cast<T>(r);
#endif
  }
  
  template<class T, class U>
  URDF_SHARED_PTR(T) static_pointer_cast(URDF_SHARED_PTR(U) const & r)
  {
#ifdef URDFDOM_USE_STD_SHARED_PTR
    return std:static_pointer_cast<T>(r);
#else
    return boost::static_pointer_cast<T>(r);
#endif
  }
}

#undef URDF_TYPEDEF_CLASS_POINTER

#endif

namespace se3
{
  namespace urdf
  {

    /// 
    /// \brief Build the model from a URDF file with a particular joint as root of the model tree inside
    /// the model given as reference argument.
    ///
    /// \param[in] filemane The URDF complete file path.
    /// \param[in] rootJoint The joint at the root of the model tree.
    /// \param[in] verbose Print parsing info.
    /// \param[out] model Reference model where to put the parsed information.
    /// \return Return the reference on argument model for convenience.
    /// 
    Model& buildModel (const std::string & filename,
                       const JointModelVariant & rootJoint,
                       Model & model, 
                       const bool verbose = false) throw (std::invalid_argument);


    ///
    /// \brief Build the model from a URDF file with a fixed joint as root of the model tree.
    ///
    /// \param[in] filemane The URDF complete file path.
    /// \param[in] verbose Print parsing info.
    /// \param[out] model Reference model where to put the parsed information.
    /// \return Return the reference on argument model for convenience.
    ///
    Model & buildModel (const std::string & filename,
                        Model & model,
                        const bool verbose = false) throw (std::invalid_argument);


    /**
     * @brief      Build The GeometryModel from a URDF file. Search for meshes
     *             in the directories specified by the user first and then in
     *             the environment variable ROS_PACKAGE_PATH
     *
     * @param[in]  model         The model of the robot, built with
     *                           urdf::buildModel
     * @param[in]  filename      The URDF complete (absolute) file path
     * @param[in]  packageDirs  A vector containing the different directories
     *                           where to search for models and meshes, typically 
     *                           obtained from calling se3::rosPaths()
     *
     * @param[in]   type         The type of objects that must be loaded (must be VISUAL or COLLISION)
     * @param[out]  geomModel    Reference where to put the parsed information.
     *
     * @return      Returns the reference on geom model for convenience.
     *
     * \warning     If hpp-fcl has not been found during compilation, COLLISION types can not be loaded
     *
     */
    GeometryModel& buildGeom(const Model & model,
                             const std::string & filename,
                             const GeometryType type,
                             GeometryModel & geomModel,
                             const std::vector<std::string> & packageDirs = std::vector<std::string> ())
    throw (std::invalid_argument);


  } // namespace urdf
} // namespace se3

#endif // ifndef __se3_parsers_urdf_hpp__
