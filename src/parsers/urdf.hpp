//
// Copyright (c) 2015-2018 CNRS
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
#include "pinocchio/multibody/geometry.hpp"
#include "pinocchio/parsers/urdf/types.hpp"

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
    template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
    ModelTpl<Scalar,Options,JointCollectionTpl> &
    buildModel(const std::string & filename,
               const typename ModelTpl<Scalar,Options,JointCollectionTpl>::JointModel & rootJoint,
               ModelTpl<Scalar,Options,JointCollectionTpl> & model,
               const bool verbose = false) throw (std::invalid_argument);


    ///
    /// \brief Build the model from a URDF file with a fixed joint as root of the model tree.
    ///
    /// \param[in] filemane The URDF complete file path.
    /// \param[in] verbose Print parsing info.
    /// \param[out] model Reference model where to put the parsed information.
    /// \return Return the reference on argument model for convenience.
    ///
    template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
    ModelTpl<Scalar,Options,JointCollectionTpl> &
    buildModel(const std::string & filename,
               ModelTpl<Scalar,Options,JointCollectionTpl> & model,
               const bool verbose = false) throw (std::invalid_argument);

    ///
    /// \brief Build the model from a URDF model with a particular joint as root of the model tree inside
    /// the model given as reference argument.
    ///
    /// \param[in] urdfTree the tree build from the URDF
    /// \param[in] rootJoint The joint at the root of the model tree.
    /// \param[in] verbose Print parsing info.
    /// \param[out] model Reference model where to put the parsed information.
    /// \return Return the reference on argument model for convenience.
    ///
    /// \note urdfTree can be build from ::urdf::parseURDF
    ///       or ::urdf::parseURDFFile
    template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
    ModelTpl<Scalar,Options,JointCollectionTpl> &
    buildModel(const ::urdf::ModelInterfaceSharedPtr & urdfTree,
               const typename ModelTpl<Scalar,Options,JointCollectionTpl>::JointModel & rootJoint,
               ModelTpl<Scalar,Options,JointCollectionTpl> & model,
               const bool verbose = false);

    ///
    /// \brief Build the model from a URDF model
    ///
    /// \param[in] urdfTree the tree build from the URDF
    /// \param[in] verbose Print parsing info.
    /// \param[out] model Reference model where to put the parsed information.
    /// \return Return the reference on argument model for convenience.
    ///
    /// \note urdfTree can be build from ::urdf::parseURDF
    ///       or ::urdf::parseURDFFile
    template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
    ModelTpl<Scalar,Options,JointCollectionTpl> &
    buildModel(const ::urdf::ModelInterfaceSharedPtr & urdfTree,
               ModelTpl<Scalar,Options,JointCollectionTpl> & model,
               const bool verbose = false);
    
    ///
    /// \brief Build the model from an XML stream with a particular joint as root of the model tree inside
    /// the model given as reference argument.
    ///
    /// \param[in] xmlStream stream containing the URDF model.
    /// \param[in] rootJoint The joint at the root of the model tree.
    /// \param[in] verbose Print parsing info.
    /// \param[out] model Reference model where to put the parsed information.
    /// \return Return the reference on argument model for convenience.
    ///
    /// \note urdfTree can be build from ::urdf::parseURDF
    ///       or ::urdf::parseURDFFile
    template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
    ModelTpl<Scalar,Options,JointCollectionTpl> &
    buildModelFromXML(const std::string & xmlStream,
                      const JointModelVariant & rootJoint,
                      ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                      const bool verbose = false)
    throw (std::invalid_argument);
    
    ///
    /// \brief Build the model from an XML stream
    ///
    /// \param[in] xmlStream stream containing the URDF model.
    /// \param[in] verbose Print parsing info.
    /// \param[out] model Reference model where to put the parsed information.
    /// \return Return the reference on argument model for convenience.
    ///
    /// \note urdfTree can be build from ::urdf::parseURDF
    ///       or ::urdf::parseURDFFile
    template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
    ModelTpl<Scalar,Options,JointCollectionTpl> &
    buildModelFromXML(const std::string & xmlStream,
                      ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                      const bool verbose = false)
    throw (std::invalid_argument);


    /**
     * @brief      Build The GeometryModel from a URDF file. Search for meshes
     *             in the directories specified by the user first and then in
     *             the environment variable ROS_PACKAGE_PATH
     *
     * @param[in]  model         The model of the robot, built with
     *                           urdf::buildModel
     * @param[in]  filename      The URDF complete (absolute) file path
     * @param[in]  packageDirs   A vector containing the different directories
     *                           where to search for models and meshes, typically 
     *                           obtained from calling se3::rosPaths()
     *
     * @param[in]   type         The type of objects that must be loaded (must be VISUAL or COLLISION)
     * @param[out]  geomModel    Reference where to put the parsed information.
     *
     * @return      Returns the reference on geom model for convenience.
     *
     * \warning     If hpp-fcl has not been found during compilation, COLLISION objects can not be loaded
     *
     */
    template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
    GeometryModel & buildGeom(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                              const std::string & filename,
                              const GeometryType type,
                              GeometryModel & geomModel,
                              const std::vector<std::string> & packageDirs = std::vector<std::string> ())
    throw (std::invalid_argument);
    
    /**
     * @brief      Build The GeometryModel from a URDF file. Search for meshes
     *             in the directories specified by the user first and then in
     *             the environment variable ROS_PACKAGE_PATH
     *
     * @param[in]  model         The model of the robot, built with
     *                           urdf::buildModel
     * @param[in]  filename      The URDF complete (absolute) file path
     * @param[in]  packageDir    A string containing the path to the directories of the meshes,
     *                           typically obtained from calling se3::rosPaths().
     *
     * @param[in]   type         The type of objects that must be loaded (must be VISUAL or COLLISION)
     * @param[out]  geomModel    Reference where to put the parsed information.
     *
     * @return      Returns the reference on geom model for convenience.
     *
     * \warning     If hpp-fcl has not been found during compilation, COLLISION objects can not be loaded
     *
     */
    template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
    GeometryModel & buildGeom(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                              const std::string & filename,
                              const GeometryType type,
                              GeometryModel & geomModel,
                              const std::string & packageDir)
    throw (std::invalid_argument)
    {
      const std::vector<std::string> dirs(1,packageDir);
      return buildGeom(model,filename,type,geomModel,dirs);
    }

    /**
     * @brief      Build The GeometryModel from a URDF model. Search for meshes
     *             in the directories specified by the user first and then in
     *             the environment variable ROS_PACKAGE_PATH
     *
     * @param[in]  model         The model of the robot, built with
     *                           urdf::buildModel
     * @param[in]  xmlStream     Stream containing the URDF model
     * @param[in]  packageDirs   A vector containing the different directories
     *                           where to search for models and meshes, typically
     *                           obtained from calling se3::rosPaths()
     *
     * @param[in]   type         The type of objects that must be loaded (must be VISUAL or COLLISION)
     * @param[out]  geomModel    Reference where to put the parsed information.
     *
     * @return      Returns the reference on geom model for convenience.
     *
     * \warning     If hpp-fcl has not been found during compilation, COLLISION objects cannot be loaded
     *
     */
    template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
    GeometryModel & buildGeom(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                              const std::istream & xmlStream,
                              const GeometryType type,
                              GeometryModel & geomModel,
                              const std::vector<std::string> & packageDirs = std::vector<std::string> ())
    throw (std::invalid_argument);
    
    /**
     * @brief      Build The GeometryModel from a URDF model. Search for meshes
     *             in the directories specified by the user first and then in
     *             the environment variable ROS_PACKAGE_PATH
     *
     * @param[in]  model         The model of the robot, built with
     *                           urdf::buildModel
     * @param[in]  xmlStream     Stream containing the URDF model
     * @param[in]  packageDir    A string containing the path to the directories of the meshes,
     *                           typically obtained from calling se3::rosPaths().
     *
     * @param[in]   type         The type of objects that must be loaded (must be VISUAL or COLLISION)
     * @param[out]  geomModel    Reference where to put the parsed information.
     *
     * @return      Returns the reference on geom model for convenience.
     *
     * \warning     If hpp-fcl has not been found during compilation, COLLISION objects cannot be loaded
     *
     */
    template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
    GeometryModel & buildGeom(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                              const std::istream & xmlStream,
                              const GeometryType type,
                              GeometryModel & geomModel,
                              const std::string & packageDir)
    throw (std::invalid_argument)
    {
      const std::vector<std::string> dirs(1,packageDir);
      return buildGeom(model,xmlStream,type,geomModel,dirs);
    }


  } // namespace urdf
} // namespace se3

#include "pinocchio/parsers/urdf/model.hxx"
#include "pinocchio/parsers/urdf/geometry.hxx"

#endif // ifndef __se3_parsers_urdf_hpp__
