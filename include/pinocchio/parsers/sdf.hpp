//
// Copyright (c) 2020 CNRS
//

#ifndef __pinocchio_parsers_sdf_hpp__
#define __pinocchio_parsers_sdf_hpp__

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/geometry.hpp"
#include "pinocchio/algorithm/contact-info.hpp"

namespace pinocchio
{
  namespace sdf
  {

    /**
     * @brief      Build The GeometryModel from a SDF file. Search for meshes
     *             in the directories specified by the user first and then in
     *             the environment variable ROS_PACKAGE_PATH
     *
     * @param[in]  model         The model of the robot, built with
     *                           sdf::buildModel
     * @param[in]  filename      The SDF complete (absolute) file path
     * @param[in]  packageDirs   A vector containing the different directories
     *                           where to search for models and meshes, typically
     *                           obtained from calling pinocchio::rosPaths()
     *
     * @param[in]   type         The type of objects that must be loaded (must be VISUAL or
     * COLLISION)
     * @param[in]   rootLinkName Name of the root link
     * @param[in]   meshLoader   object used to load meshes: hpp::fcl::MeshLoader [default] or
     * hpp::fcl::CachedMeshLoader.
     * @param[out]  geomModel    Reference where to put the parsed information.
     *
     * @return      Returns the reference on geom model for convenience.
     *
     * \warning     If hpp-fcl has not been found during compilation, COLLISION objects can not be
     * loaded
     *
     */
    template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
    GeometryModel & buildGeom(
      const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
      const std::string & filename,
      const GeometryType type,
      GeometryModel & geomModel,
      const std::string & rootLinkName = "",
      const std::vector<std::string> & packageDirs = std::vector<std::string>(),
      ::hpp::fcl::MeshLoaderPtr meshLoader = ::hpp::fcl::MeshLoaderPtr());

    /**
     * @brief      Build The GeometryModel from a SDF file. Search for meshes
     *             in the directories specified by the user first and then in
     *             the environment variable ROS_PACKAGE_PATH
     *
     * @param[in]  model         The model of the robot, built with
     *                           sdf::buildModel
     * @param[in]  filename      The SDF complete (absolute) file path
     * @param[in]  package_path    A string containing the path to the directories of the meshes,
     *                           typically obtained from calling pinocchio::rosPaths().
     *
     * @param[in]   type         The type of objects that must be loaded (must be VISUAL or
     * COLLISION)
     * @param[in]   meshLoader   object used to load meshes: hpp::fcl::MeshLoader [default] or
     * hpp::fcl::CachedMeshLoader.
     * @param[out]  geomModel    Reference where to put the parsed information.
     *
     * @return      Returns the reference on geom model for convenience.
     *
     * \warning     If hpp-fcl has not been found during compilation, COLLISION objects can not be
     * loaded
     *
     */
    template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
    GeometryModel & buildGeom(
      const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
      PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) & contact_models,
      const std::string & filename,
      const GeometryType type,
      GeometryModel & geomModel,
      const std::string & packagePath,
      ::hpp::fcl::MeshLoaderPtr meshLoader = ::hpp::fcl::MeshLoaderPtr())
    {
      const std::vector<std::string> dirs(1, packagePath);
      return buildGeom(model, contact_models, filename, type, geomModel, dirs, meshLoader);
    };

    /**
     * @brief      Build The GeometryModel from a SDF file. Search for meshes
     *             in the directories specified by the user first and then in
     *             the environment variable ROS_PACKAGE_PATH
     *
     * @param[in]  model         The model of the robot, built with
     *                           sdf::buildModel
     * @param[in]  filename      The SDF complete (absolute) file path
     * @param[in]  package_path    A string containing the path to the directories of the meshes,
     *                           typically obtained from calling pinocchio::rosPaths().
     *
     * @param[in]   type         The type of objects that must be loaded (must be VISUAL or
     * COLLISION)
     * @param[in]   rootLinkName Name of the root link
     * @param[in]   meshLoader   object used to load meshes: hpp::fcl::MeshLoader [default] or
     * hpp::fcl::CachedMeshLoader.
     * @param[out]  geomModel    Reference where to put the parsed information.
     *
     * @return      Returns the reference on geom model for convenience.
     *
     * \warning     If hpp-fcl has not been found during compilation, COLLISION objects can not be
     * loaded
     *
     */
    template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
    GeometryModel & buildGeom(
      const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
      const std::string & filename,
      const GeometryType type,
      GeometryModel & geomModel,
      const std::string & rootLinkName,
      const std::string & packagePath,
      ::hpp::fcl::MeshLoaderPtr meshLoader = ::hpp::fcl::MeshLoaderPtr())
    {
      const std::vector<std::string> dirs(1, packagePath);
      return buildGeom(model, filename, type, geomModel, rootLinkName, dirs, meshLoader);
    };

    ///
    /// \brief Build the model from an XML stream with a particular joint as root of the model tree
    /// inside the model given as reference argument.
    ///
    /// \param[in] xmlStream xml stream containing SDF model
    /// \param[in] rootJoint The joint at the root of the model tree.
    /// \param[in] rootJointName Name of the rootJoint.
    /// \param[in] parentGuidance Joint names which should be preferred for cases where two joints
    /// can qualify as parent. The other joint appears in the constraint_model. If empty, joint
    /// appearance order in .sdf is taken as default.
    /// \param[in] verbose Print parsing info.
    /// \param[out] model Reference model where to put the parsed information.
    /// \return Return the reference on argument model for convenience.
    ///
    template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
    ModelTpl<Scalar, Options, JointCollectionTpl> & buildModelFromXML(
      const std::string & xmlStream,
      const typename ModelTpl<Scalar, Options, JointCollectionTpl>::JointModel & rootJoint,
      const std::string & rootJointName,
      ModelTpl<Scalar, Options, JointCollectionTpl> & model,
      PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) & contact_models,
      const std::string & rootLinkName = "",
      const std::vector<std::string> & parentGuidance = {},
      const bool verbose = false);

    ///
    /// \brief Build the model from a SDF file with a particular joint as root of the model tree
    /// inside the model given as reference argument.
    ///
    /// \param[in] filename The SDF complete file path.
    /// \param[in] rootJoint The joint at the root of the model tree.*
    /// \param[in] rootJointName Name of the rootJoint.
    /// \param[in] parentGuidance Joint names which should be preferred for cases where two joints
    /// can qualify as parent. The other joint appears in the constraint_model. If empty, joint
    /// appearance order in .sdf is taken as default.
    /// \param[in] verbose Print parsing info.
    /// \param[out] model Reference model where to put the parsed information.
    /// \return Return the reference on argument model for convenience.
    ///
    template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
    ModelTpl<Scalar, Options, JointCollectionTpl> & buildModel(
      const std::string & filename,
      const typename ModelTpl<Scalar, Options, JointCollectionTpl>::JointModel & rootJoint,
      const std::string & rootJointName,
      ModelTpl<Scalar, Options, JointCollectionTpl> & model,
      PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) & contact_models,
      const std::string & rootLinkName = "",
      const std::vector<std::string> & parentGuidance = {},
      const bool verbose = false);

    ///
    /// \brief Build the model from an xml stream with a fixed joint as root of the model tree.
    ///
    /// \param[in] xmlStream The xml stream containing the SDF Model.
    /// \param[in] verbose Print parsing info.
    /// \param[out] model Reference model where to put the parsed information.
    /// \return Return the reference on argument model for convenience.
    ///
    template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
    ModelTpl<Scalar, Options, JointCollectionTpl> & buildModelFromXML(
      const std::string & xmlStream,
      ModelTpl<Scalar, Options, JointCollectionTpl> & model,
      PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) & contact_models,
      const std::string & rootLinkName = "",
      const std::vector<std::string> & parentGuidance = {},
      const bool verbose = false);

    ///
    /// \brief Build the model from a SDF file with a fixed joint as root of the model tree.
    ///
    /// \param[in] filename The SDF complete file path.
    /// \param[in] verbose Print parsing info.
    /// \param[out] model Reference model where to put the parsed information.
    /// \return Return the reference on argument model for convenience.
    ///
    template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
    ModelTpl<Scalar, Options, JointCollectionTpl> & buildModel(
      const std::string & filename,
      ModelTpl<Scalar, Options, JointCollectionTpl> & model,
      PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) & contact_models,
      const std::string & rootLinkName = "",
      const std::vector<std::string> & parentGuidance = {},
      const bool verbose = false);

  } // namespace sdf
} // namespace pinocchio

#include "pinocchio/parsers/sdf/model.hxx"
#include "pinocchio/parsers/sdf/geometry.hxx"

#endif // ifndef __pinocchio_parsers_sdf_hpp__
