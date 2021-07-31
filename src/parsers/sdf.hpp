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

#ifdef PINOCCHIO_WITH_HPP_FCL
#ifdef PINOCCHIO_WITH_SDF
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
     * @param[in]   type         The type of objects that must be loaded (must be VISUAL or COLLISION)
     * @param[in]   meshLoader   object used to load meshes: hpp::fcl::MeshLoader [default] or hpp::fcl::CachedMeshLoader.
     * @param[out]  geomModel    Reference where to put the parsed information.
     *
     * @return      Returns the reference on geom model for convenience.
     *
     * \warning     If hpp-fcl has not been found during compilation, COLLISION objects can not be loaded
     *
     */
    template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
    GeometryModel & buildGeom(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                              PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel)& contact_models,
                              const std::string & filename,
                              const GeometryType type,
                              GeometryModel & geomModel,
                              const std::vector<std::string> & packageDirs = std::vector<std::string> (),
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
     * @param[in]   type         The type of objects that must be loaded (must be VISUAL or COLLISION)
     * @param[in]   meshLoader   object used to load meshes: hpp::fcl::MeshLoader [default] or hpp::fcl::CachedMeshLoader.
     * @param[out]  geomModel    Reference where to put the parsed information.
     *
     * @return      Returns the reference on geom model for convenience.
     *
     * \warning     If hpp-fcl has not been found during compilation, COLLISION objects can not be loaded
     *
     */
    template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
    GeometryModel & buildGeom(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                              PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel)& contact_models,
                              const std::string & filename,
                              const GeometryType type,
                              GeometryModel & geomModel,
                              const std::string & packagePath,
                              ::hpp::fcl::MeshLoaderPtr meshLoader = ::hpp::fcl::MeshLoaderPtr())
    {
      const std::vector<std::string> dirs(1,packagePath);
      return buildGeom(model,contact_models,filename,type,geomModel,dirs,meshLoader);
    };

    

    ///
    /// \brief Build the model from a SDF file with a particular joint as root of the model tree inside
    /// the model given as reference argument.
    ///
    /// \param[in] filename The SDF complete file path.
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
               PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel)& contact_models,
               const bool verbose = false);


    ///
    /// \brief Build the model from a SDF file with a fixed joint as root of the model tree.
    ///
    /// \param[in] filename The SDF complete file path.
    /// \param[in] verbose Print parsing info.
    /// \param[out] model Reference model where to put the parsed information.
    /// \return Return the reference on argument model for convenience.
    ///
    template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
    ModelTpl<Scalar,Options,JointCollectionTpl> &
    buildModel(const std::string & filename,
               ModelTpl<Scalar,Options,JointCollectionTpl> & model,
               PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel)& contact_models,
               const bool verbose = false);

#endif
#endif
    
  } // namespace sdf
} // namespace pinocchio

#ifdef PINOCCHIO_WITH_SDF
#ifdef PINOCCHIO_WITH_HPP_FCL
#include "pinocchio/parsers/sdf/model.hxx"
#include "pinocchio/parsers/sdf/geometry.hxx"
#endif
#endif

#endif // ifndef __pinocchio_parsers_sdf_hpp__
