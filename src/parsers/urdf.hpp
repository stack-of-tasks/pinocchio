//
// Copyright (c) 2015-2018 CNRS
// Copyright (c) 2015 Wandercraft, 86 rue de Paris 91400 Orsay, France.
//

#ifndef __pinocchio_parsers_urdf_hpp__
#define __pinocchio_parsers_urdf_hpp__

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/geometry.hpp"

#ifdef PINOCCHIO_WITH_CXX11_SUPPORT
#include <memory>
#endif

/// \cond
// forward declaration of the unique type from urdfdom which is expose.
namespace urdf {
  class ModelInterface;
}

namespace hpp
{
  namespace fcl
  {
    class MeshLoader;
    typedef boost::shared_ptr<MeshLoader> MeshLoaderPtr;
  }
}
/// \endcond

namespace pinocchio
{
  namespace urdf
  {

    ///
    /// \brief Build the model from a URDF file with a particular joint as root of the model tree inside
    /// the model given as reference argument.
    ///
    /// \param[in] filename The URDF complete file path.
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
               const bool verbose = false);


    ///
    /// \brief Build the model from a URDF file with a fixed joint as root of the model tree.
    ///
    /// \param[in] filename The URDF complete file path.
    /// \param[in] verbose Print parsing info.
    /// \param[out] model Reference model where to put the parsed information.
    /// \return Return the reference on argument model for convenience.
    ///
    template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
    ModelTpl<Scalar,Options,JointCollectionTpl> &
    buildModel(const std::string & filename,
               ModelTpl<Scalar,Options,JointCollectionTpl> & model,
               const bool verbose = false);

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
    buildModel(const boost::shared_ptr< ::urdf::ModelInterface> urdfTree,
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
    buildModel(const boost::shared_ptr< ::urdf::ModelInterface> urdfTree,
               ModelTpl<Scalar,Options,JointCollectionTpl> & model,
               const bool verbose = false);

#ifdef PINOCCHIO_WITH_CXX11_SUPPORT
    /// copydoc buildModel<Scalar,Options,JointCollectionTpl>(const boost::shared_ptr< ::urdf::ModelInterface>, const typename ModelTpl<Scalar,Options,JointCollectionTpl>::JointModel &, ModelTpl<Scalar,Options,JointCollectionTpl> &, const bool verbose)
    template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
    ModelTpl<Scalar,Options,JointCollectionTpl> &
    buildModel(const std::shared_ptr< ::urdf::ModelInterface> urdfTree,
               const typename ModelTpl<Scalar,Options,JointCollectionTpl>::JointModel & rootJoint,
               ModelTpl<Scalar,Options,JointCollectionTpl> & model,
               const bool verbose = false);

    /// copydoc buildModel<Scalar,Options,JointCollectionTpl>(const boost::shared_ptr< ::urdf::ModelInterface>, ModelTpl<Scalar,Options,JointCollectionTpl> &, const bool verbose)
    template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
    ModelTpl<Scalar,Options,JointCollectionTpl> &
    buildModel(const std::shared_ptr< ::urdf::ModelInterface> urdfTree,
               ModelTpl<Scalar,Options,JointCollectionTpl> & model,
               const bool verbose = false);
#endif

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
                      const typename ModelTpl<Scalar,Options,JointCollectionTpl>::JointModel & rootJoint,
                      ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                      const bool verbose = false);
    
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
                      const bool verbose = false);


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
                              const std::string & filename,
                              const GeometryType type,
                              GeometryModel & geomModel,
                              const std::vector<std::string> & packageDirs = std::vector<std::string> (),
                              ::hpp::fcl::MeshLoaderPtr meshLoader = ::hpp::fcl::MeshLoaderPtr());
    
    /**
     * @brief      Build The GeometryModel from a URDF file. Search for meshes
     *             in the directories specified by the user first and then in
     *             the environment variable ROS_PACKAGE_PATH
     *
     * @param[in]  model         The model of the robot, built with
     *                           urdf::buildModel
     * @param[in]  filename      The URDF complete (absolute) file path
     * @param[in]  packageDir    A string containing the path to the directories of the meshes,
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
                              const std::string & filename,
                              const GeometryType type,
                              GeometryModel & geomModel,
                              const std::string & packageDir,
                              hpp::fcl::MeshLoaderPtr meshLoader = hpp::fcl::MeshLoaderPtr())
   
    {
      const std::vector<std::string> dirs(1,packageDir);
      return buildGeom(model,filename,type,geomModel,dirs,meshLoader);
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
     *                           obtained from calling pinocchio::rosPaths()
     *
     * @param[in]   type         The type of objects that must be loaded (must be VISUAL or COLLISION)
     * @param[in]   meshLoader   object used to load meshes: hpp::fcl::MeshLoader [default] or hpp::fcl::CachedMeshLoader.
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
                              const std::vector<std::string> & packageDirs = std::vector<std::string> (),
                              hpp::fcl::MeshLoaderPtr meshLoader = hpp::fcl::MeshLoaderPtr());
    
    /**
     * @brief      Build The GeometryModel from a URDF model. Search for meshes
     *             in the directories specified by the user first and then in
     *             the environment variable ROS_PACKAGE_PATH
     *
     * @param[in]  model         The model of the robot, built with
     *                           urdf::buildModel
     * @param[in]  xmlStream     Stream containing the URDF model
     * @param[in]  packageDir    A string containing the path to the directories of the meshes,
     *                           typically obtained from calling pinocchio::rosPaths().
     *
     * @param[in]   type         The type of objects that must be loaded (must be VISUAL or COLLISION)
     * @param[in]   meshLoader   object used to load meshes: hpp::fcl::MeshLoader [default] or hpp::fcl::CachedMeshLoader.
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
                              const std::string & packageDir,
                              hpp::fcl::MeshLoaderPtr meshLoader = hpp::fcl::MeshLoaderPtr())
   
    {
      const std::vector<std::string> dirs(1,packageDir);
      return buildGeom(model,xmlStream,type,geomModel,dirs, meshLoader);
    }


  } // namespace urdf
} // namespace pinocchio

#include "pinocchio/parsers/urdf/model.hxx"
#include "pinocchio/parsers/urdf/geometry.hxx"

#endif // ifndef __pinocchio_parsers_urdf_hpp__
