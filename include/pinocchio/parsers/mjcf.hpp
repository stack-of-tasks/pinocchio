//
// Copyright (c) 2024 INRIA CNRS
//

#ifndef __pinocchio_parsers_mjcf_hpp__
#define __pinocchio_parsers_mjcf_hpp__

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/geometry.hpp"
#include "pinocchio/algorithm/contact-info.hpp"

namespace pinocchio
{
  namespace mjcf
  {

    ///
    /// \brief Build the model from a MJCF file with a fixed joint as root of the model tree.
    ///
    /// \param[in] filename The MJCF complete file path.
    /// \param[in] verbose Print parsing info.
    /// \param[out] model Reference model where to put the parsed information.
    /// \return Return the reference on argument model for convenience.
    ///
    template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
    ModelTpl<Scalar, Options, JointCollectionTpl> & buildModel(
      const std::string & filename,
      ModelTpl<Scalar, Options, JointCollectionTpl> & model,
      const bool verbose = false);

    ///
    /// \brief Build the model from a MJCF file with a fixed joint as root of the model tree
    ///
    /// \param[in] xmlStream The xml stream containing the MJCF Model.
    /// \param[in] verbose Print parsing info.
    /// \param[out] model Reference model where to put the parsed information.
    /// \return Return the reference on argument model for convenience.
    ///
    template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
    ModelTpl<Scalar, Options, JointCollectionTpl> & buildModelFromXML(
      const std::string & xmlStream,
      ModelTpl<Scalar, Options, JointCollectionTpl> & model,
      const bool verbose = false);

    // TODO: update description, buildModel with contact model
    template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
    ModelTpl<Scalar, Options, JointCollectionTpl> & buildModel(
      const std::string & filename,
      ModelTpl<Scalar, Options, JointCollectionTpl> & model,
      PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) & contact_models,
      const bool verbose = false);

    template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
    ModelTpl<Scalar, Options, JointCollectionTpl> & buildModelFromXML(
      const std::string & xmlStream,
      ModelTpl<Scalar, Options, JointCollectionTpl> & model,
      PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) & contact_models,
      const bool verbose = false);

    ///
    /// \brief Build the model from a MJCF file with a particular joint as root of the model tree
    /// inside the model given as reference argument.
    ///
    /// \param[in] filename The MJCF complete file path.
    /// \param[in] rootJoint The joint at the root of the model tree.
    /// \param[in] verbose Print parsing info.
    /// \param[out] model Reference model where to put the parsed information.
    /// \return Return the reference on argument model for convenience.
    ///
    template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
    ModelTpl<Scalar, Options, JointCollectionTpl> & buildModel(
      const std::string & filename,
      const typename ModelTpl<Scalar, Options, JointCollectionTpl>::JointModel & rootJoint,
      ModelTpl<Scalar, Options, JointCollectionTpl> & model,
      const bool verbose = false);

    template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
    ModelTpl<Scalar, Options, JointCollectionTpl> & buildModelFromXML(
      const std::string & xmlStream,
      const typename ModelTpl<Scalar, Options, JointCollectionTpl>::JointModel & rootJoint,
      ModelTpl<Scalar, Options, JointCollectionTpl> & model,
      const bool verbose = false);

    ///
    /// \brief Build the model from a MJCF file with a particular joint as root of the model tree
    /// inside the model given as reference argument.
    ///
    /// \param[in] filename The MJCF complete file path.
    /// \param[in] rootJoint The joint at the root of the model tree.
    /// \param[in] rootJointName Name of the rootJoint.
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
      const bool verbose = false);

    template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
    ModelTpl<Scalar, Options, JointCollectionTpl> & buildModelFromXML(
      const std::string & xmlStream,
      const typename ModelTpl<Scalar, Options, JointCollectionTpl>::JointModel & rootJoint,
      const std::string & rootJointName,
      ModelTpl<Scalar, Options, JointCollectionTpl> & model,
      const bool verbose = false);

    ///
    /// \brief Build the model from a MJCF file with a particular joint as root of the model tree
    /// inside the model given as reference argument.
    ///
    /// \param[in] filename The MJCF complete file path.
    /// \param[in] rootJoint The joint at the root of the model tree.
    /// \param[in] verbose Print parsing info.
    /// \param[out] model Reference model where to put the parsed information.
    /// \return Return the reference on argument model for convenience.
    ///
    template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
    ModelTpl<Scalar, Options, JointCollectionTpl> & buildModel(
      const std::string & filename,
      const typename ModelTpl<Scalar, Options, JointCollectionTpl>::JointModel & rootJoint,
      ModelTpl<Scalar, Options, JointCollectionTpl> & model,
      PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) & contact_models,
      const bool verbose = false);

    template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
    ModelTpl<Scalar, Options, JointCollectionTpl> & buildModelFromXML(
      const std::string & xmlStream,
      const typename ModelTpl<Scalar, Options, JointCollectionTpl>::JointModel & rootJoint,
      ModelTpl<Scalar, Options, JointCollectionTpl> & model,
      PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) & contact_models,
      const bool verbose);

    ///
    /// \brief Build the model from a MJCF file with a particular joint as root of the model tree
    /// inside the model given as reference argument.
    ///
    /// \param[in] filename The MJCF complete file path.
    /// \param[in] rootJoint The joint at the root of the model tree.
    /// \param[in] rootJointName Name of the rootJoint.
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
      const bool verbose = false);

    ///
    /// \brief Build the model from an XML stream with a particular joint as root of the model tree
    /// inside the model given as reference argument.
    ///
    /// \param[in] xmlStream xml stream containing MJCF model
    /// \param[in] rootJoint The joint at the root of the model tree.
    /// \param[in] rootJointName Name of the rootJoint.
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
      const bool verbose = false);

    /**
     * @brief      Build The GeometryModel from a Mjcf file
     *
     * @param[in]  model         The model of the robot, built with
     *                           mjcf::buildModel
     * @param[in]  filename      The mjcf complete (absolute) file path
     * @param[in]   type         The type of objects that must be loaded (must be VISUAL or
     * COLLISION)
     * @param[in]   mesh_loader   object used to load meshes: hpp::fcl::MeshLoader [default] or
     * hpp::fcl::CachedMeshLoader.
     * @param[out]  geom_model    Reference where to put the parsed information.
     *
     * @return      Returns the reference on geom model for convenience.
     *
     * \warning     If hpp-fcl has not been found during compilation, COLLISION objects can not be
     * loaded
     *
     */
    template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
    GeometryModel & buildGeom(
      ModelTpl<Scalar, Options, JointCollectionTpl> & model,
      const std::string & filename,
      const GeometryType type,
      GeometryModel & geom_model,
      ::hpp::fcl::MeshLoaderPtr mesh_loader = ::hpp::fcl::MeshLoaderPtr());

  } // namespace mjcf
} // namespace pinocchio

#include "pinocchio/parsers/mjcf/model.hxx"
#include "pinocchio/parsers/mjcf/geometry.hxx"

#endif // ifndef __pinocchio_parsers_mjcf_hpp__
