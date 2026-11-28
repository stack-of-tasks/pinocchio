//
// Copyright (c) 2024 INRIA CNRS
//

#pragma once

// IWYU pragma: private, include "pinocchio/parsers/mjcf.hpp"

#ifdef PINOCCHIO_LSP
  #undef PINOCCHIO_LSP
  #include "pinocchio/parsers/mjcf.hpp"
#endif // PINOCCHIO_LSP

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

    /// This function is badly nammed and have the same behavior than
    /// \sa buildModel(const std::string &, ModelTpl<Scalar, Options, JointCollectionTpl> &, const
    /// bool)
    template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
    ModelTpl<Scalar, Options, JointCollectionTpl> & buildModelFromXML(
      const std::string & filename,
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
      const bool verbose = false);

    /// This function is badly nammed and have the same behavior than
    /// \sa buildModel(const std::string &, const typename ModelTpl<Scalar, Options,
    /// JointCollectionTpl>::JointModel &, ModelTpl<Scalar, Options, JointCollectionTpl>
    /// &, const bool)
    template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
    ModelTpl<Scalar, Options, JointCollectionTpl> & buildModelFromXML(
      const std::string & filename,
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

    /// This function is badly nammed and have the same behavior than
    /// \sa buildModel(const std::string &, const typename ModelTpl<Scalar, Options,
    /// JointCollectionTpl>::JointModel &, const std::string& ModelTpl<Scalar, Options,
    /// JointCollectionTpl> &, const bool)
    template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
    ModelTpl<Scalar, Options, JointCollectionTpl> & buildModelFromXML(
      const std::string & filename,
      const typename ModelTpl<Scalar, Options, JointCollectionTpl>::JointModel & rootJoint,
      const std::string & rootJointName,
      ModelTpl<Scalar, Options, JointCollectionTpl> & model,
      const bool verbose = false);

    ///
    /// \brief Build the model from MJCF string content with a fixed joint as root of the model
    /// tree.
    ///
    /// \param[in] xml_stream The MJCF model given as XML string content.
    /// \param[in] verbose Print parsing info.
    /// \param[out] model Reference model in which put the parsed information.
    /// \return Return the reference on argument model for convenience.
    ///
    template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
    ModelTpl<Scalar, Options, JointCollectionTpl> & buildModelFromXMLContent(
      const std::string & xml_stream,
      ModelTpl<Scalar, Options, JointCollectionTpl> & model,
      const bool verbose = false);

    ///
    /// \brief Build the model from MJCF string content with a particular joint as root of the
    /// model tree inside the model given as reference argument.
    ///
    /// \param[in] xml_stream The MJCF model given as XML string content.
    /// \param[in] rootJoint The joint at the root of the model tree.
    /// \param[in] verbose Print parsing info.
    /// \param[out] model Reference model in which to put the parsed information.
    /// \return Return the reference on argument model for convenience.
    ///
    template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
    ModelTpl<Scalar, Options, JointCollectionTpl> & buildModelFromXMLContent(
      const std::string & xml_stream,
      const typename ModelTpl<Scalar, Options, JointCollectionTpl>::JointModel & rootJoint,
      ModelTpl<Scalar, Options, JointCollectionTpl> & model,
      const bool verbose = false);

    ///
    /// \brief Build the model from MJCF string content with a particular joint as root of the
    /// model tree inside the model given as reference argument.
    ///
    /// \param[in] xml_stream The MJCF model given as an XML string content.
    /// \param[in] rootJoint The joint at the root of the model tree.
    /// \param[in] rootJointName Name of the rootJoint.
    /// \param[in] verbose Print parsing info.
    /// \param[out] model Reference model in which to put the parsed information.
    /// \return Return the reference on argument model for convenience.
    ///
    template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
    ModelTpl<Scalar, Options, JointCollectionTpl> & buildModelFromXMLContent(
      const std::string & xml_stream,
      const typename ModelTpl<Scalar, Options, JointCollectionTpl>::JointModel & rootJoint,
      const std::string & rootJointName,
      ModelTpl<Scalar, Options, JointCollectionTpl> & model,
      const bool verbose = false);

    ///
    /// \brief Build the model from a MJCF file with a fixed joint as root of the model tree and
    /// with point and frame anchor cosntraints.
    //
    /// \param[in] filename The MJCF complete file path.
    /// \param[in,out] model Reference model where to put the parsed information.
    /// \param[in,out] point_anchor_constraint_models Reference constraint models where to put the
    /// parsed information for point anchor model.
    /// \param[in,out] frame_anchor_constraint_models Reference constraint models where to put the
    /// parsed information for frame anchor constraints.
    /// \param[in] verbose Print parsing info.
    ///
    template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
    ModelTpl<Scalar, Options, JointCollectionTpl> & buildModel(
      const std::string & filename,
      ModelTpl<Scalar, Options, JointCollectionTpl> & model,
      std::vector<PointAnchorConstraintModel> & point_anchor_constraint_models,
      std::vector<FrameAnchorConstraintModel> & frame_anchor_constraint_models,
      const bool verbose = false);

    ///
    /// \brief Build the model from a MJCF file with a particular joint as root of the model tree
    /// and with point and frame anchor cosntraints.
    //
    /// \param[in] filename The MJCF complete file path.
    /// \param[in] rootJoint The joint at the root of the model tree.
    /// \param[in,out] model Reference model where to put the parsed information.
    /// \param[in,out] point_anchor_constraint_models Reference constraint models where to put the
    /// parsed information for point anchor model.
    /// \param[in,out] frame_anchor_constraint_models Reference constraint models where to put the
    /// parsed information for frame anchor constraints.
    /// \param[in] verbose Print parsing info.
    ///
    template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
    ModelTpl<Scalar, Options, JointCollectionTpl> & buildModel(
      const std::string & filename,
      const typename ModelTpl<Scalar, Options, JointCollectionTpl>::JointModel & rootJoint,
      ModelTpl<Scalar, Options, JointCollectionTpl> & model,
      std::vector<PointAnchorConstraintModel> & point_anchor_constraint_models,
      std::vector<FrameAnchorConstraintModel> & frame_anchor_constraint_models,
      const bool verbose = false);

    ///
    /// \brief Build the model from a MJCF file with a particular joint as root of the model tree
    /// and with point and frame anchor cosntraints.
    //
    /// \param[in] filename The MJCF complete file path.
    /// \param[in] rootJoint The joint at the root of the model tree.
    /// \param[in] rootJointName Name of the rootJoint.
    /// \param[in,out] model Reference model where to put the parsed information.
    /// \param[in,out] point_anchor_constraint_models Reference constraint models where to put the
    /// parsed information for point anchor model.
    /// \param[in,out] frame_anchor_constraint_models Reference constraint models where to put the
    /// parsed information for frame anchor constraints.
    /// \param[in] verbose Print parsing info.
    ///
    template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
    ModelTpl<Scalar, Options, JointCollectionTpl> & buildModel(
      const std::string & filename,
      const typename ModelTpl<Scalar, Options, JointCollectionTpl>::JointModel & rootJoint,
      const std::string & rootJointName,
      ModelTpl<Scalar, Options, JointCollectionTpl> & model,
      std::vector<PointAnchorConstraintModel> & point_anchor_constraint_models,
      std::vector<FrameAnchorConstraintModel> & frame_anchor_constraint_models,
      const bool verbose = false);

    ///
    /// \brief Build the model from a MJCF file with a fixed joint as root of the model tree and
    /// with point and frame anchor cosntraints.
    //
    /// \param[in] filename The MJCF complete file path.
    /// \param[in,out] model Reference model where to put the parsed information.
    /// \param[in,out] rigid_constraint_model Reference constraint models where to put the
    /// parsed information for constraints.
    /// \param[in] verbose Print parsing info.
    ///
    template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
    PINOCCHIO_UNSUPPORTED ModelTpl<Scalar, Options, JointCollectionTpl> & buildModel(
      const std::string & filename,
      ModelTpl<Scalar, Options, JointCollectionTpl> & model,
      std::vector<RigidConstraintModel> & rigid_constraint_model,
      const bool verbose = false);

    /// This function is badly nammed and have the same behavior than
    /// \sa buildModel(const std::string &, ModelTpl<Scalar, Options, JointCollectionTpl> &,
    /// std::vector<RigidConstraintModel> &, const bool)
    template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
    PINOCCHIO_UNSUPPORTED ModelTpl<Scalar, Options, JointCollectionTpl> & buildModelFromXML(
      const std::string & filename,
      ModelTpl<Scalar, Options, JointCollectionTpl> & model,
      std::vector<RigidConstraintModel> & rigid_constraint_model,
      const bool verbose = false);

    ///
    /// \brief Build the model from a MJCF file with a particular joint as root of the model tree
    /// and with point and frame anchor cosntraints.
    //
    /// \param[in] filename The MJCF complete file path.
    /// \param[in] rootJoint The joint at the root of the model tree.
    /// \param[in,out] model Reference model where to put the parsed information.
    /// \param[in,out] rigid_constraint_model Reference constraint models where to put the
    /// parsed information for constraints.
    /// \param[in] verbose Print parsing info.
    ///
    template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
    PINOCCHIO_UNSUPPORTED ModelTpl<Scalar, Options, JointCollectionTpl> & buildModel(
      const std::string & filename,
      const typename ModelTpl<Scalar, Options, JointCollectionTpl>::JointModel & rootJoint,
      ModelTpl<Scalar, Options, JointCollectionTpl> & model,
      std::vector<RigidConstraintModel> & rigid_constraint_model,
      const bool verbose = false);

    /// This function is badly nammed and have the same behavior than
    /// \sa buildModel(const std::string &, const typename ModelTpl<Scalar, Options,
    /// JointCollectionTpl>::JointModel &, ModelTpl<Scalar, Options,
    /// JointCollectionTpl> &, std::vector<RigidConstraintModel> &, const bool)
    template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
    PINOCCHIO_UNSUPPORTED ModelTpl<Scalar, Options, JointCollectionTpl> & buildModelFromXML(
      const std::string & filename,
      const typename ModelTpl<Scalar, Options, JointCollectionTpl>::JointModel & rootJoint,
      ModelTpl<Scalar, Options, JointCollectionTpl> & model,
      std::vector<RigidConstraintModel> & rigid_constraint_model,
      const bool verbose = false);

    ///
    /// \brief Build the model from a MJCF file with a particular joint as root of the model tree
    /// and with point and frame anchor cosntraints.
    //
    /// \param[in] filename The MJCF complete file path.
    /// \param[in] rootJoint The joint at the root of the model tree.
    /// \param[in] rootJointName Name of the rootJoint.
    /// \param[in,out] model Reference model where to put the parsed information.
    /// \param[in,out] rigid_constraint_model Reference constraint models where to put the
    /// parsed information for constraints.
    /// \param[in] verbose Print parsing info.
    ///
    template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
    PINOCCHIO_UNSUPPORTED ModelTpl<Scalar, Options, JointCollectionTpl> & buildModel(
      const std::string & filename,
      const typename ModelTpl<Scalar, Options, JointCollectionTpl>::JointModel & rootJoint,
      const std::string & rootJointName,
      ModelTpl<Scalar, Options, JointCollectionTpl> & model,
      std::vector<RigidConstraintModel> & rigid_constraint_model,
      const bool verbose = false);

    /// This function is badly nammed and have the same behavior than
    /// \sa buildModel(const std::string &, const typename ModelTpl<Scalar, Options,
    /// JointCollectionTpl>::JointModel &, const std::string&, ModelTpl<Scalar, Options,
    /// JointCollectionTpl> &, std::vector<RigidConstraintModel> &, const bool)
    template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
    PINOCCHIO_UNSUPPORTED ModelTpl<Scalar, Options, JointCollectionTpl> & buildModelFromXML(
      const std::string & filename,
      const typename ModelTpl<Scalar, Options, JointCollectionTpl>::JointModel & rootJoint,
      const std::string & rootJointName,
      ModelTpl<Scalar, Options, JointCollectionTpl> & model,
      std::vector<RigidConstraintModel> & rigid_constraint_model,
      const bool verbose = false);

    /**
     * @brief      Build The GeometryModel from a Mjcf file
     *
     * @param[in]  model         The model of the robot, built with
     *                           mjcf::buildModel
     * @param[in]  filename      The mjcf complete (absolute) file path
     * @param[in]   type         The type of objects that must be loaded (must be VISUAL or
     * COLLISION)
     * @param[in]   mesh_loader   object used to load meshes: coal::MeshLoader [default] or
     * coal::CachedMeshLoader.
     * @param[out]  geom_model    Reference where to put the parsed information.
     *
     * @return      Returns the reference on geom model for convenience.
     *
     * \warning     If coal has not been found during compilation, COLLISION objects can not be
     * loaded
     *
     */
    template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
    GeometryModel & buildGeom(
      ModelTpl<Scalar, Options, JointCollectionTpl> & model,
      const std::string & filename,
      const GeometryType type,
      GeometryModel & geom_model,
      ::coal::MeshLoaderPtr mesh_loader = ::coal::MeshLoaderPtr());

    /**
     * @brief      Build The GeometryModel from MJCF string content
     *
     * @param[in]  model         The model of the robot, built with
     *                           mjcf::buildModel or mjcf::buildModelFromXMLContent
     * @param[in]  xml_stream    The MJCF model given as XML string content
     * @param[in]   type         The type of objects that must be loaded (must be VISUAL or
     * COLLISION)
     * @param[in]   mesh_loader   object used to load meshes: coal::MeshLoader [default] or
     * coal::CachedMeshLoader.
     * @param[out]  geom_model    Reference geometry model in which to put the parsed information.
     *
     * @return      Returns the reference on geom model for convenience.
     *
     * \note       Without a file path, relative mesh and texture paths are resolved from the
     * current working directory.
     *
     * \warning     If coal has not been found during compilation, COLLISION objects can not be
     * loaded
     *
     */
    template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
    GeometryModel & buildGeomFromXMLContent(
      ModelTpl<Scalar, Options, JointCollectionTpl> & model,
      const std::string & xml_stream,
      const GeometryType type,
      GeometryModel & geom_model,
      ::coal::MeshLoaderPtr mesh_loader = ::coal::MeshLoaderPtr());

  } // namespace mjcf
} // namespace pinocchio
