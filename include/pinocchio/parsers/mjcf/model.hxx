//
// Copyright (c) 2016-2023 CNRS INRIA
//

#ifndef __pinocchio_parsers_mjcf_model_hxx__
#define __pinocchio_parsers_mjcf_model_hxx__

#include "pinocchio/parsers/mjcf.hpp"
#include "pinocchio/parsers/mjcf/mjcf-graph.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/algorithm/contact-info.hpp"

namespace pinocchio
{
  namespace mjcf
  {
    template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
    ModelTpl<Scalar, Options, JointCollectionTpl> & buildModel(
      const std::string & filename,
      ModelTpl<Scalar, Options, JointCollectionTpl> & model,
      const bool verbose)
    {
      return buildModelFromXML(filename, model, verbose);
    }

    template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
    ModelTpl<Scalar, Options, JointCollectionTpl> & buildModelFromXML(
      const std::string & xmlStream,
      ModelTpl<Scalar, Options, JointCollectionTpl> & model,
      const bool verbose)
    {
      PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) contact_models;
      return buildModelFromXML(xmlStream, model, contact_models, verbose);
    }

    template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
    ModelTpl<Scalar, Options, JointCollectionTpl> & buildModel(
      const std::string & filename,
      ModelTpl<Scalar, Options, JointCollectionTpl> & model,
      PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) & contact_models,
      const bool verbose)
    {
      return buildModelFromXML(filename, model, contact_models, verbose);
    }

    template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
    ModelTpl<Scalar, Options, JointCollectionTpl> & buildModelFromXML(
      const std::string & xmlStream,
      ModelTpl<Scalar, Options, JointCollectionTpl> & model,
      PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) & contact_models,
      const bool verbose)
    {
      ::pinocchio::urdf::details::UrdfVisitor<Scalar, Options, JointCollectionTpl> visitor(model);

      typedef ::pinocchio::mjcf::details::MjcfGraph MjcfGraph;

      MjcfGraph graph(visitor, xmlStream);
      if (verbose)
        visitor.log = &std::cout;

      graph.parseGraphFromXML(xmlStream);

      // Use the Mjcf graph to create the model
      graph.parseRootTree();
      graph.parseContactInformation(model, contact_models);

      return model;
    }

    template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
    ModelTpl<Scalar, Options, JointCollectionTpl> & buildModel(
      const std::string & filename,
      const typename ModelTpl<Scalar, Options, JointCollectionTpl>::JointModel & rootJoint,
      ModelTpl<Scalar, Options, JointCollectionTpl> & model,
      const bool verbose)
    {
      return buildModelFromXML(filename, rootJoint, model, verbose);
    }

    template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
    ModelTpl<Scalar, Options, JointCollectionTpl> & buildModelFromXML(
      const std::string & xmlStream,
      const typename ModelTpl<Scalar, Options, JointCollectionTpl>::JointModel & rootJoint,
      ModelTpl<Scalar, Options, JointCollectionTpl> & model,
      const bool verbose)
    {
      PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) contact_models;
      return buildModelFromXML(xmlStream, rootJoint, "root_joint", model, contact_models, verbose);
    }

    template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
    ModelTpl<Scalar, Options, JointCollectionTpl> & buildModel(
      const std::string & filename,
      const typename ModelTpl<Scalar, Options, JointCollectionTpl>::JointModel & rootJoint,
      const std::string & rootJointName,
      ModelTpl<Scalar, Options, JointCollectionTpl> & model,
      const bool verbose)
    {
      return buildModelFromXML(filename, rootJoint, rootJointName, model, verbose);
    }

    template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
    ModelTpl<Scalar, Options, JointCollectionTpl> & buildModelFromXML(
      const std::string & xmlStream,
      const typename ModelTpl<Scalar, Options, JointCollectionTpl>::JointModel & rootJoint,
      const std::string & rootJointName,
      ModelTpl<Scalar, Options, JointCollectionTpl> & model,
      const bool verbose)
    {
      PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) contact_models;
      return buildModelFromXML(xmlStream, rootJoint, rootJointName, model, contact_models, verbose);
    }

    template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
    ModelTpl<Scalar, Options, JointCollectionTpl> & buildModel(
      const std::string & filename,
      const typename ModelTpl<Scalar, Options, JointCollectionTpl>::JointModel & rootJoint,
      ModelTpl<Scalar, Options, JointCollectionTpl> & model,
      PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) & contact_models,
      const bool verbose)
    {
      return buildModelFromXML(filename, rootJoint, model, contact_models, verbose);
    }

    template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
    ModelTpl<Scalar, Options, JointCollectionTpl> & buildModelFromXML(
      const std::string & xmlStream,
      const typename ModelTpl<Scalar, Options, JointCollectionTpl>::JointModel & rootJoint,
      ModelTpl<Scalar, Options, JointCollectionTpl> & model,
      PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) & contact_models,
      const bool verbose)
    {
      return buildModelFromXML(xmlStream, rootJoint, "root_joint", model, contact_models, verbose);
    }

    template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
    ModelTpl<Scalar, Options, JointCollectionTpl> & buildModel(
      const std::string & filename,
      const typename ModelTpl<Scalar, Options, JointCollectionTpl>::JointModel & rootJoint,
      const std::string & rootJointName,
      ModelTpl<Scalar, Options, JointCollectionTpl> & model,
      PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) & contact_models,
      const bool verbose)
    {
      return buildModelFromXML(filename, rootJoint, rootJointName, model, contact_models, verbose);
    }

    template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
    ModelTpl<Scalar, Options, JointCollectionTpl> & buildModelFromXML(
      const std::string & xmlStream,
      const typename ModelTpl<Scalar, Options, JointCollectionTpl>::JointModel & rootJoint,
      const std::string & rootJointName,
      ModelTpl<Scalar, Options, JointCollectionTpl> & model,
      PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) & contact_models,
      const bool verbose)
    {
      if (rootJointName.empty())
        throw std::invalid_argument(
          "rootJoint was given without a name. Please fill the argument rootJointName");

      ::pinocchio::urdf::details::UrdfVisitorWithRootJoint<Scalar, Options, JointCollectionTpl>
        visitor(model, rootJoint, rootJointName);

      typedef ::pinocchio::mjcf::details::MjcfGraph MjcfGraph;

      MjcfGraph graph(visitor, xmlStream);
      if (verbose)
        visitor.log = &std::cout;

      graph.parseGraphFromXML(xmlStream);

      // Use the Mjcf graph to create the model
      graph.parseRootTree();
      graph.parseContactInformation(model, contact_models);

      return model;
    }

  } // namespace mjcf
} // namespace pinocchio

#endif // __pinocchio_parsers_mjcf_model_hxx__
