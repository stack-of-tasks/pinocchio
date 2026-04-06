//
// Copyright (c) 2019-2024 INRIA
//

#include "pinocchio/bindings/python/algorithm/algorithms.hpp"
#include "pinocchio/bindings/python/utils/std-vector.hpp"
#include "pinocchio/algorithm/model.hpp"

namespace pinocchio
{
  namespace python
  {

    namespace bp = boost::python;

    template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
    bp::tuple appendModel_proxy(
      const ModelTpl<Scalar, Options, JointCollectionTpl> & modelA,
      const ModelTpl<Scalar, Options, JointCollectionTpl> & modelB,
      const GeometryModel & geomModelA,
      const GeometryModel & geomModelB,
      const FrameIndex frameInModelA,
      const SE3Tpl<Scalar, Options> & aMb)
    {
      typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
      Model model;
      GeometryModel geom_model;

      appendModel(modelA, modelB, geomModelA, geomModelB, frameInModelA, aMb, model, geom_model);

      return bp::make_tuple(model, geom_model);
    }

    template<
      typename Scalar,
      int Options,
      template<typename, int> class JointCollectionTpl,
      typename ConfigVectorType>
    bp::tuple buildReducedModel(
      const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
      const GeometryModel & geom_model,
      const std::vector<JointIndex> & list_of_joints_to_lock,
      const Eigen::MatrixBase<ConfigVectorType> & reference_configuration)
    {
      typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
      Model reduced_model;
      GeometryModel reduced_geom_model;

      buildReducedModel(
        model, geom_model, list_of_joints_to_lock, reference_configuration, reduced_model,
        reduced_geom_model);

      return bp::make_tuple(reduced_model, reduced_geom_model);
    }

    template<
      typename Scalar,
      int Options,
      template<typename, int> class JointCollectionTpl,
      typename ConfigVectorType>
    bp::tuple buildReducedModel(
      const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
      const std::vector<GeometryModel, Eigen::aligned_allocator<GeometryModel>> &
        list_of_geom_models,
      const std::vector<JointIndex> & list_of_joints_to_lock,
      const Eigen::MatrixBase<ConfigVectorType> & reference_configuration)
    {
      typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
      std::vector<GeometryModel, Eigen::aligned_allocator<GeometryModel>> reduced_geom_models;
      Model reduced_model;
      buildReducedModel(
        model, list_of_geom_models, list_of_joints_to_lock, reference_configuration, reduced_model,
        reduced_geom_models);
      return bp::make_tuple(reduced_model, reduced_geom_models);
    }

    bp::tuple findCommonAncestor_proxy(
      const context::Model & model, const JointIndex joint1_id, const JointIndex joint2_id)
    {
      size_t index_ancestor_in_support1, index_ancestor_in_support2;
      JointIndex ancestor_id = findCommonAncestor(
        model, joint1_id, joint2_id, index_ancestor_in_support1, index_ancestor_in_support2);
      return bp::make_tuple(ancestor_id, index_ancestor_in_support1, index_ancestor_in_support2);
    }

    template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
    ModelTpl<Scalar, Options, JointCollectionTpl> transformJointIntoMimic_proxy(
      const ModelTpl<Scalar, Options, JointCollectionTpl> & input_model,
      const JointIndex & index_mimicked,
      const JointIndex & index_mimicking,
      const Scalar & scaling,
      const Scalar & offset)
    {
      typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;

      Model model;
      pinocchio::transformJointIntoMimic(
        input_model, index_mimicked, index_mimicking, scaling, offset, model);
      return model;
    }

    void exposeModelAlgo()
    {
      using namespace Eigen;

      typedef std::vector<GeometryModel, Eigen::aligned_allocator<GeometryModel>>
        GeometryModelVector;
      StdVectorPythonVisitor<GeometryModelVector>::expose("StdVec_GeometryModel");

      bp::def(
        "appendModel",
        (Model (*)(
          const Model &, const Model &, const FrameIndex,
          const SE3 &))&appendModel<double, 0, JointCollectionDefaultTpl>,
        bp::args("modelA", "modelB", "frame_in_modelA", "aMb"),
        "Append a child model into a parent model, after a specific frame given by its index.\n\n"
        "Parameters:\n"
        "\tmodelA: the parent model\n"
        "\tmodelB: the child model\n"
        "\tframeInModelA:  index of the frame of modelA where to append modelB\n"
        "\taMb: pose of modelB universe joint (index 0) in frameInModelA\n");

      bp::def(
        "appendModel", &appendModel_proxy<double, 0, JointCollectionDefaultTpl>,
        bp::args("modelA", "modelB", "geomModelA", "geomModelB", "frame_in_modelA", "aMb"),
        "Append a child (geometry) model into a parent (geometry) model, after a specific "
        "frame given by its index.\n\n"
        "Parameters:\n"
        "\tmodelA: the parent model\n"
        "\tmodelB: the child model\n"
        "\tgeomModelA: the parent geometry model\n"
        "\tgeomModelB: the child geometry model\n"
        "\tframeInModelA:  index of the frame of modelA where to append modelB\n"
        "\taMb: pose of modelB universe joint (index 0) in frameInModelA\n");

      bp::def(
        "buildReducedModel",
        (Model (*)(
          const Model &, const std::vector<JointIndex> &,
          const Eigen::MatrixBase<VectorXd> &))&pinocchio::
          buildReducedModel<double, 0, JointCollectionDefaultTpl, VectorXd>,
        bp::args("model", "list_of_joints_to_lock", "reference_configuration"),
        "Build a reduce model from a given input model and a list of joint to lock.\n\n"
        "Parameters:\n"
        "\tmodel: input kinematic modell to reduce\n"
        "\tlist_of_joints_to_lock: list of joint indexes to lock\n"
        "\treference_configuration: reference configuration to compute the placement of the "
        "lock joints\n");

      bp::def(
        "buildReducedModel",
        (bp::tuple (*)(
          const Model &, const GeometryModel &, const std::vector<JointIndex> &,
          const Eigen::MatrixBase<
            VectorXd> &))&buildReducedModel<double, 0, JointCollectionDefaultTpl, VectorXd>,
        bp::args("model", "geom_model", "list_of_joints_to_lock", "reference_configuration"),
        "Build a reduced model and a reduced geometry model from a given input model,"
        "an input geometry model and a list of joints to lock.\n\n"
        "Parameters:\n"
        "\tmodel: input kinematic model to reduce\n"
        "\tgeom_model: input geometry model to reduce\n"
        "\tlist_of_joints_to_lock: list of joint indexes to lock\n"
        "\treference_configuration: reference configuration to compute the placement of the "
        "locked joints\n");

      bp::def(
        "buildReducedModel",
        (bp::tuple (*)(
          const Model &,
          const std::vector<GeometryModel, Eigen::aligned_allocator<GeometryModel>> &,
          const std::vector<JointIndex> &, const Eigen::MatrixBase<VectorXd> &))
          buildReducedModel<double, 0, JointCollectionDefaultTpl, VectorXd>,
        bp::args(
          "model", "list_of_geom_models", "list_of_joints_to_lock", "reference_configuration"),
        "Build a reduced model and the related reduced geometry models from a given "
        "input model,"
        "a list of input geometry models and a list of joints to lock.\n\n"
        "Parameters:\n"
        "\tmodel: input kinematic model to reduce\n"
        "\tlist_of_geom_models: input geometry models to reduce\n"
        "\tlist_of_joints_to_lock: list of joint indexes to lock\n"
        "\treference_configuration: reference configuration to compute the "
        "placement of the locked joints\n");

      bp::def(
        "findCommonAncestor", findCommonAncestor_proxy, bp::args("model", "joint1_id", "joint2_id"),
        "Computes the common ancestor between two joints belonging to the same kinematic tree.\n\n"
        "Parameters:\n"
        "\tmodel: input model\n"
        "\tjoint1_id: index of the first joint\n"
        "\tjoint2_id: index of the second joint\n"
        "Returns a tuple containing the index of the common joint ancestor, the position of this "
        "ancestor in model.supports[joint1_id] and model.supports[joint2_id].\n");

      bp::def(
        "transformJointIntoMimic",
        transformJointIntoMimic_proxy<double, 0, JointCollectionDefaultTpl>,
        bp::args("input_model", "index_mimicked", "index_mimicking", "scaling", "offset"),
        "Transform of a joint of the model into a mimic joint. Keep the type of the joint as it "
        "was previously. \n\n"
        "Parameters:\n"
        "\tinput_model: model the input model to take joints from.\n"
        "\tindex_mimicked: index of the joint to mimic\n"
        "\tindex_mimicking: index of the joint that will mimic\n"
        "\tscaling: Scaling of joint velocity and configuration\n"
        "\toffset: Offset of joint configuration\n");
    }

  } // namespace python
} // namespace pinocchio
