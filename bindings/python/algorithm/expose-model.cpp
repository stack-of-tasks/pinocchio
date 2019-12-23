//
// Copyright (c) 2019 INRIA
//

#include "pinocchio/bindings/python/algorithm/algorithms.hpp"
#include "pinocchio/bindings/python/utils/list.hpp"
#include "pinocchio/algorithm/model.hpp"

namespace pinocchio
{
  namespace python
  {

    namespace bp = boost::python;

    template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType>
    ModelTpl<Scalar,Options,JointCollectionTpl>
    buildReducedModel_list(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                           const bp::list & list_of_joints_to_lock,
                           const Eigen::MatrixBase<ConfigVectorType> & reference_configuration)
    {
      std::vector<JointIndex> list_of_joints_to_lock_vec = extract<JointIndex>(list_of_joints_to_lock);
      return buildReducedModel(model,list_of_joints_to_lock_vec,reference_configuration);
    }
  
    template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
    bp::tuple appendModel(const ModelTpl<Scalar,Options,JointCollectionTpl> & modelA,
                          const ModelTpl<Scalar,Options,JointCollectionTpl> & modelB,
                          const GeometryModel & geomModelA,
                          const GeometryModel & geomModelB,
                          const FrameIndex frameInModelA,
                          const SE3Tpl<Scalar,Options> & aMb)
    {
      typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
      Model model; GeometryModel geom_model;
      
      appendModel(modelA,modelB,geomModelA,geomModelB,frameInModelA,aMb,model,geom_model);
      
      return bp::make_tuple(model,geom_model);
    }
     
    void exposeModelAlgo()
    {
      using namespace Eigen;
      
      bp::def("appendModel",
              (Model (*)(const Model &, const Model &, const FrameIndex, const SE3 &))&appendModel<double,0,JointCollectionDefaultTpl>,
              bp::args("modelA","modelB","frame_in_modelA","aMb"),
              "Append a child model into a parent model, after a specific frame given by its index.\n\n"
              " modelA: the parent model\n"
              " modelB: the child model\n"
              " frameInModelA:  index of the frame of modelA where to append modelB\n"
              " aMb: pose of modelB universe joint (index 0) in frameInModelA\n");
      
      bp::def("appendModel",
              (bp::tuple (*)(const Model &, const Model &, const GeometryModel &, const GeometryModel &, const FrameIndex, const SE3 &))&appendModel<double,0,JointCollectionDefaultTpl>,
              bp::args("modelA","modelB","frame_in_modelA","aMb"),
              "Append a child (geometry) model into a parent (geometry) model, after a specific frame given by its index.\n\n"
              " modelA: the parent model\n"
              " modelB: the child model\n"
              " geomModelA: the parent geometry model\n"
              " geomModelB: the child geometry model\n"
              " frameInModelA:  index of the frame of modelA where to append modelB\n"
              " aMb: pose of modelB universe joint (index 0) in frameInModelA\n");

      bp::def("buildReducedModel",
              buildReducedModel_list<double,0,JointCollectionDefaultTpl,VectorXd>,
              bp::args("model: input kinematic model",
                       "list_of_joints_to_lock: list of joint indexes to lock",
                       "reference_configuration: reference configuration to compute the placement of the lock joints"),
              "Build a reduce model from a given input model and a list of joint to lock.");

      bp::def("buildReducedModel",
              (Model (*)(const Model &, const std::vector<JointIndex> &, const Eigen::MatrixBase<VectorXd> &))&buildReducedModel<double,0,JointCollectionDefaultTpl,VectorXd>,
              bp::args("model: input kinematic model",
                       "list_of_joints_to_lock: list of joint indexes to lock",
                       "reference_configuration: reference configuration to compute the placement of the lock joints"),
              "Build a reduce model from a given input model and a list of joint to lock.");
      
    }
    
  } // namespace python
} // namespace pinocchio
