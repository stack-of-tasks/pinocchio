//
// Copyright (c) 2019-2020 INRIA
//

#include "pinocchio/bindings/python/algorithm/algorithms.hpp"
#include "pinocchio/bindings/python/utils/list.hpp"
#include "pinocchio/algorithm/model.hpp"

namespace pinocchio
{
  namespace python
  {

    namespace bp = boost::python;
  
    template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
    bp::tuple appendModel_proxy(const ModelTpl<Scalar,Options,JointCollectionTpl> & modelA,
                                const ModelTpl<Scalar,Options,JointCollectionTpl> & modelB,
                                const GeometryModel & geomModelA,
                                const GeometryModel & geomModelB,
                                const FrameIndex frameInModelA,
                                const SE3Tpl<Scalar,Options> & aMb)
    {
      typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
      Model model;
      GeometryModel geom_model;
      
      appendModel(modelA,modelB,geomModelA,geomModelB,frameInModelA,aMb,model,geom_model);
      
      return bp::make_tuple(model,geom_model);
    }
  
    template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType>
    bp::tuple
    buildReducedModel(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                      const GeometryModel & geom_model,
                      const std::vector<JointIndex> & list_of_joints_to_lock,
                      const Eigen::MatrixBase<ConfigVectorType> & reference_configuration)
    {
      typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
      Model reduced_model; GeometryModel reduced_geom_model;
      
      buildReducedModel(model,geom_model,list_of_joints_to_lock,
                        reference_configuration,reduced_model,reduced_geom_model);
      
      return bp::make_tuple(reduced_model,reduced_geom_model);
    }
  
    void exposeModelAlgo()
    {
      using namespace Eigen;
      
      bp::def("appendModel",
              (Model (*)(const Model &, const Model &, const FrameIndex, const SE3 &))&appendModel<double,0,JointCollectionDefaultTpl>,
              bp::args("modelA","modelB","frame_in_modelA","aMb"),
              "Append a child model into a parent model, after a specific frame given by its index.\n\n"
              "Parameters:\n"
              "\tmodelA: the parent model\n"
              "\tmodelB: the child model\n"
              "\tframeInModelA:  index of the frame of modelA where to append modelB\n"
              "\taMb: pose of modelB universe joint (index 0) in frameInModelA\n");
      
      bp::def("appendModel",
              &appendModel_proxy<double,0,JointCollectionDefaultTpl>,
              bp::args("modelA","modelB","frame_in_modelA","aMb"),
              "Append a child (geometry) model into a parent (geometry) model, after a specific frame given by its index.\n\n"
              "Parameters:\n"
              "\tmodelA: the parent model\n"
              "\tmodelB: the child model\n"
              "\tgeomModelA: the parent geometry model\n"
              "\tgeomModelB: the child geometry model\n"
              "\tframeInModelA:  index of the frame of modelA where to append modelB\n"
              "\taMb: pose of modelB universe joint (index 0) in frameInModelA\n");

      bp::def("buildReducedModel",
              (Model (*)(const Model &, const std::vector<JointIndex> &, const Eigen::MatrixBase<VectorXd> &))
              &pinocchio::buildReducedModel<double,0,JointCollectionDefaultTpl,VectorXd>,
              bp::args("model",
                       "list_of_joints_to_lock",
                       "reference_configuration"),
              "Build a reduce model from a given input model and a list of joint to lock.\n\n"
              "Parameters:\n"
              "\tmodel: input kinematic modell to reduce\n"
              "\tlist_of_joints_to_lock: list of joint indexes to lock\n"
              "\treference_configuration: reference configuration to compute the placement of the lock joints\n");

      bp::def("buildReducedModel",
              (bp::tuple (*)(const Model &, const GeometryModel &, const std::vector<JointIndex> &, const Eigen::MatrixBase<VectorXd> &))
              &buildReducedModel<double,0,JointCollectionDefaultTpl,VectorXd>,
              bp::args("model",
                       "geom_model",
                       "list_of_joints_to_lock",
                       "reference_configuration"),
              "Build a reduced model and a rededuced geometry model  from a given input model,"
              "a given input geometry model and a list of joint to lock.\n\n"
              "Parameters:\n"
              "\tmodel: input kinematic modell to reduce\n"
              "\tgeom_model: input geometry model to reduce\n"
              "\tlist_of_joints_to_lock: list of joint indexes to lock\n"
              "\treference_configuration: reference configuration to compute the placement of the lock joints\n");
      
    }
    
  } // namespace python
} // namespace pinocchio
