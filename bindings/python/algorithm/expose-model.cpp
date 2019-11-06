//
// Copyright (c) 2019 INRIA
//

#include "pinocchio/bindings/python/algorithm/algorithms.hpp"
#include "pinocchio/algorithm/model.hpp"
#include "pinocchio/algorithm/aba.hpp"

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
      std::vector<JointIndex> list_of_joints_to_lock_vec;
      list_of_joints_to_lock_vec.reserve((size_t)bp::len(list_of_joints_to_lock));
      for(int k = 0; k < bp::len(list_of_joints_to_lock); ++k)
      {
        JointIndex joint_id = bp::extract<JointIndex>(list_of_joints_to_lock[k]);
        list_of_joints_to_lock_vec.push_back(joint_id);
      }
      
      return buildReducedModel(model,list_of_joints_to_lock_vec,reference_configuration);
    }
     
    void exposeModelAlgo()
    {
      using namespace Eigen;

      bp::def("buildReducedModel",
              buildReducedModel_list<double,0,JointCollectionDefaultTpl,VectorXd>,
              bp::args("model: input kinematic model",
                       "list_of_joints_to_lock: list of joint indexes to lock",
                       "reference_configuration: reference configuration to compute the placement of the lock joints"),
              "Build a reduce model from a given input model and a list of joint to lock.");

      bp::def("buildReducedModel",
              &buildReducedModel<double,0,JointCollectionDefaultTpl,VectorXd>,
              bp::args("model: input kinematic model",
                       "list_of_joints_to_lock: list of joint indexes to lock",
                       "reference_configuration: reference configuration to compute the placement of the lock joints"),
              "Build a reduce model from a given input model and a list of joint to lock.");
      
    }
    
  } // namespace python
} // namespace pinocchio
