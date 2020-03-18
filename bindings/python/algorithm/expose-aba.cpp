//
// Copyright (c) 2015-2020 CNRS INRIA
//

#include "pinocchio/bindings/python/algorithm/algorithms.hpp"
#include "pinocchio/bindings/python/utils/namespace.hpp"

#include "pinocchio/algorithm/aba.hpp"

namespace pinocchio
{
  namespace python
  {
    
    const Data::RowMatrixXs &
    computeMinverse_proxy(const Model & model, Data & data, const Eigen::VectorXd & q)
    {
      computeMinverse(model,data,q);
      data.Minv.triangularView<Eigen::StrictlyLower>() =
      data.Minv.transpose().triangularView<Eigen::StrictlyLower>();
      return data.Minv;
    }
  
    namespace optimized
    {
      const Data::RowMatrixXs &
      computeMinverse_proxy(const Model & model, Data & data)
      {
        pinocchio::optimized::computeMinverse(model,data);
        data.Minv.triangularView<Eigen::StrictlyLower>() =
        data.Minv.transpose().triangularView<Eigen::StrictlyLower>();
        return data.Minv;
      }
    }
    
    void exposeABA()
    {
      using namespace Eigen;

      bp::def("aba",
              &aba<double,0,JointCollectionDefaultTpl,VectorXd,VectorXd,VectorXd>,
              bp::args("model","data","q","v","tau"),
              "Compute ABA, store the result in data.ddq and return it.\n"
              "Parameters:\n"
              "\t model: Model of the kinematic tree\n"
              "\t data: Data related to the kinematic tree\n"
              "\t q: joint configuration (size model.nq)\n"
              "\t tau: joint velocity (size model.nv)\n"
              "\t v: joint torque (size model.nv)",
              bp::return_value_policy<bp::return_by_value>());

      bp::def("aba",
              &aba<double,0,JointCollectionDefaultTpl,VectorXd,VectorXd,VectorXd,Force>,
              bp::args("model","data","q","v","tau","fext"),
              "Compute ABA with external forces, store the result in data.ddq and return it.\n"
              "Parameters:\n"
              "\t model: Model of the kinematic tree\n"
              "\t data: Data related to the kinematic tree\n"
              "\t q: joint configuration (size model.nq)\n"
              "\t v: joint velocity (size model.nv)\n"
              "\t tau: joint torque (size model.nv)\n"
              "\t fext: vector of external forces expressed in the local frame of the joint (size model.njoints)",
              bp::return_value_policy<bp::return_by_value>());
      
      bp::def("computeMinverse",
              &computeMinverse_proxy,
              bp::args("model","data","q"),
              "Computes the inverse of the joint space inertia matrix using an extension of the Articulated Body algorithm.\n"
              "The result is stored in data.Minv.\n"
              "Parameters:\n"
              "\t model: Model of the kinematic tree\n"
              "\t data: Data related to the kinematic tree\n"
              "\t q: joint configuration (size model.nq)",
              bp::return_value_policy<bp::return_by_value>());
      
      {
        bp::scope current_scope = getOrCreatePythonNamespace("optimized");
        
        bp::def("aba",
                &pinocchio::optimized::aba<double,0,JointCollectionDefaultTpl,VectorXd,VectorXd,VectorXd>,
                bp::args("model","data","q","v","tau"),
                "Compute ABA, store the result in data.ddq and return it.\n"
                "Parameters:\n"
                "\t model: Model of the kinematic tree\n"
                "\t data: Data related to the kinematic tree\n"
                "\t q: joint configuration (size model.nq)\n"
                "\t tau: joint velocity (size model.nv)\n"
                "\t v: joint torque (size model.nv)",
                bp::return_value_policy<bp::return_by_value>());

        bp::def("aba",
                &pinocchio::optimized::aba<double,0,JointCollectionDefaultTpl,VectorXd,VectorXd,VectorXd,Force>,
                bp::args("model","data","q","v","tau","fext"),
                "Compute ABA with external forces, store the result in data.ddq and return it.\n"
                "Parameters:\n"
                "\t model: Model of the kinematic tree\n"
                "\t data: Data related to the kinematic tree\n"
                "\t q: joint configuration (size model.nq)\n"
                "\t v: joint velocity (size model.nv)\n"
                "\t tau: joint torque (size model.nv)\n"
                "\t fext: vector of external forces expressed in the local frame of the joint (size model.njoints)",
                bp::return_value_policy<bp::return_by_value>());
        
        bp::def("computeMinverse",
                &optimized::computeMinverse_proxy,
                bp::args("model","data"),
                "Computes the inverse of the joint space inertia matrix using an extension of the Articulated Body algorithm.\n"
                "The result is stored in data.Minv.\n"
                "Remarks: pinocchio.optimized.aba should have been called first.\n"
                "Parameters:\n"
                "\t model: Model of the kinematic tree\n"
                "\t data: Data related to the kinematic tree",
                bp::return_value_policy<bp::return_by_value>());
      }
      
    }
    
  } // namespace python
} // namespace pinocchio
