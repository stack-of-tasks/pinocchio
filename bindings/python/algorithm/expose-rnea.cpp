//
// Copyright (c) 2015-2020 CNRS INRIA
//

#include "pinocchio/bindings/python/algorithm/algorithms.hpp"
#include "pinocchio/bindings/python/utils/list.hpp"
#include "pinocchio/algorithm/rnea.hpp"

namespace pinocchio
{
  namespace python
  {
  
    void exposeRNEA()
    {
      using namespace Eigen;
      
      bp::def("rnea",
              &rnea<double,0,JointCollectionDefaultTpl,VectorXd,VectorXd,VectorXd>,
              bp::args("model","Data","q","v","a"),
              "Compute the RNEA, store the result in Data and return it.\n\n"
              "Parameters:\n"
              "\tmodel: model of the kinematic tree\n"
              "\tdata: data related to the model\n"
              "\tq: the joint configuration vector (size model.nq)\n"
              "\tv: the joint velocity vector (size model.nv)\n"
              "\ta: the joint acceleration vector (size model.nv)\n",
              bp::return_value_policy<bp::return_by_value>());

      bp::def("rnea",
              &rnea<double,0,JointCollectionDefaultTpl,VectorXd,VectorXd,VectorXd,Force>,
              bp::args("model","Data","q","v","a","fext"),
              "Compute the RNEA with external forces, store the result in Data and return it.\n\n"
              "Parameters:\n"
              "\tmodel: model of the kinematic tree\n"
              "\tdata: data related to the model\n"
              "\tq: the joint configuration vector (size model.nq)\n"
              "\tv: the joint velocity vector (size model.nv)\n"
              "\ta: the joint acceleration vector (size model.nv)\n"
              "\tfext: list of external forces expressed in the local frame of the joints (size model.njoints)\n",
              bp::return_value_policy<bp::return_by_value>());

      bp::def("nonLinearEffects",
              &nonLinearEffects<double,0,JointCollectionDefaultTpl,VectorXd,VectorXd>,
              bp::args("model","Data","q","v"),
              "Compute the Non Linear Effects (coriolis, centrifugal and gravitational effects), store the result in Data and return it.\n\n"
              "Parameters:\n"
              "\tmodel: model of the kinematic tree\n"
              "\tdata: data related to the model\n"
              "\tq: the joint configuration vector (size model.nq)\n"
              "\tv: the joint velocity vector (size model.nv)\n",
              bp::return_value_policy<bp::return_by_value>());

      bp::def("computeGeneralizedGravity",
              &computeGeneralizedGravity<double,0,JointCollectionDefaultTpl,VectorXd>,
              bp::args("model","Data","q"),
              "Compute the generalized gravity contribution g(q) of the Lagrangian dynamics, store the result in data.g and return it.\n\n"
              "Parameters:\n"
              "\tmodel: model of the kinematic tree\n"
              "\tdata: data related to the model\n"
              "\tq: the joint configuration vector (size model.nq)\n",
              bp::return_value_policy<bp::return_by_value>());

      bp::def("computeStaticTorque",
              &computeStaticTorque<double,0,JointCollectionDefaultTpl,VectorXd>,
              bp::args("model","Data","q","fext"),
              "Computes the generalized static torque contribution g(q) - J.T f_ext of the Lagrangian dynamics, store the result in data.tau and return it.\n\n"
              "Parameters:\n"
              "\tmodel: model of the kinematic tree\n"
              "\tdata: data related to the model\n"
              "\tq: the joint configuration vector (size model.nq)\n"
              "\tfext: list of external forces expressed in the local frame of the joints (size model.njoints)\n",
              bp::return_value_policy<bp::return_by_value>());

      bp::def("computeCoriolisMatrix",
              &computeCoriolisMatrix<double,0,JointCollectionDefaultTpl,VectorXd,VectorXd>,
              bp::args("model","Data","q","v"),
              "Compute the Coriolis Matrix C(q,v) of the Lagrangian dynamics, store the result in data.C and return it.\n\n"
              "Parameters:\n"
              "\tmodel: model of the kinematic tree\n"
              "\tdata: data related to the model\n"
              "\tq: the joint configuration vector (size model.nq)\n"
              "\tv: the joint velocity vector (size model.nv)\n",
              bp::return_value_policy<bp::return_by_value>());

      bp::def("getCoriolisMatrix",
              &getCoriolisMatrix<double,0,JointCollectionDefaultTpl>,
              bp::args("model","Data"),
              "Retrives the Coriolis Matrix C(q,v) of the Lagrangian dynamics after calling one of the derivative algorithms, store the result in data.C and return it.\n\n"
              "Parameters:\n"
              "\tmodel: model of the kinematic tree\n"
              "\tdata: data related to the model\n",
              bp::return_value_policy<bp::return_by_value>());
      
    }
    
  } // namespace python
} // namespace pinocchio
