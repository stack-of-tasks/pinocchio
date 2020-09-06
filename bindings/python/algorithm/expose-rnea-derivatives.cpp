//
// Copyright (c) 2018-2020 CNRS INRIA
//

#include "pinocchio/bindings/python/algorithm/algorithms.hpp"
#include "pinocchio/algorithm/rnea-derivatives.hpp"
#include "pinocchio/bindings/python/utils/eigen.hpp"

namespace pinocchio
{
  namespace python
  {
    
    namespace bp = boost::python;
    typedef PINOCCHIO_ALIGNED_STD_VECTOR(Force) ForceAlignedVector;
    
    Data::MatrixXs computeGeneralizedGravityDerivatives(const Model & model, Data & data,
                                                        const Eigen::VectorXd & q)
    {
      Data::MatrixXs res(model.nv,model.nv);
      res.setZero();
      pinocchio::computeGeneralizedGravityDerivatives(model,data,q,res);
      return res;
    }
    
    Data::MatrixXs computeStaticTorqueDerivatives(const Model & model, Data & data,
                                                  const Eigen::VectorXd & q,
                                                  const ForceAlignedVector & fext)
    {
      Data::MatrixXs res(model.nv,model.nv);
      res.setZero();
      pinocchio::computeStaticTorqueDerivatives(model,data,q,fext,res);
      return res;
    }
    
    bp::tuple computeRNEADerivatives(const Model & model, Data & data,
                                     const Eigen::VectorXd & q,
                                     const Eigen::VectorXd & v,
                                     const Eigen::VectorXd & a)
    {
      pinocchio::computeRNEADerivatives(model,data,q,v,a);
      make_symmetric(data.M);
      return bp::make_tuple(make_ref(data.dtau_dq),
                            make_ref(data.dtau_dv),
                            make_ref(data.M));
    }
    
    bp::tuple computeRNEADerivatives_fext(const Model & model, Data & data,
                                          const Eigen::VectorXd & q,
                                          const Eigen::VectorXd & v,
                                          const Eigen::VectorXd & a,
                                          const ForceAlignedVector & fext)
    {
      pinocchio::computeRNEADerivatives(model,data,q,v,a,fext);
      make_symmetric(data.M);
      return bp::make_tuple(make_ref(data.dtau_dq),
                            make_ref(data.dtau_dv),
                            make_ref(data.M));
    }
    
    void exposeRNEADerivatives()
    {
      using namespace Eigen;
      
      bp::def("computeGeneralizedGravityDerivatives",
              computeGeneralizedGravityDerivatives,
              bp::args("model","data","q"),
              "Computes the partial derivative of the generalized gravity contribution\n"
              "with respect to the joint configuration.\n\n"
              "Parameters:\n"
              "\tmodel: model of the kinematic tree\n"
              "\tdata: data related to the model\n"
              "\tq: the joint configuration vector (size model.nq)\n");
              
      bp::def("computeStaticTorqueDerivatives",
              computeStaticTorqueDerivatives,
              bp::args("model","data","q","fext"),
              "Computes the partial derivative of the generalized gravity and external forces contributions (a.k.a static torque vector)\n"
              "with respect to the joint configuration.\n\n"
              "Parameters:\n"
              "\tmodel: model of the kinematic tree\n"
              "\tdata: data related to the model\n"
              "\tq: the joint configuration vector (size model.nq)\n"
              "\tfext: list of external forces expressed in the local frame of the joints (size model.njoints)\n");
      
      bp::def("computeRNEADerivatives",
              computeRNEADerivatives,
              bp::args("model","data","q","v","a"),
              "Computes the RNEA partial derivatives, store the result in data.dtau_dq, data.dtau_dv and data.dtau_da\n"
              "which correspond to the partial derivatives of the torque output with respect to the joint configuration,\n"
              "velocity and acceleration vectors.\n\n"
              "Parameters:\n"
              "\tmodel: model of the kinematic tree\n"
              "\tdata: data related to the model\n"
              "\tq: the joint configuration vector (size model.nq)\n"
              "\tv: the joint velocity vector (size model.nv)\n"
              "\ta: the joint acceleration vector (size model.nv)\n");
      
      bp::def("computeRNEADerivatives",
              computeRNEADerivatives_fext,
              bp::args("model","data","q","v","a","fext"),
              "Computes the RNEA partial derivatives with external contact foces,\n"
              "store the result in data.dtau_dq, data.dtau_dv and data.dtau_da\n"
              "which correspond to the partial derivatives of the torque output with respect to the joint configuration,\n"
              "velocity and acceleration vectors.\n\n"
              "Parameters:\n"
              "\tmodel: model of the kinematic tree\n"
              "\tdata: data related to the model\n"
              "\tq: the joint configuration vector (size model.nq)\n"
              "\tv: the joint velocity vector (size model.nv)\n"
              "\ta: the joint acceleration vector (size model.nv)\n"
              "\tfext: list of external forces expressed in the local frame of the joints (size model.njoints)\n");
    }
    
    
    
  } // namespace python
} // namespace pinocchio
