//
// Copyright (c) 2018-2019 CNRS INRIA
//

#include "pinocchio/bindings/python/algorithm/algorithms.hpp"
#include "pinocchio/algorithm/rnea-derivatives.hpp"

namespace pinocchio
{
  namespace python
  {
    
    typedef container::aligned_vector<Force> ForceAlignedVector;
    
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
    
    void computeRNEADerivatives(const Model & model, Data & data,
                                      const Eigen::VectorXd & q,
                                      const Eigen::VectorXd & v,
                                      const Eigen::VectorXd & a)
    {
      pinocchio::computeRNEADerivatives(model,data,q,v,a);
      // Symmetrize M
      data.M.triangularView<Eigen::StrictlyLower>()
      = data.M.transpose().triangularView<Eigen::StrictlyLower>();
    }
    
    void computeRNEADerivatives_fext(const Model & model, Data & data,
                                     const Eigen::VectorXd & q,
                                     const Eigen::VectorXd & v,
                                     const Eigen::VectorXd & a,
                                     const ForceAlignedVector & fext)
    {
      pinocchio::computeRNEADerivatives(model,data,q,v,a,fext);
      // Symmetrize M
      data.M.triangularView<Eigen::StrictlyLower>()
      = data.M.transpose().triangularView<Eigen::StrictlyLower>();
    }
    
    void exposeRNEADerivatives()
    {
      using namespace Eigen;
      
      bp::def("computeGeneralizedGravityDerivatives",
              computeGeneralizedGravityDerivatives,
              bp::args("Model","Data",
                       "q: configuration vector (size model.nq)"),
              "Computes the partial derivative of the generalized gravity contribution\n"
              "with respect to the joint configuration.");
              
      bp::def("computeStaticTorqueDerivatives",
              computeStaticTorqueDerivatives,
              bp::args("Model: model of the kinematic tree",
                       "Data: data of the kinematic tree",
                       "q: configuration vector (size model.nq)",
                       "fext: vector of external forces expressed in the local frame of the joints (size model.njoints)"),
              "Computes the partial derivative of the generalized gravity and external forces contributions (a.k.a static torque vector)\n"
              "with respect to the joint configuration.");
      
      bp::def("computeRNEADerivatives",
              computeRNEADerivatives,
              bp::args("Model","Data",
                       "q: configuration vector (size model.nq)",
                       "v: velocity vector (size model.nv)",
                       "a: acceleration vector (size model.nv)"),
              "Computes the RNEA partial derivatives, put the result in data.dtau_dq, data.dtau_dv and data.dtau_da\n"
              "which correspond to the partial derivatives of the torque output with respect to the joint configuration,\n"
              "velocity and acceleration vectors.");
      
      bp::def("computeRNEADerivatives",
              computeRNEADerivatives_fext,
              bp::args("Model","Data",
                       "q: configuration vector (size model.nq)",
                       "v: velocity vector (size model.nv)",
                       "a: acceleration vector (size model.nv)",
                       "fext: vector of external forces expressed in the local frame of the joints (size model.njoints)"),
              "Computes the RNEA partial derivatives with external contact foces,\n"
              "put the result in data.dtau_dq, data.dtau_dv and data.dtau_da\n"
              "which correspond to the partial derivatives of the torque output with respect to the joint configuration,\n"
              "velocity and acceleration vectors.");
    }
    
    
    
  } // namespace python
} // namespace pinocchio
