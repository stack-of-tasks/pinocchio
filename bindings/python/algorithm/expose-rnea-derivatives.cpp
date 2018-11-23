//
// Copyright (c) 2018 CNRS INRIA
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
              "Computes the derivative of the generalized gravity contribution\n"
              "with respect to the joint configuration.");
      
      bp::def("computeRNEADerivatives",
              computeRNEADerivatives,
              bp::args("Model","Data",
                       "q: configuration vector (size model.nq)",
                       "v: velocity vector (size model.nv)",
                       "a: acceleration vector (size model.nv)"),
              "Computes the RNEA derivatives, put the result in data.dtau_dq, data.dtau_dv and data.dtau_da\n"
              "which correspond to the partial derivatives of the torque output with respect to the joint configuration,\n"
              "velocity and acceleration vectors.");
      
      bp::def("computeRNEADerivatives",
              computeRNEADerivatives_fext,
              bp::args("Model","Data",
                       "q: configuration vector (size model.nq)",
                       "v: velocity vector (size model.nv)",
                       "a: acceleration vector (size model.nv)",
                       "fext: vector external forces (size model.njoints)"),
              "Computes the RNEA derivatives with external contact foces,\n"
              "put the result in data.dtau_dq, data.dtau_dv and data.dtau_da\n"
              "which correspond to the partial derivatives of the torque output with respect to the joint configuration,\n"
              "velocity and acceleration vectors.");
    }
    
    
    
  } // namespace python
} // namespace pinocchio
