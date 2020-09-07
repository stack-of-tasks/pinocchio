//
// Copyright (c) 2019 CNRS
//

#include "pinocchio/bindings/python/algorithm/algorithms.hpp"
#include "pinocchio/algorithm/centroidal-derivatives.hpp"

#include <boost/python/tuple.hpp>

namespace pinocchio
{
  namespace python
  {
    
    bp::tuple computeCentroidalDynamicsDerivatives_proxy(const Model & model,
                                                         Data & data,
                                                         const Eigen::VectorXd & q,
                                                         const Eigen::VectorXd & v,
                                                         const Eigen::VectorXd & a)
    {
      typedef Data::Matrix6x Matrix6x;
      Matrix6x partialh_dq(Matrix6x::Zero(6,model.nv));
      Matrix6x partial_dq(Matrix6x::Zero(6,model.nv));
      Matrix6x partial_dv(Matrix6x::Zero(6,model.nv));
      Matrix6x partial_da(Matrix6x::Zero(6,model.nv));
      
      computeCentroidalDynamicsDerivatives(model,data,q, v, a,
                                           partialh_dq, partial_dq, partial_dv, partial_da);
      
      return bp::make_tuple(partialh_dq, partial_dq,partial_dv,partial_da);
    }
    
    bp::tuple getCentroidalDynamicsDerivatives_proxy(const Model & model,
                                                     Data & data)
    {
      typedef Data::Matrix6x Matrix6x;

      Matrix6x partialh_dq(Matrix6x::Zero(6,model.nv));
      Matrix6x partial_dq(Matrix6x::Zero(6,model.nv));
      Matrix6x partial_dv(Matrix6x::Zero(6,model.nv));
      Matrix6x partial_da(Matrix6x::Zero(6,model.nv));

      getCentroidalDynamicsDerivatives(model,data, partialh_dq, partial_dq, partial_dv, partial_da);
      return bp::make_tuple(partialh_dq,partial_dq,partial_dv,partial_da);
    }

    void exposeCentroidalDerivatives()
    {
      using namespace Eigen;
      
      bp::def("computeCentroidalDynamicsDerivatives",
              computeCentroidalDynamicsDerivatives_proxy,
              bp::args("Model","Data",
                       "q: configuration vector (size model.nq)",
                       "v: velocity vector (size model.nv)",
                       "a: acceleration vector (size model.nv)"),
              "Computes the analytical derivatives of the centroidal dynamics\n"
              "with respect to the joint configuration vector, velocity and acceleration.");
      
      bp::def("getCentroidalDynamicsDerivatives",
              getCentroidalDynamicsDerivatives_proxy,
              bp::args("Model","Data"),
              "Retrive the analytical derivatives of the centroidal dynamics\n"
              "from the RNEA derivatives.\n"
              "pinocchio.computeRNEADerivatives should have been called first."); 
    }
    
  } // namespace python
} // namespace pinocchio
