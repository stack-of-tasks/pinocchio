//
// Copyright (c) 2020-2021 CNRS INRIA
//

#include "pinocchio/bindings/python/algorithm/algorithms.hpp"
#include "pinocchio/bindings/python/utils/std-vector.hpp"
#include "pinocchio/algorithm/impulse-dynamics-derivatives.hpp"

namespace bp = boost::python;

namespace pinocchio
{
    namespace python
    {
    
      typedef PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(context::RigidContactModel) RigidContactModelVector;
      typedef PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(context::RigidContactData) RigidContactDataVector;
  
      static void impulseDynamicsDerivatives_proxy(const context::Model & model,
                                                   context::Data & data,
                                                   const RigidContactModelVector & contact_models,
                                                   RigidContactDataVector & contact_datas,
                                                   const context::Scalar & r_coeff = context::Scalar(0),
                                                   const context::Scalar & mu = context::Scalar(0))
      {
        computeImpulseDynamicsDerivatives(model, data, contact_models,
                                          contact_datas, r_coeff, mu);
        return;
      }
    
      void exposeImpulseDynamicsDerivatives()
      {
        bp::def("computeImpulseDynamicsDerivatives",impulseDynamicsDerivatives_proxy,
                (bp::arg("model"),bp::arg("data"),bp::arg("contact_models"),bp::arg("contact_datas"),bp::arg("r_coeff") = 0,bp::arg("mu") = 0),
                 "Computes the impulse dynamics derivatives with contact constraints according to a given list of Contact information.\n"
                 "impulseDynamics should have been called before.");
      }
    }
}

