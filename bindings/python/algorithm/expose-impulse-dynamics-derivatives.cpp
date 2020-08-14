//
// Copyright (c) 2020 CNRS INRIA
//

#include "pinocchio/bindings/python/algorithm/algorithms.hpp"
#include "pinocchio/bindings/python/algorithm/contact-info.hpp"
#include "pinocchio/bindings/python/algorithm/proximal.hpp"
#include "pinocchio/bindings/python/algorithm/contact-cholesky.hpp"

#include "pinocchio/bindings/python/utils/std-vector.hpp"

#include "pinocchio/algorithm/impulse-dynamics-derivatives.hpp"

namespace bp = boost::python;

namespace pinocchio
{
    namespace python
    {
    
      static void impulseDynamicsDerivatives_proxy(const Model & model,
                                                   Data & data,
                                                   const PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidContactModel) & contact_models,
                                                   PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidContactData) & contact_datas,
                                                   const double r_coeff =0.0,
                                                   const double mu = 0.0)
      {
        computeImpulseDynamicsDerivatives(model, data, contact_models,
                                                 contact_datas, r_coeff, mu);
        return;
      }
    
      BOOST_PYTHON_FUNCTION_OVERLOADS(impulseDynamicsDerivatives_overloads,
                                      impulseDynamicsDerivatives_proxy, 4, 6)
    
    
      void exposeImpulseDynamicsDerivatives()
      {
        using namespace Eigen;
        
        bp::def("computeImpulseDynamicsDerivatives",
                impulseDynamicsDerivatives_proxy,
                impulseDynamicsDerivatives_overloads(bp::args("model","data","contact_models","contact_datas","r_coeff","mu"),
                                          "Computes the impulse dynamics derivatives with contact constraints according to a given list of Contact information.\n"
                                          "impulseDynamics should have been called before."));
      }
    }
}

