//
// Copyright (c) 2021 INRIA
//

#include "pinocchio/bindings/python/algorithm/algorithms.hpp"
#include "pinocchio/algorithm/contact-dynamics-derivatives.hpp"
#include "pinocchio/bindings/python/utils/eigen.hpp"

namespace bp = boost::python;

namespace pinocchio
{
    namespace python
    {
      typedef PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(context::RigidContactModel) RigidContactModelVector;
      typedef PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(context::RigidContactData) RigidContactDataVector;

      bp::tuple computeContactDynamicsDerivatives_proxy(const context::Model & model,
                                                        context::Data & data,
                                                        const RigidContactModelVector & contact_models,
                                                        RigidContactDataVector & contact_datas,
                                                        const context::Scalar mu = 0.0)
      {
        pinocchio::computeContactDynamicsDerivatives(model, data,
                                                     contact_models, contact_datas, mu);
        return bp::make_tuple(make_ref(data.ddq_dq),
                              make_ref(data.ddq_dv),
                              make_ref(data.ddq_dtau),
                              make_ref(data.dlambda_dq),
                              make_ref(data.dlambda_dv),
                              make_ref(data.dlambda_dtau));
      }
      
      BOOST_PYTHON_FUNCTION_OVERLOADS(computeContactDynamicsDerivatives_overloads,
                                      computeContactDynamicsDerivatives_proxy, 4, 5)
    
    
      void exposeContactDynamicsDerivatives()
      {
        using namespace Eigen;

        typedef Eigen::aligned_allocator<context::RigidContactModel> RigidContactModelAllocator;
        
        bp::def("computeContactDynamicsDerivatives",
                computeContactDynamicsDerivatives_proxy,
                computeContactDynamicsDerivatives_overloads(bp::args("model","data",
                                                                     "contact_models",
                                                                     "contact_datas","mu"),
                                                            "Computes the derivatives of the forward dynamics with kinematic constraints (given in the list of Contact information).\n"
                                                            "Assumes that contactDynamics has been called first. See contactDynamics for more details.\n"
                                          "This function returns derivatives of joint acceleration (ddq) and contact forces (lambda_c) of the system.\n"
                                                            "The output is a tuple with ddq_dq, ddq_dv, ddq_da, dlambda_dq, dlambda_dv, dlambda_da"));
      }
    }
}

