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
                                                        const context::ProximalSettings & settings = context::ProximalSettings())
      {
        pinocchio::computeContactDynamicsDerivatives(model, data,
                                                     contact_models, contact_datas,
                                                     const_cast<context::ProximalSettings &>(settings));
        
        return bp::make_tuple(make_ref(data.ddq_dq),
                              make_ref(data.ddq_dv),
                              make_ref(data.ddq_dtau),
                              make_ref(data.dlambda_dq),
                              make_ref(data.dlambda_dv),
                              make_ref(data.dlambda_dtau));
      }
    
    
      void exposeContactDynamicsDerivatives()
      {
        using namespace Eigen;

        typedef Eigen::aligned_allocator<context::RigidContactModel> RigidContactModelAllocator;
        
        bp::def("computeContactDynamicsDerivatives",
                computeContactDynamicsDerivatives_proxy,
                (bp::arg("model"),bp::arg("data"),bp::arg("contact_models"),bp::arg("contact_datas"),bp::arg("settings") = context::ProximalSettings()),
                "Computes the derivatives of the forward dynamics with kinematic constraints (given in the list of constraint models).\n"
                "Assumes that contactDynamics has been called first. See contactDynamics for more details.\n"
                "This function returns the derivatives of joint acceleration (ddq) and contact forces (lambda_c) of the system with respect to q, v and tau.\n"
                "The output is a tuple with ddq_dq, ddq_dv, ddq_da, dlambda_dq, dlambda_dv, dlambda_da.");
      }
    }
}

