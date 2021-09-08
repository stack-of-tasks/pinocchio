//
// Copyright (c) 2021 INRIA
//

#include "pinocchio/bindings/python/algorithm/algorithms.hpp"
#include "pinocchio/algorithm/constrained-dynamics-derivatives.hpp"
#include "pinocchio/bindings/python/utils/eigen.hpp"

namespace bp = boost::python;

namespace pinocchio
{
    namespace python
    {
      typedef PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(context::RigidConstraintModel) RigidConstraintModelVector;
      typedef PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(context::RigidConstraintData) RigidConstraintDataVector;

      bp::tuple computeConstraintDynamicsDerivatives_proxy(const context::Model & model,
                                                        context::Data & data,
                                                        const RigidConstraintModelVector & contact_models,
                                                        RigidConstraintDataVector & contact_datas)
      {
        pinocchio::computeConstraintDynamicsDerivatives(model, data,
                                                        contact_models, contact_datas);
        return bp::make_tuple(make_ref(data.ddq_dq),
                              make_ref(data.ddq_dv),
                              make_ref(data.ddq_dtau),
                              make_ref(data.dlambda_dq),
                              make_ref(data.dlambda_dv),
                              make_ref(data.dlambda_dtau));
      }
    

      bp::tuple computeConstraintDynamicsDerivatives_mu_proxy(const context::Model & model,
                                                              context::Data & data,
                                                              const RigidConstraintModelVector & contact_models,
                                                              RigidConstraintDataVector & contact_datas,
                                                              const context::ProximalSettings & prox_settings
                                                           )
      {
        pinocchio::computeConstraintDynamicsDerivatives(model, data,
                                                        contact_models, contact_datas, prox_settings);
        return bp::make_tuple(make_ref(data.ddq_dq),
                              make_ref(data.ddq_dv),
                              make_ref(data.ddq_dtau),
                              make_ref(data.dlambda_dq),
                              make_ref(data.dlambda_dv),
                              make_ref(data.dlambda_dtau));
      }


      
      void exposeConstraintDynamicsDerivatives()
      {
        using namespace Eigen;

        typedef Eigen::aligned_allocator<context::RigidConstraintModel> RigidConstraintModelAllocator;
        
        bp::def("computeConstraintDynamicsDerivatives",
                computeConstraintDynamicsDerivatives_proxy,bp::args("model","data",
                                                                     "contact_models",
                                                                     "contact_datas"),
                "Computes the derivatives of the forward dynamics with kinematic constraints (given in the list of Contact information).\n"
                "Assumes that constraintDynamics has been called first. See constraintDynamics for more details.\n"
                "This function returns derivatives of joint acceleration (ddq) and contact forces (lambda_c) of the system.\n"
		"The output is a tuple with ddq_dq, ddq_dv, ddq_da, dlambda_dq, dlambda_dv, dlambda_da");
        bp::def("computeConstraintDynamicsDerivatives",
                computeConstraintDynamicsDerivatives_mu_proxy,bp::args("model","data",
                                                                       "contact_models",
                                                                       "contact_datas",
                                                                       "prox_settings"),
                "Computes the derivatives of the forward dynamics with kinematic constraints (given in the list of Contact information).\n"
                "Assumes that constraintDynamics has been called first. See constraintDynamics for more details.\n"
                "This function returns derivatives of joint acceleration (ddq) and contact forces (lambda_c) of the system.\n"
		"The output is a tuple with ddq_dq, ddq_dv, ddq_da, dlambda_dq, dlambda_dv, dlambda_da");        
      }
    }
}

