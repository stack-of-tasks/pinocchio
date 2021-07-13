//
// Copyright (c) 2020-2021 INRIA
//

#include "pinocchio/bindings/python/algorithm/algorithms.hpp"
#include "pinocchio/bindings/python/algorithm/contact-info.hpp"
#include "pinocchio/bindings/python/algorithm/proximal.hpp"
#include "pinocchio/bindings/python/algorithm/contact-cholesky.hpp"

#include "pinocchio/bindings/python/utils/std-vector.hpp"
#include "pinocchio/bindings/python/utils/registration.hpp"

#include "pinocchio/algorithm/constrained-dynamics.hpp"

namespace bp = boost::python;

namespace pinocchio
{
    namespace python
    {
      typedef PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(context::RigidConstraintModel) RigidConstraintModelVector;
      typedef PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(context::RigidConstraintData) RigidConstraintDataVector;
    
      static const context::VectorXs constraintDynamics_proxy(const context::Model & model,
                                                           context::Data & data,
                                                           const context::VectorXs & q,
                                                           const context::VectorXs & v,
                                                           const context::VectorXs & tau,
                                                           const RigidConstraintModelVector & contact_models,
                                                           RigidConstraintDataVector & contact_datas,
                                                           context::ProximalSettings & prox_settings)
      {
        return constraintDynamics(model, data, q, v, tau, contact_models, contact_datas, prox_settings);
      }
    
      static const context::VectorXs constraintDynamics_proxy_default(const context::Model & model,
                                                                   context::Data & data,
                                                                   const context::VectorXs & q,
                                                                   const context::VectorXs & v,
                                                                   const context::VectorXs & tau,
                                                                   const RigidConstraintModelVector & contact_models,
                                                                   RigidConstraintDataVector & contact_datas)
    {
        return constraintDynamics(model, data, q, v, tau, contact_models, contact_datas);
    }
    
    
      void exposeConstraintDynamics()
      {
        using namespace Eigen;
        
        // Expose type of contacts
        if(!register_symbolic_link_to_registered_type<ContactType>())
        {
          bp::enum_<ContactType>("ContactType")
          .value("CONTACT_3D",CONTACT_3D)
          .value("CONTACT_6D",CONTACT_6D)
          .value("CONTACT_UNDEFINED",CONTACT_UNDEFINED)
          ;
        }
        
        ContactCholeskyDecompositionPythonVisitor<context::ContactCholeskyDecomposition>::expose();
        
        RigidConstraintModelPythonVisitor<context::RigidConstraintModel>::expose();
        RigidConstraintDataPythonVisitor<context::RigidConstraintData>::expose();
        
        typedef Eigen::aligned_allocator<context::RigidConstraintModel> RigidConstraintModelAllocator;
        StdVectorPythonVisitor<context::RigidConstraintModel,RigidConstraintModelAllocator>::expose("StdVec_RigidConstraintModel");
        
        typedef Eigen::aligned_allocator<context::RigidConstraintData> RigidConstraintDataAllocator;
        StdVectorPythonVisitor<context::RigidConstraintData,RigidConstraintDataAllocator>::expose("StdVec_RigidConstraintData");
        
        ProximalSettingsPythonVisitor<context::ProximalSettings>::expose();
        
        bp::def("initConstraintDynamics",
                &initConstraintDynamics<context::Scalar,context::Options,JointCollectionDefaultTpl,RigidConstraintModelAllocator>,
                bp::args("model","data","contact_models"),
                "This function allows to allocate the memory before hand for contact dynamics algorithms.\n"
                "This allows to avoid online memory allocation when running these algorithms.");
        
        bp::def("constraintDynamics",
                constraintDynamics_proxy,
                bp::args("model","data","q","v","tau","contact_models","contact_datas","prox_settings"),
                "Computes the forward dynamics with contact constraints according to a given list of Contact information.\n"
                "When using constraintDynamics for the first time, you should call first initConstraintDynamics to initialize the internal memory used in the algorithm.\n"
                "This function returns joint acceleration of the system. The contact forces are stored in the list data.contact_forces.");
        
        bp::def("constraintDynamics",
                constraintDynamics_proxy_default,
                bp::args("model","data","q","v","tau","contact_models","contact_datas"),
                "Computes the forward dynamics with contact constraints according to a given list of Contact information.\n"
                "When using constraintDynamics for the first time, you should call first initConstraintDynamics to initialize the internal memory used in the algorithm.\n"
                "This function returns joint acceleration of the system. The contact forces are stored in the list data.contact_forces.");
      }
    }
}

