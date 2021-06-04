//
// Copyright (c) 2020-2021 INRIA
//

#include "pinocchio/bindings/python/algorithm/algorithms.hpp"
#include "pinocchio/bindings/python/algorithm/contact-info.hpp"
#include "pinocchio/bindings/python/algorithm/proximal.hpp"
#include "pinocchio/bindings/python/algorithm/contact-cholesky.hpp"

#include "pinocchio/bindings/python/utils/std-vector.hpp"
#include "pinocchio/bindings/python/utils/registration.hpp"

#include "pinocchio/algorithm/contact-dynamics.hpp"

namespace bp = boost::python;

namespace pinocchio
{
    namespace python
    {
      typedef PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(context::RigidContactModel) RigidContactModelVector;
      typedef PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(context::RigidContactData) RigidContactDataVector;
    
      static const context::VectorXs contactDynamics_proxy(const context::Model & model,
                                                           context::Data & data,
                                                           const context::VectorXs & q,
                                                           const context::VectorXs & v,
                                                           const context::VectorXs & tau,
                                                           const RigidContactModelVector & contact_models,
                                                           RigidContactDataVector & contact_datas,
                                                           context::ProximalSettings & prox_settings)
      {
        return contactDynamics(model, data, q, v, tau, contact_models, contact_datas, prox_settings);
      }
    
      static const context::VectorXs contactDynamics_proxy_default(const context::Model & model,
                                                                   context::Data & data,
                                                                   const context::VectorXs & q,
                                                                   const context::VectorXs & v,
                                                                   const context::VectorXs & tau,
                                                                   const RigidContactModelVector & contact_models,
                                                                   RigidContactDataVector & contact_datas)
    {
        return contactDynamics(model, data, q, v, tau, contact_models, contact_datas);
      }
    
    
      void exposeContactDynamics()
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
        
        RigidContactModelPythonVisitor<context::RigidContactModel>::expose();
        RigidContactDataPythonVisitor<context::RigidContactData>::expose();
        
        typedef Eigen::aligned_allocator<context::RigidContactModel> RigidContactModelAllocator;
        StdVectorPythonVisitor<context::RigidContactModel,RigidContactModelAllocator>::expose("StdVec_RigidContactModel");
        
        typedef Eigen::aligned_allocator<context::RigidContactData> RigidContactDataAllocator;
        StdVectorPythonVisitor<context::RigidContactData,RigidContactDataAllocator>::expose("StdVec_RigidContactData");
        
        ProximalSettingsPythonVisitor<context::ProximalSettings>::expose();
        
        bp::def("initContactDynamics",
                &initContactDynamics<context::Scalar,context::Options,JointCollectionDefaultTpl,RigidContactModelAllocator>,
                bp::args("model","data","contact_models"),
                "This function allows to allocate the memory before hand for contact dynamics algorithms.\n"
                "This allows to avoid online memory allocation when running these algorithms.");
        
        bp::def("contactDynamics",
                contactDynamics_proxy,
                bp::args("model","data","q","v","tau","contact_models","contact_datas","prox_settings"),
                "Computes the forward dynamics with contact constraints according to a given list of Contact information.\n"
                "When using contactDynamics for the first time, you should call first initContactDynamics to initialize the internal memory used in the algorithm.\n"
                "This function returns joint acceleration of the system. The contact forces are stored in the list data.contact_forces.");
        
        bp::def("contactDynamics",
                contactDynamics_proxy_default,
                bp::args("model","data","q","v","tau","contact_models","contact_datas"),
                "Computes the forward dynamics with contact constraints according to a given list of Contact information.\n"
                "When using contactDynamics for the first time, you should call first initContactDynamics to initialize the internal memory used in the algorithm.\n"
                "This function returns joint acceleration of the system. The contact forces are stored in the list data.contact_forces.");
      }
    }
}

