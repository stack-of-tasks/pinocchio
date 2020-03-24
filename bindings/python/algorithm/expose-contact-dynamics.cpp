//
// Copyright (c) 2020 INRIA
//

#include "pinocchio/bindings/python/algorithm/algorithms.hpp"
#include "pinocchio/bindings/python/algorithm/contact-info.hpp"
#include "pinocchio/bindings/python/algorithm/proximal.hpp"
#include "pinocchio/bindings/python/algorithm/contact-cholesky.hpp"

#include "pinocchio/bindings/python/utils/std-vector.hpp"

#include "pinocchio/algorithm/contact-dynamics.hpp"

namespace bp = boost::python;

namespace pinocchio
{
    namespace python
    {
    
      static const Eigen::VectorXd contactDynamics_proxy(const Model & model,
                                                         Data & data,
                                                         const Eigen::VectorXd & q,
                                                         const Eigen::VectorXd & v,
                                                         const Eigen::VectorXd & tau,
                                                         const PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidContactModel) & contact_models,
                                                         PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidContactData) & contact_datas,
                                                         const double mu = 0.0)
      {
        return contactDynamics(model, data, q, v, tau, contact_models, contact_datas, mu);
      }
    
      BOOST_PYTHON_FUNCTION_OVERLOADS(contactDynamics_overloads, contactDynamics_proxy, 7, 8)
    
    
      void exposeContactDynamics()
      {
        using namespace Eigen;
        
        // Expose type of contacts
        bp::enum_<ContactType>("ContactType")
        .value("CONTACT_3D",CONTACT_3D)
        .value("CONTACT_6D",CONTACT_6D)
        .value("CONTACT_UNDEFINED",CONTACT_UNDEFINED)
        ;
        
        ContactCholeskyDecompositionPythonVisitor<cholesky::ContactCholeskyDecomposition>::expose();
        
        RigidContactModelPythonVisitor<RigidContactModel>::expose();
        RigidContactDataPythonVisitor<RigidContactData>::expose();
        
        typedef Eigen::aligned_allocator<RigidContactModel> RigidContactModelAllocator;
        StdVectorPythonVisitor<RigidContactModel,RigidContactModelAllocator>::expose("StdVec_RigidContactModel");
        
        typedef Eigen::aligned_allocator<RigidContactData> RigidContactDataAllocator;
        StdVectorPythonVisitor<RigidContactData,RigidContactDataAllocator>::expose("StdVec_RigidContactData");
        
        ProximalSettingsPythonVisitor<ProximalSettings>::expose();
        
        bp::def("initContactDynamics",
                &initContactDynamics<double,0,JointCollectionDefaultTpl,RigidContactModelAllocator>,
                bp::args("model","data","contact_models"),
                "This function allows to allocate the memory before hand for contact dynamics algorithms.\n"
                "This allows to avoid online memory allocation when running these algorithms.");
        
        bp::def("contactDynamics",
                contactDynamics_proxy,
                contactDynamics_overloads(bp::args("model","data","q","v","tau","contact_models","contact_datas","mu"),
                                          "Computes the forward dynamics with contact constraints according to a given list of Contact information.\n"
                                          "When using contactDynamics for the first time, you should call first initContactDynamics to initialize the internal memory used in the algorithm.\n"
                                          "This function returns joint acceleration of the system. The contact forces are stored in the list data.contact_forces."));
      }
    }
}

