//
// Copyright (c) 2021 INRIA
//

#include "pinocchio/bindings/python/algorithm/algorithms.hpp"
#include "pinocchio/algorithm/contact-jacobian.hpp"

#include "pinocchio/bindings/python/utils/std-vector.hpp"

namespace bp = boost::python;

namespace pinocchio
{
  namespace python
  {
  
    typedef PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(context::RigidContactModel) RigidContactModelVector;
    typedef PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(context::RigidContactData) RigidContactDataVector;
    
    static context::MatrixXs getConstraintJacobian_proxy(const context::Model & model,
                                                         const context::Data & data,
                                                         const context::RigidContactModel & contact_model,
                                                         context::RigidContactData & contact_data)
    {
      context::MatrixXs J(contact_model.size(),model.nv); J.setZero();
      getConstraintJacobian(model, data, contact_model, contact_data, J);
      return J;
    }
  
    static context::MatrixXs getConstraintsJacobian_proxy(const context::Model & model,
                                                          const context::Data & data,
                                                          const RigidContactModelVector & contact_models,
                                                          RigidContactDataVector & contact_datas)
    {
      const Eigen::DenseIndex constraint_size = getTotalConstraintSize(contact_models);
      context::MatrixXs J(constraint_size,model.nv); J.setZero();
      getConstraintsJacobian(model, data, contact_models, contact_datas, J);
      return J;
    }

    void exposeContactJacobian()
    {
      bp::def("getConstraintJacobian",
              getConstraintJacobian_proxy,
              bp::args("model","data","contact_model","contact_data"),
              "Computes the kinematic Jacobian associatied to a given constraint model.");
      bp::def("getConstraintsJacobian",
              getConstraintsJacobian_proxy,
              bp::args("model","data","contact_models","contact_datas"),
              "Computes the kinematic Jacobian associatied to a given set of constraint models.");
    }
  }
}
