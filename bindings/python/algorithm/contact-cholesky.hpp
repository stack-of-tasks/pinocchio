//
// Copyright (c) 2020 INRIA
//

#ifndef __pinocchio_python_algorithm_contact_cholesky_hpp__
#define __pinocchio_python_algorithm_contact_cholesky_hpp__

#include <eigenpy/memory.hpp>
#include "pinocchio/algorithm/contact-cholesky.hpp"

#include "pinocchio/bindings/python/utils/macros.hpp"
#include "pinocchio/bindings/python/utils/std-vector.hpp"
#include "pinocchio/bindings/python/utils/comparable.hpp"

EIGENPY_DEFINE_STRUCT_ALLOCATOR_SPECIALIZATION(pinocchio::python::context::ContactCholeskyDecomposition)

namespace pinocchio
{
  namespace python
  {
    namespace bp = boost::python;
    
    template<typename ContactCholeskyDecomposition>
    struct ContactCholeskyDecompositionPythonVisitor
    : public boost::python::def_visitor< ContactCholeskyDecompositionPythonVisitor<ContactCholeskyDecomposition> >
    {
      typedef ContactCholeskyDecomposition Self;
      typedef typename ContactCholeskyDecomposition::Scalar Scalar;
      typedef typename ContactCholeskyDecomposition::RigidConstraintModel RigidConstraintModel;
      typedef typename ContactCholeskyDecomposition::RigidConstraintData RigidConstraintData;
      typedef typename ContactCholeskyDecomposition::Matrix Matrix;
      typedef typename PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) RigidConstraintModelVector;
      typedef typename PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintData) RigidConstraintDataVector;
      
      typedef pinocchio::python::context::Model Model;
      typedef pinocchio::python::context::Data Data;

    public:
      
      template<class PyClass>
      void visit(PyClass& cl) const
      {
        cl
        .def(bp::init<>(bp::arg("self"),"Default constructor."))
        .def(bp::init<Model>(bp::args("self","model"),"Constructor from a model."))
        .def(bp::init<Model,RigidConstraintModelVector>((bp::arg("self"),bp::arg("model"),bp::arg("contact_models")),
                                                     "Constructor from a model and a collection of RigidConstraintModels."))
        
        .PINOCCHIO_ADD_PROPERTY_READONLY_BYVALUE(Self,U,"")
        .PINOCCHIO_ADD_PROPERTY_READONLY_BYVALUE(Self,D,"")
        .PINOCCHIO_ADD_PROPERTY_READONLY_BYVALUE(Self,Dinv,"")
        
        .def("size", &Self::size, bp::arg("self"), "Size of the decomposition.")
        .def("constraintDim", &Self::constraintDim, bp::arg("self"),
             "Returns the total dimension of the constraints contained in the Cholesky factorization.")
        .def("numContacts", &Self::numContacts, bp::arg("self"),
             "Returns the number of contacts associated to this decomposition.")
	.def("getConstraintSparsityPattern", &Self::getConstraintSparsityPattern,
	     bp::args("self", "constraint_id"),
	     "Returns the associated sparsity of the constraints.",
	     bp::return_value_policy<bp::return_by_value>())
	.def("getLoopSparsityPattern", &Self::getLoopSparsityPattern,
	     bp::args("self", "constraint_id"),
	     "Returns the sparsity of the loop constraints (indexes that connect c1 with c2)",
	     bp::return_value_policy<bp::return_by_value>())	  
	.def("getJoint1SparsityPattern", &Self::getJoint1SparsityPattern,
	     bp::args("self", "constraint_id"),
	     "Returns the associated sparsity introduced because of first joint",
	     bp::return_value_policy<bp::return_by_value>())
	.def("getJoint2SparsityPattern", &Self::getJoint2SparsityPattern,
	     bp::args("self", "constraint_id"),
	     "Returns the associated sparsity introduces because of second joint.",
	     bp::return_value_policy<bp::return_by_value>())	  
        .def("matrix",
             (Matrix (Self::*)(void) const)&Self::matrix,
             bp::arg("self"),
             "Returns the matrix resulting from the decomposition.")
        .def("compute",
             (void (*)(Self & self, const Model &, Data &, const RigidConstraintModelVector &, RigidConstraintDataVector &, const Scalar))&compute,
             bp::args("self","model","data","contact_models","contact_datas","mu"),
             "Computes the Cholesky decompostion of the augmented matrix containing the KKT matrix\n"
             "related to the system mass matrix and the Jacobians of the contact patches contained in\n"
             "the vector of RigidConstraintModel named contact_models. The decomposition is regularized with a factor mu.\n")
        
        .def("compute",
             (void (*)(Self & self, const Model &, Data &, const RigidConstraintModelVector &, RigidConstraintDataVector &))&compute,
             bp::args("self","model","data","contact_models","contact_datas"),
             "Computes the Cholesky decompostion of the augmented matrix containing the KKT matrix\n"
             "related to the system mass matrix and the Jacobians of the contact patches contained in\n"
             "the vector of RigidConstraintModel named contact_models.")
        
        .def("getInverseOperationalSpaceInertiaMatrix",
             (Matrix (Self::*)(void) const)&Self::getInverseOperationalSpaceInertiaMatrix,
             bp::arg("self"),
             "Returns the Inverse of the Operational Space Inertia Matrix resulting from the decomposition.",
             bp::return_value_policy<bp::return_by_value>())
        
        .def("getOperationalSpaceInertiaMatrix",
             (Matrix (Self::*)(void) const)&Self::getOperationalSpaceInertiaMatrix,
             bp::arg("self"),
             "Returns the Operational Space Inertia Matrix resulting from the decomposition.",
             bp::return_value_policy<bp::return_by_value>())
        
        .def("solve",
             &solve<Matrix>,
             bp::args("self","matrix"),
             "Computes the solution of \f$ A x = b \f$ where self corresponds to the Cholesky decomposition of A.",
             bp::return_value_policy<bp::return_by_value>())
        
        .def("inverse",
             (Matrix (Self::*)(void) const)&Self::inverse,
             bp::arg("self"),
             "Returns the inverse matrix resulting from the decomposition.")
        
        .def("getMassMatrixChoeslkyDecomposition",
             &Self::template getMassMatrixChoeslkyDecomposition<Scalar,0,JointCollectionDefaultTpl>,
             bp::arg("self"),
             "Retrieves the Cholesky decomposition of the Mass Matrix contained in the current decomposition.")
        
        .def(ComparableVisitor<Self,pinocchio::is_floating_point<Scalar>::value>())
        ;
      }
      
      static void expose()
      {
        bp::class_<ContactCholeskyDecomposition>("ContactCholeskyDecomposition",
                                                 "Contact information container for contact dynamic algorithms.",
                                                 bp::no_init)
        .def(ContactCholeskyDecompositionPythonVisitor<ContactCholeskyDecomposition>())
        ;
        
      }
      
      template<typename MatrixType>
      static Matrix solve(const Self & self, const MatrixType & mat)
      {
        return self.solve(mat);
      }
      
      static void compute(Self & self, const Model & model, Data & data, const RigidConstraintModelVector & contact_models, RigidConstraintDataVector & contact_datas, const Scalar mu)
      {
        self.compute(model,data,contact_models,contact_datas,mu);
      }
      
      static void compute(Self & self, const Model & model, Data & data, const RigidConstraintModelVector & contact_models, RigidConstraintDataVector & contact_datas)
      {
        self.compute(model,data,contact_models,contact_datas);
      }
    };
    
  } // namespace python
} // namespace pinocchio

#endif // ifndef __pinocchio_python_algorithm_contact_cholesky_hpp__
