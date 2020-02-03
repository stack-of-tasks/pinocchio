//
// Copyright (c) 2020 INRIA
//

#ifndef __pinocchio_python_algorithm_contact_cholesky_hpp__
#define __pinocchio_python_algorithm_contact_cholesky_hpp__

#include <eigenpy/memory.hpp>
#include "pinocchio/algorithm/contact-cholesky.hpp"

#include "pinocchio/bindings/python/utils/macros.hpp"

EIGENPY_DEFINE_STRUCT_ALLOCATOR_SPECIALIZATION(pinocchio::cholesky::ContactCholeskyDecomposition)

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
      typedef typename ContactCholeskyDecomposition::ContactInfo ContactInfo;
      typedef typename ContactCholeskyDecomposition::Matrix Matrix;
      typedef typename PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(ContactInfo) ContactInfoVector;

    public:
      
      template<class PyClass>
      void visit(PyClass& cl) const
      {
        cl
        .def(bp::init<>("Default constructor."))
        .def(bp::init<Model>(bp::arg("model"),"Constructor from a model."))
        .def(bp::init<Model,ContactInfoVector>((bp::arg("model"),bp::arg("contact_infos")),
                                               "Constructor from a model and a collection of ContactInfo objects."))
        
        .PINOCCHIO_ADD_PROPERTY_READONLY_BYVALUE(Self,U,"")
        .PINOCCHIO_ADD_PROPERTY_READONLY_BYVALUE(Self,D,"")
        .PINOCCHIO_ADD_PROPERTY_READONLY_BYVALUE(Self,Dinv,"")
        
        .def("size", &Self::size, bp::arg("self"), "Size of the decomposition.")
        .def("constraintDim", &Self::constraintDim, bp::arg("self"),
             "Returns the total dimension of the constraints contained in the Cholesky factorization.")
        .def("numContacts", &Self::numContacts, bp::arg("self"),
             "Returns the number of contacts associated to this decomposition.")
        
        .def("matrix",
             (Matrix (Self::*)(void) const)&Self::matrix,
             bp::arg("self"),
             "Returns the matrix resulting from the decomposition.")
        
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
        
        .def(bp::self == bp::self)
        .def(bp::self != bp::self)
        ;
      }
      
      static void expose()
      {
        bp::class_<ContactCholeskyDecomposition>("ContactCholeskyDecomposition",
                                                 "Contact information container for contact dynamic algorithms.")
        .def(ContactCholeskyDecompositionPythonVisitor<ContactCholeskyDecomposition>())
        ;
        
      }
      
      template<typename MatrixType>
      static Matrix solve(const Self & self, const MatrixType & mat)
      {
        return self.solve(mat);
      }
    };
    
  } // namespace python
} // namespace pinocchio

#endif // ifndef __pinocchio_python_algorithm_contact_cholesky_hpp__



