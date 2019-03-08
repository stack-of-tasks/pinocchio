//
// Copyright (c) 2019 INRIA
//

#ifndef __pinocchio_python_algorithm_contact_info_hpp__
#define __pinocchio_python_algorithm_contact_info_hpp__

#include <eigenpy/memory.hpp>
#include "pinocchio/algorithm/contact-info.hpp"

EIGENPY_DEFINE_STRUCT_ALLOCATOR_SPECIALIZATION(pinocchio::ContactInfo)

namespace pinocchio
{
  namespace python
  {
    namespace bp = boost::python;
    
    template<typename ContactInfo>
    struct ContactInfoPythonVisitor
    : public boost::python::def_visitor< ContactInfoPythonVisitor<ContactInfo> >
    {
      typedef typename ContactInfo::Scalar Scalar;
      typedef typename ContactInfo::SE3 SE3;

    public:
      
      template<class PyClass>
      void visit(PyClass& cl) const
      {
        cl
        .def(bp::init<>("Default constructor."))
        .def(bp::init<ContactType,FrameIndex,SE3>
             ((bp::arg("Type of the contact"),
               bp::arg("Index of the parent Frame in the model tree"),
               bp::arg("Placement of the contact with respect to the parent Frame")),
              "Contructor with from a given type, parent and placement."))
        
        .add_property("type",&ContactInfo::type,"Type of the contact.")
        .add_property("parent",&ContactInfo::parent,"Index of the parent Frame in the model tree.")
        .add_property("placement",&ContactInfo::placement,"Placement of the contact with respect to the parent Frame.")
        
        .def("dim", &ContactInfo::dim)
        
        .def(bp::self == bp::self)
        .def(bp::self != bp::self)
        ;
      }
      
      static void expose()
      {
        bp::class_<ContactInfo>("ContactInfo",
                                "Contact information container for contact dynamic algorithms.")
        .def(ContactInfoPythonVisitor<ContactInfo>())
        ;
        
      }
    };
    
  } // namespace python
} // namespace pinocchio

#endif // ifndef __pinocchio_python_algorithm_contact_info_hpp__


