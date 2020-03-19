//
// Copyright (c) 2019-2020 INRIA
//

#ifndef __pinocchio_python_algorithm_contact_info_hpp__
#define __pinocchio_python_algorithm_contact_info_hpp__

#include <eigenpy/memory.hpp>
#include "pinocchio/algorithm/contact-info.hpp"

EIGENPY_DEFINE_STRUCT_ALLOCATOR_SPECIALIZATION(pinocchio::RigidContactModel)

namespace pinocchio
{
  namespace python
  {
    namespace bp = boost::python;
    
    template<typename RigidContactModel>
    struct RigidContactModelPythonVisitor
    : public boost::python::def_visitor< RigidContactModelPythonVisitor<RigidContactModel> >
    {
      typedef typename RigidContactModel::Scalar Scalar;
      typedef typename RigidContactModel::SE3 SE3;

    public:
      
      template<class PyClass>
      void visit(PyClass& cl) const
      {
        cl
        .def(bp::init<>(bp::arg("self"),
                        "Default constructor."))
        .def(bp::init<ContactType,FrameIndex,SE3,bp::optional<ReferenceFrame> >
             ((bp::arg("self"),
               bp::arg("contact_type"),
               bp::arg("frame_id"),
               bp::arg("placement"),
               bp::arg("reference_frame")),
              "Contructor from a given ContactType, frame parent index and placement with respect to the parent Frame."))
        
        .add_property("type",&RigidContactModel::type,"Type of the contact.")
        .add_property("frame_id",&RigidContactModel::frame_id,"Index of the parent Frame in the model tree.")
        .add_property("placement",&RigidContactModel::placement,"Placement of the contact with respect to the parent Frame.")
        .add_property("reference_frame",&RigidContactModel::reference_frame,"Reference frame where the constraint is expressed (WORLD, LOCAL_WORLD_ALIGNED or LOCAL).")
        .add_property("desired_contact_placement",&RigidContactModel::desired_contact_placement,"Desired contact placement.")
        .add_property("desired_contact_velocity",&RigidContactModel::desired_contact_velocity,"Desired contact spatial velocity.")
        .add_property("desired_contact_acceleration",&RigidContactModel::desired_contact_acceleration,"Desired contact spatial acceleration.")
        
        .def("size", &RigidContactModel::size, "Size of the contact")
        
        .def(bp::self == bp::self)
        .def(bp::self != bp::self)
        ;
      }
      
      static void expose()
      {
        bp::class_<RigidContactModel>("RigidContactModel",
                                      "Rigid contact model for contact dynamic algorithms.",
                                      bp::no_init)
        .def(RigidContactModelPythonVisitor<RigidContactModel>())
        ;
        
      }
    };
    
  } // namespace python
} // namespace pinocchio

#endif // ifndef __pinocchio_python_algorithm_contact_info_hpp__


