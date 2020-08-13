//
// Copyright (c) 2019-2020 INRIA
//

#ifndef __pinocchio_python_algorithm_contact_info_hpp__
#define __pinocchio_python_algorithm_contact_info_hpp__

#include <eigenpy/memory.hpp>
#include "pinocchio/algorithm/contact-info.hpp"
#include "pinocchio/bindings/python/utils/macros.hpp"

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
      typedef RigidContactModel Self;
      typedef typename RigidContactModel::ContactData ContactData;

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
               bp::arg("reference_frame")),
              "Contructor from a given ContactType and frame parent index."))
        
        .PINOCCHIO_ADD_PROPERTY(Self,type,"Type of the contact.")
        .PINOCCHIO_ADD_PROPERTY(Self,frame_id,"Index of the parent Frame in the model tree.")
        .PINOCCHIO_ADD_PROPERTY(Self,reference_frame,"Reference frame where the constraint is expressed (WORLD, LOCAL_WORLD_ALIGNED or LOCAL).")
        .PINOCCHIO_ADD_PROPERTY(Self,desired_contact_placement,"Desired contact placement.")
        .PINOCCHIO_ADD_PROPERTY(Self,desired_contact_velocity,"Desired contact spatial velocity.")
        .PINOCCHIO_ADD_PROPERTY(Self,desired_contact_acceleration,"Desired contact spatial acceleration.")
        
        .def("size", &RigidContactModel::size, "Size of the contact")
        
        .def("createData",
             &RigidContactModelPythonVisitor::createData,
             "Create a Data object for the given model.")
        
        .def(bp::self == bp::self)
        .def(bp::self != bp::self)
        ;
      }
      
      static void expose()
      {
        bp::class_<RigidContactModel>("RigidContactModel",
                                      "Rigid contact model for contact dynamic algorithms.",
                                      bp::no_init)
        .def(RigidContactModelPythonVisitor())
        ;
        
      }
      
      static ContactData createData(const Self & self)
      {
        return ContactData(self);
      }
    };
  
    template<typename RigidContactData>
    struct RigidContactDataPythonVisitor
    : public boost::python::def_visitor< RigidContactDataPythonVisitor<RigidContactData> >
    {
      typedef typename RigidContactData::Scalar Scalar;
      typedef typename RigidContactData::SE3 SE3;
      typedef RigidContactData Self;
      typedef typename RigidContactData::ContactModel ContactModel;

    public:
      
      template<class PyClass>
      void visit(PyClass& cl) const
      {
        cl
        .def(bp::init<ContactModel>(bp::args("self","contact_model"),
                                    "Default constructor."))
        
        .PINOCCHIO_ADD_PROPERTY(Self,contact_force,
                                "Contact force.")
        .PINOCCHIO_ADD_PROPERTY(Self,contact_velocity,
                                "Current contact Spatial velocity.")
        .PINOCCHIO_ADD_PROPERTY(Self,contact_acceleration,
                                "Current contact Spatial acceleration.")
        .PINOCCHIO_ADD_PROPERTY(Self,contact_acceleration_drift,
                                "Current contact drift acceleration (acceleration only due to the Coriolis and centrifugal effects).")
        .PINOCCHIO_ADD_PROPERTY(Self,contact_acceleration_deviation,
                                "Contact deviation from the reference acceleration (a.k.a the error).")
        
        .def(bp::self == bp::self)
        .def(bp::self != bp::self)
        ;
      }
      
      static void expose()
      {
        bp::class_<RigidContactData>("RigidContactData",
                                     "Rigid contact data associated to a RigidContactModel for contact dynamic algorithms.",
                                     bp::no_init)
        .def(RigidContactDataPythonVisitor())
        ;
        
      }
    };
    
  } // namespace python
} // namespace pinocchio

#endif // ifndef __pinocchio_python_algorithm_contact_info_hpp__


