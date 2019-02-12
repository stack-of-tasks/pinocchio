//
// Copyright (c) 2017 CNRS
//

#ifndef __pinocchio_python_fcl_contact_hpp__
#define __pinocchio_python_fcl_contact_hpp__

#include "pinocchio/spatial/fcl-pinocchio-conversions.hpp"
#include <hpp/fcl/collision_data.h>

#include <boost/python.hpp>
#include <boost/python/copy_const_reference.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>

namespace pinocchio
{
  namespace python
  {
    namespace fcl
    {
    
      namespace bp = boost::python;
      
      struct ContactPythonVisitor : public bp::def_visitor<ContactPythonVisitor>
      {
        typedef ::hpp::fcl::Contact Contact;
        typedef ::hpp::fcl::CollisionGeometry CollisionGeometry;
        
        template<class PyClass>
        void visit(PyClass& cl) const
        {
          cl
          .def(bp::init<>("Default constructor"))
          // TODO: add a kind of bp::with_custodian_and_ward_postcall<0,1>() for managing arguments lifetime
          .def("__init__",bp::make_constructor(&make_contact,
                                               bp::default_call_policies(),
                                               bp::args("o1","o2","b1","b2")),
               "Constructor from two Collision Geometry objects")
          
          .def_readwrite("b1", &Contact::b1, "Contact primitive in object 1.\n"
                         "if object 1 is mesh or point cloud, it is the triangle or point id\n"
                         "if object 1 is geometry shape, it is NONE (-1),\n"
                         "if object 1 is octree, it is the id of the cell.")
          
          .def_readwrite("b2", &Contact::b2, "Contact primitive in object 2.\n"
                         "if object 2 is mesh or point cloud, it is the triangle or point id\n"
                         "if object 2 is geometry shape, it is NONE (-1),\n"
                         "if object 2 is octree, it is the id of the cell.")
          .add_property("normal", &getNormal, &setNormal,
                        "Contact normal, pointing from o1 to o2.")
          .add_property("pos", &getPos, &setPos,
                        "Contact position, in world space.")
          
          .def_readwrite("penetration_depth",&Contact::penetration_depth,"Penetration depth.")
          
          .add_property("o1",bp::make_function(&getObject1,bp::return_value_policy<bp::copy_const_reference>()),"Returns a copy of collision object 1")
          .add_property("o2",bp::make_function(&getObject2,bp::return_value_policy<bp::copy_const_reference>()),"Returns a copy of collision object 2.")
          ;
        }
        
        static void expose()
        {
          bp::class_<Contact>("Contact",
                              "Contact information returned by collision.",
                              bp::no_init)
          .def(ContactPythonVisitor())
          ;
        }
        
      private:
        
        static Contact * make_contact(const CollisionGeometry & o1, const CollisionGeometry & o2, int b1, int b2)
        { return new Contact(&o1,&o2,b1,b2); }
        
        static Eigen::Vector3d getNormal(const Contact & self)
        { return self.normal; }
        static void setNormal(Contact & self, const Eigen::Vector3d & normal)
        { self.normal = normal; }
        
        static Eigen::Vector3d getPos(const Contact & self)
        { return self.pos; }
        static void setPos(Contact & self, const Eigen::Vector3d & pos)
        { self.pos = pos; }
        
        static const CollisionGeometry & getObject1(const Contact & self)
        { return *self.o1; }
        static const CollisionGeometry & getObject2(const Contact & self)
        { return *self.o2; }
      };
      
    } // namespace fcl
    
  } // namespace python
} // namespace pinocchio

#endif // namespace __pinocchio_python_fcl_contact_hpp__
