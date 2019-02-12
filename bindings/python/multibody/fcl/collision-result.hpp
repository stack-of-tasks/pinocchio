//
// Copyright (c) 2017 CNRS
//

#ifndef __pinocchio_python_fcl_collision_result_hpp__
#define __pinocchio_python_fcl_collision_result_hpp__

#include "pinocchio/spatial/fcl-pinocchio-conversions.hpp"
#include <hpp/fcl/collision_data.h>

#include <boost/python/copy_const_reference.hpp>
#include <boost/python/return_internal_reference.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>

namespace pinocchio
{
  namespace python
  {
    namespace fcl
    {
      
      namespace bp = boost::python;
      
      struct CollisionResultPythonVisitor : public bp::def_visitor<CollisionResultPythonVisitor>
      {
        typedef ::hpp::fcl::CollisionResult CollisionResult;
        
        template<class PyClass>
        void visit(PyClass& cl) const
        {
          cl
          .def(bp::init<>("Default constructor"))
          
          .def("addContact",&CollisionResult::addContact,bp::arg("contact"),"Adds one contact into result structure")
          .def("isCollision",&CollisionResult::isCollision,"Returns binary collision result")
          .def("numContacts",&CollisionResult::numContacts,"Returns the number of contacts found")
          
          .def("clear",&CollisionResult::clear,"Clears the results obtained")
          
          .def("getContacts",&CollisionResult::getContacts,"Get all the contacts",bp::return_internal_reference<>())
          .def("getContact",&CollisionResult::getContact,bp::arg("index"),"Get the i-th contact calculated",bp::return_value_policy<bp::copy_const_reference>())
          
          .def_readwrite("distance_lower_bound",&CollisionResult::distance_lower_bound,"Lower bound on distance between objects if they are disjoint (computed only on request).")
          
          ;
        }
        
        static void expose()
        {
          bp::class_<CollisionResult>("CollisionResult",
                              "Contact information returned by collision.",
                              bp::no_init)
          .def(CollisionResultPythonVisitor())
          ;
        }
        
      private:
        
        
      };
      
    } // namespace fcl
    
  } // namespace python
} // namespace pinocchio

#endif // namespace __pinocchio_python_fcl_collision_result_hpp__
