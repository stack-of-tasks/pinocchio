//
// Copyright (c) 2018 CNRS
//
// This file is part of Pinocchio
// Pinocchio is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// Pinocchio is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// General Lesser Public License for more details. You should have
// received a copy of the GNU Lesser General Public License along with
// Pinocchio If not, see
// <http://www.gnu.org/licenses/>.

#ifndef __se3_python_fcl_distance_result_hpp__
#define __se3_python_fcl_distance_result_hpp__

#include "pinocchio/spatial/fcl-pinocchio-conversions.hpp"
#include <hpp/fcl/collision_data.h>

#include <boost/python/copy_const_reference.hpp>
#include <boost/python/return_internal_reference.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>

namespace se3
{
  namespace python
  {
    namespace fcl
    {
      
      namespace bp = boost::python;
      
      struct DistanceResultPythonVisitor : public bp::def_visitor<DistanceResultPythonVisitor>
      {
        typedef ::fcl::DistanceResult DistanceResult;
        
        template<class PyClass>
        void visit(PyClass& cl) const
        {
          cl
          .def(bp::init<>("Default constructor"))
          
          .def_readonly("min_distance",&DistanceResult::min_distance,
                        "minimum distance between two objects.\n"
                        "If two objects are in collision, min_distance <= 0.")
          
          .def("getNearestPoint1",getNearestPoint1,
               "Returns the nearest point on collision object 1.")
          .def("getNearestPoint2",getNearestPoint2,
               "Returns the nearest point on collision object 2.")
          
          ;
        }
        
        static void expose()
        {
          bp::class_<DistanceResult>("DistanceResult",
                                      "Contact information returned by collision.",
                                      bp::no_init)
          .def(DistanceResultPythonVisitor())
          ;
        }
        
      private:
        
        static ::fcl::Vec3f getNearestPoint1(const DistanceResult & res) { return res.nearest_points[0]; }
        static ::fcl::Vec3f getNearestPoint2(const DistanceResult & res) { return res.nearest_points[1]; }
        
      };
      
    } // namespace fcl
    
  } // namespace python
} // namespace se3

#endif // namespace __se3_python_fcl_distance_result_hpp__
