//
// Copyright (c) 2016 CNRS
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

#ifndef __se3_python_geometry_object_hpp__
#define __se3_python_geometry_object_hpp__

#include <eigenpy/exception.hpp>
#include <eigenpy/eigenpy.hpp>

#include <boost/python/suite/indexing/vector_indexing_suite.hpp>

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/geometry.hpp"


namespace se3
{
  namespace python
  {
    namespace bp = boost::python;

    struct GeometryObjectPythonVisitor
      : public boost::python::def_visitor< GeometryObjectPythonVisitor >
    {
      typedef eigenpy::UnalignedEquivalent<SE3>::type SE3_fx;
      typedef Model::Index Index;
      typedef Model::JointIndex JointIndex;
      typedef Model::FrameIndex FrameIndex;

    public:

      static PyObject* convert(Frame const& f)
      {
        return boost::python::incref(boost::python::object(f).ptr());
      }

      template<class PyClass>
      void visit(PyClass& cl) const 
      {
        cl
          .add_property("name", &GeometryObjectPythonVisitor::getName, &GeometryObjectPythonVisitor::setName)
          .add_property("parent_id", &GeometryObjectPythonVisitor::getParentId, &GeometryObjectPythonVisitor::setParentId)
          .add_property("placement", &GeometryObjectPythonVisitor::getPlacementWrtParentJoint, &GeometryObjectPythonVisitor::setMeshPath)
          .add_property("mesh_path", &GeometryObjectPythonVisitor::getMeshPath, &GeometryObjectPythonVisitor::setMeshPath)
          ;
      }


      static std::string getName( const GeometryObject & self) { return self.name; }
      static void setName(GeometryObject & self, const std::string & name) { self.name = name; }
      static JointIndex getParentId( const GeometryObject & self) { return self.parent; }
      static void setParentId(GeometryObject & self, const JointIndex parent) { self.parent = parent; }
      static SE3_fx getPlacementWrtParentJoint( const GeometryObject & self) { return self.placement; }
      static void setPlacementWrtParentJoint(GeometryObject & self, const SE3_fx & placement) { self.placement = placement; }
      static std::string getMeshPath( const GeometryObject & self) { return self.mesh_path; }
      static void setMeshPath(GeometryObject & self, const std::string & mesh_path) { self.mesh_path = mesh_path; }

      static void expose()
      {
        bp::class_<GeometryObject>("GeometryObject",
                           "A wrapper on a collision geometry including its parent joint, placement in parent frame.\n\n",
	                         bp::no_init
                         )
	                       .def(GeometryObjectPythonVisitor())
	                       ;
    
        // bp::to_python_converter< GeometryObject,GeometryObjectPythonVisitor >();

        bp::class_< std::vector<GeometryObject> >("StdVec_GeometryObject")
        .def(bp::vector_indexing_suite< std::vector<GeometryObject> >());
      }


    };
    

  } // namespace python
} // namespace se3

#endif // ifndef __se3_python_geometry_object_hpp__

