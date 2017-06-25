//
// Copyright (c) 2015-2017 CNRS
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

#ifndef __se3_python_geometry_model_hpp__
#define __se3_python_geometry_model_hpp__

#include <eigenpy/memory.hpp>

#include "pinocchio/bindings/python/utils/eigen_container.hpp"
#include "pinocchio/bindings/python/utils/printable.hpp"
#include "pinocchio/multibody/geometry.hpp"

EIGENPY_DEFINE_STRUCT_ALLOCATOR_SPECIALIZATION(se3::GeometryModel)

namespace se3
{
  namespace python
  {
    namespace bp = boost::python;
    
    struct GeometryModelPythonVisitor
    : public boost::python::def_visitor< GeometryModelPythonVisitor >
    {
    public:
      
      /* --- Exposing C++ API to python through the handler ----------------- */
      template<class PyClass>
      void visit(PyClass& cl) const
      {
        cl
        .def(bp::init<>("Default constructor"))
        .add_property("ngeoms", &GeometryModel::ngeoms, "Number of geometries contained in the Geometry Model.")
        .add_property("geometryObjects",
                      &GeometryModel::geometryObjects,"Vector of geometries objects.")
        
        .def("addGeometryObject", &GeometryModel::addGeometryObject,
             bp::args("GeometryObject", "Model", "bool"),
             "Add a GeometryObject to a GeometryModel")
        .def("getGeometryId",&GeometryModel::getGeometryId)
        .def("existGeometryName",&GeometryModel::existGeometryName)
#ifdef WITH_HPP_FCL
        .add_property("collisionPairs",
                      &GeometryModel::collisionPairs,
                      "Vector of collision pairs.")
        .def("addCollisionPair",&GeometryModel::addCollisionPair,
             bp::args("co1 (index)","co2 (index)"),
             "Add a collision pair given by the index of the two collision objects."
             " Remark: co1 < co2")
        .def("addAllCollisionPairs",&GeometryModel::addAllCollisionPairs,
             "Add all collision pairs.")
        .def("removeCollisionPair",&GeometryModel::removeCollisionPair,
             bp::args("co1 (index)","co2 (index)"),
             "Remove a collision pair given by the index of the two collision objects."
             " Remark: co1 < co2")
        .def("removeAllCollisionPairs",&GeometryModel::removeAllCollisionPairs,
             "Remove all collision pairs.")
        .def("existCollisionPair",&GeometryModel::existCollisionPair,
             bp::args("co1 (index)","co2 (index)"),
             "Check if a collision pair given by the index of the two collision objects exists or not."
             " Remark: co1 < co2")
        .def("findCollisionPair", &GeometryModel::findCollisionPair,
             bp::args("co1 (index)","co2 (index)"),
             "Return the index of a collision pair given by the index of the two collision objects exists or not."
             " Remark: co1 < co2")
#endif // WITH_HPP_FCL
        ;
      }
      
      

      /* --- Expose --------------------------------------------------------- */
      static void expose()
      {
        bp::class_<GeometryModel>("GeometryModel",
                                  "Geometry model (const)",
                                  bp::no_init)
        .def(GeometryModelPythonVisitor())
        .def(PrintableVisitor<GeometryModel>())
        ;
      }
      
    };
    
  } // namespace python
} // namespace se3

#endif // ifndef __se3_python_geometry_model_hpp__
