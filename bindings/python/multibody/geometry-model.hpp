//
// Copyright (c) 2015-2018 CNRS
//

#ifndef __pinocchio_python_geometry_model_hpp__
#define __pinocchio_python_geometry_model_hpp__

#include <eigenpy/memory.hpp>

#include "pinocchio/bindings/python/utils/eigen_container.hpp"
#include "pinocchio/bindings/python/utils/printable.hpp"
#include "pinocchio/multibody/geometry.hpp"

EIGENPY_DEFINE_STRUCT_ALLOCATOR_SPECIALIZATION(pinocchio::GeometryModel)

namespace pinocchio
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

        .def("addGeometryObject",static_cast <GeometryModel::GeomIndex (GeometryModel::*)(const GeometryObject &)>(&GeometryModel::addGeometryObject),
             bp::arg("GeometryObject"),
             "Add a GeometryObject to a GeometryModel").def("addGeometryObject",static_cast <GeometryModel::GeomIndex (GeometryModel::*)(const GeometryObject &,
                                                                                                                                         const Model &)>(&GeometryModel::addGeometryObject),
                                                            bp::args("GeometryObject",
                                                                     "model: a moddel of the system"),
                                                            "Add a GeometryObject to a GeometryModel and set its parent joint by reading its value in model")
        .def("getGeometryId",&GeometryModel::getGeometryId)
        .def("existGeometryName",&GeometryModel::existGeometryName)
#ifdef PINOCCHIO_WITH_HPP_FCL
        .add_property("collisionPairs",
                      &GeometryModel::collisionPairs,
                      "Vector of collision pairs."
                      " Note: for technical reasons, when you read a collision pair from this vector it is automatically copied")
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
#endif // PINOCCHIO_WITH_HPP_FCL
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
} // namespace pinocchio

#endif // ifndef __pinocchio_python_geometry_model_hpp__
