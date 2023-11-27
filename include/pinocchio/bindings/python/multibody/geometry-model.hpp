//
// Copyright (c) 2015-2021 CNRS INRIA
//

#ifndef __pinocchio_python_geometry_model_hpp__
#define __pinocchio_python_geometry_model_hpp__

#include <eigenpy/memory.hpp>

#include "pinocchio/bindings/python/utils/printable.hpp"
#include "pinocchio/bindings/python/utils/copyable.hpp"
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
        .def(bp::init<>(bp::arg("self"),"Default constructor"))
        .add_property("ngeoms", &GeometryModel::ngeoms, "Number of geometries contained in the Geometry Model.")
        .add_property("geometryObjects",
                      &GeometryModel::geometryObjects,"Vector of geometries objects.")

        .def("addGeometryObject",
             static_cast <GeometryModel::GeomIndex (GeometryModel::*)(const GeometryObject &)>(&GeometryModel::addGeometryObject),
             bp::args("self","geometry_object"),
             "Add a GeometryObject to a GeometryModel.\n"
             "Parameters\n"
             "\tgeometry_object : a GeometryObject\n")
        .def("addGeometryObject",
             static_cast <GeometryModel::GeomIndex (GeometryModel::*)(const GeometryObject &,
                                                                      const Model &)>(&GeometryModel::addGeometryObject),
             bp::args("self","geometry_object","model"),
             "Add a GeometryObject to a GeometryModel and set its parent joint by reading its value in the model.\n"
             "Parameters\n"
             "\tgeometry_object : a GeometryObject\n"
             "\tmodel : a Model of the system\n")
         .def("removeGeometryObject",
             &GeometryModel::removeGeometryObject,
             bp::args("self","name"),
             "Remove a GeometryObject. Remove also the collision pairs that contain the object.")
        .def("getGeometryId",
             &GeometryModel::getGeometryId,
             bp::args("self","name"),
             "Returns the index of a GeometryObject given by its name.")
        .def("existGeometryName",
             &GeometryModel::existGeometryName,
             bp::args("self","name"),
             "Checks if a GeometryObject  given by its name exists.")
        .def("createData",
             &GeometryModelPythonVisitor::createData,
             bp::arg("self"),
             "Create a GeometryData associated to the current model.")
        .add_property("collisionPairs",
                      &GeometryModel::collisionPairs,
                      "Vector of collision pairs.")
        .def("addCollisionPair",&GeometryModel::addCollisionPair,
             bp::args("self","collision_pair"),
             "Add a collision pair given by the index of the two collision objects.")
        .def("addAllCollisionPairs",&GeometryModel::addAllCollisionPairs,
             "Add all collision pairs.\n"
             "note : collision pairs between geometries having the same parent joint are not added.")
        .def("setCollisionPairs",
             &GeometryModel::setCollisionPairs,
             setCollisionPairs_overload(bp::args("self","collision_map","upper"),
                                        "Set the collision pairs from a given input array.\n"
                                        "Each entry of the input matrix defines the activation of a given collision pair"
                                        "(map[i,j] == True means that the pair (i,j) is active)."))
        .def("removeCollisionPair",&GeometryModel::removeCollisionPair,
             bp::args("self","collision_pair"),
             "Remove a collision pair.")
        .def("removeAllCollisionPairs",&GeometryModel::removeAllCollisionPairs,
             "Remove all collision pairs.")
        .def("existCollisionPair",&GeometryModel::existCollisionPair,
             bp::args("self","collision_pair"),
             "Check if a collision pair exists.")
        .def("findCollisionPair", &GeometryModel::findCollisionPair,
             bp::args("self","collision_pair"),
             "Return the index of a collision pair.")

        .def(bp::self == bp::self)
        .def(bp::self != bp::self)
        ;
      }
      
      static GeometryData createData(const GeometryModel & geomModel)
      {
        return GeometryData(geomModel);
      }

      /* --- Expose --------------------------------------------------------- */
      static void expose()
      {
        bp::class_<GeometryModel>("GeometryModel",
                                  "Geometry model containing the collision or visual geometries associated to a model.",
                                  bp::no_init)
        .def(GeometryModelPythonVisitor())
        .def(PrintableVisitor<GeometryModel>())
        .def(CopyableVisitor<GeometryModel>())
        ;
      }
      
    protected:
      
      BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(setCollisionPairs_overload,GeometryModel::setCollisionPairs,1,2)
      
    };
    
  } // namespace python
} // namespace pinocchio

#endif // ifndef __pinocchio_python_geometry_model_hpp__
