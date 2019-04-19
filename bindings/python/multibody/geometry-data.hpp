//
// Copyright (c) 2015-2018 CNRS
//

#ifndef __pinocchio_python_geometry_data_hpp__
#define __pinocchio_python_geometry_data_hpp__

#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <eigenpy/memory.hpp>

#include "pinocchio/bindings/python/utils/eigen_container.hpp"
#include "pinocchio/bindings/python/utils/printable.hpp"
#include "pinocchio/bindings/python/utils/copyable.hpp"

EIGENPY_DEFINE_STRUCT_ALLOCATOR_SPECIALIZATION(pinocchio::GeometryData)

namespace pinocchio
{
  namespace python
  {
    namespace bp = boost::python;
    
    /* --- COLLISION PAIR --------------------------------------------------- */
    /* --- COLLISION PAIR --------------------------------------------------- */
    /* --- COLLISION PAIR --------------------------------------------------- */
    struct CollisionPairPythonVisitor
    : public boost::python::def_visitor<CollisionPairPythonVisitor>
    {

      static void expose()
      {
        bp::class_<CollisionPair> ("CollisionPair",
                                   "Pair of ordered index defining a pair of collisions",
                                   bp::no_init)
        .def(bp::init<const GeomIndex &, const GeomIndex &>(bp::args("co1 (index)", "co2 (index)"),
                                                        "Initializer of collision pair."))
        .def(PrintableVisitor<CollisionPair>())
        .def(CopyableVisitor<CollisionPair>())
        .def(bp::self == bp::self)
        .def(bp::self != bp::self)
        .def_readwrite("first",&CollisionPair::first)
        .def_readwrite("second",&CollisionPair::second);
        
        bp::class_< std::vector<CollisionPair> >("StdVec_CollisionPair")
        .def(bp::vector_indexing_suite< std::vector<CollisionPair>, true >());
      }

    }; // struct CollisionPairPythonVisitor

    struct GeometryDataPythonVisitor
      : public boost::python::def_visitor< GeometryDataPythonVisitor >
    {

      /* --- Exposing C++ API to python through the handler ----------------- */
      template<class PyClass>
      void visit(PyClass& cl) const
      {
        cl
        .def(bp::init<GeometryModel>(bp::arg("geometry_model"), "Default constructor from a given GeometryModel"))
        
        .def_readonly("oMg",
                      &GeometryData::oMg,
                      "Vector of collision objects placement relative to the world.")
#ifdef PINOCCHIO_WITH_HPP_FCL
        .def_readonly("activeCollisionPairs",
                      &GeometryData::activeCollisionPairs,
                      "Vector of active CollisionPairs")
        .def_readonly("distanceRequest",
                      &GeometryData::distanceRequest,
                      "Defines which information should be computed by fcl for distances")
        .def_readonly("distanceResults",
                      &GeometryData::distanceResults,
                      "Vector of distance results.")
        .def_readonly("collisionRequest",
                      &GeometryData::collisionRequest,
                      "Defines which information should be computed by fcl for collisions.")
        .def_readonly("collisionResults",
                      &GeometryData::collisionResults,
                      "Vector of collision results.")
        .def_readonly("radius",
                      &GeometryData::radius,
                      "Vector of radius of bodies, ie distance of the further point of the geometry object from the joint center ")
        
        .def("fillInnerOuterObjectMaps", &GeometryData::fillInnerOuterObjectMaps,
             bp::args("GeometryModel"),
             "Fill inner and outer objects maps")
        .def("activateCollisionPair",static_cast<void (GeometryData::*)(const PairIndex)>(&GeometryData::activateCollisionPair),
             bp::args("pairIndex (int)"),
             "Activate pair ID <pairIndex> in geomModel.collisionPairs."
             "Only active pairs are check for collision and distance computation.")
        .def("deactivateCollisionPair",&GeometryData::deactivateCollisionPair,
             bp::args("pairIndex (int)"),
             "Deactivate pair ID <pairIndex> in geomModel.collisionPairs.")

#endif // PINOCCHIO_WITH_HPP_FCL        
        ;
      }
             

      /* --- Expose --------------------------------------------------------- */
      static void expose()
      {
        bp::class_<GeometryData>("GeometryData",
                                 "Geometry data linked to a geometry model and data struct.",
                                 bp::no_init)
        .def(GeometryDataPythonVisitor())
        .def(PrintableVisitor<GeometryData>())
        ;
     
      }

    };
    
  }} // namespace pinocchio::python

#endif // ifndef __pinocchio_python_geometry_data_hpp__

