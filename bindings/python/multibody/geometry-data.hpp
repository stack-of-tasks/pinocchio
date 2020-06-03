//
// Copyright (c) 2015-2020 CNRS INRIA
//

#ifndef __pinocchio_python_geometry_data_hpp__
#define __pinocchio_python_geometry_data_hpp__

#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <eigenpy/memory.hpp>

#include "pinocchio/bindings/python/utils/printable.hpp"
#include "pinocchio/bindings/python/utils/copyable.hpp"
#include "pinocchio/bindings/python/utils/deprecation.hpp"

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
        .def(bp::init<const GeomIndex &, const GeomIndex &>
             (bp::args("self","index 1", "index 2"),
              "Initializer of collision pair."))
        .def(PrintableVisitor<CollisionPair>())
        .def(CopyableVisitor<CollisionPair>())
        .def(bp::self == bp::self)
        .def(bp::self != bp::self)
        .def_readwrite("first",&CollisionPair::first)
        .def_readwrite("second",&CollisionPair::second);
        
        bp::class_< std::vector<CollisionPair> >("StdVec_CollisionPair")
        .def(bp::vector_indexing_suite< std::vector<CollisionPair> >());
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
        .def(bp::init<GeometryModel>(bp::args("self","geometry_model"),
                                     "Default constructor from a given GeometryModel"))
        
        .def_readonly("oMg",
                      &GeometryData::oMg,
                      "Vector of collision objects placement relative to the world frame.\n"
                      "note: These quantities have to be updated by calling updateGeometryPlacements.")
        .def_readonly("activeCollisionPairs",
                      &GeometryData::activeCollisionPairs,
                      "Vector of active CollisionPairs")
        
#ifdef PINOCCHIO_WITH_HPP_FCL
        .def_readonly("distanceRequests",
                      &GeometryData::distanceRequests,
                      "Defines which information should be computed by FCL for distance computations")
        .def_readonly("distanceResults",
                      &GeometryData::distanceResults,
                      "Vector of distance results.")
        .def_readonly("collisionRequests",
                      &GeometryData::collisionRequests,
                      "Defines which information should be computed by FCL for collision computations.\n\n"
                      "Note: it is possible to define a security_margin and a break_distance for a collision request.\n"
                      "Most likely, for robotics application, these thresholds will be different for each collision pairs\n"
                      "(e.g. the two hands can have a large security margin while the two hips cannot.)")
        .def_readonly("collisionResults",
                      &GeometryData::collisionResults,
                      "Vector of collision results.")
        .def_readonly("radius",
                      &GeometryData::radius,
                      "Vector of radius of bodies, i.e. the distance between the further point of the geometry object from the joint center.\n"
                      "note: This radius information might be usuful in continuous collision checking")
#endif // PINOCCHIO_WITH_HPP_FCL
        
        .def("fillInnerOuterObjectMaps", &GeometryData::fillInnerOuterObjectMaps,
             bp::args("self","GeometryModel"),
             "Fill inner and outer objects maps")
        .def("activateCollisionPair",
             static_cast<void (GeometryData::*)(const PairIndex)>(&GeometryData::activateCollisionPair),
             bp::args("self","pair_id"),
             "Activate the collsion pair pair_id in geomModel.collisionPairs if it exists.\n"
             "note: Only active pairs are check for collision and distance computations.")
        .def("deactivateCollisionPair",&GeometryData::deactivateCollisionPair,
             bp::args("self","pair_id"),
             "Deactivate the collsion pair pair_id in geomModel.collisionPairs if it exists.")
        ;

#ifdef PINOCCHIO_WITH_HPP_FCL  
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
        cl
        .add_property("distanceRequest",
                      bp::make_getter(&GeometryData::distanceRequest,
                                      deprecated_member<bp::return_internal_reference<> >()),
                      "Deprecated. Use distanceRequests attribute instead.")
        .add_property("collisionRequest",
                      bp::make_getter(&GeometryData::collisionRequest,
                                      deprecated_member<bp::return_internal_reference<> >()),
                      "Deprecated. Use collisionRequests attribute instead.")
        ;
#pragma GCC diagnostic pop
#endif // PINOCCHIO_WITH_HPP_FCL
      }
             
      /* --- Expose --------------------------------------------------------- */
      static void expose()
      {
        bp::class_<GeometryData>("GeometryData",
                                 "Geometry data linked to a Geometry Model and a Data struct.",
                                 bp::no_init)
        .def(GeometryDataPythonVisitor())
        .def(PrintableVisitor<GeometryData>())
        .def(CopyableVisitor<GeometryData>())
        ;
     
      }

    };
    
  }} // namespace pinocchio::python

#endif // ifndef __pinocchio_python_geometry_data_hpp__
