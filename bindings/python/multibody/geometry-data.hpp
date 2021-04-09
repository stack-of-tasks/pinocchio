//
// Copyright (c) 2015-2021 CNRS INRIA
//

#ifndef __pinocchio_python_geometry_data_hpp__
#define __pinocchio_python_geometry_data_hpp__

#include <eigenpy/memory.hpp>

#include "pinocchio/serialization/geometry.hpp"

#include "pinocchio/bindings/python/utils/printable.hpp"
#include "pinocchio/bindings/python/utils/copyable.hpp"
#include "pinocchio/bindings/python/utils/deprecation.hpp"
#include "pinocchio/bindings/python/utils/std-vector.hpp"
#include "pinocchio/bindings/python/serialization/serializable.hpp"

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
        .def(bp::init<>
             (bp::args("self"),
              "Empty constructor."))
        .def(bp::init<const GeomIndex &, const GeomIndex &>
             (bp::args("self","index1", "index2"),
              "Initializer of collision pair."))
        .def(PrintableVisitor<CollisionPair>())
        .def(CopyableVisitor<CollisionPair>())
        .def(bp::self == bp::self)
        .def(bp::self != bp::self)
        .def_readwrite("first",&CollisionPair::first)
        .def_readwrite("second",&CollisionPair::second);
        
        StdVectorPythonVisitor<CollisionPair>::expose("StdVec_CollisionPair");
        serialize< std::vector<CollisionPair> >();
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
        
        .def("fillInnerOuterObjectMaps",
             &GeometryData::fillInnerOuterObjectMaps,
             bp::args("self","geometry_model"),
             "Fill inner and outer objects maps")
        .def("activateCollisionPair",
             static_cast<void (GeometryData::*)(const PairIndex)>(&GeometryData::activateCollisionPair),
             bp::args("self","pair_id"),
             "Activate the collsion pair pair_id in geomModel.collisionPairs if it exists.\n"
             "note: Only active pairs are check for collision and distance computations.")
        .def("setGeometryCollisionStatus",
             &GeometryData::setGeometryCollisionStatus,
             bp::args("self","geom_model","geom_id","enable_collision"),
             "Enable or disable collision for the given geometry given by its geometry id with all the other geometries registered in the list of collision pairs.")
        .def("setActiveCollisionPairs",
             &GeometryData::setActiveCollisionPairs,
             setActiveCollisionPairs_overload(bp::args("self","geometry_model","collision_map","upper"),
                                              "Set the collision pair association from a given input array.\n"
                                              "Each entry of the input matrix defines the activation of a given collision pair."))
        .def("deactivateCollisionPair",
             &GeometryData::deactivateCollisionPair,
             bp::args("self","pair_id"),
             "Deactivate the collsion pair pair_id in geomModel.collisionPairs if it exists.")
        .def("deactivateAllCollisionPairs",
             &GeometryData::deactivateAllCollisionPairs,
             bp::args("self"),
             "Deactivate all collision pairs.")
#ifdef PINOCCHIO_WITH_HPP_FCL
        .def("setSecurityMargins",
             &GeometryData::setSecurityMargins,
             setSecurityMargins_overload(bp::args("self","geometry_model","security_margin_map","upper"),
                                         "Set the security margin of all the collision request in a row, according to the values stored in the associative map."))
#endif // PINOCCHIO_WITH_HPP_FCL
        
        .def(bp::self == bp::self)
        .def(bp::self != bp::self)
        
        ;

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
        .def(SerializableVisitor<GeometryData>())
        ;
     
      }
      
    protected:
      
      BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(setActiveCollisionPairs_overload,GeometryData::setActiveCollisionPairs,2,3)
      BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(setSecurityMargins_overload,GeometryData::setSecurityMargins,2,3)

    };
    
  }} // namespace pinocchio::python

#endif // ifndef __pinocchio_python_geometry_data_hpp__
