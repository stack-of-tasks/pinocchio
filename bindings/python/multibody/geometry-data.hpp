//
// Copyright (c) 2015-2018 CNRS
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

#ifndef __se3_python_geometry_data_hpp__
#define __se3_python_geometry_data_hpp__

#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <eigenpy/memory.hpp>

#include "pinocchio/bindings/python/utils/eigen_container.hpp"
#include "pinocchio/bindings/python/utils/printable.hpp"

EIGENPY_DEFINE_STRUCT_ALLOCATOR_SPECIALIZATION(se3::GeometryData)

namespace se3
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
        .def(bp::init<GeometryModel>(bp::arg("geometry_model"), "Default constructor from a given GeometryModel"))
        
        .def_readonly("oMg",
                      &GeometryData::oMg,
                      "Vector of collision objects placement relative to the world.")
#ifdef WITH_HPP_FCL
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
        .def("activateCollisionPair",&GeometryData::activateCollisionPair,
             bp::args("pairIndex (int)"),
             "Activate pair ID <pairIndex> in geomModel.collisionPairs."
             "Only active pairs are check for collision and distance computation.")
        .def("deactivateCollisionPair",&GeometryData::deactivateCollisionPair,
             bp::args("pairIndex (int)"),
             "Deactivate pair ID <pairIndex> in geomModel.collisionPairs.")

#endif // WITH_HPP_FCL        
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
    
  }} // namespace se3::python

#endif // ifndef __se3_python_geometry_data_hpp__

