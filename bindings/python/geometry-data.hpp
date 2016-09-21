//
// Copyright (c) 2015-2016 CNRS
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
#include <eigenpy/exception.hpp>
#include <eigenpy/eigenpy.hpp>

#include "pinocchio/bindings/python/se3.hpp"
#include "pinocchio/bindings/python/eigen_container.hpp"
#include "pinocchio/bindings/python/handler.hpp"
#include "pinocchio/bindings/python/data.hpp"
#include "pinocchio/bindings/python/geometry-model.hpp"

namespace fcl
{
#ifdef WITH_HPP_FCL
  // This operator is defined here temporary, as it is needed by vector_indexing_suite
  // It has also been defined in hpp-fcl in a pending pull request.
  // Once it has been integrated in releases of hpp-fcl, please remove this operator
  inline bool operator ==(const DistanceResult & dr1, const DistanceResult& dr2)
  {
    return dr1.min_distance == dr2.min_distance
        && dr1.o1 == dr2.o1
        && dr1.o2 == dr2.o2
        && dr1.nearest_points[0] == dr2.nearest_points[0]
        && dr1.nearest_points[1] == dr2.nearest_points[1];
  }
#endif
}
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
                                   bp::init<const GeomIndex &, const GeomIndex &> (bp::args("co1 (index)", "co2 (index)"),
                                                                           "Initializer of collision pair"))
        .def("__str__",&CollisionPairPythonVisitor::toString)
        .def_readwrite("first",&CollisionPair::first)
        .def_readwrite("second",&CollisionPair::second);
        
        bp::class_< std::vector<CollisionPair> >("StdVec_CollisionPair")
        .def(bp::vector_indexing_suite< std::vector<CollisionPair> >());
      }
      
      static std::string toString(const CollisionPair & cp)
      {	std::ostringstream s; s << cp; return s.str(); }
      
    }; // struct CollisionPairPythonVisitor

    /* ---------------------------------------------------------------------- */
    /* ---------------------------------------------------------------------- */
    /* ---------------------------------------------------------------------- */
    typedef Handler<GeometryData> GeometryDataHandler;
    
    struct GeometryDataPythonVisitor
      : public boost::python::def_visitor< GeometryDataPythonVisitor >
    {
      typedef eigenpy::UnalignedEquivalent<SE3>::type SE3_fx;
      
      /* --- Convert From C++ to Python ------------------------------------- */
      // static PyObject* convert(Model const& modelConstRef)
      // {
      // 	Model * ptr = const_cast<Model*>(&modelConstRef);
      // 	return boost::python::incref(boost::python::object(ModelHandler(ptr)).ptr());
      // }
      static PyObject* convert(GeometryDataHandler::SmartPtr_t const& ptr)
      {
        return boost::python::incref(boost::python::object(GeometryDataHandler(ptr)).ptr());
      }

      /* --- Exposing C++ API to python through the handler ----------------- */
      template<class PyClass>
      void visit(PyClass& cl) const
      {
        cl
        .def("__init__",
             bp::make_constructor(&GeometryDataPythonVisitor::makeDefault,
                                  bp::default_call_policies(),
                                  (bp::arg("geometry_model"))),
             "Initialize from the geometry model.")
        
        .add_property("oMg",
                      bp::make_function(&GeometryDataPythonVisitor::oMg,
                                        bp::return_internal_reference<>()),
                      "Vector of collision objects placement relative to the world.")
#ifdef WITH_HPP_FCL
        .add_property("activeCollisionPairs",
                      bp::make_function(&GeometryDataPythonVisitor::activeCollisionPairs,
                                        bp::return_internal_reference<>()))
        .add_property("distanceRequest",
                      bp::make_function(&GeometryDataPythonVisitor::distanceRequest,
                                        bp::return_internal_reference<>()),
                      "Defines what information should be computed by fcl for distances")
        .add_property("distanceResults",
                      bp::make_function(&GeometryDataPythonVisitor::distanceResults,
                                        bp::return_internal_reference<>()),
                      "Vector of distance results computed in ")
        .add_property("CollisionRequest",
                      bp::make_function(&GeometryDataPythonVisitor::CollisionRequest,
                                        bp::return_internal_reference<>()),
                      "Defines what information should be computed by fcl for collision tests")
        .add_property("collision_results",
                      bp::make_function(&GeometryDataPythonVisitor::collision_results,
                                        bp::return_internal_reference<>())  )
        .add_property("radius",
                      bp::make_function(&GeometryDataPythonVisitor::radius,
                                        bp::return_internal_reference<>()),
                      "Vector of radius of bodies, ie distance of the further point of the geometry object from the joint center ")
        
        .def("fillInnerOuterObjectMaps", &GeometryDataPythonVisitor::fillInnerOuterObjectMaps,
             bp::args("GeometryModel"),
             "Fill inner and outer objects maps")
        .def("activateCollisionPair",&GeometryDataPythonVisitor::activateCollisionPair,
             bp::args("pairIndex (int)"),
             "Activate pair ID <pairIndex> in geomModel.collisionPairs."
             "Only active pairs are check for collision and distance computation.")
        .def("deactivateCollisionPair",&GeometryDataPythonVisitor::deactivateCollisionPair,
             bp::args("pairIndex (int)"),
             "Deactivate pair ID <pairIndex> in geomModel.collisionPairs.")

#endif // WITH_HPP_FCL        
        .def("__str__",&GeometryDataPythonVisitor::toString)
        ;
      }
      
      static GeometryDataHandler* makeDefault(const GeometryModelHandler & geometry_model)
      {
        return new GeometryDataHandler(new GeometryData(*geometry_model), true);
      }

      static std::vector<SE3> & oMg(GeometryDataHandler & m) { return m->oMg; }
#ifdef WITH_HPP_FCL      
      static std::vector<bool> & activeCollisionPairs(GeometryDataHandler & m) { return m->activeCollisionPairs; }
      static fcl::DistanceRequest & distanceRequest( GeometryDataHandler & m ) { return m->distanceRequest; }
      static std::vector<fcl::DistanceResult> & distanceResults( GeometryDataHandler & m ) { return m->distanceResults; }
      static fcl::CollisionRequest & CollisionRequest( GeometryDataHandler & m ) { return m->collisionRequest; } 
      static std::vector<fcl::CollisionResult> & collision_results( GeometryDataHandler & m ) { return m->collisionResults; }
      static std::vector<double> & radius( GeometryDataHandler & m ) { return m->radius; }

      static void fillInnerOuterObjectMaps(GeometryDataHandler & m, const GeometryModelHandler & model)
      {m->fillInnerOuterObjectMaps(*model);}

      static void activateCollisionPair(GeometryDataHandler & m,
                                        Index pairID) { m->activateCollisionPair(pairID); } 
      static void deactivateCollisionPair(GeometryDataHandler & m,
                                          Index pairID) { m->deactivateCollisionPair(pairID); } 
      
#endif // WITH_HPP_FCL      
      
      static std::string toString(const GeometryDataHandler& m)
      {	  std::ostringstream s; s << *m; return s.str();       }

      /* --- Expose --------------------------------------------------------- */
      static void expose()
      {
#ifdef WITH_HPP_FCL        
        bp::class_< std::vector<fcl::DistanceResult> >("StdVec_DistanceResult")
        .def(bp::vector_indexing_suite< std::vector<fcl::DistanceResult> >());
  
        bp::class_< std::vector<fcl::CollisionResult> >("StdVec_CollisionResult")
        .def(bp::vector_indexing_suite< std::vector<fcl::CollisionResult> >());
#endif // WITH_HPP_FCL
        bp::class_<GeometryDataHandler>("GeometryData",
                                        "Geometry data linked to a geometry model and data struct.",
                                        bp::no_init)
        .def(GeometryDataPythonVisitor());
     
        bp::to_python_converter< GeometryDataHandler::SmartPtr_t,GeometryDataPythonVisitor >();
      }

    };
    
  }} // namespace se3::python

#endif // ifndef __se3_python_geometry_data_hpp__

