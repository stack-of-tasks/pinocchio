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

#include "pinocchio/python/se3.hpp"
#include "pinocchio/python/eigen_container.hpp"
#include "pinocchio/python/handler.hpp"
#include "pinocchio/python/data.hpp"
#include "pinocchio/python/geometry-model.hpp"

namespace se3
{
  namespace python
  {
    namespace bp = boost::python;
    
    struct CollisionPairPythonVisitor
    : public boost::python::def_visitor<CollisionPairPythonVisitor>
    {
      typedef CollisionPair::Index Index;
      typedef CollisionPair::GeomIndex GeomIndex;
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

    typedef Handler<GeometryData> GeometryDataHandler;

    struct GeometryDataPythonVisitor
      : public boost::python::def_visitor< GeometryDataPythonVisitor >
    {
      typedef GeometryData::Index Index;
      typedef GeometryData::GeomIndex GeomIndex;
      typedef se3::DistanceResult DistanceResult;
      typedef se3::CollisionResult CollisionResult;
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
        .add_property("distance_results",
                      bp::make_function(&GeometryDataPythonVisitor::distance_results,
                                        bp::return_internal_reference<>()),
                      "Vector of distance results computed in ")
        .add_property("collision_results",
                      bp::make_function(&GeometryDataPythonVisitor::collision_results,
                                        bp::return_internal_reference<>())  )
        
        .def("computeCollision",&GeometryDataPythonVisitor::computeCollision,
             bp::args("co1 (index)","co2 (index)"),
             "Check if the two collision objects of a collision pair are in collision."
             "The collision pair is given by the two index of the collision objects.")
        .def("computeAllCollisions",&GeometryDataPythonVisitor::computeAllCollisions,
             "Same as computeCollision. It applies computeCollision to all collision pairs contained collision_pairs."
             "The results are stored in collision_results.")
        .def("isColliding",&GeometryDataPythonVisitor::isColliding,
             "Check if at least one of the collision pairs is in collision.")
        
        .def("computeDistance",&GeometryDataPythonVisitor::computeDistance,
             bp::args("co1 (index)","co2 (index)"),
             "Compute the distance result between two collision objects of a collision pair."
             "The collision pair is given by the two index of the collision objects.")
        .def("computeAllDistances",&GeometryDataPythonVisitor::computeAllDistances,
             "Same as computeDistance. It applies computeDistance to all collision pairs contained collision_pairs."
             "The results are stored in collision_distances.")
        
        .def("__str__",&GeometryDataPythonVisitor::toString)
        ;
      }
      
      static GeometryDataHandler* makeDefault(const GeometryModelHandler & geometry_model)
      {
        return new GeometryDataHandler(new GeometryData(*geometry_model), true);
      }

      static std::vector<SE3> & oMg(GeometryDataHandler & m) { return m->oMg; }
      static std::vector<DistanceResult> & distance_results( GeometryDataHandler & m ) { return m->distance_results; }
      static std::vector<CollisionResult> & collision_results( GeometryDataHandler & m ) { return m->collision_results; }

      static CollisionResult computeCollision(const GeometryDataHandler & m, const GeomIndex co1, const GeomIndex co2)
      {
        return m->computeCollision(CollisionPair(co1, co2));
      }
      static bool isColliding(const GeometryDataHandler & m) { return m->isColliding(); }
      static void computeAllCollisions(GeometryDataHandler & m) { m->computeAllCollisions(); }
      
      static DistanceResult computeDistance(const GeometryDataHandler & m, const GeomIndex co1, const GeomIndex co2)
      {
        return m->computeDistance(CollisionPair(co1, co2));
      }
      static void computeAllDistances(GeometryDataHandler & m) { m->computeAllDistances(); }
      
      
      static std::string toString(const GeometryDataHandler& m)
      {	  std::ostringstream s; s << *m; return s.str();       }

      /* --- Expose --------------------------------------------------------- */
      static void expose()
      {
        
        bp::class_< std::vector<DistanceResult> >("StdVec_DistanceResult")
        .def(bp::vector_indexing_suite< std::vector<DistanceResult> >());
  
        bp::class_< std::vector<CollisionResult> >("StdVec_CollisionResult")
        .def(bp::vector_indexing_suite< std::vector<CollisionResult> >());

        bp::class_<GeometryDataHandler>("GeometryData",
                                        "Geometry data linked to a geometry model and data struct.",
                                        bp::no_init)
        .def(GeometryDataPythonVisitor());
     
        bp::to_python_converter< GeometryDataHandler::SmartPtr_t,GeometryDataPythonVisitor >();
      }

    };
    
  }} // namespace se3::python

#endif // ifndef __se3_python_geometry_data_hpp__

