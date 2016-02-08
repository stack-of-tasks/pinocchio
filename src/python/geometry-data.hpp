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
      static void expose()
      {
        bp::class_<CollisionPair> ("CollisionPair",
                                   "Pair of ordered index defining a pair of collisions",
                                   bp::init<const Index &, const Index &> (bp::args("co1 (index)", "co2 (index)"),
                                                                           "Initializer of collision pair"))
        .def("__str__",&CollisionPairPythonVisitor::toString);
        
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
    public:
      typedef GeometryData::Index Index;
      typedef GeometryData::CollisionPair_t CollisionPair_t;
      typedef se3::DistanceResult DistanceResult;
      typedef eigenpy::UnalignedEquivalent<SE3>::type SE3_fx;
      
    public:

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
                                  (bp::arg("data"),bp::arg("geometry_model"))),
             "Initialize from data and the geometry model.")
        
        .add_property("nCollisionPairs", &GeometryDataPythonVisitor::nCollisionPairs)
        
        .add_property("oMg",
                      bp::make_function(&GeometryDataPythonVisitor::oMg,
                                        bp::return_internal_reference<>()),
                      "Vector of collision objects placement relative to the world.")
        .add_property("collision_pairs",
                      bp::make_function(&GeometryDataPythonVisitor::collision_pairs,
                                        bp::return_internal_reference<>()),
                      "Vector of collision pairs.")
        .add_property("distances",
                      bp::make_function(&GeometryDataPythonVisitor::distances,
                                        bp::return_internal_reference<>()),
                      "Vector of distance results computed in ")
        .add_property("collisions",
                      bp::make_function(&GeometryDataPythonVisitor::collisions,
                                        bp::return_internal_reference<>())  )
        
        .def("addCollisionPair",&GeometryDataPythonVisitor::addCollisionPair,
             bp::args("co1 (index)","co2 (index)"),
             "Add a collision pair given by the index of the two collision objects."
             " Remark: co1 < co2")
        .def("addAllCollisionPairs",&GeometryDataPythonVisitor::addAllCollisionPairs,
             "Add all collision pairs.")
        .def("removeCollisionPair",&GeometryDataPythonVisitor::removeCollisionPair,
             bp::args("co1 (index)","co2 (index)"),
             "Remove a collision pair given by the index of the two collision objects."
             " Remark: co1 < co2")
        .def("removeAllCollisionPairs",&GeometryDataPythonVisitor::removeAllCollisionPairs,
             "Remove all collision pairs.")
        .def("existCollisionPair",&GeometryDataPythonVisitor::existCollisionPair,
             bp::args("co1 (index)","co2 (index)"),
             "Check if a collision pair given by the index of the two collision objects exists or not."
             " Remark: co1 < co2")
        .def("findCollisionPair", &GeometryDataPythonVisitor::findCollisionPair,
             bp::args("co1 (index)","co2 (index)"),
             "Return the index of a collision pair given by the index of the two collision objects exists or not."
             " Remark: co1 < co2")
        .def("collide",&GeometryDataPythonVisitor::collide,
             bp::args("co1 (index)","co2 (index)"),
             "Check if the two collision objects of a collision pair are in collision."
             "The collision pair is given by the two index of the collision objects.")
        
        .def("__str__",&GeometryDataPythonVisitor::toString)
	  ;
      }
      
      static GeometryDataHandler* makeDefault(const DataHandler & data, const GeometryModelHandler & geometry_model)
      {
        return new GeometryDataHandler(new GeometryData(*data, *geometry_model), true);
      }

      static Index nCollisionPairs(const GeometryDataHandler & m ) { return m->nCollisionPairs; }
      
      static std::vector<SE3> & oMg(GeometryDataHandler & m) { return m->oMg; }
      static std::vector<CollisionPair_t> & collision_pairs( GeometryDataHandler & m ) { return m->collision_pairs; }
      static std::vector<DistanceResult> & distances( GeometryDataHandler & m ) { return m->distances; }
      static std::vector<bool> & collisions( GeometryDataHandler & m ) { return m->collisions; }

      static void addCollisionPair (GeometryDataHandler & m, const Index co1, const Index co2)
      {
        m->addCollisionPair(co1, co2);
      }
      static void addAllCollisionPairs (GeometryDataHandler & m)
      {
        m->addAllCollisionPairs();
      }
      
      static void removeCollisionPair (GeometryDataHandler & m, const Index co1, const Index co2)
      {
        m->removeCollisionPair(co1, co2);
      }
      static void removeAllCollisionPairs (GeometryDataHandler & m)
      {
        m->removeAllCollisionPairs();
      }
      
      static bool existCollisionPair (const GeometryDataHandler & m, const Index co1, const Index co2)
      {
        return m->existCollisionPair(co1, co2);
      }
      static GeometryData::Index findCollisionPair (const GeometryDataHandler & m, const Index co1, const Index co2)
      {
        return m->findCollisionPair(co1, co2);
      }

      static bool collide(const GeometryDataHandler & m, const Index co1, const Index co2) { return m -> collide(co1, co2); };
      
      static std::string toString(const GeometryDataHandler& m)
      {	  std::ostringstream s; s << *m; return s.str();       }

      /* --- Expose --------------------------------------------------------- */
      static void expose()
      {
        
        bp::class_< std::vector<DistanceResult> >("StdVec_DistanceResult")
        .def(bp::vector_indexing_suite< std::vector<DistanceResult> >());
  
        bp::class_<GeometryDataHandler>("GeometryData",
                                 "Geometry data linked to a geometry model",
                                 bp::no_init)
        .def(GeometryDataPythonVisitor());
     
	bp::to_python_converter< GeometryDataHandler::SmartPtr_t,GeometryDataPythonVisitor >();
      }


    };
    


  }} // namespace se3::python

#endif // ifndef __se3_python_geometry_data_hpp__

