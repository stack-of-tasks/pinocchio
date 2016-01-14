//
// Copyright (c) 2015 CNRS
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

// #include "pinocchio/multibody/parser/sample-models.hpp"
#include "pinocchio/python/se3.hpp"
#include "pinocchio/python/eigen_container.hpp"
#include "pinocchio/python/handler.hpp"

#include "pinocchio/multibody/geometry.hpp"

namespace se3
{
  namespace python
  {
    namespace bp = boost::python;

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
    .add_property("nCollisionPairs", &GeometryDataPythonVisitor::nCollisionPairs)

    .add_property("oMg",
      bp::make_function(&GeometryDataPythonVisitor::oMg,
            bp::return_internal_reference<>())  )
    .add_property("collision_pairs",
      bp::make_function(&GeometryDataPythonVisitor::collision_pairs,
            bp::return_internal_reference<>())  )
    .add_property("distances",
      bp::make_function(&GeometryDataPythonVisitor::distances,
            bp::return_internal_reference<>())  )
    .add_property("collisions",
      bp::make_function(&GeometryDataPythonVisitor::collisions,
            bp::return_internal_reference<>())  )

    .def("addCollisionPair",&GeometryDataPythonVisitor::isCollisionPair)
    .def("removeCollisionPair",&GeometryDataPythonVisitor::isCollisionPair)
    .def("isCollisionPair",&GeometryDataPythonVisitor::isCollisionPair)
    .def("collide",&GeometryDataPythonVisitor::collide)

    .def("__str__",&GeometryDataPythonVisitor::toString)


	  ;
      }

      static GeometryModel::Index nCollisionPairs( GeometryDataHandler & m ) { return m->nCollisionPairs; }
      
      static std::vector<SE3> & oMg( GeometryDataHandler & m ) { return m->oMg; }
      static std::vector<CollisionPair_t> & collision_pairs( GeometryDataHandler & m ) { return m->collision_pairs; }
      static std::vector<DistanceResult> & distances( GeometryDataHandler & m ) { return m->distances; }
      static std::vector<bool> & collisions( GeometryDataHandler & m ) { return m->collisions; }

      static void addCollisionPair (GeometryDataHandler & m, Index co1, Index co2) { m -> addCollisionPair(co1, co2);}
      static void removeCollisionPair (GeometryDataHandler & m, Index co1, Index co2) { m -> removeCollisionPair(co1, co2);}
      static bool isCollisionPair (const GeometryDataHandler & m, Index co1, Index co2) { return m -> isCollisionPair(co1, co2);}

      static bool collide(const GeometryDataHandler & m, Index co1, Index co2) { return m -> collide(co1, co2); };
      
 

      static std::string toString(const GeometryDataHandler& m) 
      {	  std::ostringstream s; s << *m; return s.str();       }

      /* --- Expose --------------------------------------------------------- */
      static void expose()
      {
  
  bp::class_<GeometryDataHandler>("GeometryData",
         "Geometry data ",
         bp::no_init)
    .def(GeometryDataPythonVisitor());

    bp::class_< std::vector<CollisionPair_t> >("StdVec_CollisionPair_t")
          .def(bp::vector_indexing_suite< std::vector<CollisionPair_t> >());
    bp::class_< std::vector<DistanceResult> >("StdVec_DistanceResult")
          .def(bp::vector_indexing_suite< std::vector<DistanceResult> >());
      
	/* Not sure if it is a good idea to enable automatic
	 * conversion. Prevent it for now */
	//bp::to_python_converter< Model,GeometryDataPythonVisitor >();
	bp::to_python_converter< GeometryDataHandler::SmartPtr_t,GeometryDataPythonVisitor >();
      }


    };
    


  }} // namespace se3::python

#endif // ifndef __se3_python_geometry_data_hpp__

