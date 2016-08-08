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

#ifndef __se3_python_geometry_model_hpp__
#define __se3_python_geometry_model_hpp__

#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <eigenpy/exception.hpp>
#include <eigenpy/eigenpy.hpp>

#include "pinocchio/bindings/python/se3.hpp"
#include "pinocchio/bindings/python/eigen_container.hpp"
#include "pinocchio/bindings/python/handler.hpp"

#include "pinocchio/multibody/geometry.hpp"

namespace se3
{
  namespace python
  {
    namespace bp = boost::python;
    
    typedef Handler<GeometryModel> GeometryModelHandler;
    
    struct GeometryModelPythonVisitor
    : public boost::python::def_visitor< GeometryModelPythonVisitor >
    {
    public:

      typedef eigenpy::UnalignedEquivalent<SE3>::type SE3_fx;
      
    public:
      
      /* --- Convert From C++ to Python ------------------------------------- */
      // static PyObject* convert(Model const& modelConstRef)
      // {
      // 	Model * ptr = const_cast<Model*>(&modelConstRef);
      // 	return boost::python::incref(boost::python::object(ModelHandler(ptr)).ptr());
      // }
      static PyObject* convert(GeometryModelHandler::SmartPtr_t const& ptr)
      {
        return boost::python::incref(boost::python::object(GeometryModelHandler(ptr)).ptr());
      }
      
      /* --- Exposing C++ API to python through the handler ----------------- */
      template<class PyClass>
      void visit(PyClass& cl) const
      {
	cl
          .add_property("ngeoms", &GeometryModelPythonVisitor::ngeoms)

          .def("getGeometryId",&GeometryModelPythonVisitor::getGeometryId)
          .def("existGeometryName",&GeometryModelPythonVisitor::existGeometryName)
          .def("getGeometryName",&GeometryModelPythonVisitor::getGeometryName)
          .add_property("geometryObjects",
                        bp::make_function(&GeometryModelPythonVisitor::geometryObjects,
                                          bp::return_internal_reference<>())  )
          .def("__str__",&GeometryModelPythonVisitor::toString)
	  .def("addEmptyGeometryObject",&GeometryModelPythonVisitor::addEmptyGeometryObject,
               bp::args("parent (index)","placement (SE3)","geom_name (string)","mesh_path (string)"),
               "Add geometry object with no collision to the GeometryModel.")

#ifdef WITH_HPP_FCL
          .add_property("collision_pairs",
                        bp::make_function(&GeometryModelPythonVisitor::collision_pairs,
                                          bp::return_internal_reference<>()),
                        "Vector of collision pairs.")
          .def("addCollisionPair",&GeometryModelPythonVisitor::addCollisionPair,
               bp::args("co1 (index)","co2 (index)"),
               "Add a collision pair given by the index of the two collision objects."
               " Remark: co1 < co2")
          .def("addAllCollisionPairs",&GeometryModelPythonVisitor::addAllCollisionPairs,
               "Add all collision pairs.")
          .def("removeCollisionPair",&GeometryModelPythonVisitor::removeCollisionPair,
               bp::args("co1 (index)","co2 (index)"),
               "Remove a collision pair given by the index of the two collision objects."
               " Remark: co1 < co2")
          .def("removeAllCollisionPairs",&GeometryModelPythonVisitor::removeAllCollisionPairs,
               "Remove all collision pairs.")
          .def("existCollisionPair",&GeometryModelPythonVisitor::existCollisionPair,
               bp::args("co1 (index)","co2 (index)"),
               "Check if a collision pair given by the index of the two collision objects exists or not."
               " Remark: co1 < co2")
          .def("findCollisionPair", &GeometryModelPythonVisitor::findCollisionPair,
               bp::args("co1 (index)","co2 (index)"),
               "Return the index of a collision pair given by the index of the two collision objects exists or not."
               " Remark: co1 < co2")
#endif // WITH_HPP_FCL
	  .def("BuildGeometryModel",&GeometryModelPythonVisitor::maker_default)
	  .staticmethod("BuildGeometryModel")
	  ;
      }

      static Index ngeoms( GeometryModelHandler & m ) { return m->ngeoms; }
      static GeomIndex getGeometryId( const GeometryModelHandler & gmodelPtr, const std::string & name )
      { return  gmodelPtr->getGeometryId(name); }
      static bool existGeometryName(const GeometryModelHandler & gmodelPtr, const std::string & name)
      { return gmodelPtr->existGeometryName(name);}
      static std::string getGeometryName(const GeometryModelHandler & gmodelPtr, const GeomIndex index)
      { return gmodelPtr->getGeometryName(index);}
      
      static std::vector<GeometryObject> & geometryObjects( GeometryModelHandler & m ) { return m->geometryObjects; }
      static GeomIndex addEmptyGeometryObject( GeometryModelHandler & gmodelPtr, const JointIndex & parent, const SE3_fx & placement, const std::string & geom_name, const std::string & mesh_path)
      { return  gmodelPtr->addGeometryObject(parent, placement, geom_name, mesh_path); }
#ifdef WITH_HPP_FCL      
      static std::vector<CollisionPair> & collision_pairs( GeometryModelHandler & m ) 
      { return m->collisionPairs; }
      
      static void addCollisionPair (GeometryModelHandler & m, const GeomIndex co1, const GeomIndex co2)
      { m->addCollisionPair(CollisionPair(co1, co2)); }
      
      static void addAllCollisionPairs (GeometryModelHandler & m)
      { m->addAllCollisionPairs(); }
      
      static void removeCollisionPair (GeometryModelHandler & m, const GeomIndex co1, const GeomIndex co2)
      { m->removeCollisionPair( CollisionPair(co1,co2) ); }
      
      static void removeAllCollisionPairs (GeometryModelHandler & m)
      { m->removeAllCollisionPairs(); }
      
      static bool existCollisionPair (const GeometryModelHandler & m, const GeomIndex co1, const GeomIndex co2)
      { return m->existCollisionPair(CollisionPair(co1,co2)); }

      static Index findCollisionPair (const GeometryModelHandler & m, const GeomIndex co1, 
                                      const GeomIndex co2)
      { return m->findCollisionPair( CollisionPair(co1,co2) ); }
#endif // WITH_HPP_FCL      
      static GeometryModelHandler maker_default()
      { return GeometryModelHandler(new GeometryModel(), true); }
      
      static std::string toString(const GeometryModelHandler& m)
      {	  std::ostringstream s; s << *m; return s.str(); }
      
      /* --- Expose --------------------------------------------------------- */
      static void expose()
      {
        
        bp::enum_<GeometryType>("GeometryType")
        .value("VISUAL",VISUAL)
        .value("COLLISION",COLLISION)
        .value("NONE",NONE)
        ;
        
        bp::class_<GeometryModelHandler>("GeometryModel",
                                         "Geometry model (const)",
                                         bp::no_init)
        .def(GeometryModelPythonVisitor());
        
        bp::to_python_converter< GeometryModelHandler::SmartPtr_t,GeometryModelPythonVisitor >();
      }
      
    };
    
  } // namespace python
} // namespace se3

#endif // ifndef __se3_python_geometry_model_hpp__
