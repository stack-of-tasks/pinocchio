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

#include "pinocchio/python/se3.hpp"
#include "pinocchio/python/eigen_container.hpp"
#include "pinocchio/python/handler.hpp"

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
      typedef GeometryModel::Index Index;
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
    .add_property("ngeom", &GeometryModelPythonVisitor::ngeom)

    .def("getGeomId",&GeometryModelPythonVisitor::getGeomId)

    .add_property("geometryPlacement",
      bp::make_function(&GeometryModelPythonVisitor::geometryPlacement,
            bp::return_internal_reference<>())  )
    .add_property("geom_parents", 
      bp::make_function(&GeometryModelPythonVisitor::geom_parents,
            bp::return_internal_reference<>())  )
    .add_property("geom_names",
      bp::make_function(&GeometryModelPythonVisitor::geom_names,
            bp::return_internal_reference<>())  )

    .def("__str__",&GeometryModelPythonVisitor::toString)

	  .def("BuildEmptyGeometryModel",&GeometryModelPythonVisitor::maker_empty)
	  .staticmethod("BuildEmptyGeometryModel")
	  ;
      }

      static GeometryModel::Index ngeom( GeometryModelHandler & m ) { return m->ngeom; }

      static Model::Index getGeomId( const GeometryModelHandler & gmodelPtr, const std::string & name )
      { return  gmodelPtr->getGeomId(name); }
      
      static std::vector<SE3> & geometryPlacement( GeometryModelHandler & m ) { return m->geometryPlacement; }
      static std::vector<Model::Index> & geom_parents( GeometryModelHandler & m ) { return m->geom_parents; }
      static std::vector<std::string> & geom_names ( GeometryModelHandler & m ) { return m->geom_names; }

      

      static GeometryModelHandler maker_empty()
      {
	return GeometryModelHandler( new GeometryModel(),true );
      }
 

      static std::string toString(const GeometryModelHandler& m) 
      {	  std::ostringstream s; s << *m; return s.str();       }

      /* --- Expose --------------------------------------------------------- */
      static void expose()
      {
  
  bp::class_<GeometryModelHandler>("GeometryModel",
         "Geometry model (const)",
         bp::no_init)
    .def(GeometryModelPythonVisitor());
      
	bp::to_python_converter< GeometryModelHandler::SmartPtr_t,GeometryModelPythonVisitor >();
      }


    };
    


  }} // namespace se3::python

#endif // ifndef __se3_python_geometry_model_hpp__

