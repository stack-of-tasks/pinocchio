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
      typedef GeometryModel::JointIndex JointIndex;
      typedef GeometryModel::GeomIndex GeomIndex;
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

	  .def("BuildGeometryModel",&GeometryModelPythonVisitor::maker_default)
	  .staticmethod("BuildGeometryModel")
	  ;
      }

      static GeometryModel::Index ngeoms( GeometryModelHandler & m ) { return m->ngeoms; }

      static Model::GeomIndex getGeometryId( const GeometryModelHandler & gmodelPtr, const std::string & name )
      { return  gmodelPtr->getGeometryId(name); }
      static bool existGeometryName(const GeometryModelHandler & gmodelPtr, const std::string & name)
      { return gmodelPtr->existGeometryName(name);}
      static std::string getGeometryName(const GeometryModelHandler & gmodelPtr, const GeomIndex index)
      { return gmodelPtr->getGeometryName(index);}

      static std::vector<GeometryObject> & geometryObjects( GeometryModelHandler & m ) { return m->geometryObjects; }
      
      

      static GeometryModelHandler maker_default(const ModelHandler & model)
      {
        return GeometryModelHandler(new GeometryModel(*model), true);
      }
 

      static std::string toString(const GeometryModelHandler& m) 
      {	  std::ostringstream s; s << *m; return s.str();       }

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
    


  }} // namespace se3::python

#endif // ifndef __se3_python_geometry_model_hpp__

