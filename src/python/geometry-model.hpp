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
    .add_property("ncollisions", &GeometryModelPythonVisitor::ncollisions)
    .add_property("nvisuals", &GeometryModelPythonVisitor::nvisuals)

    .def("getCollisionId",&GeometryModelPythonVisitor::getCollisionId)
    .def("getVisualId",&GeometryModelPythonVisitor::getVisualId)
    .def("existCollisionName",&GeometryModelPythonVisitor::existCollisionName)
    .def("existVisualName",&GeometryModelPythonVisitor::existVisualName)
    .def("getCollisionName",&GeometryModelPythonVisitor::getCollisionName)
    .def("getVisualName",&GeometryModelPythonVisitor::getVisualName)
    .add_property("collision_objects",
      bp::make_function(&GeometryModelPythonVisitor::collision_objects,
            bp::return_internal_reference<>())  )
    .add_property("visual_objects",
      bp::make_function(&GeometryModelPythonVisitor::visual_objects,
            bp::return_internal_reference<>())  )


    .def("__str__",&GeometryModelPythonVisitor::toString)

	  .def("BuildGeometryModel",&GeometryModelPythonVisitor::maker_default)
	  .staticmethod("BuildGeometryModel")
	  ;
      }

      static GeometryModel::Index ncollisions( GeometryModelHandler & m ) { return m->ncollisions; }
      static GeometryModel::Index nvisuals( GeometryModelHandler & m ) { return m->nvisuals; }

      static std::vector<GeometryObject> & collision_objects( GeometryModelHandler & m ) { return m->collision_objects; }
      static std::vector<GeometryObject> & visual_objects( GeometryModelHandler & m ) { return m->visual_objects; }
      
      static Model::GeomIndex getCollisionId( const GeometryModelHandler & gmodelPtr, const std::string & name )
      { return  gmodelPtr->getCollisionId(name); }
      static Model::GeomIndex getVisualId( const GeometryModelHandler & gmodelPtr, const std::string & name )
      { return  gmodelPtr->getVisualId(name); }
      static bool existCollisionName(const GeometryModelHandler & gmodelPtr, const std::string & name)
      { return gmodelPtr->existCollisionName(name);}
      static bool existVisualName(const GeometryModelHandler & gmodelPtr, const std::string & name)
      { return gmodelPtr->existVisualName(name);}
      static std::string getCollisionName(const GeometryModelHandler & gmodelPtr, const GeomIndex index)
      { return gmodelPtr->getCollisionName(index);}
      static std::string getVisualName(const GeometryModelHandler & gmodelPtr, const GeomIndex index)
      { return gmodelPtr->getVisualName(index);}
      // static std::vector<Model::JointIndex> & geom_parents( GeometryModelHandler & m ) { return m->geom_parents; }
      // static std::vector<std::string> & geom_names ( GeometryModelHandler & m ) { return m->geom_names; }

      

      static GeometryModelHandler maker_default(const ModelHandler & model)
      {
        return GeometryModelHandler(new GeometryModel(*model), true);
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

