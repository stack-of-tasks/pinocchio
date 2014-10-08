#ifndef __se3_python_model_hpp__
#define __se3_python_model_hpp__

#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <eigenpy/exception.hpp>
#include <eigenpy/eigenpy.hpp>

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/parser/sample-models.hpp"
#include "pinocchio/python/eigen_container.hpp"
#include "pinocchio/python/handler.hpp"


namespace se3
{
  namespace python
  {
    namespace bp = boost::python;

    typedef Handler<Model> ModelHandler;

    struct ModelPythonVisitor
      : public boost::python::def_visitor< ModelPythonVisitor >
    {
    public:
      typedef Model::Index Index;
      typedef typename eigenpy::UnalignedEquivalent<Motion>::type Motion_fx;

    public:

      /* --- Convert From C++ to Python ------------------------------------- */
      // static PyObject* convert(Model const& modelConstRef)
      // {
      // 	Model * ptr = const_cast<Model*>(&modelConstRef);
      // 	return boost::python::incref(boost::python::object(ModelHandler(ptr)).ptr());
      // }
      static PyObject* convert(ModelHandler::SmartPtr_t const& ptr)
      {
	return boost::python::incref(boost::python::object(ModelHandler(ptr)).ptr());
      }

      /* --- Exposing C++ API to python through the handler ----------------- */
    template<class PyClass>
      void visit(PyClass& cl) const 
      {
	cl
	  .def("getBodyId",&ModelPythonVisitor::getBodyId)
	  .def("createData",&ModelPythonVisitor::createData)

	  .def("__str__",&ModelPythonVisitor::toString)

	  .add_property("nq", &ModelPythonVisitor::nq)
	  .add_property("nv", &ModelPythonVisitor::nv)
	  .add_property("nbody", &ModelPythonVisitor::nbody)
	  .add_property("inertias",
			bp::make_function(&ModelPythonVisitor::inertias,
					  bp::return_internal_reference<>())  )
	  .add_property("jointPlacements",
			bp::make_function(&ModelPythonVisitor::jointPlacements,
					  bp::return_internal_reference<>())  )
	  .add_property("parents", 
			bp::make_function(&ModelPythonVisitor::parents,
					  bp::return_internal_reference<>())  )
	  .add_property("names",
			bp::make_function(&ModelPythonVisitor::names,
					  bp::return_internal_reference<>())  )
	  .add_property("gravity",&ModelPythonVisitor::gravity,&ModelPythonVisitor::setGravity)

	  .def("BuildEmptyModel",&ModelPythonVisitor::maker_empty)
	  .staticmethod("BuildEmptyModel")
	  .def("BuildHumanoidSimple",&ModelPythonVisitor::maker_humanoidSimple)
	  .staticmethod("BuildHumanoidSimple")
	  ;
      }

      static Model::Index getBodyId( const ModelHandler & modelPtr, const std::string & name )
      { return  modelPtr->getBodyId(name); }
      static boost::shared_ptr<Data> createData(const ModelHandler& m )
      {	return boost::shared_ptr<Data>( new Data(*m) );      } 
      
      static int nq( ModelHandler & m ) { return m->nq; }
      static int nv( ModelHandler & m ) { return m->nv; }
      static int nbody( ModelHandler & m ) { return m->nbody; }
      static std::vector<Inertia> & inertias( ModelHandler & m ) { return m->inertias; }
      static std::vector<SE3> & jointPlacements( ModelHandler & m ) { return m->jointPlacements; }
      static std::vector<Model::Index> & parents( ModelHandler & m ) { return m->parents; }
      static std::vector<std::string> & names ( ModelHandler & m ) { return m->names; }
      static Motion gravity( ModelHandler & m ) { return m->gravity; }
      static void setGravity( ModelHandler & m,const Motion_fx & g ) { m->gravity = g; }

      static ModelHandler maker_empty()
      {
	return ModelHandler( new Model(),true );
      }
      static ModelHandler maker_humanoidSimple()
      {
	Model * model = new Model();
	buildModels::humanoidSimple(*model);
	return ModelHandler( model,true );
      }

      static std::string toString(const ModelHandler& m) 
      {	  std::ostringstream s; s << *m; return s.str();       }

      /* --- Expose --------------------------------------------------------- */
      static void expose()
      {
	bp::class_< std::vector<Index> >("StdVec_Index")
	  .def(bp::vector_indexing_suite< std::vector<Index> >());
	bp::class_< std::vector<std::string> >("StdVec_StdString")
	  .def(bp::vector_indexing_suite< std::vector<std::string> >());

	bp::class_<ModelHandler>("Model",
				 "Articulated rigid body model (const)",
				 bp::no_init)
	  .def(ModelPythonVisitor());
    
	/* Not sure if it is a good idea to enable automatic
	 * conversion. Prevent it for now */
	//bp::to_python_converter< Model,ModelPythonVisitor >();
	bp::to_python_converter< ModelHandler::SmartPtr_t,ModelPythonVisitor >();
      }


    };
    


  }} // namespace se3::python

#endif // ifndef __se3_python_model_hpp__

