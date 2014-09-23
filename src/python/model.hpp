#ifndef __se3_python_model_hpp__
#define __se3_python_model_hpp__

#include <eigenpy/exception.hpp>
#include <eigenpy/eigenpy.hpp>
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/parser/sample-models.hpp"

#include <boost/shared_ptr.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <boost/python/return_internal_reference.hpp>

namespace se3
{
  namespace python
  {

    /* There might be several way of building and owning a model object:

     *   - build from C++ (as static or dynamic, wathever), and owned by
     * C++. In that case, use a ModelHandler with no ownership ModelHandler(
     * Model*,false).
     *   - build from C++ but owned by python. Python would be in charge of
     * destroying it when it is done. In that case, use
     * ModelHandler(Model*,true). Take care not to destroy the object by
     * yourself, otherwise there might have access to deallocated memory and
     * double destruction.
     *   - build and managed by shared_ptr. It is the best way, in the sense
     * that the object is manage both by C++ and python. It will be released
     * only when both are done with it.  In that case, simply give Python the
     * shared_ptr ModelHandler( shared_ptr<Model> ).
     * 
     * It is not possible to construct a Model from python, because of Eigen
     * memory alignement.  The python visitor does not define any
     * constructor. Instead, some static makers (function allocating memory and
     * returning a pointer Model *) are implemented.  When Python is making
     * such a Model, it is stored in a shared_ptr and can be access directly
     * from C++. By default, when passing a Model to python, it is kept as a
     * raw pointer. Access is then done from python to the C++ memory without
     * any guarantee. If you want to enforce memory access, prefer transmitting
     * a shared_ptr to python.
     */
    struct ModelHandler
    {
      typedef boost::shared_ptr<Model> SmartPtr_t;
      typedef Model * Ptr_t;

      SmartPtr_t smptr;
      Ptr_t rawptr;
      bool smart;

      ModelHandler(Model * model,bool transmitOwnership=false)
	: smptr( transmitOwnership ? model : NULL )
	, rawptr( model )
	, smart( transmitOwnership ) {}
      ModelHandler( SmartPtr_t model )
	: smptr(model), rawptr(NULL), smart(true) {}
      ~ModelHandler()
      {    
	std::cout << "Destroy model handler " << std::endl;
	if( (!smart) && (rawptr!=NULL) ) delete rawptr;
      }

      Model *       ptr()              { return smart ?  smptr.get() :  rawptr; }
      const Model * ptr()        const { return smart ?  smptr.get() :  rawptr; }
      Model *       operator->()       { return ptr(); }
      const Model * operator->() const { return ptr(); }

      Model &       get()              { return smart ? *smptr       : *rawptr; }
      const Model & get()        const { return smart ? *smptr       : *rawptr; }
      Model &       operator*()        { return get(); }
      const Model&  operator*()  const { return get(); }
    };


    namespace bp = boost::python;

    struct InertiasVisitor
      : public boost::python::def_visitor< InertiasVisitor >
    {
      typedef typename eigenpy::UnalignedEquivalent<Inertia>::type Inertia_fx;
      typedef std::vector<Inertia> Inertias;
      
      template<class PyClass>
      void visit(PyClass& cl) const 
      {
	cl
	  .def("__getitem__", &InertiasVisitor::getItem)
	  .def("__setitem__", &InertiasVisitor::setItem)
	  .def("__len__",&InertiasVisitor::length)
	  ;
      }

      static Inertia getItem( const Inertias & Ys,int i) { return Ys[i]; }
      static void setItem( Inertias & Ys,int i,const Inertia_fx & Y)
      { 
	std::cout << "Y = " << Y << std::endl;
	Ys[i] = Y; 
      }
      static int length( const Inertias & Ys ) { return Ys.size(); }

    };


    struct ParentsVisitor
     : public boost::python::def_visitor< ParentsVisitor >
    {
      typedef Model::Index Index;
      typedef std::vector<Index> Parents;
      
      template<class PyClass>
      void visit(PyClass& cl) const 
      {
	cl
	  .def("__getitem__", &ParentsVisitor::getItem)
	  .def("__setitem__", &ParentsVisitor::setItem)
	  .def("__len__",&ParentsVisitor::length)
	  ;
      }

      static Index getItem( const Parents & Ys,int i) { return Ys[i]; }
      static void setItem( Parents & Ys,int i,const Index & Y)
      { 
	std::cout << "p = " << Y << std::endl;
	Ys[i] = Y; 
      }
      static int length( const Parents & Ys ) { return Ys.size(); }
    };


    struct ModelPythonVisitor
      : public boost::python::def_visitor< ModelPythonVisitor >
    {

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

	  .add_property("inertias",
			bp::make_function(&ModelPythonVisitor::inertias,
					  bp::return_internal_reference<>())  )

	  .add_property("parents", 
			bp::make_function(&ModelPythonVisitor::parents,
					  bp::return_internal_reference<>())  )

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
      
      typedef std::vector<Inertia> Inertias_t;
      static Inertias_t & inertias( ModelHandler & m ) { return m->inertias; }
      static void set_inertias( ModelHandler & m,const Inertias_t & Ys )
      { m->inertias = Ys; }
      static ParentsVisitor::Parents & parents( ModelHandler & m ) { return m->parents; }

      

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
	bp::class_<InertiasVisitor::Inertias>("ListInertia")
	  .def(InertiasVisitor());

	bp::class_<ParentsVisitor::Parents>("ListInertia")
	  .def(ParentsVisitor());
	  //.def(bp::vector_indexing_suite<Inertias_t>());

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

