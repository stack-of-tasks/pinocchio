#ifndef __se3_python_data_hpp__
#define __se3_python_data_hpp__

#include <eigenpy/exception.hpp>
#include <eigenpy/eigenpy.hpp>
#include "pinocchio/multibody/model.hpp"

#include <boost/shared_ptr.hpp>

namespace se3
{
  namespace python
  {

    /* See the strategy applied for handling the Model object. The same applies
     * here. */
    struct DataHandler
    {
      typedef boost::shared_ptr<Data> SmartPtr_t;
      typedef Data * Ptr_t;

      SmartPtr_t smptr;
      Ptr_t rawptr;
      bool smart;

      DataHandler(Data * data,bool transmitOwnership=false)
	: smptr( transmitOwnership ? data : NULL )
	, rawptr( data )
	, smart( transmitOwnership ) {}
      DataHandler( SmartPtr_t data )
	: smptr(data), rawptr(NULL), smart(true) {}
      ~DataHandler()
      {    
	std::cout << "Destroy data handler " << std::endl;
	if( (!smart) && (rawptr!=NULL) ) delete rawptr;
      }

      Data *       ptr()              { return smart ?  smptr.get() :  rawptr; }
      const Data * ptr()        const { return smart ?  smptr.get() :  rawptr; }
      Data *       operator->()       { return ptr(); }
      const Data * operator->() const { return ptr(); }

      Data &       get()              { return smart ? *smptr       : *rawptr; }
      const Data & get()        const { return smart ? *smptr       : *rawptr; }
      Data &       operator*()        { return get(); }
      const Data&  operator*()  const { return get(); }
    };


    namespace bp = boost::python;

    struct DataPythonVisitor
      : public boost::python::def_visitor< DataPythonVisitor >
    {

    public:

      /* --- Convert From C++ to Python ------------------------------------- */
      static PyObject* convert(DataHandler::SmartPtr_t const& ptr)
      {
	return boost::python::incref(boost::python::object(DataHandler(ptr)).ptr());
      }

      /* --- Exposing C++ API to python through the handler ----------------- */
      template<class PyClass>
      void visit(PyClass& cl) const 
      {
	cl
	  ;
      }


      /* --- Expose --------------------------------------------------------- */
      static void expose()
      {
	bp::class_<DataHandler>("Data",
				 "Articulated rigid body data (const)",
				 bp::no_init)
	  .def(DataPythonVisitor());
    
	/* Not sure if it is a good idea to enable automatic
	 * conversion. Prevent it for now */
	//bp::to_python_converter< Data,DataPythonVisitor >();
	bp::to_python_converter< DataHandler::SmartPtr_t,DataPythonVisitor >();
      }


    };
    


  }} // namespace se3::python

#endif // ifndef __se3_python_data_hpp__

