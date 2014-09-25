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
    namespace bp = boost::python;

    typedef Handler<Data> DataHandler;

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
    
	bp::to_python_converter< DataHandler::SmartPtr_t,DataPythonVisitor >();
      }


    };
    


  }} // namespace se3::python

#endif // ifndef __se3_python_data_hpp__

