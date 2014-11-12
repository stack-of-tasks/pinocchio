#ifndef __se3_python_parsers_hpp__
#define __se3_python_parsers_hpp__

#include <eigenpy/exception.hpp>
#include <eigenpy/eigenpy.hpp>

#include "pinocchio/python/model.hpp"
#include "pinocchio/python/data.hpp"

#include "pinocchio/multibody/parser/urdf.hpp"

namespace se3
{
  namespace python
  {
    struct ParsersPythonVisitor
    {
      static ModelHandler buildModelFromUrdf( const std::string & filename,
					      bool ff )
      {
	Model * model = new Model();
	*model = se3::urdf::buildModel(filename,ff);
	return ModelHandler(model,true);
      }

      /* --- Expose --------------------------------------------------------- */
      static void expose()
      {
	bp::def("buildModelFromUrdf",buildModelFromUrdf,
		bp::args("Filename (string)",
			 "Free flyer (bool, false for a fixed robot)"),
		"Parse the urdf file given in input and return a proper pinocchio model "
		"(remember to create the corresponding data structure).");
      }

    };
    
  }} // namespace se3::python

#endif // ifndef __se3_python_data_hpp__

