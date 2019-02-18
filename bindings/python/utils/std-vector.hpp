//
// Copyright (c) 2016 CNRS
//

#ifndef __pinocchio_python_utils_std_vector_hpp__
#define __pinocchio_python_utils_std_vector_hpp__

#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <string>
#include <vector>
#include "pinocchio/bindings/python/utils/pickle-vector.hpp"

namespace pinocchio
{
  namespace python
  {
    
    namespace bp = boost::python;
    
    ///
    /// \brief Expose an std::vector from a type given as template argument.
    ///
    /// \tparam T Type to expose as std::vector<T>.
    ///
    /// \sa StdAlignedVectorPythonVisitor
    ///
    template<class T, bool NoProxy = false>
    struct StdVectorPythonVisitor
    : public bp::vector_indexing_suite< typename std::vector<T>,NoProxy >
    {
      
      static void expose(const std::string & class_name,
                         const std::string & doc_string = "")
      {
        bp::class_< std::vector<T> >(class_name.c_str(),doc_string.c_str())
        .def(StdVectorPythonVisitor())
        .def_pickle(PickleVector<typename std::vector<T> >());        
      }
    };
  } // namespace python
} // namespace pinocchio

#endif // ifndef __pinocchio_python_utils_std_vector_hpp__
