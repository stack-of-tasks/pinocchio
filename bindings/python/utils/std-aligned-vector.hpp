//
// Copyright (c) 2016-2020 CNRS INRIA
//

#ifndef __pinocchio_python_utils_std_aligned_vector_hpp__
#define __pinocchio_python_utils_std_aligned_vector_hpp__

#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <string>

#include "pinocchio/container/aligned-vector.hpp"

#include "pinocchio/bindings/python/utils/pickle-vector.hpp"
#include "pinocchio/bindings/python/utils/std-vector.hpp"

namespace pinocchio
{
  namespace python
  {
    
    ///
    /// \brief Expose an container::aligned_vector from a type given as template argument.
    ///
    /// \tparam T Type to expose as container::aligned_vector<T>.
    /// \tparam EnableFromPythonListConverter Enables the conversion from a Python list to a container::aligned_vector<T>.
    ///
    /// \sa StdAlignedVectorPythonVisitor
    ///
    template<class T, bool NoProxy = false, bool EnableFromPythonListConverter = true>
    struct StdAlignedVectorPythonVisitor
    : public ::boost::python::vector_indexing_suite<typename container::aligned_vector<T>,NoProxy>
    , public StdContainerFromPythonList< container::aligned_vector<T> >
    {
      typedef container::aligned_vector<T> vector_type;
      typedef StdContainerFromPythonList<vector_type> FromPythonListConverter;
      
      static ::boost::python::class_<vector_type> expose(const std::string & class_name,
                                                         const std::string & doc_string = "")
      {
        namespace bp = boost::python;
        
        bp::class_<vector_type> cl(class_name.c_str(),doc_string.c_str());
        cl
        .def(StdAlignedVectorPythonVisitor())
        .def("tolist",&FromPythonListConverter::tolist,bp::arg("self"),
             "Returns the aligned_vector as a Python list.")
        .def_pickle(PickleVector<vector_type>());
        
        // Register conversion
        if(EnableFromPythonListConverter)
          FromPythonListConverter::register_converter();
        
        return cl;
      }
    };
  } // namespace python
} // namespace pinocchio

#endif // ifndef __pinocchio_python_utils_std_aligned_vector_hpp__
