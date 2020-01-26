//
// Copyright (c) 2016-2020 CNRS INRIA
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
    
    ///
    /// \brief Register the conversion from a Python list to a std::vector
    ///
    /// \tparam vector_type A std container (e.g. std::vector or std::list)
    ///
    template<typename vector_type>
    struct StdContainerFromPythonList
    {
      typedef typename vector_type::value_type T;
      
      /// \brief Check if obj_ptr can be converted
      static void* convertible(PyObject* obj_ptr)
      {
        // Check if it is a list
        if(!PyList_Check(obj_ptr)) return 0;
        
        // Retrieve the underlying list
        ::boost::python::object bp_obj(::boost::python::handle<>(::boost::python::borrowed(obj_ptr)));
        ::boost::python::list bp_list(bp_obj);
        ::boost::python::ssize_t list_size = ::boost::python::len(bp_list);
        
        // Check if all the elements contained in the current vector is of type T
        for(::boost::python::ssize_t k = 0; k < list_size; ++k)
        {
          ::boost::python::extract<T> elt(bp_list[k]);
          if(!elt.check()) return 0;
        }
        
        return obj_ptr;
      }
      
      /// \brief Allocate the std::vector and fill it with the element contained in the list
      static void construct(PyObject* obj_ptr,
                            boost::python::converter::rvalue_from_python_stage1_data * memory)
      {
        // Extract the list
        ::boost::python::object bp_obj(::boost::python::handle<>(::boost::python::borrowed(obj_ptr)));
        ::boost::python::list bp_list(bp_obj);
        ::boost::python::ssize_t list_size = ::boost::python::len(bp_list);
        
        void * storage = reinterpret_cast<::boost::python::converter::rvalue_from_python_storage<vector_type>*>
        (reinterpret_cast<void*>(memory))->storage.bytes;
        
        // Build the std::vector
        new (storage) vector_type;
        vector_type & vector = *reinterpret_cast<vector_type*>(storage);
        vector.reserve((size_t)list_size);
        
        // Populate the vector
        for(::boost::python::ssize_t k = 0; k < list_size; ++k)
        {
          ::boost::python::extract<T> elt(bp_list[k]);
          vector.push_back(elt());
        }
        
        // Validate the construction
        memory->convertible = storage;
      }
      
      static void register_converter()
      {
        ::boost::python::converter::registry::push_back(&convertible,
                                                        &construct,
                                                        ::boost::python::type_id<vector_type>());
      }
    };
    
    ///
    /// \brief Expose an std::vector from a type given as template argument.
    ///
    /// \tparam T Type to expose as std::vector<T>.
    /// \tparam Allocator Type for the Allocator in std::vector<T,Allocator>.
    /// \tparam NoProxy When set to false, the elements will be copied when returned to Python.
    /// \tparam EnableFromPythonListConverter Enables the conversion from a Python list to a std::vector<T,Allocator>
    ///
    /// \sa StdAlignedVectorPythonVisitor
    ///
    template<class T, class Allocator = std::allocator<T>, bool NoProxy = false, bool EnableFromPythonListConverter = true>
    struct StdVectorPythonVisitor
    : public ::boost::python::vector_indexing_suite<typename std::vector<T,Allocator>, NoProxy>
    , public StdContainerFromPythonList< std::vector<T,Allocator> >
    {
      typedef std::vector<T,Allocator> vector_type;
      typedef StdContainerFromPythonList<vector_type> FromPythonListConverter;
      
      static void expose(const std::string & class_name,
                         const std::string & doc_string = "")
      {
        ::boost::python::class_<vector_type>(class_name.c_str(),doc_string.c_str())
        .def(StdVectorPythonVisitor())
        .def_pickle(PickleVector<vector_type>());
        
        // Register conversion
        if(EnableFromPythonListConverter)
          FromPythonListConverter::register_converter();
      }
    };
    
  } // namespace python
} // namespace pinocchio

#endif // ifndef __pinocchio_python_utils_std_vector_hpp__
