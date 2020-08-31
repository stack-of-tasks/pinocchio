//
// Copyright (c) 2016-2020 CNRS INRIA
//

#ifndef __pinocchio_python_utils_std_vector_hpp__
#define __pinocchio_python_utils_std_vector_hpp__

#include <boost/python.hpp>
#include <boost/python/stl_iterator.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>

#include <string>
#include <vector>
#include <iterator>

#include "pinocchio/bindings/python/utils/pickle-vector.hpp"

namespace pinocchio
{
  namespace python
  {
  
    namespace details
    {
      template<typename Container>
      struct overload_base_get_item_for_std_vector
      : public boost::python::def_visitor< overload_base_get_item_for_std_vector<Container> >
      {
        typedef typename Container::value_type value_type;
        typedef typename Container::value_type data_type;
        typedef size_t index_type;
        
        template <class Class>
        void visit(Class& cl) const
        {
          cl
          .def("__getitem__", &base_get_item);
        }
        
      private:
        
        static boost::python::object
        base_get_item(boost::python::back_reference<Container&> container, PyObject* i_)
        {
          namespace bp = ::boost::python;

          index_type idx = convert_index(container.get(), i_);
          typename Container::iterator i = container.get().begin();
          std::advance(i, idx);
          if (i == container.get().end())
          {
              PyErr_SetString(PyExc_KeyError, "Invalid index");
              bp::throw_error_already_set();
          }
          
          typename bp::to_python_indirect<data_type&,bp::detail::make_reference_holder> convert;
          return bp::object(bp::handle<>(convert(*i)));
        }
        
        static index_type
        convert_index(Container & container, PyObject* i_)
        {
          namespace bp = boost::python;
          bp::extract<long> i(i_);
          if (i.check())
          {
            long index = i();
            if (index < 0)
              index += container.size();
            if (index >= long(container.size()) || index < 0)
            {
              PyErr_SetString(PyExc_IndexError, "Index out of range");
              bp::throw_error_already_set();
            }
            return index;
          }
          
          PyErr_SetString(PyExc_TypeError, "Invalid index type");
          bp::throw_error_already_set();
          return index_type();
        }
      };
    }
  
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
        namespace bp = boost::python;
        
        // Check if it is a list
        if(!PyList_Check(obj_ptr)) return 0;
        
        // Retrieve the underlying list
        bp::object bp_obj(bp::handle<>(bp::borrowed(obj_ptr)));
        bp::list bp_list(bp_obj);
        bp::ssize_t list_size = bp::len(bp_list);
        
        // Check if all the elements contained in the current vector is of type T
        for(bp::ssize_t k = 0; k < list_size; ++k)
        {
          bp::extract<T> elt(bp_list[k]);
          if(!elt.check()) return 0;
        }
        
        return obj_ptr;
      }
      
      /// \brief Allocate the std::vector and fill it with the element contained in the list
      static void construct(PyObject* obj_ptr,
                            boost::python::converter::rvalue_from_python_stage1_data * memory)
      {
        namespace bp = boost::python;
        
        // Extract the list
        bp::object bp_obj(bp::handle<>(bp::borrowed(obj_ptr)));
        bp::list bp_list(bp_obj);
        
        void * storage = reinterpret_cast< bp::converter::rvalue_from_python_storage<vector_type>*>
        (reinterpret_cast<void*>(memory))->storage.bytes;
        
        typedef bp::stl_input_iterator<T> iterator;
        
        // Build the std::vector
        new (storage) vector_type(iterator(bp_list),
                                  iterator());
        
        // Validate the construction
        memory->convertible = storage;
      }
      
      static void register_converter()
      {
        ::boost::python::converter::registry::push_back(&convertible,
                                                        &construct,
                                                        ::boost::python::type_id<vector_type>());
      }
      
      static ::boost::python::list tolist(vector_type & self)
      {
        namespace bp = boost::python;
        
        typedef bp::iterator<vector_type> iterator;
        bp::list python_list(iterator()(self));
        
        return python_list;
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
      
      static ::boost::python::class_<vector_type> expose(const std::string & class_name,
                                                         const std::string & doc_string = "")
      {
        namespace bp = boost::python;
        
        bp::class_<vector_type> cl(class_name.c_str(),doc_string.c_str());
        cl
        .def(StdVectorPythonVisitor())
        .def("tolist",&FromPythonListConverter::tolist,bp::arg("self"),
             "Returns the std::vector as a Python list.")
        .def_pickle(PickleVector<vector_type>());
        
        // Register conversion
        if(EnableFromPythonListConverter)
          FromPythonListConverter::register_converter();
        
        return cl;
      }
    };
    
  } // namespace python
} // namespace pinocchio

#endif // ifndef __pinocchio_python_utils_std_vector_hpp__
