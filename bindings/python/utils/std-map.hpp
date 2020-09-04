//
// Copyright (c) 2020 INRIA
//

#ifndef __pinocchio_python_utils_map_hpp__
#define __pinocchio_python_utils_map_hpp__

#include <boost/python/suite/indexing/map_indexing_suite.hpp>

namespace pinocchio
{
  namespace python
  {
    namespace details
    {
      template<typename Container>
      struct overload_base_get_item_for_std_map
      : public boost::python::def_visitor< overload_base_get_item_for_std_map<Container> >
      {
        typedef typename Container::value_type value_type;
        typedef typename Container::value_type::second_type data_type;
        typedef typename Container::key_type key_type;
        typedef typename Container::key_type index_type;
        
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
          typename Container::iterator i = container.get().find(idx);
          if (i == container.get().end())
          {
              PyErr_SetString(PyExc_KeyError, "Invalid key");
              bp::throw_error_already_set();
          }
          
          typename bp::to_python_indirect<data_type&,bp::detail::make_reference_holder> convert;
          return bp::object(bp::handle<>(convert(i->second)));
        }
        
        static index_type
        convert_index(Container& /*container*/, PyObject* i_)
        {
          namespace bp = ::boost::python;
          bp::extract<key_type const&> i(i_);
          if (i.check())
          {
            return i();
          }
          else
          {
            bp::extract<key_type> i(i_);
            if (i.check())
              return i();
          }
          
          PyErr_SetString(PyExc_TypeError, "Invalid index type");
          bp::throw_error_already_set();
          return index_type();
        }
      };
      
    }
  }
}

#endif // ifndef __pinocchio_python_utils_map_hpp__
