//
// Copyright (c) 2019 INRIA
//

#ifndef __pinocchio_python_utils_list_hpp__
#define __pinocchio_python_utils_list_hpp__

#include "pinocchio/bindings/python/fwd.hpp"

#include <eigenpy/exception.hpp>

#include <boost/python.hpp>
#include <boost/python/type_id.hpp>
#include <vector>
#include <sstream>
#include <string>

namespace pinocchio
{
  namespace python
  {
  
    template<typename T, class Allocator>
    void extract(const boost::python::list & list, std::vector<T,Allocator> & vec)
    {
      namespace bp = boost::python;
      
      size_t size_list = (size_t)bp::len(list);
      vec.resize(size_list);
      for(size_t i = 0; i < size_list; ++i)
      {
        bp::extract<T> input_T(list[i]);
        if(input_T.check())
          vec[i] = input_T();
        else
        {
          const std::string classname
          = bp::extract<std::string>(list[i].attr("__class__").attr("__name__"));
          std::stringstream ss;
          ss << "The conversion from " << classname << " to "
             << bp::type_id<T>().name() << " has failed." << std::endl;
          throw eigenpy::Exception(ss.str());
        }
      }
    }
    
    template<typename T>
    std::vector<T,std::allocator<T> > extract(const boost::python::list & list)
    {
      std::vector<T,std::allocator<T> > vec;
      extract(list,vec);
      return vec;
    }
    
    template<typename T, class Allocator>
    std::vector<T,Allocator> extract(const boost::python::list & list)
    {
      std::vector<T,Allocator> vec;
      extract(list,vec);
      return vec;
    }
    
  } // namespace python
} // namespace pinocchio

#endif // ifndef __pinocchio_python_utils_list_hpp__
