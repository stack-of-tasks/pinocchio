//
// Copyright (c) 2019 INRIA
//

#ifndef __pinocchio_python_utils_list_hpp__
#define __pinocchio_python_utils_list_hpp__

#include "pinocchio/bindings/python/fwd.hpp"

#include <boost/python.hpp>
#include <vector>

namespace pinocchio
{
  namespace python
  {
    
    namespace bp = boost::python;
    
    template<typename T>
    std::vector<T> extractList(const bp::list & input_list)
    {
      long size_list = bp::len(input_list);
      std::vector<T> res((size_t)size_list);
      for(long i = 0; i < size_list; ++i)
      {
        res.push_back(bp::extract<T>(input_list[i]));
      }
      
      return res;
    }
    
  } // namespace python
} // namespace pinocchio

#endif // ifndef __pinocchio_python_utils_list_hpp__
