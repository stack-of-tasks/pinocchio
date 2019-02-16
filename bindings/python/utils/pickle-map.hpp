//
// Copyright (c) 2019 CNRS
//

#ifndef __pinocchio_python_utils_pickle_map_hpp__
#define __pinocchio_python_utils_pickle_map_hpp__

#include "pinocchio/bindings/python/utils/pickle-vector.hpp"

namespace pinocchio
{
  namespace python
  {
    namespace bp = boost::python;
    ///
    /// \brief Create a pickle interface for the std::map and aligned map
    ///
    /// \tparam VecType Map Type to pickle
    ///
    /// \sa Pickle
    ///
    template<typename VecType>
    struct PickleMap : public PickleVector<VecType>
    { 
      static void setstate(bp::object op, bp::tuple tup)
      {
        VecType& o = bp::extract<VecType&>(op)(); 
        bp::stl_input_iterator<typename VecType::value_type> begin(tup[0]), end;
        o.insert(begin,end);
      }
    };
  }
}

#endif // ifndef __pinocchio_python_utils_pickle_map_hpp__
