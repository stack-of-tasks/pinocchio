//
// Copyright (c) 2019 CNRS
//

#ifndef __pinocchio_python_utils_pickle_vector_hpp__
#define __pinocchio_python_utils_pickle_vector_hpp__

#include <boost/python.hpp>
#include <boost/python/tuple.hpp>
#include <boost/python/stl_iterator.hpp>

namespace pinocchio
{
  namespace python
  {
    namespace bp = boost::python;
    ///
    /// \brief Create a pickle interface for the std::vector and aligned vector
    ///
    /// \tparam VecType Vector Type to pickle
    ///
    /// \sa Pickle
    ///
    template<typename VecType>
    struct PickleVector : bp::pickle_suite
    { 
      static bp::tuple getinitargs(const VecType&) {    return bp::make_tuple();      }
      static bp::tuple getstate(bp::object op)
      { return bp::make_tuple(bp::list(bp::extract<const VecType&>(op)()));           }
      static void setstate(bp::object op, bp::tuple tup)
      {
        VecType& o = bp::extract<VecType&>(op)(); 
        bp::stl_input_iterator<typename VecType::value_type> begin(tup[0]), end;
        o.insert(o.begin(),begin,end);
      }
    };
  }
}

#endif // ifndef __pinocchio_python_utils_pickle_vector_hpp__
