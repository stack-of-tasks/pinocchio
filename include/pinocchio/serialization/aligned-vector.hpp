//
// Copyright (c) 2017-2024 CNRS INRIA
//

#ifndef __pinocchio_serialization_aligned_vector_hpp__
#define __pinocchio_serialization_aligned_vector_hpp__

#include "pinocchio/container/aligned-vector.hpp"

#include <boost/serialization/nvp.hpp>
#include <boost/serialization/split_free.hpp>
#include "pinocchio/serialization/vector.hpp"

namespace boost
{

  namespace serialization
  {

    template<class Archive, typename T>
    void
    serialize(Archive & ar, pinocchio::container::aligned_vector<T> & v, const unsigned int version)
    {
      split_free(ar, v, version);
    }

#if BOOST_VERSION / 100 % 1000 == 58
    template<class T>
    inline const fixme::nvp<pinocchio::container::aligned_vector<T>>
    make_nvp(const char * name, pinocchio::container::aligned_vector<T> & t)
    {
      return fixme::nvp<pinocchio::container::aligned_vector<T>>(name, t);
    }
#else
    template<class T>
    inline const nvp<pinocchio::container::aligned_vector<T>>
    make_nvp(const char * name, pinocchio::container::aligned_vector<T> & t)
    {
      return nvp<pinocchio::container::aligned_vector<T>>(name, t);
    }
#endif

  } // namespace serialization

} // namespace boost

#endif // ifndef __pinocchio_serialization_aligned_vector_hpp__
