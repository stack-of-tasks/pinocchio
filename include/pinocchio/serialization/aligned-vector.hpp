//
// Copyright (c) 2017-2019 CNRS INRIA
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
    
    template <class Archive, typename T>
    void serialize(Archive & ar,
                   pinocchio::container::aligned_vector<T> & v,
                   const unsigned int version)
    {
      typedef typename pinocchio::container::aligned_vector<T>::vector_base vector_base;
      split_free(ar,*static_cast<vector_base*>(&v),version);
    }
    
#if BOOST_VERSION / 100 % 1000 == 58
    template<class T>
    inline
    const fixme::nvp< typename pinocchio::container::aligned_vector<T>::vector_base >
    make_nvp(const char * name, pinocchio::container::aligned_vector<T> & t)
    {
      typedef typename pinocchio::container::aligned_vector<T>::vector_base vector_base;
      return fixme::nvp< vector_base >(name, *static_cast<vector_base*>(&t));
    }
#else
    template<class T>
    inline
    const nvp< typename pinocchio::container::aligned_vector<T>::vector_base >
    make_nvp(const char * name, pinocchio::container::aligned_vector<T> & t)
    {
      typedef typename pinocchio::container::aligned_vector<T>::vector_base vector_base;
      return nvp< vector_base >(name, *static_cast<vector_base*>(&t));
    }
#endif

  }
  
}

#endif // ifndef __pinocchio_serialization_aligned_vector_hpp__
