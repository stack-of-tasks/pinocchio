//
// Copyright (c) 2017-2019 CNRS INRIA
//

#ifndef __pinocchio_serialization_aligned_vector_hpp__
#define __pinocchio_serialization_aligned_vector_hpp__

#include "pinocchio/container/aligned-vector.hpp"

#include <boost/serialization/split_free.hpp>
#include <boost/serialization/vector.hpp>

namespace boost
{
  
  namespace serialization
  {
    
    template <class Archive, typename T>
    void save(Archive & ar, const se3::container::aligned_vector<T> & v, const unsigned int version)
    {
      typedef typename se3::container::aligned_vector<T>::vector_base vector_base;
      save(ar, *static_cast<const vector_base*>(&v), version);
    }
    
    template <class Archive, typename T>
    void load(Archive & ar, se3::container::aligned_vector<T> & v, const unsigned int version)
    {
      typedef typename se3::container::aligned_vector<T>::vector_base vector_base;
      load(ar, *static_cast<vector_base*>(&v), version);
    }
    
    template <class Archive, typename T>
    void serialize(Archive & ar, se3::container::aligned_vector<T> & v, const unsigned int version)
    {
      split_free(ar,v,version);
    }
    
  }
  
}

#endif // ifndef __pinocchio_serialization_aligned_vector_hpp__
