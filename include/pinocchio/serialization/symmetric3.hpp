//
// Copyright (c) 2019 INRIA
//

#ifndef __pinocchio_serialization_symmetric3_hpp__
#define __pinocchio_serialization_symmetric3_hpp__

#include "pinocchio/spatial/symmetric3.hpp"
#include "pinocchio/serialization/fwd.hpp"

#include <boost/serialization/split_free.hpp>
#include <boost/serialization/vector.hpp>

namespace boost
{
  namespace serialization
  {
    
    template <class Archive, typename Scalar, int Options>
    void save(Archive & ar,
              const pinocchio::Symmetric3Tpl<Scalar,Options> & S,
              const unsigned int /*version*/)
    {
      ar & make_nvp("data",make_array(S.data().data(),6));
    }
    
    template <class Archive, typename Scalar, int Options>
    void load(Archive & ar,
              pinocchio::Symmetric3Tpl<Scalar,Options> & S,
              const unsigned int /*version*/)
    {
      ar >> make_nvp("data",make_array(S.data().data(),6));
    }
    
    template <class Archive, typename Scalar, int Options>
    void serialize(Archive & ar,
                   pinocchio::Symmetric3Tpl<Scalar,Options> & S,
                   const unsigned int version)
    {
      split_free(ar,S,version);
    }
    
  }
}

#endif // ifndef __pinocchio_serialization_symmetric3_hpp__
