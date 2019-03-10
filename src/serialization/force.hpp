//
// Copyright (c) 2019 INRIA
//

#ifndef __pinocchio_serialization_force_hpp__
#define __pinocchio_serialization_force_hpp__

#include "pinocchio/spatial/force.hpp"
#include "pinocchio/serialization/fwd.hpp"

#include <boost/serialization/split_free.hpp>
#include <boost/serialization/vector.hpp>

namespace boost
{
  namespace serialization
  {
    
    template <class Archive, typename Scalar, int Options>
    void save(Archive & ar,
              const pinocchio::ForceTpl<Scalar,Options> & f,
              const unsigned int /*version*/)
    {
      ar & make_nvp("linear",make_array(f.linear().data(),3));
      ar & make_nvp("angular",make_array(f.angular().data(),3));
    }
    
    template <class Archive, typename Scalar, int Options>
    void load(Archive & ar,
              pinocchio::ForceTpl<Scalar,Options> & f,
              const unsigned int /*version*/)
    {
      ar >> make_nvp("linear",make_array(f.linear().data(),3));
      ar >> make_nvp("angular",make_array(f.angular().data(),3));
    }
    
    template <class Archive, typename Scalar, int Options>
    void serialize(Archive & ar,
                   pinocchio::ForceTpl<Scalar,Options> & f,
                   const unsigned int version)
    {
      split_free(ar,f,version);
    }
    
  }
}

#endif // ifndef __pinocchio_serialization_force_hpp__

