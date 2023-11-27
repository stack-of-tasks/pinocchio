//
// Copyright (c) 2019 INRIA
//

#ifndef __pinocchio_serialization_se3_hpp__
#define __pinocchio_serialization_se3_hpp__

#include "pinocchio/spatial/se3.hpp"
#include "pinocchio/serialization/fwd.hpp"

#include <boost/serialization/split_free.hpp>
#include <boost/serialization/vector.hpp>

namespace boost
{
  namespace serialization
  {
    
    template <class Archive, typename Scalar, int Options>
    void save(Archive & ar,
              const pinocchio::SE3Tpl<Scalar,Options> & M,
              const unsigned int /*version*/)
    {
      ar & make_nvp("translation",make_array(M.translation().data(),3));
      ar & make_nvp("rotation",make_array(M.rotation().data(),9));
    }
    
    template <class Archive, typename Scalar, int Options>
    void load(Archive & ar,
              pinocchio::SE3Tpl<Scalar,Options> & M,
              const unsigned int /*version*/)
    {
      ar >> make_nvp("translation",make_array(M.translation().data(),3));
      ar >> make_nvp("rotation",make_array(M.rotation().data(),9));
    }
    
    template <class Archive, typename Scalar, int Options>
    void serialize(Archive & ar,
                   pinocchio::SE3Tpl<Scalar,Options> & M,
                   const unsigned int version)
    {
      split_free(ar,M,version);
    }
    
  }
}

#endif // ifndef __pinocchio_serialization_se3_hpp__
