//
// Copyright (c) 2019 INRIA
//

#ifndef __pinocchio_serialization_inertia_hpp__
#define __pinocchio_serialization_inertia_hpp__

#include "pinocchio/spatial/inertia.hpp"
#include "pinocchio/serialization/fwd.hpp"
#include "pinocchio/serialization/symmetric3.hpp"

#include <boost/serialization/split_free.hpp>
#include <boost/serialization/vector.hpp>

namespace boost
{
  namespace serialization
  {
    
    template <class Archive, typename Scalar, int Options>
    void save(Archive & ar,
              const pinocchio::InertiaTpl<Scalar,Options> & I,
              const unsigned int /*version*/)
    {
      const Scalar mass = I.mass();
      ar & make_nvp("mass",mass);
      ar & make_nvp("lever",make_array(I.lever().data(),3));
      ar & make_nvp("inertia",I.inertia());
    }
    
    template <class Archive, typename Scalar, int Options>
    void load(Archive & ar,
              pinocchio::InertiaTpl<Scalar,Options> & I,
              const unsigned int /*version*/)
    {
      ar >> make_nvp("mass",I.mass());
      ar >> make_nvp("lever",make_array(I.lever().data(),3));
      ar >> make_nvp("inertia",I.inertia());
    }
    
    template <class Archive, typename Scalar, int Options>
    void serialize(Archive & ar,
                   pinocchio::InertiaTpl<Scalar,Options> & I,
                   const unsigned int version)
    {
      split_free(ar,I,version);
    }
    
  }
}

#endif // ifndef __pinocchio_serialization_inertia_hpp__
