//
// Copyright (c) 2019 INRIA
//

#ifndef __pinocchio_serialization_motion_hpp__
#define __pinocchio_serialization_motion_hpp__

#include "pinocchio/spatial/motion.hpp"
#include "pinocchio/serialization/fwd.hpp"

#include <boost/serialization/split_free.hpp>
#include <boost/serialization/vector.hpp>

namespace boost
{
  namespace serialization
  {
    
    template <class Archive, typename Scalar, int Options>
    void save(Archive & ar,
              const pinocchio::MotionTpl<Scalar,Options> & m,
              const unsigned int /*version*/)
    {
      ar & make_nvp("linear",make_array(m.linear().data(),3));
      ar & make_nvp("angular",make_array(m.angular().data(),3));
    }
    
    template <class Archive, typename Scalar, int Options>
    void load(Archive & ar,
              pinocchio::MotionTpl<Scalar,Options> & m,
              const unsigned int /*version*/)
    {
      ar >> make_nvp("linear",make_array(m.linear().data(),3));
      ar >> make_nvp("angular",make_array(m.angular().data(),3));
    }
    
    template <class Archive, typename Scalar, int Options>
    void serialize(Archive & ar,
                   pinocchio::MotionTpl<Scalar,Options> & m,
                   const unsigned int version)
    {
      split_free(ar,m,version);
    }
  
    template <class Archive, typename Scalar, int Options>
    void serialize(Archive & /*ar*/,
                   pinocchio::MotionZeroTpl<Scalar,Options> & /*m*/,
                   const unsigned int /*version*/)
    {
      // Nothing to do
    }
    
  }
}

#endif // ifndef __pinocchio_serialization_motion_hpp__


