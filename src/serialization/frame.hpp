//
// Copyright (c) 2019 INRIA
//

#ifndef __pinocchio_serialization_frame_hpp__
#define __pinocchio_serialization_frame_hpp__

#include "pinocchio/multibody/frame.hpp"
#include "pinocchio/serialization/fwd.hpp"
#include "pinocchio/serialization/se3.hpp"

#include <boost/serialization/split_free.hpp>
#include <boost/serialization/vector.hpp>

namespace boost
{
  namespace serialization
  {
    
    template <class Archive, typename Scalar, int Options>
    void save(Archive & ar,
              const pinocchio::FrameTpl<Scalar,Options> & f,
              const unsigned int /*version*/)
    {
      ar & make_nvp("name",f.name);
      ar & make_nvp("parent",f.parent);
      ar & make_nvp("previousFrame",f.previousFrame);
      ar & make_nvp("placement",f.placement);
      ar & make_nvp("type",f.type);
    }
    
    template <class Archive, typename Scalar, int Options>
    void load(Archive & ar,
              pinocchio::FrameTpl<Scalar,Options> & f,
              const unsigned int /*version*/)
    {
      ar >> make_nvp("name",f.name);
      ar >> make_nvp("parent",f.parent);
      ar >> make_nvp("previousFrame",f.previousFrame);
      ar >> make_nvp("placement",f.placement);
      ar >> make_nvp("type",f.type);
    }
    
    template <class Archive, typename Scalar, int Options>
    void serialize(Archive & ar,
                   pinocchio::FrameTpl<Scalar,Options> & f,
                   const unsigned int version)
    {
      split_free(ar,f,version);
    }
    
  }
}

#endif // ifndef __pinocchio_serialization_frame_hpp__

