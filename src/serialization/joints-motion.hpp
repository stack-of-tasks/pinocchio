//
// Copyright (c) 2019 INRIA
//

#ifndef __pinocchio_serialization_joints_motion_hpp__
#define __pinocchio_serialization_joints_motion_hpp__

#include "pinocchio/serialization/fwd.hpp"

#include <boost/serialization/split_free.hpp>
#include <boost/serialization/vector.hpp>

namespace boost
{
  namespace serialization
  {
    
    template <class Archive, typename Scalar, int Options, int axis>
    void serialize(Archive & ar,
                   pinocchio::MotionRevoluteTpl<Scalar,Options,axis> & m,
                   const unsigned int /*version*/)
    {
      ar & make_nvp("w",m.angularRate());
    }
  
    template <class Archive, typename Scalar, int Options, int axis>
    void serialize(Archive & ar,
                   pinocchio::MotionPrismaticTpl<Scalar,Options,axis> & m,
                   const unsigned int /*version*/)
    {
      ar & make_nvp("v",m.linearRate());
    }
  
    template <class Archive, typename Scalar, int Options>
    void serialize(Archive & ar,
                   pinocchio::MotionSphericalTpl<Scalar,Options> & m,
                   const unsigned int /*version*/)
    {
      ar & make_nvp("angular",m.angular());
    }
  
    template <class Archive, typename Scalar, int Options>
    void serialize(Archive & ar,
                   pinocchio::MotionTranslationTpl<Scalar,Options> & m,
                   const unsigned int /*version*/)
    {
      ar & make_nvp("linear",m.linear());
    }
    
    template <class Archive, typename Scalar, int Options>
    void serialize(Archive & ar,
                   pinocchio::MotionPlanarTpl<Scalar,Options> & m,
                   const unsigned int /*version*/)
    {
      ar & make_nvp("data",m.data());
    }
    
    template <class Archive, typename Scalar, int Options>
    void serialize(Archive & ar,
                   pinocchio::MotionRevoluteUnalignedTpl<Scalar,Options> & m,
                   const unsigned int /*version*/)
    {
      ar & make_nvp("axis",m.axis());
      ar & make_nvp("w",m.angularRate());
    }
    
    template <class Archive, typename Scalar, int Options>
    void serialize(Archive & ar,
                   pinocchio::MotionPrismaticUnalignedTpl<Scalar,Options> & m,
                   const unsigned int /*version*/)
    {
      ar & make_nvp("axis",m.axis());
      ar & make_nvp("v",m.linearRate());
    }
    
  }
}

#endif // ifndef __pinocchio_serialization_joints_motion_hpp__
