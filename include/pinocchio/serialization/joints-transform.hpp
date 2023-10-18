//
// Copyright (c) 2019 INRIA
//

#ifndef __pinocchio_serialization_joints_transform_hpp__
#define __pinocchio_serialization_joints_transform_hpp__

#include "pinocchio/spatial/motion.hpp"
#include "pinocchio/serialization/fwd.hpp"

#include <boost/serialization/split_free.hpp>
#include <boost/serialization/vector.hpp>

namespace boost
{
  namespace serialization
  {
    
    template <class Archive, typename Scalar, int Options, int axis>
    void serialize(Archive & ar,
                   pinocchio::TransformRevoluteTpl<Scalar,Options,axis> & m,
                   const unsigned int /*version*/)
    {
      ar & make_nvp("sin",m.sin());
      ar & make_nvp("cos",m.cos());
    }
  
    template <class Archive, typename Scalar, int Options, int axis>
    void serialize(Archive & ar,
                   pinocchio::TransformPrismaticTpl<Scalar,Options,axis> & m,
                   const unsigned int /*version*/)
    {
      ar & make_nvp("displacement",m.displacement());
    }
  
    template <class Archive, typename Scalar, int Options>
    void serialize(Archive & ar,
                   pinocchio::TransformTranslationTpl<Scalar,Options> & m,
                   const unsigned int /*version*/)
    {
      ar & make_nvp("translation",m.translation());
    }
    
  }
}

#endif // ifndef __pinocchio_serialization_joints_transform_hpp__
