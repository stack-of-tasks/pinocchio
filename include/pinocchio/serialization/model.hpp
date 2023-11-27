//
// Copyright (c) 2019 INRIA
//

#ifndef __pinocchio_multibody_model_serialization_hpp__
#define __pinocchio_multibody_model_serialization_hpp__

#include <boost/serialization/string.hpp>
#include <boost/serialization/variant.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/map.hpp>

#include "pinocchio/serialization/fwd.hpp"
#include "pinocchio/serialization/aligned-vector.hpp"
#include "pinocchio/serialization/spatial.hpp"
#include "pinocchio/serialization/joints.hpp"
#include "pinocchio/serialization/frame.hpp"

namespace boost
{
  namespace serialization
  {
    template<class Archive, typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
    void serialize(Archive & ar,
                   pinocchio::ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                   const unsigned int /*version*/)
    {
      ar & make_nvp("nq",model.nq);
      ar & make_nvp("nqs",model.nqs);
      ar & make_nvp("idx_qs",model.idx_qs);
      ar & make_nvp("nv",model.nv);
      ar & make_nvp("nvs",model.nvs);
      ar & make_nvp("idx_vs",model.idx_vs);
      ar & make_nvp("njoints",model.njoints);
      ar & make_nvp("nbodies",model.nbodies);
      ar & make_nvp("nframes",model.nframes);
      ar & make_nvp("parents",model.parents);
      ar & make_nvp("names",model.names);
      ar & make_nvp("supports",model.supports);
      ar & make_nvp("subtrees",model.subtrees);
      ar & make_nvp("gravity",model.gravity);
      ar & make_nvp("name",model.name);
      
      ar & make_nvp("referenceConfigurations",model.referenceConfigurations);
      ar & make_nvp("rotorInertia",model.rotorInertia);
      ar & make_nvp("rotorGearRatio",model.rotorGearRatio);
      ar & make_nvp("friction",model.friction);
      ar & make_nvp("damping",model.damping);
      ar & make_nvp("effortLimit",model.effortLimit);
      ar & make_nvp("velocityLimit",model.velocityLimit);
      ar & make_nvp("lowerPositionLimit",model.lowerPositionLimit);
      ar & make_nvp("upperPositionLimit",model.upperPositionLimit);
      
      ar & make_nvp("inertias",model.inertias);
      ar & make_nvp("jointPlacements",model.jointPlacements);
      
      ar & make_nvp("joints",model.joints);
      ar & make_nvp("frames",model.frames);
    }
    
  } // namespace serialization
} // namespace boost

#endif // ifndef __pinocchio_multibody_model_serialization_hpp__
