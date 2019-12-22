//
// Copyright (c) 2019 INRIA
//

#ifndef __pinocchio_serialization_joints_constraint_hpp__
#define __pinocchio_serialization_joints_constraint_hpp__

#include "pinocchio/serialization/fwd.hpp"

#include <boost/serialization/split_free.hpp>
#include <boost/serialization/vector.hpp>

namespace boost
{
  namespace serialization
  {
    
    template <class Archive, typename Scalar, int Options, int axis>
    void serialize(Archive & /*ar*/,
                   pinocchio::ConstraintRevoluteTpl<Scalar,Options,axis> & /*S*/,
                   const unsigned int /*version*/)
    {}
  
    template <class Archive, typename Scalar, int Options, int axis>
    void serialize(Archive & /*ar*/,
                   pinocchio::ConstraintPrismaticTpl<Scalar,Options,axis> & /*S*/,
                   const unsigned int /*version*/)
    {}
  
    template <class Archive, typename Scalar, int Options>
    void serialize(Archive & /*ar*/,
                   pinocchio::ConstraintSphericalTpl<Scalar,Options> & /*S*/,
                   const unsigned int /*version*/)
    {}
  
    template <class Archive, typename Scalar, int Options>
    void serialize(Archive & /*ar*/,
                   pinocchio::ConstraintTranslationTpl<Scalar,Options> & /*S*/,
                   const unsigned int /*version*/)
    {}
  
    template <class Archive, typename Scalar, int Options>
    void serialize(Archive & /*ar*/,
                   pinocchio::ConstraintIdentityTpl<Scalar,Options> & /*S*/,
                   const unsigned int /*version*/)
    {}
  
    template <class Archive, typename Scalar, int Options>
    void serialize(Archive & ar,
                   pinocchio::ConstraintRevoluteUnalignedTpl<Scalar,Options> & S,
                   const unsigned int /*version*/)
    {
      ar & make_nvp("axis",S.axis());
    }
  
    template <class Archive, typename Scalar, int Options>
    void serialize(Archive & ar,
                   pinocchio::ConstraintPrismaticUnalignedTpl<Scalar,Options> & S,
                   const unsigned int /*version*/)
    {
      ar & make_nvp("axis",S.axis());
    }
  
    template <class Archive, int Dim, typename Scalar, int Options>
    void serialize(Archive & ar,
                   pinocchio::ConstraintTpl<Dim,Scalar,Options> & S,
                   const unsigned int /*version*/)
    {
      ar & make_nvp("matrix",S.matrix());
    }
  
    template <class Archive, class Constraint>
    void serialize(Archive & ar,
                   pinocchio::ScaledConstraint<Constraint> & S,
                   const unsigned int /*version*/)
    {
      ar & make_nvp("scaling",S.scaling());
      ar & make_nvp("constraint",S.constraint());
    }
  
    template <class Archive, typename Scalar, int Options>
    void serialize(Archive & /*ar*/,
                   pinocchio::ConstraintPlanarTpl<Scalar,Options> & /*S*/,
                   const unsigned int /*version*/)
    {}
  
    template <class Archive, typename Scalar, int Options>
    void serialize(Archive & ar,
                   pinocchio::ConstraintSphericalZYXTpl<Scalar,Options> & S,
                   const unsigned int /*version*/)
    {
      ar & make_nvp("angularSubspace",S.angularSubspace());
    }
    
  }
}

#endif // ifndef __pinocchio_serialization_joints_constraint_hpp__

