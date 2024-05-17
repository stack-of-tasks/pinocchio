//
// Copyright (c) 2019-2020 INRIA
//

#ifndef __pinocchio_serialization_joints_motion_subspace_hpp__
#define __pinocchio_serialization_joints_motion_subspace_hpp__

#include "pinocchio/serialization/fwd.hpp"

#include <boost/serialization/split_free.hpp>
#include <boost/serialization/vector.hpp>

namespace boost
{
  namespace serialization
  {

    template<class Archive, typename Scalar, int Options, int axis>
    void serialize(
      Archive & /*ar*/,
      pinocchio::JointMotionSubspaceRevoluteTpl<Scalar, Options, axis> & /*S*/,
      const unsigned int /*version*/)
    {
    }

    template<class Archive, typename Scalar, int Options, int axis>
    void serialize(
      Archive & /*ar*/,
      pinocchio::JointMotionSubspacePrismaticTpl<Scalar, Options, axis> & /*S*/,
      const unsigned int /*version*/)
    {
    }

    template<class Archive, typename Scalar, int Options, int axis>
    void serialize(
      Archive & ar,
      pinocchio::JointMotionSubspaceHelicalTpl<Scalar, Options, axis> & S,
      const unsigned int /*version*/)
    {
      ar & make_nvp("h", S.h());
    }

    template<class Archive, typename Scalar, int Options>
    void serialize(
      Archive & /*ar*/,
      pinocchio::JointMotionSubspaceSphericalTpl<Scalar, Options> & /*S*/,
      const unsigned int /*version*/)
    {
    }

    template<class Archive, typename Scalar, int Options>
    void serialize(
      Archive & /*ar*/,
      pinocchio::JointMotionSubspaceTranslationTpl<Scalar, Options> & /*S*/,
      const unsigned int /*version*/)
    {
    }

    template<class Archive, typename Scalar, int Options>
    void serialize(
      Archive & /*ar*/,
      pinocchio::JointMotionSubspaceIdentityTpl<Scalar, Options> & /*S*/,
      const unsigned int /*version*/)
    {
    }

    template<class Archive, typename Scalar, int Options>
    void serialize(
      Archive & ar,
      pinocchio::JointMotionSubspaceRevoluteUnalignedTpl<Scalar, Options> & S,
      const unsigned int /*version*/)
    {
      ar & make_nvp("axis", S.axis());
    }

    template<class Archive, typename Scalar, int Options>
    void serialize(
      Archive & ar,
      pinocchio::JointMotionSubspacePrismaticUnalignedTpl<Scalar, Options> & S,
      const unsigned int /*version*/)
    {
      ar & make_nvp("axis", S.axis());
    }

    template<class Archive, typename Scalar, int Options>
    void serialize(
      Archive & ar,
      pinocchio::JointMotionSubspaceHelicalUnalignedTpl<Scalar, Options> & S,
      const unsigned int /*version*/)
    {
      ar & make_nvp("axis", S.axis());
      ar & make_nvp("h", S.h());
    }

    template<class Archive, typename Scalar, int Options>
    void serialize(
      Archive & ar,
      pinocchio::JointMotionSubspaceUniversalTpl<Scalar, Options> & S,
      const unsigned int /*version*/)
    {
      ar & make_nvp("angularSubspace", S.angularSubspace());
    }

    template<class Archive, int Dim, typename Scalar, int Options>
    void serialize(
      Archive & ar,
      pinocchio::JointMotionSubspaceTpl<Dim, Scalar, Options> & S,
      const unsigned int /*version*/)
    {
      ar & make_nvp("matrix", S.matrix());
    }

    template<class Archive, class Constraint>
    void serialize(
      Archive & ar,
      pinocchio::ScaledJointMotionSubspace<Constraint> & S,
      const unsigned int /*version*/)
    {
      ar & make_nvp("scaling", S.scaling());
      ar & make_nvp("constraint", S.constraint());
    }

    template<class Archive, typename Scalar, int Options>
    void serialize(
      Archive & /*ar*/,
      pinocchio::JointMotionSubspacePlanarTpl<Scalar, Options> & /*S*/,
      const unsigned int /*version*/)
    {
    }

    template<class Archive, typename Scalar, int Options>
    void serialize(
      Archive & ar,
      pinocchio::JointMotionSubspaceSphericalZYXTpl<Scalar, Options> & S,
      const unsigned int /*version*/)
    {
      ar & make_nvp("angularSubspace", S.angularSubspace());
    }

  } // namespace serialization
} // namespace boost

#endif // ifndef __pinocchio_serialization_joints_motion_subspace_hpp__
