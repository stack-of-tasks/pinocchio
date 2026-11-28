//
// Copyright (c) 2025 INRIA
//

#pragma once

// IWYU pragma: private, include "pinocchio/serialization.hpp"

#ifdef PINOCCHIO_LSP
  #undef PINOCCHIO_LSP
  #include "pinocchio/serialization.hpp"
#endif // PINOCCHIO_LSP

namespace boost
{
  namespace serialization
  {

    template<typename Archive>
    void serialize(Archive &, ::pinocchio::BlankConstraintData &, const unsigned int)
    {
      // do nothing
    }

    template<typename Archive, typename Derived>
    void serialize(
      Archive & ar, ::pinocchio::ConstraintDataBase<Derived> & cdata, const unsigned int version)
    {
      PINOCCHIO_UNUSED_VARIABLE(ar);
      PINOCCHIO_UNUSED_VARIABLE(cdata);
      PINOCCHIO_UNUSED_VARIABLE(version);
    }

    namespace internal
    {
      template<typename Scalar, int Options>
      struct JointLimitConstraintDataAccessor
      : public ::pinocchio::JointLimitConstraintDataTpl<Scalar, Options>
      {
        typedef ::pinocchio::JointLimitConstraintDataTpl<Scalar, Options> Base;

        using Base::compact_tangent_map;
        using Base::constraint_residual_storage;
        using Base::rowise_tangent_map;
      };
    } // namespace internal

    template<typename Archive, typename Scalar, int Options>
    void serialize(
      Archive & ar,
      ::pinocchio::JointLimitConstraintDataTpl<Scalar, Options> & cdata,
      const unsigned int /*version*/)
    {
      typedef ::pinocchio::JointLimitConstraintDataTpl<Scalar, Options> Self;
      typedef typename Self::Base Base;
      ar & make_nvp("base", boost::serialization::base_object<Base>(cdata));

      ar & make_nvp("constraint_residual_storage", cdata.constraint_residual_storage);
      if (Archive::is_loading::value)
      {
        cdata.constraint_residual = cdata.constraint_residual_storage.map();
      }
      ar & make_nvp("compact_tangent_map", cdata.compact_tangent_map);
      ar & make_nvp("rowise_tangent_map", cdata.rowise_tangent_map);
    }

    template<typename Archive, typename Scalar, int Options>
    void serialize(
      Archive & ar,
      ::pinocchio::JointFrictionConstraintDataTpl<Scalar, Options> & cdata,
      const unsigned int /*version*/)
    {
      typedef ::pinocchio::JointFrictionConstraintDataTpl<Scalar, Options> Self;
      typedef typename Self::Base Base;
      ar & make_nvp("base", boost::serialization::base_object<Base>(cdata));
      ar & make_nvp("friction_impulse_lower_limit", cdata.friction_impulse_lower_limit);
      ar & make_nvp("friction_impulse_upper_limit", cdata.friction_impulse_upper_limit);
    }

    template<typename Archive, typename Derived>
    void serialize(
      Archive & ar,
      ::pinocchio::PointConstraintDataBase<Derived> & cdata,
      const unsigned int /*version*/)
    {
      typedef ::pinocchio::PointConstraintDataBase<Derived> Self;
      typedef typename Self::Base Base;
      ar & make_nvp("base", boost::serialization::base_object<Base>(cdata));

      ar & make_nvp("constraint_force", cdata.constraint_force);
      ar & make_nvp("oMc1", cdata.oMc1);
      ar & make_nvp("oMc2", cdata.oMc2);
      ar & make_nvp("c1Mc2", cdata.c1Mc2);
      ar & make_nvp("constraint_position_error", cdata.constraint_position_error);
      ar & make_nvp("constraint_velocity_error", cdata.constraint_velocity_error);
      ar & make_nvp("constraint_acceleration_error", cdata.constraint_acceleration_error);
      ar & make_nvp("constraint_acceleration_biais_term", cdata.constraint_acceleration_biais_term);
      ar & make_nvp("A1_world", cdata.A1_world);
      ar & make_nvp("A2_world", cdata.A2_world);
      ar & make_nvp("A_world", cdata.A_world);
      ar & make_nvp("A1_local", cdata.A1_local);
      ar & make_nvp("A2_local", cdata.A2_local);
      ar & make_nvp("A_local", cdata.A_local);
    }

    template<typename Archive, typename Scalar, int Options>
    void serialize(
      Archive & ar,
      ::pinocchio::PointAnchorConstraintDataTpl<Scalar, Options> & cdata,
      const unsigned int /*version*/)
    {
      typedef ::pinocchio::PointAnchorConstraintDataTpl<Scalar, Options> Self;
      typedef typename Self::Base Base;
      ar & make_nvp("base", boost::serialization::base_object<Base>(cdata));
    }

    template<typename Archive, typename Scalar, int Options>
    void serialize(
      Archive & ar,
      ::pinocchio::PointContactConstraintDataTpl<Scalar, Options> & cdata,
      const unsigned int /*version*/)
    {
      typedef ::pinocchio::PointContactConstraintDataTpl<Scalar, Options> Self;
      typedef typename Self::Base Base;
      ar & make_nvp("base", boost::serialization::base_object<Base>(cdata));
    }

    template<typename Archive, typename Scalar, int Options>
    void serialize(
      Archive & ar,
      ::pinocchio::ConstantLengthConstraintDataTpl<Scalar, Options> & cdata,
      const unsigned int /*version*/)
    {
      typedef ::pinocchio::ConstantLengthConstraintDataTpl<Scalar, Options> Self;
      typedef typename Self::Base Base;
      ar & make_nvp("base", boost::serialization::base_object<Base>(cdata));

      ar & make_nvp("constraint_force", cdata.constraint_force);
      ar & make_nvp("oMc1", cdata.oMc1);
      ar & make_nvp("oMc2", cdata.oMc2);
      ar & make_nvp("c1Mc2", cdata.c1Mc2);
      ar & make_nvp("relative_position", cdata.relative_position);
      ar & make_nvp("distance", cdata.distance);
      ar & make_nvp("direction", cdata.direction);
      ar & make_nvp("constraint_position_error", cdata.constraint_position_error);
      ar & make_nvp("constraint_velocity_error", cdata.constraint_velocity_error);
      ar & make_nvp("constraint_acceleration_error", cdata.constraint_acceleration_error);
      ar & make_nvp("constraint_acceleration_biais_term", cdata.constraint_acceleration_biais_term);
      ar & make_nvp("A1_world", cdata.A1_world);
      ar & make_nvp("A2_world", cdata.A2_world);
      ar & make_nvp("A_world", cdata.A_world);
      ar & make_nvp("A1_local", cdata.A1_local);
      ar & make_nvp("A2_local", cdata.A2_local);
      ar & make_nvp("A_local", cdata.A_local);
    }

    template<typename Archive, typename Derived>
    void serialize(
      Archive & ar,
      ::pinocchio::FrameConstraintDataBase<Derived> & cdata,
      const unsigned int /*version*/)
    {
      typedef ::pinocchio::FrameConstraintDataBase<Derived> Self;
      typedef typename Self::Base Base;
      ar & make_nvp("base", boost::serialization::base_object<Base>(cdata));

      ar & make_nvp("constraint_force", cdata.constraint_force);
      ar & make_nvp("oMc1", cdata.oMc1);
      ar & make_nvp("oMc2", cdata.oMc2);
      ar & make_nvp("c1Mc2", cdata.c1Mc2);
      ar & make_nvp("constraint_position_error", cdata.constraint_position_error);
      ar & make_nvp("constraint_velocity_error", cdata.constraint_velocity_error);
      ar & make_nvp("constraint_acceleration_error", cdata.constraint_acceleration_error);
      ar & make_nvp("constraint_acceleration_biais_term", cdata.constraint_acceleration_biais_term);
      ar & make_nvp("A1_world", cdata.A1_world);
      ar & make_nvp("A2_world", cdata.A2_world);
      ar & make_nvp("A_world", cdata.A_world);
      ar & make_nvp("A1_local", cdata.A1_local);
      ar & make_nvp("A2_local", cdata.A2_local);
      ar & make_nvp("A_local", cdata.A_local);
    }

    template<typename Archive, typename Scalar, int Options>
    void serialize(
      Archive & ar,
      ::pinocchio::FrameAnchorConstraintDataTpl<Scalar, Options> & cdata,
      const unsigned int /*version*/)
    {
      typedef ::pinocchio::FrameAnchorConstraintDataTpl<Scalar, Options> Self;
      typedef typename Self::Base Base;
      ar & make_nvp("base", boost::serialization::base_object<Base>(cdata));
    }

    template<
      typename Archive,
      typename Scalar,
      int Options,
      template<typename, int> class ConstraintCollectionTpl>
    void serialize(
      Archive & ar,
      pinocchio::ConstraintDataTpl<Scalar, Options, ConstraintCollectionTpl> & cdata,
      const unsigned int /*version*/)
    {
      typedef ::pinocchio::ConstraintDataTpl<Scalar, Options, ConstraintCollectionTpl> Self;
      typedef typename Self::Base Base;
      ar & make_nvp("base", boost::serialization::base_object<Base>(cdata));

      typedef typename ConstraintCollectionTpl<Scalar, Options>::ConstraintDataVariant
        ConstraintDataVariant;
      ar & make_nvp("base_variant", base_object<ConstraintDataVariant>(cdata));
    }

  } // namespace serialization
} // namespace boost
