//
// Copyright (c) 2025 INRIA

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
    void serialize(Archive &, ::pinocchio::BlankConstraintModel &, const unsigned int)
    {
      // do nothing
    }

    template<typename Archive, typename Scalar>
    void serialize(
      Archive & ar,
      ::pinocchio::BaumgarteCorrectorParametersTpl<Scalar> & baumgarte_parameters,
      const unsigned int /*version*/)
    {
      ar & make_nvp("Kp", baumgarte_parameters.Kp);
      ar & make_nvp("Kd", baumgarte_parameters.Kd);
    }

    namespace internal
    {
      template<typename Derived>
      struct ConstraintModelCommonParametersAccessor
      : public ::pinocchio::ConstraintModelCommonParameters<Derived>
      {
        typedef ::pinocchio::ConstraintModelCommonParameters<Derived> Base;
        using Base::m_baumgarte_parameters;
        using Base::m_compliance;
      };
    } // namespace internal

    template<typename Archive, typename Derived>
    void serialize(
      Archive & ar,
      ::pinocchio::ConstraintModelCommonParameters<Derived> & cmodel,
      const unsigned int version)
    {
      PINOCCHIO_UNUSED_VARIABLE(version);
      typedef internal::ConstraintModelCommonParametersAccessor<Derived> Accessor;
      auto & cmodel_ = reinterpret_cast<Accessor &>(cmodel);
      ar & make_nvp("m_compliance", cmodel_.m_compliance);
      ar & make_nvp("m_baumgarte_parameters", cmodel_.m_baumgarte_parameters);
    }

    // ---------------
    // Bases
    // ---------------

    template<typename Archive, typename Derived>
    void serialize(
      Archive & ar, ::pinocchio::ConstraintModelBase<Derived> & cmodel, const unsigned int version)
    {
      PINOCCHIO_UNUSED_VARIABLE(version);
      ar & make_nvp("name", cmodel.name);
    }

    template<typename Archive, typename Derived>
    void serialize(
      Archive & ar,
      ::pinocchio::KinematicsConstraintModelBase<Derived> & cmodel,
      const unsigned int /*version*/)
    {
      typedef ::pinocchio::KinematicsConstraintModelBase<Derived> Self;
      typedef typename Self::Base Base;
      ar & make_nvp("base", boost::serialization::base_object<Base>(cmodel));
    }

    template<typename Archive, typename Derived>
    void serialize(
      Archive & ar,
      ::pinocchio::JointWiseConstraintModelBase<Derived> & cmodel,
      const unsigned int /*version*/)
    {
      typedef ::pinocchio::JointWiseConstraintModelBase<Derived> Self;
      typedef typename Self::Base Base;
      ar & make_nvp("base", boost::serialization::base_object<Base>(cmodel));
    }

    template<typename Archive, typename Derived>
    void serialize(
      Archive & ar,
      ::pinocchio::BinaryKinematicsConstraintModelBase<Derived> & cmodel,
      const unsigned int /*version*/)
    {
      typedef ::pinocchio::BinaryKinematicsConstraintModelBase<Derived> Self;
      typedef typename Self::Base KinematicsBase;
      ar & make_nvp("base", boost::serialization::base_object<KinematicsBase>(cmodel));
      typedef typename Self::BaseCommonParameters BaseCommonParameters;
      ar & make_nvp(
        "base_common_parameters", boost::serialization::base_object<BaseCommonParameters>(cmodel));

      ar & make_nvp("joint1_id", cmodel.joint1_id);
      ar & make_nvp("joint2_id", cmodel.joint2_id);
      ar & make_nvp("joint1_placement", cmodel.joint1_placement);
      ar & make_nvp("joint2_placement", cmodel.joint2_placement);
      ar & make_nvp("desired_constraint_offset", cmodel.desired_constraint_offset);
      ar & make_nvp("desired_constraint_velocity", cmodel.desired_constraint_velocity);
      ar & make_nvp("desired_constraint_acceleration", cmodel.desired_constraint_acceleration);
      ar & make_nvp("nv", cmodel.nv);
      ar & make_nvp("depth_joint1", cmodel.depth_joint1);
      ar & make_nvp("depth_joint2", cmodel.depth_joint2);
    }

    template<typename Archive, typename Derived>
    void serialize(
      Archive & ar,
      ::pinocchio::PointConstraintModelBase<Derived> & cmodel,
      const unsigned int /*version*/)
    {
      typedef ::pinocchio::PointConstraintModelBase<Derived> Self;
      typedef typename Self::Base Base;
      ar & make_nvp("base", boost::serialization::base_object<Base>(cmodel));
    }

    template<typename Archive, typename Scalar, int Options>
    void serialize(
      Archive & ar,
      ::pinocchio::FrameAnchorConstraintModelTpl<Scalar, Options> & cmodel,
      const unsigned int /*version*/)
    {
      typedef ::pinocchio::FrameAnchorConstraintModelTpl<Scalar, Options> Self;
      typedef typename Self::Base Base;
      ar & make_nvp("base", boost::serialization::base_object<Base>(cmodel));
    }

    // ---------------
    // Constraints
    // ---------------

    namespace internal
    {
      template<typename Scalar, int Options>
      struct JointLimitConstraintModelAccessor
      : public ::pinocchio::JointLimitConstraintModelTpl<Scalar, Options>
      {
        typedef ::pinocchio::JointLimitConstraintModelTpl<Scalar, Options> Base;

        using Base::m_activable_idx_in_selected;
        using Base::m_activable_idx_qs;
        using Base::m_activable_idx_qs_reduce;
        using Base::m_activable_position_limit;
        using Base::m_activable_position_margin;
        using Base::m_cursel_active_idx_in_activable;
        using Base::m_cursel_active_idx_in_selected;
        using Base::m_cursel_active_idx_qs;
        using Base::m_cursel_active_idx_qs_reduce;
        using Base::m_cursel_lower_active_residual_size;
        using Base::m_lower_activable_residual_size;
        using Base::m_max_of_nvs;
        using Base::m_nq_reduce;
        using Base::m_selected_joint_idx_vs;
        using Base::m_selected_joint_nqs;
        using Base::m_selected_joint_nvs;
        using Base::m_selected_joints;
      };
    } // namespace internal

    template<typename Archive, typename Scalar, int Options>
    void serialize(
      Archive & ar,
      ::pinocchio::JointLimitConstraintModelTpl<Scalar, Options> & cmodel,
      const unsigned int /*version*/)
    {
      typedef ::pinocchio::JointLimitConstraintModelTpl<Scalar, Options> Self;
      typedef typename Self::Base Base;
      ar & make_nvp("base", boost::serialization::base_object<Base>(cmodel));
      typedef typename Self::BaseCommonParameters BaseCommonParameters;
      ar & make_nvp(
        "base_common_parameters", boost::serialization::base_object<BaseCommonParameters>(cmodel));

      typedef internal::JointLimitConstraintModelAccessor<Scalar, Options> Accessor;
      auto & cmodel_ = reinterpret_cast<Accessor &>(cmodel);
      ar & make_nvp("m_selected_joints", cmodel_.m_selected_joints);
      ar & make_nvp("m_selected_joint_nqs", cmodel_.m_selected_joint_nqs);
      ar & make_nvp("m_selected_joint_nvs", cmodel_.m_selected_joint_nvs);
      ar & make_nvp("m_selected_joint_idx_vs", cmodel_.m_selected_joint_idx_vs);
      ar & make_nvp("m_nq_reduce", cmodel_.m_nq_reduce);
      ar & make_nvp("m_max_of_nvs", cmodel_.m_max_of_nvs);
      ar & make_nvp("m_lower_activable_residual_size", cmodel_.m_lower_activable_residual_size);
      ar & make_nvp("m_activable_idx_in_selected", cmodel_.m_activable_idx_in_selected);
      ar & make_nvp("m_activable_idx_qs", cmodel_.m_activable_idx_qs);
      ar & make_nvp("m_activable_idx_qs_reduce", cmodel_.m_activable_idx_qs_reduce);
      ar & make_nvp("m_activable_position_limit", cmodel_.m_activable_position_limit);
      ar & make_nvp("m_activable_position_margin", cmodel_.m_activable_position_margin);
      ar & make_nvp("m_cursel_active_idx_in_activable", cmodel_.m_cursel_active_idx_in_activable);
      ar & make_nvp(
        "m_cursel_lower_active_residual_size", cmodel_.m_cursel_lower_active_residual_size);
      ar & make_nvp("m_cursel_active_idx_in_selected", cmodel_.m_cursel_active_idx_in_selected);
      ar & make_nvp("m_cursel_active_idx_qs", cmodel_.m_cursel_active_idx_qs);
      ar & make_nvp("m_cursel_active_idx_qs_reduce", cmodel_.m_cursel_active_idx_qs_reduce);
    }

    namespace internal
    {
      template<typename Scalar, int Options>
      struct JointFrictionConstraintModelAccessor
      : public ::pinocchio::JointFrictionConstraintModelTpl<Scalar, Options>
      {
        typedef ::pinocchio::JointFrictionConstraintModelTpl<Scalar, Options> Base;
        using Base::m_active_dofs;
        using Base::m_active_joint_ids;
        using Base::m_active_joints;
        using Base::m_dt;
        using Base::m_friction_lower_limit;
        using Base::m_friction_upper_limit;
      };
    } // namespace internal

    template<typename Archive, typename Scalar, int Options>
    void serialize(
      Archive & ar,
      ::pinocchio::JointFrictionConstraintModelTpl<Scalar, Options> & cmodel,
      const unsigned int /*version*/)
    {
      typedef ::pinocchio::JointFrictionConstraintModelTpl<Scalar, Options> Self;
      typedef typename Self::Base Base;
      ar & make_nvp("base", boost::serialization::base_object<Base>(cmodel));
      typedef typename Self::BaseCommonParameters BaseCommonParameters;
      ar & make_nvp(
        "base_common_parameters", boost::serialization::base_object<BaseCommonParameters>(cmodel));

      typedef internal::JointFrictionConstraintModelAccessor<Scalar, Options> Accessor;
      auto & cmodel_ = reinterpret_cast<Accessor &>(cmodel);
      ar & make_nvp("m_active_joints", cmodel_.m_active_joints);
      ar & make_nvp("m_active_dofs", cmodel_.m_active_dofs);
      ar & make_nvp("m_active_joint_ids", cmodel_.m_active_joint_ids);
      ar & make_nvp("friction_lower_limit", cmodel_.m_friction_lower_limit);
      ar & make_nvp("friction_upper_limit", cmodel_.m_friction_upper_limit);
      ar & make_nvp("dt", cmodel_.m_dt);
    }

    template<typename Archive, typename Scalar, int Options>
    void serialize(
      Archive & ar,
      ::pinocchio::PointAnchorConstraintModelTpl<Scalar, Options> & cmodel,
      const unsigned int /*version*/)
    {
      typedef ::pinocchio::PointAnchorConstraintModelTpl<Scalar, Options> Self;
      typedef typename Self::Base Base;
      ar & make_nvp("base", boost::serialization::base_object<Base>(cmodel));
    }

    namespace internal
    {
      template<typename Scalar, int Options>
      struct PointContactConstraintModelAccessor
      : public ::pinocchio::PointContactConstraintModelTpl<Scalar, Options>
      {
        typedef ::pinocchio::PointContactConstraintModelTpl<Scalar, Options> Base;
        using Base::m_friction;
      };
    } // namespace internal

    template<typename Archive, typename Scalar, int Options>
    void serialize(
      Archive & ar,
      ::pinocchio::PointContactConstraintModelTpl<Scalar, Options> & cmodel,
      const unsigned int /*version*/)
    {
      typedef ::pinocchio::PointContactConstraintModelTpl<Scalar, Options> Self;
      typedef typename Self::Base Base;
      ar & make_nvp("base", boost::serialization::base_object<Base>(cmodel));
      typedef internal::PointContactConstraintModelAccessor<Scalar, Options> Accessor;
      auto & cmodel_ = reinterpret_cast<Accessor &>(cmodel);
      ar & make_nvp("m_friction", cmodel_.m_friction);
    }

    namespace internal
    {
      template<typename Scalar, int Options>
      struct ConstantLengthConstraintModelAccessor
      : public ::pinocchio::ConstantLengthConstraintModelTpl<Scalar, Options>
      {
        typedef ::pinocchio::ConstantLengthConstraintModelTpl<Scalar, Options> Base;
        using Base::m_length;
      };
    } // namespace internal

    template<typename Archive, typename Scalar, int Options>
    void serialize(
      Archive & ar,
      ::pinocchio::ConstantLengthConstraintModelTpl<Scalar, Options> & cmodel,
      const unsigned int /*version*/)
    {
      typedef ::pinocchio::ConstantLengthConstraintModelTpl<Scalar, Options> Self;
      typedef typename Self::Base Base;
      ar & make_nvp("base", boost::serialization::base_object<Base>(cmodel));
      typedef internal::ConstantLengthConstraintModelAccessor<Scalar, Options> Accessor;
      auto & cmodel_ = reinterpret_cast<Accessor &>(cmodel);
      ar & make_nvp("m_length", cmodel_.m_length);
    }

    template<typename Archive, typename Derived>
    void serialize(
      Archive & ar,
      ::pinocchio::FrameConstraintModelBase<Derived> & cmodel,
      const unsigned int /*version*/)
    {
      typedef ::pinocchio::FrameConstraintModelBase<Derived> Self;
      typedef typename Self::Base Base;
      ar & make_nvp("base", boost::serialization::base_object<Base>(cmodel));
    }

    // ---------------
    // Generic
    // ---------------

    template<
      typename Archive,
      typename Scalar,
      int Options,
      template<typename, int> class ConstraintCollectionTpl>
    void serialize(
      Archive & ar,
      pinocchio::ConstraintModelTpl<Scalar, Options, ConstraintCollectionTpl> & cmodel,
      const unsigned int /*version*/)
    {
      typedef ::pinocchio::ConstraintModelTpl<Scalar, Options, ConstraintCollectionTpl> Self;
      typedef typename Self::Base Base;
      ar & make_nvp("base", boost::serialization::base_object<Base>(cmodel));

      typedef typename ConstraintCollectionTpl<Scalar, Options>::ConstraintModelVariant
        ConstraintModelVariant;
      ar & make_nvp(
        "base_variant", boost::serialization::base_object<ConstraintModelVariant>(cmodel));
    }

  } // namespace serialization
} // namespace boost
