//
// Copyright (c) 2019-2025 INRIA CNRS
//

#pragma once

// IWYU pragma: private, include "pinocchio/constraints.hpp"

#ifdef PINOCCHIO_LSP
  #undef PINOCCHIO_LSP
  #include "pinocchio/constraints.hpp"
#endif // PINOCCHIO_LSP

namespace pinocchio
{
  // --------------------------------------------------------------
  // Declaration
  // --------------------------------------------------------------
  template<typename Derived>
  struct PointConstraintDataBase;

  // --------------------------------------------------------------
  // Helpers
  // --------------------------------------------------------------
  template<typename Derived>
  using enable_if_point_data_t =
    std::enable_if_t<std::is_base_of_v<PointConstraintDataBase<Derived>, Derived>>;

  ///
  /// \brief Data structure associated with PointConstraint
  ///
  template<typename Derived>
  struct PointConstraintDataBase : ConstraintDataBase<Derived>
  {
    // --------------------------------------------------------------
    // Type defs
    // --------------------------------------------------------------
    // CRTP related types -------------------------------------------
    typedef PointConstraintDataBase Self;
    typedef ConstraintDataBase<Derived> Base;

    // Retrieving traits --------------------------------------------
    typedef typename traits<Derived>::ConstraintModel ConstraintModel;
    typedef typename traits<Derived>::ConstraintData ConstraintData;

    typedef typename traits<Derived>::Scalar Scalar;
    static constexpr int Options = traits<Derived>::Options;

    // Useful types ------------------------------------------------
    typedef SE3Tpl<Scalar, Options> SE3;
    typedef MotionTpl<Scalar, Options> Motion;
    typedef ForceTpl<Scalar, Options> Force;
    typedef Eigen::Matrix<Scalar, 3, 1, Options> Vector3;
    typedef Eigen::Matrix<Scalar, 6, 1, Options> Vector6;
    typedef Eigen::Matrix<Scalar, 6, 6, Options> Matrix6;
    typedef Eigen::Matrix<Scalar, 3, 6, Options> Matrix36;
    typedef Matrix36 MatrixSize6;
    typedef Eigen::Matrix<Scalar, 3, 6, Eigen::RowMajor> RowMatrix36;
    typedef Eigen::Matrix<Scalar, 6, Eigen::Dynamic, Options> Matrix6x;
    typedef Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, Options> MatrixX;

    // -------------------------------
    // METHODS SPECIFIC TO CLASS
    // -------------------------------

    // CRTP related ------------------

    /// \brief Cast to base
    Base & base()
    {
      return static_cast<Base &>(*this);
    }

    /// \brief Const cast to base
    const Base & base() const
    {
      return static_cast<const Base &>(*this);
    }

    // Constructors ------------------

    /// \brief Default constructor
    PointConstraintDataBase()
    : constraint_force(Vector3::Zero())
    , oMc1(SE3::Identity())
    , oMc2(SE3::Identity())
    , c1Mc2(SE3::Identity())
    , constraint_position_error(Vector3::Zero())
    , constraint_velocity_error(Vector3::Zero())
    , constraint_acceleration_error(Vector3::Zero())
    , constraint_acceleration_biais_term(Vector3::Zero())
    , A1_world(Matrix36::Zero())
    , A2_world(Matrix36::Zero())
    , A_world(Matrix36::Zero())
    , A1_local(Matrix36::Zero())
    , A2_local(Matrix36::Zero())
    , A_local(Matrix36::Zero())
    {
    }

    /// \brief Constructor from a given ConstraintModel
    ///
    /// \param[in] cmodel input constraint model
    ///
    explicit PointConstraintDataBase(const ConstraintModel & cmodel)
    : constraint_force(Vector3::Zero())
    , oMc1(SE3::Identity())
    , oMc2(SE3::Identity())
    , c1Mc2(SE3::Identity())
    , constraint_position_error(Vector3::Zero())
    , constraint_velocity_error(Vector3::Zero())
    , constraint_acceleration_error(Vector3::Zero())
    , constraint_acceleration_biais_term(Vector3::Zero())
    , A1_world(Matrix36::Zero())
    , A2_world(Matrix36::Zero())
    , A_world(Matrix36::Zero())
    , A1_local(Matrix36::Zero())
    , A2_local(Matrix36::Zero())
    , A_local(Matrix36::Zero())
    {
      PINOCCHIO_UNUSED_VARIABLE(cmodel);
    }

    // Operators ---------------------

    /// \brief Comparison operator
    bool operator==(const PointConstraintDataBase & other) const
    {
      return constraint_force == other.constraint_force && oMc1 == other.oMc1 && oMc2 == other.oMc2
             && c1Mc2 == other.c1Mc2 && constraint_position_error == other.constraint_position_error
             && constraint_velocity_error == other.constraint_velocity_error
             && constraint_acceleration_error == other.constraint_acceleration_error
             && constraint_acceleration_biais_term == other.constraint_acceleration_biais_term
             && A1_world == other.A1_world && A2_world == other.A2_world && A_world == other.A_world
             && A1_local == other.A1_local && A2_local == other.A2_local
             && A_local == other.A_local;
    }

    /// \brief Comparison operator
    bool operator!=(const PointConstraintDataBase & other) const
    {
      return !(*this == other);
    }

    // ------------------------------
    // MEMBERS
    // ------------------------------
    // note: data is always public - use at your own risk

    /// \brief Resulting contact forces
    Vector3 constraint_force;

    /// \brief Placement of the constraint frame 1 with respect to the WORLD frame
    SE3 oMc1;

    /// \brief Placement of the constraint frame 2 with respect to the WORLD frame
    SE3 oMc2;

    /// \brief Relative displacement between the two frames
    SE3 c1Mc2;

    /// \brief Constraint position error
    Vector3 constraint_position_error;

    /// \brief Constraint velocity error
    Vector3 constraint_velocity_error;

    /// \brief Constraint acceleration error
    Vector3 constraint_acceleration_error;

    /// \brief Constraint acceleration bias
    Vector3 constraint_acceleration_biais_term;

    Matrix36 A1_world;
    Matrix36 A2_world;
    Matrix36 A_world; // A1 + A2

    Matrix36 A1_local;
    Matrix36 A2_local;
    Matrix36 A_local; // A1 + A2
  }; // struct PointConstraintDataBase

} // namespace pinocchio
