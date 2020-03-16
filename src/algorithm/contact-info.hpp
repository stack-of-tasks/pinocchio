//
// Copyright (c) 2019-2020 INRIA
//

#ifndef __pinocchio_algorithm_contact_info_hpp__
#define __pinocchio_algorithm_contact_info_hpp__

#include "pinocchio/multibody/fwd.hpp"
#include "pinocchio/spatial/se3.hpp"
#include "pinocchio/spatial/motion.hpp"
#include "pinocchio/spatial/force.hpp"

#include <limits>

namespace pinocchio
{
  /// \brief Type of contact
  enum ContactType
  {
    CONTACT_3D = 0,       /// \brief Point contact model
    CONTACT_6D,           /// \brief Frame contact model
    CONTACT_UNDEFINED     /// \brief The default contact is undefined
  };
  
  template<ContactType contact_type>
  struct contact_dim
  {
    enum { value = 0 };
  };
  
  template<>
  struct contact_dim<CONTACT_3D>
  {
    enum { value  = 3 };
  };
  
  template<>
  struct contact_dim<CONTACT_6D>
  {
    enum { value  = 6 };
  };

  template<typename Scalar, int Options> struct RigidContactModelTpl;
  template<typename Scalar, int Options> struct RigidContactDataTpl;
  
  ///
  /// \brief Contact model structure containg all the info describing the rigid contact model
  ///
  template<typename _Scalar, int _Options>
  struct RigidContactModelTpl
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    typedef _Scalar Scalar;
    enum { Options = _Options };
    
    typedef RigidContactModelTpl ContactModel;
    typedef RigidContactDataTpl<Scalar,Options> ContactData;
    
    typedef SE3Tpl<Scalar,Options> SE3;
    typedef MotionTpl<Scalar,Options> Motion;
    typedef ForceTpl<Scalar,Options> Force;
    typedef pinocchio::FrameIndex FrameIndex;
    
    /// \brief Type of the contact.
    ContactType type;
    
    /// \brief Index of the parent Frame in the model tree.
    FrameIndex frame_id;
    
    /// \brief Relative placement with respect to the parent frame.
    SE3 placement;
    
    /// \brief Reference frame where the constraint is expressed (WORLD, LOCAL_WORLD_ALIGNED or LOCAL)
    ReferenceFrame reference_frame;
    
    /// \brief Desired contact placement
    SE3 desired_contact_placement;
    
    /// \brief Desired contact spatial velocity
    Motion desired_contact_velocity;
    
    /// \brief Desired contact spatial acceleration
    Motion desired_contact_acceleration;
    
    /// \brief Default constructor.
    RigidContactModelTpl()
    : type(CONTACT_UNDEFINED)
    , frame_id(std::numeric_limits<FrameIndex>::max())
    , reference_frame(WORLD)
    , desired_contact_placement(SE3::Identity())
    , desired_contact_velocity(Motion::Zero())
    , desired_contact_acceleration(Motion::Zero())
    {}
    
    ///
    /// \brief Contructor with from a given type, parent and placement.
    ///
    /// \param[in] type Type of the contact.
    /// \param[in] frame_id Index of the parent Frame in the model tree.
    /// \param[in] placement Placement of the contact with respect to the parent Frame.
    /// \param[in] reference_frame Placement of the contact with respect to the parent Frame.
    ///
    template<typename S2, int O2>
    RigidContactModelTpl(const ContactType type,
                         const FrameIndex frame_id,
                         const SE3Tpl<S2,O2> & placement,
                         const ReferenceFrame & reference_frame = WORLD)
    : type(type)
    , frame_id(frame_id)
    , placement(placement)
    , reference_frame(reference_frame)
    , desired_contact_placement(SE3::Identity())
    , desired_contact_velocity(Motion::Zero())
    , desired_contact_acceleration(Motion::Zero())
    {}
    
    ///
    /// \brief Contructor with from a given type, parent and placement.
    ///
    /// \param[in] type Type of the contact.
    /// \param[in] frame_id Index of the parent Frame in the model tree.
    ///
    RigidContactModelTpl(const ContactType type,
                         const FrameIndex frame_id,
                         const ReferenceFrame & reference_frame = WORLD)
    : type(type)
    , frame_id(frame_id)
    , placement(SE3::Identity())
    , reference_frame(reference_frame)
    , desired_contact_placement(SE3::Identity())
    , desired_contact_velocity(Motion::Zero())
    , desired_contact_acceleration(Motion::Zero())
    {}
    
    ///
    /// \brief Comparison operator
    ///
    /// \param[in] other Other RigidContactModelTpl to compare with.
    ///
    /// \returns true if the two *this is equal to other (type, parent and placement attributs must be the same).
    ///
    template<int OtherOptions>
    bool operator==(const RigidContactModelTpl<Scalar,OtherOptions> & other) const
    {
      return
         type == other.type
      && frame_id == other.frame_id
      && placement == other.placement
      && reference_frame == other.reference_frame;
    }
    
    ///
    /// \brief Oposite of the comparison operator.
    ///
    /// \param[in] other Other RigidContactModelTpl to compare with.
    ///
    /// \returns false if the two *this is not equal to other (at least type, parent or placement attributs is different).
    ///
    template<int OtherOptions>
    bool operator!=(const RigidContactModelTpl<Scalar,OtherOptions> & other) const
    {
      return !(*this == other);
    }
    
    int size() const
    {
      switch(type)
      {
        case CONTACT_3D:
          return contact_dim<CONTACT_3D>::value;
        case CONTACT_6D:
          return contact_dim<CONTACT_6D>::value;
        default:
          return contact_dim<CONTACT_UNDEFINED>::value;
      }
      return -1;
    }
    
  };

  ///
  /// \brief Contact model structure containg all the info describing the rigid contact model
  ///
  template<typename _Scalar, int _Options>
  struct RigidContactDataTpl
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    typedef _Scalar Scalar;
    enum { Options = _Options };
    
    typedef RigidContactModelTpl<Scalar,Options> ContactModel;
    typedef RigidContactDataTpl ContactData;
    
    typedef SE3Tpl<Scalar,Options> SE3;
    typedef MotionTpl<Scalar,Options> Motion;
    typedef ForceTpl<Scalar,Options> Force;
    typedef Eigen::Matrix<Scalar,6,Eigen::Dynamic,Options> Matrix6x;
    
    RigidContactDataTpl(const ContactModel & /*contact_model*/)
    : contact_force(Force::Zero())
    , contact_placement(SE3::Identity())
    , contact_velocity(Motion::Zero())
    , contact_acceleration(Motion::Zero())
    , contact_acceleration_drift(Motion::Zero())
    , contact_acceleration_deviation(Motion::Zero())
    {}
    
    // data
    
    /// \brief Resulting contact forces
    Force contact_force;
    
    /// \brief Current contact placement with respect to the world frame
    SE3 contact_placement;
    
    /// \brief Contact placement with respect to the supporting joint frame
    SE3 joint_contact_placement;
    
    /// \brief Current contact spatial velocity
    Motion contact_velocity;
    
    /// \brief Current contact spatial acceleration
    Motion contact_acceleration;
    
    /// \brief Current contact drift acceleration (acceleration only due to the Coriolis and centrifugal effects).
    Motion contact_acceleration_drift;
    
    /// \brief Contact deviation from the reference acceleration (a.k.a the error)
    Motion contact_acceleration_deviation;
    
    bool operator==(const RigidContactDataTpl & other) const
    {
      return
         contact_force == other.contact_force
      && contact_placement == other.contact_placement
      && joint_contact_placement == other.joint_contact_placement
      && contact_velocity == other.contact_velocity
      && contact_acceleration == other.contact_acceleration
      && contact_acceleration_drift == other.contact_acceleration_drift
      && contact_acceleration_deviation == other.contact_acceleration_deviation
      ;
    }
    
    bool operator!=(const RigidContactDataTpl & other) const
    {
      return !(*this == other);
    }
  };
  
  typedef RigidContactModelTpl<double,0> RigidContactModel;
  typedef RigidContactDataTpl<double,0> RigidContactData;
}

#endif // ifndef __pinocchio_algorithm_contact_info_hpp__
