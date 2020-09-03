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
    typedef pinocchio::JointIndex JointIndex;
    
    /// \brief Type of the contact.
    ContactType type;
    
    /// \brief Index of the first joint in the model tree
    JointIndex joint1_id;
    
    /// \brief Index of the second joint in the model tree
    JointIndex joint2_id;
    
    /// \brief Relative placement with respect to the frame of joint1.
    SE3 joint1_placement;
    
    /// \brief Relative placement with respect to the frame of joint2.
    SE3 joint2_placement;
    
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
    , joint1_id(std::numeric_limits<FrameIndex>::max())
    , joint2_id(std::numeric_limits<FrameIndex>::max())
    , joint1_placement(SE3::Identity())
    , joint2_placement(SE3::Identity())
    , reference_frame(WORLD)
    , desired_contact_placement(SE3::Identity())
    , desired_contact_velocity(Motion::Zero())
    , desired_contact_acceleration(Motion::Zero())
    {}
    
    ///
    /// \brief Contructor with from a given type, joint indexes and placements.
    ///
    /// \param[in] type Type of the contact.
    /// \param[in] joint1_id Index of the joint 1 in the model tree.
    /// \param[in] joint2_id Index of the joint 2 in the model tree.
    /// \param[in] joint1_placement Placement of the constraint w.r.t the frame of joint1.
    /// \param[in] joint2_placement Placement of the constraint w.r.t the frame of joint2.
    /// \param[in] reference_frame Reference frame in which the constraints quantities are expressed.
    ///
    template<typename S2, int O2>
    RigidContactModelTpl(const ContactType type,
                         const JointIndex joint1_id,
                         const SE3Tpl<S2,O2> & joint1_placement,
                         const JointIndex joint2_id,
                         const SE3Tpl<S2,O2> & joint2_placement,
                         const ReferenceFrame & reference_frame = WORLD)
    : type(type)
    , joint1_id(joint1_id)
    , joint2_id(joint2_id)
    , joint1_placement(joint1_placement)
    , joint2_placement(joint2_placement)
    , reference_frame(reference_frame)
    , desired_contact_placement(SE3::Identity())
    , desired_contact_velocity(Motion::Zero())
    , desired_contact_acceleration(Motion::Zero())
    {}
    
    ///
    /// \brief Contructor with from a given type, joint1_id and placement.
    ///
    /// \param[in] type Type of the contact.
    /// \param[in] joint1_id Index of the joint 1 in the model tree.
    /// \param[in] joint1_placement Placement of the constraint w.r.t the frame of joint1.
    /// \param[in] reference_frame Reference frame in which the constraints quantities are expressed.
    ///
    template<typename S2, int O2>
    RigidContactModelTpl(const ContactType type,
                         const JointIndex joint1_id,
                         const SE3Tpl<S2,O2> & joint1_placement,
                         const ReferenceFrame & reference_frame = WORLD)
    : type(type)
    , joint1_id(joint1_id)
    , joint2_id(0)
    , joint1_placement(joint1_placement)
    , joint2_placement(SE3::Identity())
    , reference_frame(reference_frame)
    , desired_contact_placement(SE3::Identity())
    , desired_contact_velocity(Motion::Zero())
    , desired_contact_acceleration(Motion::Zero())
    {}
    
    ///
    /// \brief Contructor with from a given type and the joint ids.
    ///
    /// \param[in] type Type of the contact.
    /// \param[in] joint1_id Index of the joint 1 in the model tree.
    /// \param[in] joint2_id Index of the joint 2 in the model tree.
    ///
    RigidContactModelTpl(const ContactType type,
                         const JointIndex joint1_id,
                         const JointIndex joint2_id,
                         const ReferenceFrame & reference_frame = WORLD)
    : type(type)
    , joint1_id(joint1_id)
    , joint2_id(joint2_id)
    , joint1_placement(SE3::Identity())
    , joint2_placement(SE3::Identity())
    , reference_frame(reference_frame)
    , desired_contact_placement(SE3::Identity())
    , desired_contact_velocity(Motion::Zero())
    , desired_contact_acceleration(Motion::Zero())
    {}
    
    ///
    /// \brief Contructor with from a given type and .
    ///
    /// \param[in] type Type of the contact.
    /// \param[in] joint1_id Index of the joint 1 in the model tree.
    ///
    /// \remarks The second joint id (joint2_id) is set to be 0 (corresponding to the index of the universe).
    ///
    RigidContactModelTpl(const ContactType type,
                         const JointIndex joint1_id,
                         const ReferenceFrame & reference_frame = WORLD)
    : type(type)
    , joint1_id(joint1_id)
    , joint2_id(0) // set to be the Universe
    , joint1_placement(SE3::Identity())
    , joint2_placement(SE3::Identity())
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
    /// \returns true if the two *this is equal to other (type, joint1_id and placement attributs must be the same).
    ///
    template<int OtherOptions>
    bool operator==(const RigidContactModelTpl<Scalar,OtherOptions> & other) const
    {
      return
         type == other.type
      && joint1_id == other.joint1_id
      && joint2_id == other.joint2_id
      && joint1_placement == other.joint1_placement
      && joint2_placement == other.joint2_placement
      && reference_frame == other.reference_frame;
    }
    
    ///
    /// \brief Oposite of the comparison operator.
    ///
    /// \param[in] other Other RigidContactModelTpl to compare with.
    ///
    /// \returns false if the two *this is not equal to other (at least type, joint1_id or placement attributs is different).
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
