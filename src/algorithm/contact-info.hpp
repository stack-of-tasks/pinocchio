//
// Copyright (c) 2019 INRIA
//

#ifndef __pinocchio_algorithm_contact_info_hpp__
#define __pinocchio_algorithm_contact_info_hpp__

#include "pinocchio/multibody/fwd.hpp"
#include "pinocchio/spatial/se3.hpp"

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
    enum { value = -1 };
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
  
  /// \brief Contact info data structure containg all the required info to
  template<typename _Scalar, int _Options>
  struct ContactInfoTpl
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    typedef _Scalar Scalar;
    enum { Options = _Options };
    
    typedef SE3Tpl<Scalar,Options> SE3;
    
    /// \brief Type of the contact.
    ContactType type;
    
    /// \brief Index of the parent Frame in the model tree.
    FrameIndex frame_id;
    
    /// \brief Relative placement with respect to the parent frame.
    SE3 placement;
    
    /// \brief Default constructor.
    ContactInfoTpl()
    : type(CONTACT_UNDEFINED)
    , frame_id(std::numeric_limits<FrameIndex>::max())
    , reference_frame(WORLD)
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
    ContactInfoTpl(const ContactType type,
                   const FrameIndex frame_id,
                   const SE3Tpl<S2,O2> & placement,
                   const ReferenceFrame & reference_frame = WORLD)
    : type(type)
    , frame_id(frame_id)
    , placement(placement)
    {}
    
    ///
    /// \brief Contructor with from a given type, parent and placement.
    ///
    /// \param[in] type Type of the contact.
    /// \param[in] frame_id Index of the parent Frame in the model tree.
    ///
    ContactInfoTpl(const ContactType type,
                   const FrameIndex frame_id,
                   const ReferenceFrame & reference_frame = WORLD)
    : type(type)
    , frame_id(frame_id)
    , placement(SE3::Identity())
    {}
    
    ///
    /// \brief Comparison operator
    ///
    /// \param[in] other Other ContactInfoTpl to compare with.
    ///
    /// \returns true if the two *this is equal to other (type, parent and placement attributs must be the same).
    ///
    template<int OtherOptions>
    bool operator==(const ContactInfoTpl<Scalar,OtherOptions> & other) const
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
    /// \param[in] other Other ContactInfoTpl to compare with.
    ///
    /// \returns false if the two *this is not equal to other (at least type, parent or placement attributs is different).
    ///
    template<int OtherOptions>
    bool operator!=(const ContactInfoTpl<Scalar,OtherOptions> & other) const
    {
      return !(*this == other);
    }
    
    int dim() const
    {
      switch(type)
      {
        case CONTACT_3D:
          return contact_dim<CONTACT_3D>::value;
        case CONTACT_6D:
          return contact_dim<CONTACT_6D>::value;
        default:
          return -1;
      }
      return -1;
    }
    
  };
  
  typedef ContactInfoTpl<double,0> ContactInfo;
}

#endif // ifndef __pinocchio_algorithm_contact_info_hpp__
