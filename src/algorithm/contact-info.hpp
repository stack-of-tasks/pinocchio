//
// Copyright (c) 2019 INRIA
//

#ifndef __pinocchio_contact_info_hpp__
#define __pinocchio_contact_info_hpp__

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
    FrameIndex parent;
    
    /// \brief Relative placement with respect to the parent frame.
    SE3 placement;
    
    /// \brief Default constructor.
    ContactInfoTpl()
    : type(CONTACT_UNDEFINED)
    , parent(std::numeric_limits<FrameIndex>::max())
    {}
    
    ///
    /// \brief Contructor with from a given type, parent and placement.
    ///
    /// \param[in] type Type of the contact.
    /// \param[in] parent Index of the parent Frame in the model tree.
    /// \param[in] placement Placement of the contact with respect to the parent Frame".
    ///
    template<typename S2, int O2>
    ContactInfoTpl(const ContactType type,
                   const FrameIndex parent,
                   const SE3Tpl<S2,O2> & placement)
    : type(type)
    , parent(parent)
    , placement(placement)
    {}
    
    ///
    /// \brief Contructor with from a given type, parent and placement.
    ///
    /// \param[in] type Type of the contact.
    /// \param[in] parent Index of the parent Frame in the model tree.
    ///
    ContactInfoTpl(const ContactType type,
                   const FrameIndex parent)
    : type(type)
    , parent(parent)
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
      && parent == other.parent
      && placement == other.placement;
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
          return 3;
        case CONTACT_6D:
          return 6;
        default:
          return -1;
      }
      return -1;
    }
    
  };
  
  typedef ContactInfoTpl<double,0> ContactInfo;
}

#endif // ifndef __pinocchio_contact_info_hpp__
