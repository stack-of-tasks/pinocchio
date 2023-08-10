//
// Copyright (c) 2019-2022 INRIA CNRS
//

#ifndef __pinocchio_algorithm_contact_info_hpp__
#define __pinocchio_algorithm_contact_info_hpp__

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/algorithm/fwd.hpp"

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

  template<typename _Scalar> struct BaumgarteCorrectorParametersTpl;

  template<typename _Scalar>
  struct traits< BaumgarteCorrectorParametersTpl<_Scalar> >
  {
    typedef _Scalar Scalar;
  };

  template<typename _Scalar>
  struct BaumgarteCorrectorParametersTpl
  : NumericalBase< BaumgarteCorrectorParametersTpl<_Scalar> >
  {
    typedef _Scalar Scalar;
    typedef Eigen::Matrix<Scalar, -1, 1, Eigen::ColMajor, 6> Vector6Max;
    
    BaumgarteCorrectorParametersTpl(int size = 6)
      : Kp(size), Kd(size)
    {
      Kp.setZero();
      Kd.setZero();
    }
    
    bool operator ==(const BaumgarteCorrectorParametersTpl & other) const
    { return Kp == other.Kp && Kd == other.Kd; }
    
    bool operator !=(const BaumgarteCorrectorParametersTpl & other) const
    { return !(*this == other); }
    
    // parameters
    /// \brief Proportional corrector value.
    Vector6Max Kp;
    
    /// \brief Damping corrector value.
    Vector6Max Kd;

    template<typename NewScalar>
    BaumgarteCorrectorParametersTpl<NewScalar> cast() const {
      typedef BaumgarteCorrectorParametersTpl<NewScalar> ReturnType;
      ReturnType res;
      res.Kp = Kp.template cast<NewScalar>();
      res.Kd = Kd.template cast<NewScalar>();
      return res;
    }  
  };

  template<typename Scalar, int Options> struct RigidConstraintModelTpl;
  template<typename Scalar, int Options> struct RigidConstraintDataTpl;

  template<typename _Scalar, int _Options>
  struct traits< RigidConstraintModelTpl<_Scalar,_Options> >
  {
    typedef _Scalar Scalar;
  };

  template<typename _Scalar, int _Options>
  struct traits< RigidConstraintDataTpl<_Scalar,_Options> >
  {
    typedef _Scalar Scalar;
  };

  ///
  /// \brief Contact model structure containg all the info describing the rigid contact model
  ///
  template<typename _Scalar, int _Options>
  struct RigidConstraintModelTpl
  : NumericalBase< RigidConstraintModelTpl<_Scalar,_Options> >
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    typedef _Scalar Scalar;
    enum { Options = _Options };
    
    typedef RigidConstraintModelTpl ContactModel;
    typedef RigidConstraintDataTpl<Scalar,Options> ContactData;
    
    typedef SE3Tpl<Scalar,Options> SE3;
    typedef MotionTpl<Scalar,Options> Motion;
    typedef ForceTpl<Scalar,Options> Force;
    typedef BaumgarteCorrectorParametersTpl<Scalar> BaumgarteCorrectorParameters;
    typedef Eigen::Matrix<bool,Eigen::Dynamic,1,Options> BooleanVector;
    typedef Eigen::Matrix<Eigen::DenseIndex,Eigen::Dynamic,1,Options> IndexVector;
    
    /// \brief Name of the contact
    std::string name;
    
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
    
    /// \brief Reference frame where the constraint is expressed (LOCAL_WORLD_ALIGNED or LOCAL)
    ReferenceFrame reference_frame;
    
    /// \brief Desired contact placement
    SE3 desired_contact_placement;
    
    /// \brief Desired contact spatial velocity
    Motion desired_contact_velocity;
    
    /// \brief Desired contact spatial acceleration
    Motion desired_contact_acceleration;
    
    /// \brief Corrector parameters
    BaumgarteCorrectorParameters corrector;
    
    /// \brief Sparsity pattern associated to joint 1.
    BooleanVector colwise_joint1_sparsity;
    
    /// \brief Sparsity pattern associated to joint 2.
    BooleanVector colwise_joint2_sparsity;
    
    /// \brief Indexes of the columns spanned by the constraints.
    IndexVector colwise_span_indexes;
    
    /// \brief Dimensions of the models
    int nv;
    
    /// \brief Depth of the kinematic tree for joint1 and joint2
    size_t depth_joint1, depth_joint2;
    
    ///
    /// \brief Default constructor
    ///
    RigidConstraintModelTpl()
    : nv(-1)
    , depth_joint1(0)
    , depth_joint2(0)
    {}
        
    ///
    /// \brief Contructor with from a given type, joint indexes and placements.
    ///
    /// \param[in] type Type of the contact.
    /// \param[in] model Model associated to the constraint.
    /// \param[in] joint1_id Index of the joint 1 in the model tree.
    /// \param[in] joint2_id Index of the joint 2 in the model tree.
    /// \param[in] joint1_placement Placement of the constraint w.r.t the frame of joint1.
    /// \param[in] joint2_placement Placement of the constraint w.r.t the frame of joint2.
    /// \param[in] reference_frame Reference frame in which the constraints quantities are expressed.
    ///
    template<int OtherOptions, template<typename,int> class JointCollectionTpl>
    RigidConstraintModelTpl(const ContactType type,
                            const ModelTpl<Scalar,OtherOptions,JointCollectionTpl> & model,
                            const JointIndex joint1_id,
                            const SE3 & joint1_placement,
                            const JointIndex joint2_id,
                            const SE3 & joint2_placement,
                            const ReferenceFrame & reference_frame = LOCAL)
    : type(type)
    , joint1_id(joint1_id)
    , joint2_id(joint2_id)
    , joint1_placement(joint1_placement)
    , joint2_placement(joint2_placement)
    , reference_frame(reference_frame)
    , desired_contact_placement(SE3::Identity())
    , desired_contact_velocity(Motion::Zero())
    , desired_contact_acceleration(Motion::Zero())
    , corrector(size())
    , colwise_joint1_sparsity(model.nv)
    , colwise_joint2_sparsity(model.nv)
    {
      init(model);
    }
    
    ///
    /// \brief Contructor with from a given type, joint1_id and placement.
    ///
    /// \param[in] type Type of the contact.
    /// \param[in] joint1_id Index of the joint 1 in the model tree.
    /// \param[in] joint1_placement Placement of the constraint w.r.t the frame of joint1.
    /// \param[in] reference_frame Reference frame in which the constraints quantities are expressed.
    ///
    template<int OtherOptions, template<typename,int> class JointCollectionTpl>
    RigidConstraintModelTpl(const ContactType type,
                            const ModelTpl<Scalar,OtherOptions,JointCollectionTpl> & model,
                            const JointIndex joint1_id,
                            const SE3 & joint1_placement,
                            const ReferenceFrame & reference_frame = LOCAL)
    : type(type)
    , joint1_id(joint1_id)
    , joint2_id(0)
    , joint1_placement(joint1_placement)
    , joint2_placement(SE3::Identity())
    , reference_frame(reference_frame)
    , desired_contact_placement(SE3::Identity())
    , desired_contact_velocity(Motion::Zero())
    , desired_contact_acceleration(Motion::Zero())
    , corrector(size())
    , colwise_joint1_sparsity(model.nv)
    , colwise_joint2_sparsity(model.nv)
    {
      init(model);
    }
    
    ///
    /// \brief Contructor with from a given type and the joint ids.
    ///
    /// \param[in] type Type of the contact.
    /// \param[in] joint1_id Index of the joint 1 in the model tree.
    /// \param[in] joint2_id Index of the joint 2 in the model tree.
    ///
    template<int OtherOptions, template<typename,int> class JointCollectionTpl>
    RigidConstraintModelTpl(const ContactType type,
                            const ModelTpl<Scalar,OtherOptions,JointCollectionTpl> & model,
                            const JointIndex joint1_id,
                            const JointIndex joint2_id,
                            const ReferenceFrame & reference_frame = LOCAL)
    : type(type)
    , joint1_id(joint1_id)
    , joint2_id(joint2_id)
    , joint1_placement(SE3::Identity())
    , joint2_placement(SE3::Identity())
    , reference_frame(reference_frame)
    , desired_contact_placement(SE3::Identity())
    , desired_contact_velocity(Motion::Zero())
    , desired_contact_acceleration(Motion::Zero())
    , corrector(size())
    , colwise_joint1_sparsity(model.nv)
    , colwise_joint2_sparsity(model.nv)
    {
      init(model);
    }
    
    ///
    /// \brief Contructor with from a given type and .
    ///
    /// \param[in] type Type of the contact.
    /// \param[in] joint1_id Index of the joint 1 in the model tree.
    ///
    /// \remarks The second joint id (joint2_id) is set to be 0 (corresponding to the index of the universe).
    ///
    template<int OtherOptions, template<typename,int> class JointCollectionTpl>
    RigidConstraintModelTpl(const ContactType type,
                            const ModelTpl<Scalar,OtherOptions,JointCollectionTpl> & model,
                            const JointIndex joint1_id,
                            const ReferenceFrame & reference_frame = LOCAL)
    : type(type)
    , joint1_id(joint1_id)
    , joint2_id(0) // set to be the Universe
    , joint1_placement(SE3::Identity())
    , joint2_placement(SE3::Identity())
    , reference_frame(reference_frame)
    , desired_contact_placement(SE3::Identity())
    , desired_contact_velocity(Motion::Zero())
    , desired_contact_acceleration(Motion::Zero())
    , corrector(size())
    , colwise_joint1_sparsity(model.nv)
    , colwise_joint2_sparsity(model.nv)
    {
      init(model);
    }
    
    ///
    /// \brief Comparison operator
    ///
    /// \param[in] other Other RigidConstraintModelTpl to compare with.
    ///
    /// \returns true if the two *this is equal to other (type, joint1_id and placement attributs must be the same).
    ///
    template<int OtherOptions>
    bool operator==(const RigidConstraintModelTpl<Scalar,OtherOptions> & other) const
    {
      return
         name == other.name
      && type == other.type
      && joint1_id == other.joint1_id
      && joint2_id == other.joint2_id
      && joint1_placement == other.joint1_placement
      && joint2_placement == other.joint2_placement
      && reference_frame == other.reference_frame
      && corrector == other.corrector
      && colwise_joint1_sparsity == other.colwise_joint1_sparsity
      && colwise_joint2_sparsity == other.colwise_joint2_sparsity
      && colwise_span_indexes == other.colwise_span_indexes
      && nv == other.nv
      && depth_joint1 == other.depth_joint1
      && depth_joint2 == other.depth_joint2
      ;
    }
    
    ///
    /// \brief Oposite of the comparison operator.
    ///
    /// \param[in] other Other RigidConstraintModelTpl to compare with.
    ///
    /// \returns false if the two *this is not equal to other (at least type, joint1_id or placement attributs is different).
    ///
    template<int OtherOptions>
    bool operator!=(const RigidConstraintModelTpl<Scalar,OtherOptions> & other) const
    {
      return !(*this == other);
    }

    template<template<typename,int> class JointCollectionTpl>
    void calc(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
              const DataTpl<Scalar,Options,JointCollectionTpl> & data,
                    RigidConstraintDataTpl<Scalar,Options> & cdata) const
    {
      PINOCCHIO_UNUSED_VARIABLE(model);

      if(joint1_id > 0)
        cdata.oMc1 = data.oMi[joint1_id] * joint1_placement;
      else
        cdata.oMc1 = joint1_placement;

      if(joint2_id > 0)
        cdata.oMc2 = data.oMi[joint2_id] * joint2_placement;
      else
        cdata.oMc2 = joint2_placement;

      // Compute relative placement
      cdata.c1Mc2 = cdata.oMc1.actInv(cdata.oMc2);
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

    /// \returns An expression of *this with the Scalar type casted to NewScalar.
    template<typename NewScalar>
    RigidConstraintModelTpl<NewScalar,Options> cast() const
    {
      typedef RigidConstraintModelTpl<NewScalar,Options> ReturnType;
      ReturnType res;
      res.type = type;
      res.joint1_id = joint1_id;
      res.joint2_id = joint2_id;
      res.joint1_placement = joint1_placement.template cast<NewScalar>();
      res.joint2_placement = joint2_placement.template cast<NewScalar>();
      res.reference_frame = reference_frame;
      res.desired_contact_placement = desired_contact_placement.template cast<NewScalar>();
      res.desired_contact_velocity = desired_contact_velocity.template cast<NewScalar>();
      res.desired_contact_acceleration = desired_contact_acceleration.template cast<NewScalar>();
      res.corrector = corrector.template cast<NewScalar>();
      res.colwise_joint1_sparsity = colwise_joint1_sparsity;
      res.colwise_joint2_sparsity = colwise_joint2_sparsity;
      res.colwise_span_indexes = colwise_span_indexes;
      res.nv = nv;
      res.depth_joint1 = depth_joint1;
      res.depth_joint2 = depth_joint2;
      
      return res;
    }
    
  protected:
    
    template<int OtherOptions, template<typename,int> class JointCollectionTpl>
    void init(const ModelTpl<Scalar,OtherOptions,JointCollectionTpl> & model)
    {
      nv = model.nv;
      depth_joint1 = static_cast<size_t>(model.supports[joint1_id].size());
      depth_joint2 = static_cast<size_t>(model.supports[joint2_id].size());
      
      typedef ModelTpl<Scalar,OtherOptions,JointCollectionTpl> Model;
      static const bool default_sparsity_value = false;
      colwise_joint1_sparsity.fill(default_sparsity_value);
      colwise_joint2_sparsity.fill(default_sparsity_value);
      
      JointIndex current1_id = 0;
      if(joint1_id > 0)
        current1_id = joint1_id;

      JointIndex current2_id = 0;
      if(joint2_id > 0)
        current2_id = joint2_id;
      
      while(current1_id != current2_id)
      {
        if(current1_id > current2_id)
        {
          const typename Model::JointModel & joint1 = model.joints[current1_id];
          Eigen::DenseIndex current1_col_id = joint1.idx_v();
          for(int k = 0; k < joint1.nv(); ++k,++current1_col_id)
          {
            colwise_joint1_sparsity[current1_col_id] = true;
          }
          current1_id = model.parents[current1_id];
        }
        else
        {
          const typename Model::JointModel & joint2 = model.joints[current2_id];
          Eigen::DenseIndex current2_col_id = joint2.idx_v();
          for(int k = 0; k < joint2.nv(); ++k,++current2_col_id)
          {
            colwise_joint2_sparsity[current2_col_id] = true;
          }
          current2_id = model.parents[current2_id];
        }
      }
      assert(current1_id == current2_id && "current1_id should be equal to current2_id");
      
      if(type == CONTACT_3D && reference_frame != WORLD)
      {
        JointIndex current_id = current1_id;
        while(current_id > 0)
        {
          const typename Model::JointModel & joint = model.joints[current_id];
          Eigen::DenseIndex current_row_id = joint.idx_v();
          for(int k = 0; k < joint.nv(); ++k,++current_row_id)
          {
            colwise_joint1_sparsity[current_row_id] = true;
            colwise_joint2_sparsity[current_row_id] = true;
          }
          current_id = model.parents[current_id];
        }
      }
      
      Eigen::DenseIndex size = 0;
      colwise_span_indexes.resize(model.nv);
      for(Eigen::DenseIndex col_id = 0; col_id < model.nv; ++col_id)
      {
        if(colwise_joint1_sparsity[col_id] || colwise_joint2_sparsity[col_id])
          colwise_span_indexes[size++] = col_id;
      }
      colwise_span_indexes.conservativeResize(size);
    }
    
  };

  template<typename Scalar, int Options, class Allocator>
  size_t getTotalConstraintSize(const std::vector< RigidConstraintModelTpl<Scalar,Options>, Allocator> & contact_models)
  {
    typedef std::vector< RigidConstraintModelTpl<Scalar,Options>, Allocator> VectorType;
    size_t total_size = 0;
    for(typename VectorType::const_iterator it = contact_models.begin(); it != contact_models.end(); ++it)
      total_size += it->size();
    
    return total_size;
  }

  ///
  /// \brief Contact model structure containg all the info describing the rigid contact model
  ///
  template<typename _Scalar, int _Options>
  struct RigidConstraintDataTpl
  : NumericalBase< RigidConstraintDataTpl<_Scalar,_Options> >
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    typedef _Scalar Scalar;
    enum { Options = _Options };
    
    typedef RigidConstraintModelTpl<Scalar,Options> ContactModel;
    typedef RigidConstraintDataTpl ContactData;
    
    typedef SE3Tpl<Scalar,Options> SE3;
    typedef MotionTpl<Scalar,Options> Motion;
    typedef ForceTpl<Scalar,Options> Force;
    typedef Eigen::Matrix<Scalar,6,6,Options> Matrix6;
    typedef PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(Matrix6) VectorOfMatrix6;
    typedef Eigen::Matrix<Scalar,6,Eigen::Dynamic,Options> Matrix6x;
    typedef Eigen::Matrix<Scalar,Eigen::Dynamic,Eigen::Dynamic,Options> MatrixX;
    
    // data
    
    /// \brief Resulting contact forces
    Force contact_force;
    
    /// \brief Placement of the constraint frame 1 with respect to the WORLD frame
    SE3 oMc1;
    
    /// \brief Placement of the constraint frame 2 with respect to the WORLD frame
    SE3 oMc2;
    
    /// \brief Relative displacement between the two frames
    SE3 c1Mc2;
    
    /// \brief Current contact placement error
    Motion contact_placement_error;
    
    /// \brief Current contact spatial velocity of the constraint 1
    Motion contact1_velocity;
    
    /// \brief Current contact spatial velocity of the constraint 2
    Motion contact2_velocity;
    
    /// \brief Current contact velocity error
    Motion contact_velocity_error;
    
    /// \brief Current contact spatial acceleration
    Motion contact_acceleration;
    
    /// \brief Contact spatial acceleration desired
    Motion contact_acceleration_desired;
    
    /// \brief Current contact spatial error (due to the integration step).
    Motion contact_acceleration_error;
    
    /// \brief Current contact drift acceleration (acceleration only due to the Coriolis and centrifugal effects) for the constraint frame 1.
    Motion contact1_acceleration_drift;
    
    /// \brief Current contact drift acceleration (acceleration only due to the Coriolis and centrifugal effects) for the constraint frame 2.
    Motion contact2_acceleration_drift;
    
    /// \brief Contact deviation from the reference acceleration (a.k.a the error)
    Motion contact_acceleration_deviation;
    
    VectorOfMatrix6 extended_motion_propagators_joint1;
    VectorOfMatrix6 lambdas_joint1;
    VectorOfMatrix6 extended_motion_propagators_joint2;
    
    Matrix6x dv1_dq, da1_dq, da1_dv, da1_da;
    Matrix6x dv2_dq, da2_dq, da2_dv, da2_da;
    MatrixX dvc_dq, dac_dq, dac_dv, dac_da;
    
    /// \brief Default constructor
    RigidConstraintDataTpl() {}

    explicit RigidConstraintDataTpl(const ContactModel & contact_model)
    : contact_force(Force::Zero())
    , contact_placement_error(Motion::Zero())
    , contact1_velocity(Motion::Zero())
    , contact2_velocity(Motion::Zero())
    , contact_velocity_error(Motion::Zero())
    , contact_acceleration(Motion::Zero())
    , contact_acceleration_desired(Motion::Zero())
    , contact_acceleration_error(Motion::Zero())
    , contact1_acceleration_drift(Motion::Zero())
    , contact2_acceleration_drift(Motion::Zero())
    , contact_acceleration_deviation(Motion::Zero())
    , extended_motion_propagators_joint1(contact_model.depth_joint1,Matrix6::Zero())
    , lambdas_joint1(contact_model.depth_joint1,Matrix6::Zero())
    , extended_motion_propagators_joint2(contact_model.depth_joint2,Matrix6::Zero())
    , dv1_dq(6,contact_model.nv)
    , da1_dq(6,contact_model.nv)
    , da1_dv(6,contact_model.nv)
    , da1_da(6,contact_model.nv)
    , dv2_dq(6,contact_model.nv)
    , da2_dq(6,contact_model.nv)
    , da2_dv(6,contact_model.nv)
    , da2_da(6,contact_model.nv)
    , dvc_dq(contact_model.size(),contact_model.nv)
    , dac_dq(contact_model.size(),contact_model.nv)
    , dac_dv(contact_model.size(),contact_model.nv)
    , dac_da(contact_model.size(),contact_model.nv)
    {}
    
    bool operator==(const RigidConstraintDataTpl & other) const
    {
      return
         contact_force == other.contact_force
      && oMc1 == other.oMc1
      && oMc2 == other.oMc2
      && c1Mc2 == other.c1Mc2
      && contact_placement_error == other.contact_placement_error
      && contact1_velocity == other.contact1_velocity
      && contact2_velocity == other.contact2_velocity
      && contact_velocity_error == other.contact_velocity_error
      && contact_acceleration == other.contact_acceleration
      && contact_acceleration_desired == other.contact_acceleration_desired
      && contact_acceleration_error == other.contact_acceleration_error
      && contact1_acceleration_drift == other.contact1_acceleration_drift
      && contact2_acceleration_drift == other.contact2_acceleration_drift
      && contact_acceleration_deviation == other.contact_acceleration_deviation
      && extended_motion_propagators_joint1 == other.extended_motion_propagators_joint1
      && lambdas_joint1 == other.lambdas_joint1
      && extended_motion_propagators_joint2 == other.extended_motion_propagators_joint2
      //
      && dv1_dq == other.dv1_dq
      && da1_dq == other.da1_dq
      && da1_dv == other.da1_dv
      && da1_da == other.da1_da
      //
      && dv2_dq == other.dv2_dq
      && da2_dq == other.da2_dq
      && da2_dv == other.da2_dv
      && da2_da == other.da2_da
      //
      && dvc_dq == other.dvc_dq
      && dac_dq == other.dac_dq
      && dac_dv == other.dac_dv
      && dac_da == other.dac_da
      ;
    }
    
    bool operator!=(const RigidConstraintDataTpl & other) const
    {
      return !(*this == other);
    }
  };
  
}

#endif // ifndef __pinocchio_algorithm_contact_info_hpp__
