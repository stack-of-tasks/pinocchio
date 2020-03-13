//
// Copyright (c) 2019-2020 INRIA
//

#ifndef __pinocchio_algorithm_contact_dynamics_hxx__
#define __pinocchio_algorithm_contact_dynamics_hxx__

#include "pinocchio/spatial/classic-acceleration.hpp"

#include "pinocchio/algorithm/check.hpp"
#include "pinocchio/algorithm/aba.hpp"
#include "pinocchio/algorithm/contact-cholesky.hxx"

#include <limits>

#include <iostream>

namespace pinocchio
{

  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, class Allocator>
  inline void initContactDynamics(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                                  DataTpl<Scalar,Options,JointCollectionTpl> & data,
                                  const std::vector<RigidContactModelTpl<Scalar,Options>,Allocator> & contact_models)
  {
    data.contact_chol.allocate(model,contact_models);
    data.contact_vector_solution.resize(data.contact_chol.size());

    data.lambda_c.resize(data.contact_chol.constraintDim());
    
    data.dlambda_dq.resize(data.contact_chol.constraintDim(), model.nv);
    data.dac_dq.resize(data.contact_chol.constraintDim(), model.nv);
    data.osim.resize(data.contact_chol.constraintDim(), data.contact_chol.constraintDim());
    data.v_partial_dq.resize(contact_models.size(),Data::Matrix6x::Zero(6,model.nv));
    data.a_partial_dq.resize(contact_models.size(),Data::Matrix6x::Zero(6,model.nv));
    data.a_partial_dv.resize(contact_models.size(),Data::Matrix6x::Zero(6,model.nv));
    data.a_partial_da.resize(contact_models.size(),Data::Matrix6x::Zero(6,model.nv));
  }
  
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType, typename TangentVectorType>
  struct ContactDynamicsForwardStep
  : public fusion::JointUnaryVisitorBase< ContactDynamicsForwardStep<Scalar,Options,JointCollectionTpl,ConfigVectorType,TangentVectorType> >
  {
    typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
    typedef DataTpl<Scalar,Options,JointCollectionTpl> Data;
    
    typedef boost::fusion::vector<const Model &,
                                  Data &,
                                  const ConfigVectorType &,
                                  const TangentVectorType &
                                  > ArgsType;
    
    template<typename JointModel>
    static void algo(const JointModelBase<JointModel> & jmodel,
                     JointDataBase<typename JointModel::JointDataDerived> & jdata,
                     const Model & model,
                     Data & data,
                     const Eigen::MatrixBase<ConfigVectorType> & q,
                     const Eigen::MatrixBase<TangentVectorType> & v)
    {
      typedef typename Model::JointIndex JointIndex;
      typedef typename Data::Motion Motion;
      typedef typename Data::Force Force;
      typedef typename Data::Inertia Inertia;
      
      const JointIndex & i = jmodel.id();
      const JointIndex & parent = model.parents[i];
      
      Motion & ov = data.ov[i];
      Motion & oa = data.oa[i];
      Motion & oa_gf = data.oa_gf[i];
      
      Inertia & oYcrb = data.oYcrb[i];
      
      Force & oh = data.oh[i];
      Force & of = data.of[i];
      
      jmodel.calc(jdata.derived(),q.derived(),v.derived());
      
      data.liMi[i] = model.jointPlacements[i]*jdata.M();
      if(parent > 0)
        data.oMi[i] = data.oMi[parent] * data.liMi[i];
      else
        data.oMi[i] = data.liMi[i];
      
      ov = data.oMi[i].act(jdata.v());
      if(parent > 0)
        ov += data.ov[parent];
      
      oa = data.oMi[i].act(jdata.c());
      
      if(parent > 0)
      {
        oa += (data.ov[parent] ^ ov);
        oa += data.oa[parent];
      }
      
      jmodel.jointCols(data.J) = data.oMi[i].act(jdata.S());
      oYcrb = data.oMi[i].act(model.inertias[i]);
      
      oa_gf = oa - model.gravity; // add gravity contribution
      
      oh = oYcrb * ov;
      of = oYcrb * oa_gf + ov.cross(oh);
    }
    
  };
  
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  struct ContactDynamicsBackwardStep
  : public fusion::JointUnaryVisitorBase< ContactDynamicsBackwardStep<Scalar,Options,JointCollectionTpl> >
  {
    typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
    typedef DataTpl<Scalar,Options,JointCollectionTpl> Data;
    
    typedef boost::fusion::vector<const Model &,
                                  Data &
                                 > ArgsType;
    
    template<typename JointModel>
    static void algo(const JointModelBase<JointModel> & jmodel,
                     const Model & model,
                     Data & data)
    {
      typedef typename Model::JointIndex JointIndex;
      typedef typename SizeDepType<JointModel::NV>::template ColsReturn<typename Data::Matrix6x>::Type ColsBlock;
      const JointIndex & i = jmodel.id();

      ColsBlock Ag_cols = jmodel.jointCols(data.Ag);
      const ColsBlock J_cols = jmodel.jointCols(data.J);
      motionSet::inertiaAction(data.oYcrb[i],J_cols,Ag_cols);
      
      data.M.block(jmodel.idx_v(),jmodel.idx_v(),jmodel.nv(),data.nvSubtree[i]).noalias()
      = J_cols.transpose()*data.Ag.middleCols(jmodel.idx_v(),data.nvSubtree[i]);
      
      jmodel.jointVelocitySelector(data.nle).noalias()
      = J_cols.transpose()*data.of[i].toVector();
      
      const JointIndex & parent = model.parents[i];
      data.of[parent] += data.of[i];
      data.oYcrb[parent] += data.oYcrb[i];
    }
    
  };
  
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType, typename TangentVectorType1, typename TangentVectorType2, class ContactModelAllocator, class ContactDataAllocator>
  inline const typename DataTpl<Scalar,Options,JointCollectionTpl>::TangentVectorType &
  contactDynamics(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                  DataTpl<Scalar,Options,JointCollectionTpl> & data,
                  const Eigen::MatrixBase<ConfigVectorType> & q,
                  const Eigen::MatrixBase<TangentVectorType1> & v,
                  const Eigen::MatrixBase<TangentVectorType2> & tau,
                  const std::vector<RigidContactModelTpl<Scalar,Options>,ContactModelAllocator> & contact_models,
                  std::vector<RigidContactDataTpl<Scalar,Options>,ContactDataAllocator> & contact_datas,
                  const Scalar mu)
  {
    assert(model.check(data) && "data is not consistent with model.");
    PINOCCHIO_CHECK_INPUT_ARGUMENT(q.size() == model.nq,
                                   "The joint configuration vector is not of right size");
    PINOCCHIO_CHECK_INPUT_ARGUMENT(v.size() == model.nv,
                                   "The joint velocity vector is not of right size");
    PINOCCHIO_CHECK_INPUT_ARGUMENT(tau.size() == model.nv,
                                   "The joint torque vector is not of right size");
    PINOCCHIO_CHECK_INPUT_ARGUMENT(mu >= Scalar(0),
                                   "mu has to be positive");
    PINOCCHIO_CHECK_INPUT_ARGUMENT(contact_models.size() == contact_datas.size(),
                                   "The contact models and data do not have the same vector size.");
    
    typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
    typedef DataTpl<Scalar,Options,JointCollectionTpl> Data;
    typedef typename Data::Motion Motion;
    
    typedef RigidContactModelTpl<Scalar,Options> RigidContactModel;
    typedef RigidContactDataTpl<Scalar,Options> RigidContactData;
    
    typename Data::TangentVectorType & a = data.ddq;
    typename Data::ContactCholeskyDecomposition & contact_chol = data.contact_chol;
    typename Data::VectorXs & contact_vector_solution = data.contact_vector_solution;
    
    data.oYcrb[0].setZero();
    data.of[0].setZero();
    typedef ContactDynamicsForwardStep<Scalar,Options,JointCollectionTpl,ConfigVectorType,TangentVectorType1> Pass1;
    for(JointIndex i=1;i<(JointIndex) model.njoints;++i)
    {
      Pass1::run(model.joints[i],data.joints[i],
                 typename Pass1::ArgsType(model,data,q.derived(),v.derived()));
    }
    
    typedef ContactDynamicsBackwardStep<Scalar,Options,JointCollectionTpl> Pass2;
    for(JointIndex i=(JointIndex)(model.njoints-1);i>0;--i)
    {
      Pass2::run(model.joints[i],
                 typename Pass2::ArgsType(model,data));
    }
    
    // Add the armature contribution
    data.M.diagonal() += model.armature;
    
    // Retrieve the Centroidal Momemtum map
    typedef typename Data::Force Force;
    typedef Eigen::Block<typename Data::Matrix6x,3,-1> Block3x;
    
    data.com[0] = data.oYcrb[0].lever();
    
    const Block3x Ag_lin = data.Ag.template middleRows<3>(Force::LINEAR);
    Block3x Ag_ang = data.Ag.template middleRows<3>(Force::ANGULAR);
    for(long i = 0; i<model.nv; ++i)
      Ag_ang.col(i) += Ag_lin.col(i).cross(data.com[0]);

    // Computes the Cholesky decomposition
    contact_chol.compute(model,data,contact_models,contact_datas,mu);

    contact_vector_solution.tail(model.nv) = tau - data.nle;

    // Temporary variables
    Motion coriolis_centrifugal_acc; // Coriolis/centrifugal acceleration of the contact frame.
    typename Motion::Vector3 coriolis_centrifugal_acc_local;

    Eigen::DenseIndex current_row_id = 0;
    for(size_t contact_id = 0; contact_id < contact_models.size(); ++contact_id)
    {
      const RigidContactModel & contact_model = contact_models[contact_id];
      RigidContactData & contact_data = contact_datas[contact_id];
      const int contact_dim = contact_model.size();

      const typename Model::FrameIndex & frame_id = contact_model.frame_id;
      const typename Model::Frame & frame = model.frames[frame_id];
      const typename Model::JointIndex & joint_id = frame.parent;
      const typename Data::SE3 & oMi = data.oMi[joint_id];
      
      SE3 & iMcontact = contact_data.joint_contact_placement;
      SE3 & oMcontact = contact_data.contact_placement;

      // Update frame placement
      iMcontact = frame.placement * contact_model.placement;
      oMcontact = oMi * iMcontact;

      classicAcceleration(data.ov[joint_id],
                          data.oa[joint_id],
                          oMcontact,
                          coriolis_centrifugal_acc_local);
      
      switch(contact_model.reference_frame)
      {
        case WORLD:
        {
          // LINEAR
          classicAcceleration(data.ov[joint_id],
                              data.oa[joint_id],
                              coriolis_centrifugal_acc.linear());
          // ANGULAR
          coriolis_centrifugal_acc.angular() = data.oa[joint_id].angular();
          
          break;
        }
        case LOCAL_WORLD_ALIGNED:
        {
          // LINEAR
          coriolis_centrifugal_acc.linear().noalias() = oMcontact.rotation() * coriolis_centrifugal_acc_local;
          // ANGULAR
          coriolis_centrifugal_acc.angular() = data.oa[joint_id].angular();
          
          break;
        }
        case LOCAL:
        {
          // LINEAR
          coriolis_centrifugal_acc.linear() = coriolis_centrifugal_acc_local;
          // ANGULAR
          coriolis_centrifugal_acc.angular().noalias() = oMcontact.rotation().transpose() * data.oa[joint_id].angular();
          
          break;
        }
        default:
          assert(false && "must never happened");
          break;
      }

      switch(contact_model.type)
      {
        case CONTACT_3D:
          contact_vector_solution.segment(current_row_id,contact_dim) = -coriolis_centrifugal_acc.linear();
          break;
        case CONTACT_6D:
          contact_vector_solution.segment(current_row_id,contact_dim) = -coriolis_centrifugal_acc.toVector();
          break;
        default:
          assert(false && "must never happened");
          break;
      }

      current_row_id += contact_dim;
    }
    
    // Solve the system
    contact_chol.solveInPlace(contact_vector_solution);
    
    // Retrieve the joint space acceleration
    a = contact_vector_solution.tail(model.nv);

    data.lambda_c = -contact_vector_solution.head(contact_chol.constraintDim());
    
    //Set contact forces to zero
    std::fill(data.contact_forces.begin(), data.contact_forces.end(), Force::Zero());
    
    // Retrieve the contact forces
    Eigen::DenseIndex current_row_sol_id = 0;
    for(size_t contact_id = 0; contact_id < contact_models.size(); ++contact_id)
    {
      const RigidContactModel & contact_model = contact_models[contact_id];
      RigidContactData & contact_data = contact_datas[contact_id];
      typename RigidContactData::Force & fext = contact_data.contact_force;
      const int contact_dim = contact_model.size();
      
      switch(contact_model.type)
      {
        case CONTACT_3D:
        {
          fext.linear() = -contact_vector_solution.template segment<3>(current_row_sol_id);
          fext.angular().setZero();
          break;
        }
        case CONTACT_6D:
        {
          typedef typename Data::VectorXs::template FixedSegmentReturnType<6>::Type Segment6d;
          const ForceRef<Segment6d> f_sol(contact_vector_solution.template segment<6>(current_row_sol_id));
          fext = -f_sol;
          break;
        }
        default:
          assert(false && "must never happened");
          break;
      }
      
      current_row_sol_id += contact_dim;
    }
    
    return a;
  }

  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType, typename TangentVectorType>
  struct ContactABAForwardStep1
  : public fusion::JointUnaryVisitorBase< ContactABAForwardStep1<Scalar,Options,JointCollectionTpl,ConfigVectorType,TangentVectorType> >
  {
    typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
    typedef DataTpl<Scalar,Options,JointCollectionTpl> Data;
    
    typedef boost::fusion::vector<const Model &,
                                  Data &,
                                  const ConfigVectorType &,
                                  const TangentVectorType &
                                  > ArgsType;
    
    template<typename JointModel>
    static void algo(const pinocchio::JointModelBase<JointModel> & jmodel,
                     pinocchio::JointDataBase<typename JointModel::JointDataDerived> & jdata,
                     const Model & model,
                     Data & data,
                     const Eigen::MatrixBase<ConfigVectorType> & q,
                     const Eigen::MatrixBase<TangentVectorType> & v)
    {
      typedef typename Model::JointIndex JointIndex;
      
      const JointIndex & i = jmodel.id();
      jmodel.calc(jdata.derived(),q.derived(),v.derived());
      
      const JointIndex & parent = model.parents[i];
      data.liMi[i] = model.jointPlacements[i] * jdata.M();
      if(parent > 0)
        data.oMi[i] = data.oMi[parent] * data.liMi[i];
      else
        data.oMi[i] = data.liMi[i];
      
      jmodel.jointCols(data.J) = data.oMi[i].act(jdata.S());
      
      data.ov[i] = data.oMi[i].act(jdata.v());
      if(parent > 0)
        data.ov[i] += data.ov[parent];

      data.oa[i] = data.oMi[i].act(jdata.c());
      if(parent > 0)
      {
        data.oa[i] += (data.ov[parent] ^ data.ov[i]);
      }
      
      data.oa_drift[i] = data.oa[i];
      if(parent > 0)
      {
        data.oa_drift[i] += data.oa_drift[parent];
      }
      
      data.oYcrb[i] = data.oMi[i].act(model.inertias[i]);
      data.oYaba[i] = data.oYcrb[i].matrix();
      data.of[i] = data.oYcrb[i].vxiv(data.ov[i]) - data.oYcrb[i] * model.gravity; // -f_ext
    }
    
  };

  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename TangentVectorType>
  struct ContactABABackwardStep1
  : public fusion::JointUnaryVisitorBase< ContactABABackwardStep1<Scalar,Options,JointCollectionTpl,TangentVectorType> >
  {
    typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
    typedef DataTpl<Scalar,Options,JointCollectionTpl> Data;
    
    typedef boost::fusion::vector<const Model &,
                                  Data &,
                                  const TangentVectorType &> ArgsType;
    
    template<typename JointModel>
    static void algo(const JointModelBase<JointModel> & jmodel,
                     JointDataBase<typename JointModel::JointDataDerived> & jdata,
                     const Model & model,
                     Data & data,
                     const Eigen::MatrixBase<TangentVectorType> & tau)
    {
      typedef typename Model::JointIndex JointIndex;
      typedef typename Data::Inertia Inertia;
      typedef typename Data::Force Force;
      typedef typename Data::Matrix6x Matrix6x;
      
      const JointIndex & i = jmodel.id();
      const JointIndex & parent  = model.parents[i];
      typename Inertia::Matrix6 & Ia = data.oYaba[i];
      
      typedef typename SizeDepType<JointModel::NV>::template ColsReturn<Matrix6x>::Type ColBlock;
      ColBlock Jcols = jmodel.jointCols(data.J);
      
      Force & fi_augmented = data.of_augmented[i];
      const Force & fi = data.of[i];
      
      fi_augmented += fi;
      jmodel.jointVelocitySelector(data.u).noalias()
      = jmodel.jointVelocitySelector(tau)
      - Jcols.transpose()*fi_augmented.toVector();
      
      jdata.U().noalias() = Ia * Jcols;
      jdata.StU().noalias() = Jcols.transpose() * jdata.U();
      
      // Account for the rotor inertia contribution
      jdata.StU().diagonal() += jmodel.jointVelocitySelector(model.armature);
      
      internal::PerformStYSInversion<Scalar>::run(jdata.StU(),jdata.Dinv());
      jdata.UDinv().noalias() = jdata.U() * jdata.Dinv();
      
      if(parent > 0)
      {
        Ia.noalias() -= jdata.UDinv() * jdata.U().transpose();
        
        fi_augmented.toVector().noalias() += Ia * data.oa[i].toVector() + jdata.UDinv() * jmodel.jointVelocitySelector(data.u);
        data.oYaba[parent] += Ia;
        data.of_augmented[parent] += fi_augmented;
      }
    }
    
  };

  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename TangentVectorType>
  struct ContactABABackwardStepAugmented
  : public fusion::JointUnaryVisitorBase< ContactABABackwardStepAugmented<Scalar,Options,JointCollectionTpl,TangentVectorType> >
    {
      typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
      typedef DataTpl<Scalar,Options,JointCollectionTpl> Data;
      
      typedef boost::fusion::vector<const Model &,
                                    Data &,
                                    const TangentVectorType &> ArgsType;
      
      template<typename JointModel>
      static void algo(const JointModelBase<JointModel> & jmodel,
                       JointDataBase<typename JointModel::JointDataDerived> & jdata,
                       const Model & model,
                       Data & data,
                       const Eigen::MatrixBase<TangentVectorType> & tau)
      {
        typedef typename Model::JointIndex JointIndex;
        typedef typename Data::Inertia Inertia;
        typedef typename Data::Force Force;
        typedef typename Data::Matrix6x Matrix6x;
        
        const JointIndex & i = jmodel.id();
        const JointIndex & parent  = model.parents[i];
        typename Inertia::Matrix6 & Ia = data.oYaba[i];
        
        typedef typename SizeDepType<JointModel::NV>::template ColsReturn<Matrix6x>::Type ColBlock;
        ColBlock Jcols = jmodel.jointCols(data.J);
        
        Force & fi_augmented = data.of_augmented[i];
        const Force & fi = data.of[i];
        
        fi_augmented += fi;
        jmodel.jointVelocitySelector(data.u).noalias()
        = jmodel.jointVelocitySelector(tau)
        - Jcols.transpose()*fi_augmented.toVector();
        
        if(parent > 0)
        {
          fi_augmented.toVector().noalias() += Ia * data.oa[i].toVector() + jdata.UDinv() * jmodel.jointVelocitySelector(data.u);
          data.of_augmented[parent] += fi_augmented;
        }
      }
      
    };
  
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  struct ContactABAForwardStep2
  : public fusion::JointUnaryVisitorBase< ContactABAForwardStep2<Scalar,Options,JointCollectionTpl> >
  {
    typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
    typedef DataTpl<Scalar,Options,JointCollectionTpl> Data;
    
    typedef boost::fusion::vector<const Model &,
                                  Data &> ArgsType;
    
    template<typename JointModel>
    static void algo(const pinocchio::JointModelBase<JointModel> & jmodel,
                     pinocchio::JointDataBase<typename JointModel::JointDataDerived> & jdata,
                     const Model & model,
                     Data & data)
    {
      typedef typename Model::JointIndex JointIndex;
      typedef typename Data::Matrix6x Matrix6x;
      
      typedef typename SizeDepType<JointModel::NV>::template ColsReturn<Matrix6x>::Type ColBlock;
      ColBlock Jcols = jmodel.jointCols(data.J);
      
      const JointIndex & i = jmodel.id();
      const JointIndex & parent = model.parents[i];
      
      data.oa_augmented[i] = data.oa[i];
      if(parent > 0)
        data.oa_augmented[i] += data.oa_augmented[parent]; // does not take into account the gravity field
      jmodel.jointVelocitySelector(data.ddq).noalias() =
      jdata.Dinv() * jmodel.jointVelocitySelector(data.u) - jdata.UDinv().transpose() * data.oa_augmented[i].toVector();
      data.oa_augmented[i].toVector() += Jcols * jmodel.jointVelocitySelector(data.ddq);
    }
    
  };
  
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType, typename TangentVectorType1, typename TangentVectorType2, class ModelAllocator, class DataAllocator>
  inline const typename DataTpl<Scalar,Options,JointCollectionTpl>::TangentVectorType &
  contactABA(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
             DataTpl<Scalar,Options,JointCollectionTpl> & data,
             const Eigen::MatrixBase<ConfigVectorType> & q,
             const Eigen::MatrixBase<TangentVectorType1> & v,
             const Eigen::MatrixBase<TangentVectorType2> & tau,
             const std::vector<RigidContactModelTpl<Scalar,Options>,ModelAllocator> & contact_models,
             std::vector<RigidContactDataTpl<Scalar,Options>,DataAllocator> & contact_data,
             ProximalSettingsTpl<Scalar> & settings)
  {
    assert(model.check(data) && "data is not consistent with model.");
    PINOCCHIO_CHECK_INPUT_ARGUMENT(q.size() == model.nq,
                                   "The joint configuration vector is not of right size");
    PINOCCHIO_CHECK_INPUT_ARGUMENT(v.size() == model.nv,
                                   "The joint velocity vector is not of right size");
    PINOCCHIO_CHECK_INPUT_ARGUMENT(tau.size() == model.nv,
                                   "The joint torque vector is not of right size");
    PINOCCHIO_CHECK_INPUT_ARGUMENT(settings.mu >= Scalar(0),
                                   "mu has to be positive");
    PINOCCHIO_CHECK_INPUT_ARGUMENT(contact_models.size() == contact_data.size(),
                                   "contact models and data size are not the same");
    
    typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
    typedef DataTpl<Scalar,Options,JointCollectionTpl> Data;
    
    typedef typename Data::Motion Motion;
    
    typedef typename Model::JointIndex JointIndex;
    typedef RigidContactModelTpl<Scalar,Options> RigidContactModel;
    typedef RigidContactDataTpl<Scalar,Options> RigidContactData;
    typedef typename Data::Force Force;
    
    typedef ContactABAForwardStep1<Scalar,Options,JointCollectionTpl,ConfigVectorType,TangentVectorType1> Pass1;
    for(JointIndex i=1; i<(JointIndex)model.njoints; ++i)
    {
      Pass1::run(model.joints[i],data.joints[i],
                 typename Pass1::ArgsType(model,data,q.derived(),v.derived()));
      data.of_augmented[i].setZero();
    }
    
    typename Data::SE3 iMc; // tmp variable
    for(size_t k = 0; k < contact_models.size(); ++k)
    {
      const RigidContactModel & cmodel = contact_models[k];
      RigidContactData & cdata = contact_data[k];
      
      const typename RigidContactModel::FrameIndex & frame_id = cmodel.frame_id;
      const typename Model::Frame & frame = model.frames[frame_id];
      const typename Model::JointIndex & joint_id = frame.parent;

      // Compute relative placement between the joint and the contact frame
      iMc = frame.placement * cmodel.placement;
      SE3 & oMc = cdata.contact_placement;
      oMc = data.oMi[joint_id] * iMc; // contact placement
      
      typedef typename Data::Inertia Inertia;
      typedef typename Inertia::Symmetric3 Symmetric3;
      
      // Add contact inertia to the joint articulated inertia
      Symmetric3 S(Symmetric3::Zero());
      if(cmodel.type == CONTACT_6D)
        S.setDiagonal(Symmetric3::Vector3::Constant(settings.mu));
      
      Inertia contact_inertia(settings.mu,oMc.translation(),S);
      data.oYaba[joint_id] += contact_inertia.matrix();
      
      typename Data::Motion & joint_velocity = data.ov[joint_id];
      Motion & contact_velocity = cdata.contact_velocity;
      contact_velocity = oMc.actInv(joint_velocity);
      
      typename Data::Motion & joint_spatial_acceleration_drift = data.oa_drift[joint_id];
      Motion & contact_acceleration_drift = cdata.contact_acceleration_drift;
      contact_acceleration_drift
      = cmodel.desired_contact_acceleration - oMc.actInv(joint_spatial_acceleration_drift);
      // Handle the classic acceleration term
      contact_acceleration_drift.linear() -= contact_velocity.angular().cross(contact_velocity.linear());
      
      // Init contact force
//      cdata.contact_force.setZero();
      
      // Add the contribution of the constraints to the force vector
      data.of_augmented[joint_id] = oMc.act(cdata.contact_force);
      if(cmodel.type == CONTACT_3D)
      {
        data.of_augmented[joint_id] -= settings.mu * oMc.act(Force(contact_acceleration_drift.linear(),
                                                                            Force::Vector3::Zero()));
      }
      else
      {
        data.of_augmented[joint_id] -= oMc.act(Force(settings.mu * contact_acceleration_drift.toVector()));
      }
    }

    typedef ContactABABackwardStep1<Scalar,Options,JointCollectionTpl,TangentVectorType2> Pass2;
    for(JointIndex i=(JointIndex)model.njoints-1;i>0; --i)
    {
      Pass2::run(model.joints[i],data.joints[i],
                 typename Pass2::ArgsType(model,data,
                                          tau.derived()));
    }

    typedef ContactABAForwardStep2<Scalar,Options,JointCollectionTpl> Pass3;
    for(JointIndex i=1; i<(JointIndex)model.njoints; ++i)
    {
      Pass3::run(model.joints[i],data.joints[i],
                 typename Pass3::ArgsType(model,data));
      data.of_augmented[i].setZero();
    }
    
    settings.iter = 0;
    bool optimal_solution_found = false;
    if(contact_models.size() == 0)
    {
      return data.ddq;
    }
    
    Scalar primal_infeasibility = Scalar(0);
    int it = 0;
    for(int it = 0; it < settings.max_iter; ++it)
    {
      // Compute contact acceleration errors and max contact errors, aka primal_infeasibility
      primal_infeasibility = Scalar(0);
      for(size_t contact_id = 0; contact_id < contact_models.size(); ++contact_id)
      {
        const RigidContactModel & cmodel = contact_models[contact_id];
        RigidContactData & cdata = contact_data[contact_id];
        
        const typename RigidContactModel::FrameIndex & frame_id = cmodel.frame_id;
        const typename Model::Frame & frame = model.frames[frame_id];
        const typename Model::JointIndex & joint_id = frame.parent;
        
        const SE3 & oMc = cdata.contact_placement;
        const Motion & contact_velocity = cdata.contact_velocity;
        
        // Compute contact acceleration error (drift)
        const typename Data::Motion & joint_spatial_acceleration = data.oa_augmented[joint_id];
        cdata.contact_acceleration_deviation = oMc.actInv(joint_spatial_acceleration) - cmodel.desired_contact_acceleration;
        cdata.contact_acceleration_deviation.linear() += contact_velocity.angular().cross(contact_velocity.linear());
        
        using std::max;
        if(cmodel.type == CONTACT_3D)
        {
          primal_infeasibility
          = max<Scalar>(primal_infeasibility,
                        cdata.contact_acceleration_deviation.linear().template lpNorm<Eigen::Infinity>());
        }
        else
        {
          primal_infeasibility
          = max<Scalar>(primal_infeasibility,
                        cdata.contact_acceleration_deviation.toVector().template lpNorm<Eigen::Infinity>());
        }
      }
      
      if(primal_infeasibility < settings.accuracy)
      {
        optimal_solution_found = true;
        break;
      }
      
      // Update contact forces
      for(size_t contact_id = 0; contact_id < contact_models.size(); ++contact_id)
      {
        const RigidContactModel & cmodel = contact_models[contact_id];
        RigidContactData & cdata = contact_data[contact_id];
        
        const typename RigidContactModel::FrameIndex & frame_id = cmodel.frame_id;
        const typename Model::Frame & frame = model.frames[frame_id];
        const typename Model::JointIndex & joint_id = frame.parent;
        
        const SE3 & oMc = cdata.contact_placement;
        // Update contact force value
        if(cmodel.type == CONTACT_3D)
        {
          cdata.contact_force.linear().noalias() += settings.mu * cdata.contact_acceleration_deviation.linear();
        }
        else
        {
          cdata.contact_force.toVector().noalias() += settings.mu * cdata.contact_acceleration_deviation.toVector();
        }

        // Add the contribution of the constraints to the force vector
        const Motion & contact_acceleration_drift = cdata.contact_acceleration_drift;
        data.of_augmented[joint_id] = oMc.act(cdata.contact_force);
        if(cmodel.type == CONTACT_3D)
        {
          data.of_augmented[joint_id] -= settings.mu * oMc.act(Force(contact_acceleration_drift.linear(),
                                                                     Force::Vector3::Zero()));
        }
        else
        {
          data.of_augmented[joint_id] -= oMc.act(Force(settings.mu * contact_acceleration_drift.toVector()));
        }
      }
      
      typedef ContactABABackwardStepAugmented<Scalar,Options,JointCollectionTpl,TangentVectorType2> Pass2Augmented;
      for(JointIndex i=(JointIndex)model.njoints-1;i>0; --i)
      {
        Pass2Augmented::run(model.joints[i],data.joints[i],
                            typename Pass2::ArgsType(model,data,tau.derived()));
      }

      typedef ContactABAForwardStep2<Scalar,Options,JointCollectionTpl> Pass3;
      for(JointIndex i=1; i<(JointIndex)model.njoints; ++i)
      {
        Pass3::run(model.joints[i],data.joints[i],
                   typename Pass3::ArgsType(model,data));
        data.of_augmented[i].setZero();
      }
    }
      
    settings.iter = it;
    settings.residual = primal_infeasibility;

    return data.ddq;
  }
  
} // namespace pinocchio

#endif // ifndef __pinocchio_algorithm_contact_dynamics_hxx__
