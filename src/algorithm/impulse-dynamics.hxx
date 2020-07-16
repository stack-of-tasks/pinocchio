//
// Copyright (c) 2020 CNRS INRIA
//

#ifndef __pinocchio_algorithm_impulse_dynamics_hxx__
#define __pinocchio_algorithm_impulse_dynamics_hxx__

#include "pinocchio/spatial/classic-acceleration.hpp"

#include "pinocchio/algorithm/check.hpp"
#include "pinocchio/algorithm/aba.hpp"
#include "pinocchio/algorithm/contact-cholesky.hxx"

#include <limits>

namespace pinocchio
{

  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, class Allocator>
  inline void initImpulseDynamics(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                                  DataTpl<Scalar,Options,JointCollectionTpl> & data,
                                  const std::vector<RigidContactModelTpl<Scalar,Options>,Allocator> & contact_models)
  {
    data.contact_chol.allocate(model,contact_models);
    data.contact_vector_solution.resize(data.contact_chol.size());

    data.impulse_c.resize(data.contact_chol.constraintDim());
    
    data.dlambda_dq.resize(data.contact_chol.constraintDim(), model.nv);
    data.dlambda_dv.resize(data.contact_chol.constraintDim(), model.nv);
    data.dlambda_dtau.resize(data.contact_chol.constraintDim(), model.nv);
    data.dvc_dq.resize(data.contact_chol.constraintDim(), model.nv);
    data.dac_dq.resize(data.contact_chol.constraintDim(), model.nv);
    data.dac_dv.resize(data.contact_chol.constraintDim(), model.nv);
    data.dac_da.resize(data.contact_chol.constraintDim(), model.nv);
    data.osim.resize(data.contact_chol.constraintDim(), data.contact_chol.constraintDim());

    data.impulse_c.setZero();
    data.dlambda_dq.setZero();
    data.dlambda_dv.setZero();
    data.dlambda_dtau.setZero();
    data.dvc_dq.setZero();
    data.dac_dq.setZero();
    data.dac_dv.setZero();
    data.dac_da.setZero();
    data.osim.setZero();
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
      
      Inertia & oinertias = data.oinertias[i];
      
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
      oinertias = data.oMi[i].act(model.inertias[i]);
      
      oa_gf = oa - model.gravity; // add gravity contribution
      
      oh = oinertias * ov;
      of = oinertias * oa_gf + ov.cross(oh);

      data.oYcrb[i] = data.oinertias[i];
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
      
      typename Data::SE3& iMcontact = contact_data.joint_contact_placement;
      typename Data::SE3& oMcontact = contact_data.contact_placement;

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

    data.impulse_c = -contact_vector_solution.head(contact_chol.constraintDim());
    
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
  
} // namespace pinocchio

#endif // ifndef __pinocchio_algorithm_contact_dynamics_hxx__
