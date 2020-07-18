//
// Copyright (c) 2020 CNRS INRIA
//

#ifndef __pinocchio_algorithm_impulse_dynamics_hxx__
#define __pinocchio_algorithm_impulse_dynamics_hxx__

#include "pinocchio/algorithm/check.hpp"
#include "pinocchio/algorithm/contact-cholesky.hxx"
#include "pinocchio/algorithm/contact-dynamics.hxx"
#include <limits>

namespace pinocchio
{
  
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType, typename TangentVectorType1, class ContactModelAllocator, class ContactDataAllocator>
  inline const typename DataTpl<Scalar,Options,JointCollectionTpl>::TangentVectorType &
  impulseDynamics(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                  DataTpl<Scalar,Options,JointCollectionTpl> & data,
                  const Eigen::MatrixBase<ConfigVectorType> & q,
                  const Eigen::MatrixBase<TangentVectorType1> & v_before,
                  const std::vector<RigidContactModelTpl<Scalar,Options>,ContactModelAllocator> & contact_models,
                  std::vector<RigidContactDataTpl<Scalar,Options>,ContactDataAllocator> & contact_datas,
                  const Scalar r_coeff,
                  const Scalar mu)
  {
    assert(model.check(data) && "data is not consistent with model.");
    PINOCCHIO_CHECK_INPUT_ARGUMENT(q.size() == model.nq,
                                   "The joint configuration vector is not of right size");
    PINOCCHIO_CHECK_INPUT_ARGUMENT(v_before.size() == model.nv,
                                   "The joint velocity vector is not of right size");
    PINOCCHIO_CHECK_INPUT_ARGUMENT(mu >= Scalar(0),
                                   "mu has to be positive");
    PINOCCHIO_CHECK_INPUT_ARGUMENT((r_coeff >= Scalar(0)) &&(r_coeff <= Scalar(1)) ,
                                   "r_coeff has to be in [0,1]");
    PINOCCHIO_CHECK_INPUT_ARGUMENT(contact_models.size() == contact_datas.size(),
                                   "The contact models and data do not have the same vector size.");
    
    typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
    typedef DataTpl<Scalar,Options,JointCollectionTpl> Data;
    typedef typename Data::Motion Motion;
    
    typedef RigidContactModelTpl<Scalar,Options> RigidContactModel;
    typedef RigidContactDataTpl<Scalar,Options> RigidContactData;
    
    typename Data::TangentVectorType & dq_after = data.dq_after;
    typename Data::ContactCholeskyDecomposition & contact_chol = data.contact_chol;
    typename Data::VectorXs & contact_vector_solution = data.contact_vector_solution;
    
    data.oYcrb[0].setZero();
    typedef ContactAndImpulseDynamicsForwardStep<Scalar,Options,JointCollectionTpl,ConfigVectorType,TangentVectorType1, false> Pass1;
    for(JointIndex i=1;i<(JointIndex) model.njoints;++i)
    {
      Pass1::run(model.joints[i],data.joints[i],
                 typename Pass1::ArgsType(model,data,q.derived(),v_before.derived()));
    }
    
    typedef ContactAndImpulseDynamicsBackwardStep<Scalar,Options,JointCollectionTpl,false> Pass2;
    for(JointIndex i=(JointIndex)(model.njoints-1);i>0;--i)
    {
      Pass2::run(model.joints[i],
                 typename Pass2::ArgsType(model,data));
    }

    data.M.diagonal() += model.armature;
    contact_chol.compute(model,data,contact_models,contact_datas,mu);
    
    //Centroidal computations
    typedef typename Data::Force Force;
    typedef Eigen::Block<typename Data::Matrix6x,3,-1> Block3x;
    data.com[0] = data.oYcrb[0].lever();
    const Block3x Ag_lin = data.Ag.template middleRows<3>(Force::LINEAR);
    Block3x Ag_ang = data.Ag.template middleRows<3>(Force::ANGULAR);
    for(long i = 0; i<model.nv; ++i)
      Ag_ang.col(i) += Ag_lin.col(i).cross(data.com[0]);
    
    //TODO: send this to the back-forward pass
    //data.M.template triangularView<Eigen::StrictlyLower>()
    //  = data.M.transpose().template triangularView<Eigen::StrictlyLower>();
    //contact_vector_solution.tail(model.nv).noalias() = data.M*v_before;

    contact_vector_solution.tail(model.nv).noalias() = data.tau;
    contact_vector_solution.tail(model.nv).noalias() += model.armature.cwiseProduct(v_before);
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
      typename Data::SE3 & oMcontact = data.oMf[frame_id];

      // Update frame placement
      oMcontact = oMi * frame.placement;

      switch(contact_model.reference_frame)
      {
        case WORLD:
        {
          //Temporary assignment
          data.ov[0] = data.ov[joint_id];
          break;
        }
        case LOCAL_WORLD_ALIGNED:
        {
          //Temporary assignment
          data.ov[0] = data.ov[joint_id];
          data.ov[0].linear() -= oMcontact.translation().cross(data.ov[joint_id].angular());
          break;
        }
        case LOCAL:
        {
          //Temporary assignment
          data.ov[0] = oMcontact.actInv(data.ov[joint_id]);
          break;
        }
        default:
          assert(false && "must never happened");
          break;
      }

      switch(contact_model.type)
      {
        case CONTACT_3D:
          contact_vector_solution.segment(current_row_id,contact_dim) = -r_coeff*data.ov[0].linear();
          break;
        case CONTACT_6D:
          contact_vector_solution.segment(current_row_id,contact_dim) = -r_coeff*data.ov[0].toVector();
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
    dq_after = contact_vector_solution.tail(model.nv);

    data.impulse_c = -contact_vector_solution.head(contact_chol.constraintDim());

    // Retrieve the impulses
    Eigen::DenseIndex current_row_sol_id = 0;
    for(size_t contact_id = 0; contact_id < contact_models.size(); ++contact_id)
    {
      const RigidContactModel & contact_model = contact_models[contact_id];
      RigidContactData & contact_data = contact_datas[contact_id];
      typename RigidContactData::Force & impulse = contact_data.contact_force;
      const int contact_dim = contact_model.size();
      
      switch(contact_model.type)
      {
        case CONTACT_3D:
        {
          impulse.linear() = -contact_vector_solution.template segment<3>(current_row_sol_id);
          impulse.angular().setZero();
          break;
        }
        case CONTACT_6D:
        {
          typedef typename Data::VectorXs::template FixedSegmentReturnType<6>::Type Segment6d;
          const ForceRef<Segment6d> i_sol(contact_vector_solution.template segment<6>(current_row_sol_id));
          impulse = -i_sol;
          break;
        }
        default:
          assert(false && "must never happened");
          break;
      }
      
      current_row_sol_id += contact_dim;
    }
    
    return dq_after;
  }
  
} // namespace pinocchio

#endif // ifndef __pinocchio_algorithm_impulse_dynamics_hxx__
