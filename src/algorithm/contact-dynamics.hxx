//
// Copyright (c) 2019 INRIA
//

#ifndef __pinocchio_algorithm_contact_dynamics_hxx__
#define __pinocchio_algorithm_contact_dynamics_hxx__

#include "pinocchio/algorithm/compute-all-terms.hpp"
#include "pinocchio/algorithm/cholesky.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/check.hpp"
#include "pinocchio/algorithm/aba.hpp"
#include "pinocchio/algorithm/contact-cholesky.hxx"

namespace pinocchio
{

  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, class Allocator>
  inline void initContactDynamics(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                                  DataTpl<Scalar,Options,JointCollectionTpl> & data,
                                  const std::vector<RigidContactModelTpl<Scalar,Options>,Allocator> & contact_models)
  {
    data.contact_chol.allocate(model,contact_models);
    data.contact_vector_solution.resize(data.contact_chol.size());
    data.contact_forces.resize(contact_models.size());
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
      
      data.v[i] = jdata.v();
      
      if(parent > 0)
      {
        data.oMi[i] = data.oMi[parent] * data.liMi[i];
        data.v[i] += data.liMi[i].actInv(data.v[parent]);
      }
      else
        data.oMi[i] = data.liMi[i];
      
      data.a[i] = jdata.c() + (data.v[i] ^ jdata.v());
      if(parent > 0)
        data.a[i] += data.liMi[i].actInv(data.a[parent]);
      
      jmodel.jointCols(data.J) = data.oMi[i].act(jdata.S());
      oYcrb = data.oMi[i].act(model.inertias[i]);
      
      ov = data.oMi[i].act(data.v[i]);
      oa = data.oMi[i].act(data.a[i]);
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
      ColsBlock J_cols = jmodel.jointCols(data.J);
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
  
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType, typename TangentVectorType1, typename TangentVectorType2, class Allocator>
  inline const typename DataTpl<Scalar,Options,JointCollectionTpl>::TangentVectorType &
  contactDynamics(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                  DataTpl<Scalar,Options,JointCollectionTpl> & data,
                  const Eigen::MatrixBase<ConfigVectorType> & q,
                  const Eigen::MatrixBase<TangentVectorType1> & v,
                  const Eigen::MatrixBase<TangentVectorType2> & tau,
                  const std::vector<RigidContactModelTpl<Scalar,Options>,Allocator> & contact_models,
                  const Scalar mu)
  {
    assert(q.size() == model.nq);
    assert(v.size() == model.nv);
    assert(tau.size() == model.nv);
    assert(model.check(data) && "data is not consistent with model.");
    
    assert(mu >= (Scalar)0 && "mu must be positive.");
    
    typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
    typedef DataTpl<Scalar,Options,JointCollectionTpl> Data;
    
    typedef RigidContactModelTpl<Scalar,Options> RigidContactModel;
    typedef std::vector<RigidContactModel,Allocator> RigidContactModelVector;
    
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
      Pass2::run(model.joints[i],data.joints[i],
                 typename Pass2::ArgsType(model,data));
    }
    
    // Retrieve the Centroidal Momemtum map
    typedef DataTpl<Scalar,Options,JointCollectionTpl> Data;
    typedef typename Data::Force Force;
    typedef Eigen::Block<typename Data::Matrix6x,3,-1> Block3x;
    
    data.com[0] = data.oYcrb[0].lever();
    
    const Block3x Ag_lin = data.Ag.template middleRows<3>(Force::LINEAR);
    Block3x Ag_ang = data.Ag.template middleRows<3>(Force::ANGULAR);
    for(long i = 0; i<model.nv; ++i)
      Ag_ang.col(i) += Ag_lin.col(i).cross(data.com[0]);

    // Computes the Cholesky decomposition
    contact_chol.compute(model,data,contact_models,mu);

    contact_vector_solution.tail(model.nv) = tau - data.nle;

    // Temporary variables
    typename Data::SE3 oMcontact; // placement of the contact frame in the world frame
    typename Data::SE3 iMcontact; // placement of the contact frame in the local frame of the joint
    typename Data::Motion coriolis_centrifugal_acc; // Coriolis/centrifugal acceleration of the contact frame.
    typename Motion::Vector3 coriolis_centrifugal_acc_local;

    Eigen::DenseIndex current_row_id = 0;
    for(typename RigidContactModelVector::const_iterator it = contact_models.begin();
        it != contact_models.end(); ++it)
    {
      const RigidContactModel & contact_info = *it;
      const int contact_dim = contact_info.size();

      const typename Model::FrameIndex & frame_id = contact_info.frame_id;
      const typename Model::Frame & frame = model.frames[frame_id];
      const typename Model::JointIndex & joint_id = frame.parent;
      const typename Data::SE3 & oMi = data.oMi[joint_id];

      // Update frame placement
      iMcontact = frame.placement * contact_info.placement;
      oMcontact = oMi * iMcontact;

      classicAcceleration(data.v[joint_id],
                          data.a[joint_id],
                          iMcontact,
                          coriolis_centrifugal_acc_local);
      // LINEAR
      coriolis_centrifugal_acc.linear().noalias()
      = oMcontact.rotation() * coriolis_centrifugal_acc_local;

      // ANGULAR
      coriolis_centrifugal_acc.angular().noalias()
      = data.oMi[joint_id].rotation() * data.a[joint_id].angular();
      
      switch(contact_info.reference_frame)
      {
        case WORLD:
        {
          data.ov[joint_id] = oMi.act(data.v[joint_id]);
          data.oa[joint_id] = oMi.act(data.a[joint_id]);
          
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
          coriolis_centrifugal_acc.linear() = oMcontact.rotation() * coriolis_centrifugal_acc_local;
          // ANGULAR
          coriolis_centrifugal_acc.angular().noalias() = oMi.rotation() * data.a[joint_id].angular();
          
          break;
        }
        case LOCAL:
        {
          // LINEAR
          coriolis_centrifugal_acc.linear() = coriolis_centrifugal_acc_local;
          // ANGULAR
          coriolis_centrifugal_acc.angular().noalias() = iMcontact.rotation().transpose() * data.a[joint_id].angular();
          
          break;
        }
        default:
          assert(false && "must never happened");
          break;
      }

      switch(contact_info.type)
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
    
    // Retrieve the contact forces
    size_t current_id = 0;
    Eigen::DenseIndex current_row_sol_id = 0;
    for(typename RigidContactModelVector::const_iterator it = contact_models.begin();
        it != contact_models.end(); ++it, current_id++)
    {
      typename Data::Force & fext = data.contact_forces[current_id];
      const RigidContactModel & contact_info = *it;
      const int contact_dim = contact_info.size();
      
      switch(contact_info.type)
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
  
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType, typename TangentVectorType1, typename TangentVectorType2, class Allocator>
  inline const typename DataTpl<Scalar,Options,JointCollectionTpl>::TangentVectorType &
  fastContactDynamics(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                      DataTpl<Scalar,Options,JointCollectionTpl> & data,
                      const Eigen::MatrixBase<ConfigVectorType> & q,
                      const Eigen::MatrixBase<TangentVectorType1> & v,
                      const Eigen::MatrixBase<TangentVectorType2> & tau,
                      const std::vector<ContactInfoTpl<Scalar,Options>,Allocator> & contact_infos)
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
    typedef typename Model::JointIndex JointIndex;
    typedef ContactInfoTpl<Scalar,Options> ContactInfo;
    typedef std::vector<ContactInfo,Allocator> ContactInfoVector;
    
    data.v[0].setZero();
    data.a[0] = -model.gravity;
    data.u = tau;
    
    typedef AbaForwardStep1<Scalar,Options,JointCollectionTpl,ConfigVectorType,TangentVectorType1> Pass1;
    for(JointIndex i=1; i<(JointIndex)model.njoints; ++i)
    {
      Pass1::run(model.joints[i],data.joints[i],
                 typename Pass1::ArgsType(model,data,q.derived(),v.derived()));
    }
    
    typename Data::SE3 iMc; // tmp variable
    for(typename ContactInfoVector::const_iterator it = contact_infos.begin();
        it != contact_infos.end(); ++it)
    {
      const ContactInfo & cinfo = *it;
      const typename ContactInfo::FrameIndex & frame_id = cinfo.frame_id;
      const typename Model::Frame & frame = model.frames[frame_id];
      const typename Model::JointIndex & joint_id = frame.parent;

      // Compute relative placement between the joint and the contact frame
      iMc = frame.placement * cinfo.placement;
      typename Data::Motion & ai = data.a[joint_id];
      typename Data::Motion & vi = data.v[joint_id];
      if(cinfo.type == CONTACT_6D or cinfo.type == CONTACT_3D)
      {
        ai.linear() += vi.angular().cross(vi.linear());
      }
    }
    
    typedef AbaBackwardStep<Scalar,Options,JointCollectionTpl> Pass2;
    for(JointIndex i=(JointIndex)model.njoints-1;i>0; --i)
    {
      Pass2::run(model.joints[i],data.joints[i],
                 typename Pass2::ArgsType(model,data));
    }
    
    typedef AbaForwardStep2<Scalar,Options,JointCollectionTpl> Pass3;
    for(JointIndex i=1; i<(JointIndex)model.njoints; ++i)
    {
      Pass3::run(model.joints[i],data.joints[i],
                 typename Pass3::ArgsType(model,data));
    }
    
    return data.ddq;
  }
  
} // namespace pinocchio

#endif // ifndef __pinocchio_algorithm_contact_dynamics_hxx__
