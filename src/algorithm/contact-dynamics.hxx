//
// Copyright (c) 2016-2019 CNRS INRIA
//

#ifndef __pinocchio_contact_dynamics_hxx__
#define __pinocchio_contact_dynamics_hxx__

#include "pinocchio/algorithm/compute-all-terms.hpp"
#include "pinocchio/algorithm/cholesky.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/check.hpp"

namespace pinocchio
{

  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename TangentVectorType,
  typename ConstraintMatrixType, typename DriftVectorType>
  inline const typename DataTpl<Scalar,Options,JointCollectionTpl>::TangentVectorType &
  forwardDynamics(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                  DataTpl<Scalar,Options,JointCollectionTpl> & data,
                  const Eigen::MatrixBase<TangentVectorType> & tau,
                  const Eigen::MatrixBase<ConstraintMatrixType> & J,
                  const Eigen::MatrixBase<DriftVectorType> & gamma,
                  const ProximalSettingsTpl<Scalar> & prox_settings)
  {
    PINOCCHIO_CHECK_INPUT_ARGUMENT(tau.size() == model.nv);
    PINOCCHIO_CHECK_INPUT_ARGUMENT(J.cols() == model.nv);
    PINOCCHIO_CHECK_INPUT_ARGUMENT(J.rows() == gamma.size());
    assert(model.check(data) && "data is not consistent with model.");
    
    typedef DataTpl<Scalar,Options,JointCollectionTpl> Data;
    
    typename Data::TangentVectorType & a = data.ddq;
    typename Data::VectorXs & lambda_c = data.lambda_c;
    typename Data::VectorXs & lambda_c_prox = data.lambda_c_prox;
    
    // Compute the UDUt decomposition of data.M
    cholesky::decompose(model, data);
    
    // Compute the dynamic drift (control - nle)
    data.torque_residual = tau - data.nle;
    cholesky::solve(model, data, data.torque_residual);
    
    data.sDUiJt = J.transpose();
    // Compute U^-1 * J.T
    cholesky::Uiv(model, data, data.sDUiJt);
    for(Eigen::DenseIndex k=0;k<model.nv;++k)
      data.sDUiJt.row(k) /= sqrt(data.D[k]);
    
    data.JMinvJt.noalias() = data.sDUiJt.transpose() * data.sDUiJt;

    const Scalar & mu = prox_settings.mu;
    assert(mu >= 0. && "mu must be positive");
    int max_it = prox_settings.max_it;
    assert(max_it >= 1 && "mu must greater or equal to 1");

    if(mu == 0.)
      max_it = 1;
    else
      data.JMinvJt.diagonal().array() += mu;

    data.llt_JMinvJt.compute(data.JMinvJt);
    
    lambda_c_prox = Data::VectorXs::Zero(gamma.size());
    for(int it = 0; it < max_it; ++it)
    {
      // Compute the Lagrange Multipliers
      lambda_c.noalias() = -J*data.torque_residual;
      lambda_c += -gamma + mu*lambda_c_prox;
      data.llt_JMinvJt.solveInPlace(lambda_c);
      
      data.diff_lambda_c = lambda_c - lambda_c_prox;
      lambda_c_prox = lambda_c;
      
      // Check termination
      if(max_it > 1)
//        std::cout << "data.diff_lambda_c.template lpNorm<Eigen::Infinity>():\n" << data.diff_lambda_c.template lpNorm<Eigen::Infinity>() << std::endl;
      if(data.diff_lambda_c.template lpNorm<Eigen::Infinity>() <= prox_settings.threshold)
        break;
    }
    
    // Compute the joint acceleration
    a.noalias() = J.transpose() * lambda_c;
    cholesky::solve(model, data, a);
    a += data.torque_residual;

    return a;
  }
  
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename TangentVectorType,
  typename ConstraintMatrixType, typename DriftVectorType>
  inline const typename DataTpl<Scalar,Options,JointCollectionTpl>::TangentVectorType &
  forwardDynamics(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                  DataTpl<Scalar,Options,JointCollectionTpl> & data,
                  const Eigen::MatrixBase<TangentVectorType> & tau,
                  const Eigen::MatrixBase<ConstraintMatrixType> & J,
                  const Eigen::MatrixBase<DriftVectorType> & gamma,
                  const Scalar mu)
  {
    ProximalSettingsTpl<Scalar> prox_settings;
    prox_settings.mu = mu;
    return forwardDynamics(model,data,tau,J,gamma,prox_settings);
  }
  
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType, typename TangentVectorType1, typename TangentVectorType2,
  typename ConstraintMatrixType, typename DriftVectorType>
  inline const typename DataTpl<Scalar,Options,JointCollectionTpl>::TangentVectorType &
  forwardDynamics(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                  DataTpl<Scalar,Options,JointCollectionTpl> & data,
                  const Eigen::MatrixBase<ConfigVectorType> & q,
                  const Eigen::MatrixBase<TangentVectorType1> & v,
                  const Eigen::MatrixBase<TangentVectorType2> & tau,
                  const Eigen::MatrixBase<ConstraintMatrixType> & J,
                  const Eigen::MatrixBase<DriftVectorType> & gamma,
                  const ProximalSettingsTpl<Scalar> & prox_settings)
  {
    computeAllTerms(model, data, q, v);
    return forwardDynamics(model,data,tau,J,gamma,prox_settings);
  }
  
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType, typename TangentVectorType1, typename TangentVectorType2,
  typename ConstraintMatrixType, typename DriftVectorType>
  inline const typename DataTpl<Scalar,Options,JointCollectionTpl>::TangentVectorType &
  forwardDynamics(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                  DataTpl<Scalar,Options,JointCollectionTpl> & data,
                  const Eigen::MatrixBase<ConfigVectorType> & q,
                  const Eigen::MatrixBase<TangentVectorType1> & v,
                  const Eigen::MatrixBase<TangentVectorType2> & tau,
                  const Eigen::MatrixBase<ConstraintMatrixType> & J,
                  const Eigen::MatrixBase<DriftVectorType> & gamma,
                  const Scalar mu)
  {
    PINOCCHIO_CHECK_INPUT_ARGUMENT(q.size() == model.nq);
    PINOCCHIO_CHECK_INPUT_ARGUMENT(v.size() == model.nv);

    computeAllTerms(model, data, q, v);
    return forwardDynamics(model,data,tau,J,gamma,mu);
  }
  
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType, typename TangentVectorType1, typename TangentVectorType2,
  typename ConstraintMatrixType, typename DriftVectorType>
  PINOCCHIO_DEPRECATED
  inline const typename DataTpl<Scalar,Options,JointCollectionTpl>::TangentVectorType &
  forwardDynamics(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                  DataTpl<Scalar,Options,JointCollectionTpl> & data,
                  const Eigen::MatrixBase<ConfigVectorType> & q,
                  const Eigen::MatrixBase<TangentVectorType1> & v,
                  const Eigen::MatrixBase<TangentVectorType2> & tau,
                  const Eigen::MatrixBase<ConstraintMatrixType> & J,
                  const Eigen::MatrixBase<DriftVectorType> & gamma,
                  const Scalar mu,
                  const bool updateKinematics)
  {
    if(updateKinematics)
      return forwardDynamics(model,data,q,v,tau,J,gamma,mu);
    else
      return forwardDynamics(model,data,tau,J,gamma,mu);
  }
  
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, class Allocator>
  inline void initContactDynamics(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                                  DataTpl<Scalar,Options,JointCollectionTpl> & data,
                                  const std::vector<ContactInfoTpl<Scalar,Options>,Allocator> & contact_infos)
  {
    data.contact_chol.allocate(model,contact_infos);
    data.contact_vector_solution.resize(data.contact_chol.dim());
    data.contact_forces.resize(contact_infos.size());
  }
  
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType, typename TangentVectorType1, typename TangentVectorType2, class Allocator>
  inline const typename DataTpl<Scalar,Options,JointCollectionTpl>::TangentVectorType &
  contactDynamics(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                  DataTpl<Scalar,Options,JointCollectionTpl> & data,
                  const Eigen::MatrixBase<ConfigVectorType> & q,
                  const Eigen::MatrixBase<TangentVectorType1> & v,
                  const Eigen::MatrixBase<TangentVectorType2> & tau,
                  const std::vector<ContactInfoTpl<Scalar,Options>,Allocator> & contact_infos,
                  const Scalar mu)
  {
    assert(q.size() == model.nq);
    assert(v.size() == model.nv);
    assert(tau.size() == model.nv);
    assert(model.check(data) && "data is not consistent with model.");
    
    assert(mu >= (Scalar)0 && "mu must be positive.");
    
    typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
    typedef DataTpl<Scalar,Options,JointCollectionTpl> Data;
    
    typedef ContactInfoTpl<Scalar,Options> ContactInfo;
    typedef std::vector<ContactInfo,Allocator> ContactInfoVector;
    
    typename Data::TangentVectorType & a = data.ddq;
    typename Data::ContactCholeskyDecomposition & contact_chol = data.contact_chol;
    typename Data::VectorXs & contact_vector_solution = data.contact_vector_solution;
    
    computeAllTerms(model,data,q.derived(),v.derived());

    // Computes the Cholesky decomposition
    contact_chol.compute(model,data,contact_infos,mu);

    contact_vector_solution.tail(model.nv) = tau - data.nle;

    // Temporary variables
    typename Data::SE3 oMcontact; // placement of the contact frame in the world frame
    typename Data::SE3 iMcontact; // placement of the contact frame in the local frame of the joint
    typename Data::Motion coriolis_centrifugal_acc; // Coriolis/centrifugal acceleration of the contact frame.
    typename Motion::Vector3 coriolis_centrifugal_acc_local;

    Eigen::DenseIndex current_row_id = 0;
    for(typename ContactInfoVector::const_iterator it = contact_infos.begin();
        it != contact_infos.end(); ++it)
    {
      const ContactInfo & contact_info = *it;
      const int contact_dim = contact_info.dim();

      const typename Model::FrameIndex & frame_id = contact_info.frame_id;
      const typename Model::Frame & frame = model.frames[frame_id];
      const typename Model::JointIndex & joint_id = frame.parent;
      const typename Data::SE3 & oMi = data.oMi[joint_id];

      // Update frame placement
      iMcontact = frame.placement * contact_info.placement;
//      data.oMf[frame_id] = data.oMi[frame.parent] * frame.placement;
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
    
//    std::cout << "contact_vector_solution:\n" << contact_vector_solution.head(12).transpose() << std::endl;
    // Solve the system
    contact_chol.solveInPlace(contact_vector_solution);
    
    // Retrieve the joint space acceleration
    a = contact_vector_solution.tail(model.nv);
    
    // Retrieve the contact forces
    size_t current_id = 0;
    Eigen::DenseIndex current_row_sol_id = 0;
    for(typename ContactInfoVector::const_iterator it = contact_infos.begin();
        it != contact_infos.end(); ++it, current_id++)
    {
      typename Data::Force & fext = data.contact_forces[current_id];
      const ContactInfo & contact_info = *it;
      const int contact_dim = contact_info.dim();
      
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
  
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl,
  typename ConstraintMatrixType, typename KKTMatrixType>
  inline void getKKTContactDynamicMatrixInverse(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                                                const DataTpl<Scalar,Options,JointCollectionTpl> & data,
                                                const Eigen::MatrixBase<ConstraintMatrixType> & J,
                                                const Eigen::MatrixBase<KKTMatrixType> & MJtJ_inv)
  {
    typedef DataTpl<Scalar,Options,JointCollectionTpl> Data;
    PINOCCHIO_CHECK_INPUT_ARGUMENT(MJtJ_inv.cols() == data.JMinvJt.cols() + model.nv);
    PINOCCHIO_CHECK_INPUT_ARGUMENT(MJtJ_inv.rows() == data.JMinvJt.rows() + model.nv);
    const typename Data::MatrixXs::Index& nc = data.JMinvJt.cols();
    
    KKTMatrixType& MJtJ_inv_ = PINOCCHIO_EIGEN_CONST_CAST(KKTMatrixType,MJtJ_inv);
    
    Eigen::Block<typename Data::MatrixXs> topLeft = MJtJ_inv_.topLeftCorner(model.nv, model.nv);
    Eigen::Block<typename Data::MatrixXs> topRight = MJtJ_inv_.topRightCorner(model.nv, nc);
    Eigen::Block<typename Data::MatrixXs> bottomLeft = MJtJ_inv_.bottomLeftCorner(nc, model.nv);
    Eigen::Block<typename Data::MatrixXs> bottomRight = MJtJ_inv_.bottomRightCorner(nc, nc);
    
    bottomRight = -Data::MatrixXs::Identity(nc,nc);    topLeft.setIdentity();
    data.llt_JMinvJt.solveInPlace(bottomRight);    cholesky::solve(model, data, topLeft);
    
    bottomLeft.noalias() = J*topLeft;
    topRight.noalias() = bottomLeft.transpose() * (-bottomRight);
    topLeft.noalias() -= topRight*bottomLeft;
    bottomLeft = topRight.transpose();
  }

  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType, typename TangentVectorType, typename ConstraintMatrixType>
  inline const typename DataTpl<Scalar,Options,JointCollectionTpl>::TangentVectorType &
  impulseDynamics(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                  DataTpl<Scalar,Options,JointCollectionTpl> & data,
                  const Eigen::MatrixBase<ConfigVectorType> & q,
                  const Eigen::MatrixBase<TangentVectorType> & v_before,
                  const Eigen::MatrixBase<ConstraintMatrixType> & J,
                  const Scalar r_coeff,
                  const Scalar inv_damping)
  {
    PINOCCHIO_CHECK_INPUT_ARGUMENT(q.size() == model.nq);
    
    // Compute the mass matrix
    crba(model, data, q);
    
    return impulseDynamics(model,data,v_before,J,r_coeff,inv_damping);
  }

  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename TangentVectorType, typename ConstraintMatrixType>
  inline const typename DataTpl<Scalar,Options,JointCollectionTpl>::TangentVectorType &
  impulseDynamics(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                  DataTpl<Scalar,Options,JointCollectionTpl> & data,
                  const Eigen::MatrixBase<TangentVectorType> & v_before,
                  const Eigen::MatrixBase<ConstraintMatrixType> & J,
                  const Scalar r_coeff,
                  const Scalar inv_damping)
  {
    PINOCCHIO_CHECK_INPUT_ARGUMENT(v_before.size() == model.nv);
    PINOCCHIO_CHECK_INPUT_ARGUMENT(J.cols() == model.nv);
    assert(model.check(data) && "data is not consistent with model.");
    
    typedef DataTpl<Scalar,Options,JointCollectionTpl> Data;
    
    typename Data::VectorXs & impulse_c = data.impulse_c;
    typename Data::TangentVectorType & dq_after = data.dq_after;
    
    // Compute the UDUt decomposition of data.M
    cholesky::decompose(model, data);
    
    data.sDUiJt = J.transpose();
    // Compute U^-1 * J.T
    cholesky::Uiv(model, data, data.sDUiJt);
    for(int k=0;k<model.nv;++k) data.sDUiJt.row(k) /= sqrt(data.D[k]);
    
    data.JMinvJt.noalias() = data.sDUiJt.transpose() * data.sDUiJt;
    
    data.JMinvJt.diagonal().array() += inv_damping;
    data.llt_JMinvJt.compute(data.JMinvJt);
    
    // Compute the Lagrange Multipliers related to the contact impulses
    impulse_c.noalias() = (-r_coeff - 1.) * (J * v_before);
    data.llt_JMinvJt.solveInPlace(impulse_c);
    
    // Compute the joint velocity after impacts
    dq_after.noalias() = J.transpose() * impulse_c;
    cholesky::solve(model, data, dq_after);
    dq_after += v_before;
    
    return dq_after;
  }
} // namespace pinocchio

#endif // ifndef __pinocchio_contact_dynamics_hxx__
