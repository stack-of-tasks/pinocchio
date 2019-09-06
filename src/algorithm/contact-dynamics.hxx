//
// Copyright (c) 2019 INRIA
//

#ifndef __pinocchio_contact_dynamics_hxx__
#define __pinocchio_contact_dynamics_hxx__

#include "pinocchio/algorithm/compute-all-terms.hpp"
#include "pinocchio/algorithm/cholesky.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/check.hpp"

namespace pinocchio
{

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
  
} // namespace pinocchio

#endif // ifndef __pinocchio_contact_dynamics_hxx__
