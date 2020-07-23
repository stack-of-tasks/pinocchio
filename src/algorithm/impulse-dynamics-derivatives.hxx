//
// Copyright (c) 2020 CNRS INRIA
//

#ifndef __pinocchio_algorithm_impulse_dynamics_derivatives_hxx__
#define __pinocchio_algorithm_impulse_dynamics_derivatives_hxx__

#include "pinocchio/algorithm/check.hpp"
#include "pinocchio/algorithm/rnea-derivatives.hpp"
#include "pinocchio/algorithm/kinematics-derivatives.hpp"
#include "pinocchio/algorithm/contact-cholesky.hpp"
#include "pinocchio/algorithm/contact-dynamics-derivatives.hxx"

namespace pinocchio
{
  
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, class ContactModelAllocator, class ContactDataAllocator, typename MatrixType1, typename MatrixType2, typename MatrixType3, typename MatrixType4>
  inline void computeImpulseDynamicsDerivatives(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                                                DataTpl<Scalar,Options,JointCollectionTpl> & data,
                                                const std::vector<RigidContactModelTpl<Scalar,Options>,ContactModelAllocator> & contact_models,
                                                std::vector<RigidContactDataTpl<Scalar,Options>,ContactDataAllocator> & contact_data,
                                                const Scalar r_coeff,
                                                const Scalar mu,
                                                const Eigen::MatrixBase<MatrixType1> & dqafter_partial_dq,
                                                const Eigen::MatrixBase<MatrixType2> & dqafter_partial_dv,
                                                const Eigen::MatrixBase<MatrixType3> & impulse_partial_dq,
                                                const Eigen::MatrixBase<MatrixType4> & impulse_partial_dv)
  {
    const Eigen::DenseIndex & nc = data.contact_chol.constraintDim();
    
    PINOCCHIO_CHECK_INPUT_ARGUMENT(contact_data.size() == contact_models.size(),
                                   "contact_data and contact_models do not have the same size");
    
    PINOCCHIO_CHECK_INPUT_ARGUMENT(dqafter_partial_dq.cols() == model.nv);
    PINOCCHIO_CHECK_INPUT_ARGUMENT(dqafter_partial_dq.rows() == model.nv);
    
    PINOCCHIO_CHECK_INPUT_ARGUMENT(dqafter_partial_dv.cols() == model.nv);
    PINOCCHIO_CHECK_INPUT_ARGUMENT(dqafter_partial_dv.rows() == model.nv);

    PINOCCHIO_CHECK_INPUT_ARGUMENT(impulse_partial_dq.cols() == model.nv);
    PINOCCHIO_CHECK_INPUT_ARGUMENT(impulse_partial_dq.rows() == nc);
    PINOCCHIO_CHECK_INPUT_ARGUMENT((r_coeff >= Scalar(0)) && (r_coeff <= Scalar(1)) && "mu must be positive.");
    PINOCCHIO_CHECK_INPUT_ARGUMENT(mu >= (Scalar)0 && "mu must be positive.");
    PINOCCHIO_CHECK_INPUT_ARGUMENT(model.gravity.angular().isZero(),
                                   "The gravity must be a pure force vector, no angular part");
    
    assert(model.check(data) && "data is not consistent with model.");
    data.dtau_dq.setZero();
    
    typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
    typedef DataTpl<Scalar,Options,JointCollectionTpl> Data;
    
    typedef RigidContactModelTpl<Scalar,Options> RigidContactModel;
    typedef RigidContactDataTpl<Scalar,Options> RigidContactData;
    
    typedef std::vector<RigidContactModel,ContactModelAllocator> RigidContactModelVector;
    typedef std::vector<RigidContactData,ContactDataAllocator> RigidContactDataVector;
    typedef typename ModelTpl<Scalar,Options,JointCollectionTpl>::JointIndex JointIndex;
    
    typedef ComputeContactDynamicsDerivativesForwardStep<Scalar,Options,JointCollectionTpl,false> Pass1;
    for(JointIndex i=1; i<(JointIndex) model.njoints; ++i)
    {
      Pass1::run(model.joints[i],data.joints[i],
                 typename Pass1::ArgsType(model,data));
      //data.of[i] -= data.oMi[i].act(data.contact_forces[i]);
    }
    
    // Add the contribution of the external forces. TODO: this should be done at the contact dynamics level.
    for(size_t k = 0; k < contact_models.size(); ++k)
    {
      const RigidContactModel & cmodel = contact_models[k];
      const RigidContactData & cdata = contact_data[k];
      const JointIndex & parent = model.frames[cmodel.frame_id].parent;
      assert(   cmodel.reference_frame == LOCAL
             && "The contact is not expressed in the expected frame");

      //TODO: add LOCAL, WORLD, LOCAL_WORLD_ALIGNED frames.
      if(cmodel.reference_frame == LOCAL){
        data.of[model.frames[cmodel.frame_id].parent] -= data.oMf[cmodel.frame_id].act(cdata.contact_force);
      }
      else if(cmodel.reference_frame == WORLD){
        data.of[model.frames[cmodel.frame_id].parent] -= cdata.contact_force;
      }
      else {
        std::cerr<<"BAD CALL> ABORT ABORT";
      }
    }

    typedef ComputeContactDynamicDerivativesBackwardStep<Scalar,Options,JointCollectionTpl,false> Pass2;
    for(JointIndex i=(JointIndex)(model.njoints-1); i>0; --i)
    {
      Pass2::run(model.joints[i],
                 typename Pass2::ArgsType(model,data));
    }

    data.contact_chol.getOperationalSpaceInertiaMatrix(data.osim);
    data.contact_chol.getInverseMassMatrix(data.Minv);
    //Temporary: dlambda_dv stores J*Minv
    typename Data::MatrixXs & JMinv = data.dlambda_dv;

    JMinv.noalias() = Jc * data.Minv;

    data.dv_dq.noalias() = - JMinv * data.dtau_dq;

    //TODO: implement the kinematic deriv
    data.dv_dq.noalias() += dJdq*((1+r_coeff)*v+dv);

    //TODO: Implem sparse.
    data.dtau_dq.noalias() -= Jc.transpose()*impulse_partial_dq;
    
    impulse_partial_dq.noalias() = -osim * data.dv_dq;    
    impulse_partial_dv.noalias() = -(1+r_coeff)*osim*Jc;
    dvimpulse_partial_dq.noalias() = -data.Minv*data.dtau_dq;
    dvimpulse_partial_dv.noalias() = JMinv.transpose()*impulse_partial_dv;

  }  
} // namespace pinocchio

#endif // ifndef __pinocchio_algorithm_contact_dynamics_derivatives_hxx__
