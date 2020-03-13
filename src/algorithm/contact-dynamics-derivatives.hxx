//
// Copyright (c) 2020 LAAS, INRIA
//

#ifndef __pinocchio_algorithm_contact_dynamics_derivatives_hxx__
#define __pinocchio_algorithm_contact_dynamics_derivatives_hxx__

#include "pinocchio/algorithm/check.hpp"
#include "pinocchio/algorithm/aba-derivatives.hpp"
#include "pinocchio/algorithm/kinematics-derivatives.hpp"
#include "pinocchio/algorithm/contact-cholesky.hpp"

namespace pinocchio
{

  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename ConfigVectorType, typename TangentVectorType1, typename TangentVectorType2, class Allocator>//, typename MatrixType1, typename MatrixType2, typename MatrixType3, typename MatrixType4, typename MatrixType5, typename MatrixType6>
  inline void computeContactDynamicsDerivatives(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                                                DataTpl<Scalar,Options,JointCollectionTpl> & data,
                                                const Eigen::MatrixBase<ConfigVectorType> & q,
                                                const Eigen::MatrixBase<TangentVectorType1> & v,
                                                const Eigen::MatrixBase<TangentVectorType2> & tau,
                                                const std::vector<RigidContactModelTpl<Scalar,Options>,Allocator> & contact_infos,
                                                const Scalar mu/*,
                                                                      const Eigen::MatrixBase<MatrixType1> & ddq_partial_dq,
                                                const Eigen::MatrixBase<MatrixType2> & ddq_partial_dv,
                                                const Eigen::MatrixBase<MatrixType3> & ddq_partial_dtau,
                                                const Eigen::MatrixBase<MatrixType4> & lambda_partial_dq,
                                                const Eigen::MatrixBase<MatrixType5> & lambda_partial_dv,
                                                const Eigen::MatrixBase<MatrixType6> & lambda_partial_dtau*/)
  {
    const Eigen::DenseIndex& nc = data.contact_chol.constraintDim();
    
    PINOCCHIO_CHECK_INPUT_ARGUMENT(q.size() == model.nq, "The joint configuration vector is not of right size");
    PINOCCHIO_CHECK_INPUT_ARGUMENT(v.size() == model.nv, "The joint velocity vector is not of right size");
    PINOCCHIO_CHECK_INPUT_ARGUMENT(tau.size() == model.nv, "The joint torque vector is not of right size");
    /*    PINOCCHIO_CHECK_INPUT_ARGUMENT(ddq_partial_dq.cols() == model.nv);
    PINOCCHIO_CHECK_INPUT_ARGUMENT(ddq_partial_dq.rows() == model.nv);
    PINOCCHIO_CHECK_INPUT_ARGUMENT(ddq_partial_dv.cols() == model.nv);
    PINOCCHIO_CHECK_INPUT_ARGUMENT(aba_partial_dv.rows() == model.nv);
    PINOCCHIO_CHECK_INPUT_ARGUMENT(ddq_partial_dtau.cols() == model.nv);
    PINOCCHIO_CHECK_INPUT_ARGUMENT(ddq_partial_dtau.rows() == model.nv);
    PINOCCHIO_CHECK_INPUT_ARGUMENT(lambda_partial_dq.cols() == model.nv);
    PINOCCHIO_CHECK_INPUT_ARGUMENT(lambda_partial_dq.rows() == nc);
    PINOCCHIO_CHECK_INPUT_ARGUMENT(lambda_partial_dv.cols() == model.nv);
    PINOCCHIO_CHECK_INPUT_ARGUMENT(lambda_partial_dv.rows() == nc);
    PINOCCHIO_CHECK_INPUT_ARGUMENT(lambda_partial_dtau.cols() == model.nv);
    PINOCCHIO_CHECK_INPUT_ARGUMENT(lambda_partial_dtau.rows() == nc);*/
    assert(model.check(data) && "data is not consistent with model.");
    assert(mu >= (Scalar)0 && "mu must be positive.");
    
    typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
    typedef DataTpl<Scalar,Options,JointCollectionTpl> Data;
    
    typedef RigidContactModelTpl<Scalar,Options> RigidContactModel;
    typedef std::vector<RigidContactModel,Allocator> RigidContactModelVector;
    typedef typename ModelTpl<Scalar,Options,JointCollectionTpl>::JointIndex JointIndex;

    data.contact_chol.getOperationalSpaceInertiaMatrix(data.osim);

    computeABADerivatives(model, data, q, v, tau, data.contact_forces);

    size_t current_id = 0;
    Eigen::DenseIndex current_row_sol_id = 0;
    
    for(typename RigidContactModelVector::const_iterator it = contact_infos.begin();
        it != contact_infos.end(); ++it, current_id++)
    {
      const RigidContactModel & contact_info = *it;

      //TODO: This is only for size 6. replace with contact_info::NC
      typedef typename SizeDepType<6>::template RowsReturn<typename Data::MatrixXs>::Type RowsBlock;
      const typename Model::FrameIndex & frame_id = contact_info.frame_id;
      const typename Model::Frame & frame = model.frames[frame_id];
      const typename Model::JointIndex & joint_id = frame.parent;

      //TODO: We don't need all these quantities. Remove those not needed.
      getJointAccelerationDerivatives(model, data,
                                      joint_id,
                                      LOCAL,
                                      data.v_partial_dq[current_id],
                                      data.a_partial_dq[current_id],
                                      data.a_partial_dv[current_id],
                                      data.a_partial_da[current_id]);

      //TODO: replace with contact_info::nc
      RowsBlock contact_dac_dq = SizeDepType<6>::middleRows(data.dac_dq, current_row_sol_id);

      //TODO: Sparse
      contact_dac_dq.noalias() = data.a_partial_da[current_id] * data.ddq_dq;
      contact_dac_dq += data.a_partial_dq[current_id];

      //TODO: remplacer par contact_info::NC
      current_row_sol_id += 6;
    }

    data.dlambda_dq.noalias() = -data.osim * data.dac_dq;

    current_id = 0;
    current_row_sol_id = 0;
    for(typename RigidContactModelVector::const_iterator it = contact_infos.begin();
        it != contact_infos.end(); ++it, current_id++)
    {
      const RigidContactModel & contact_info = *it;

      //TODO: This is only for size 6. replace with contact_info::NC
      typedef typename SizeDepType<6>::template RowsReturn<typename Data::MatrixXs>::Type RowsBlock;
      
      RowsBlock contact_dlambdac_dq = SizeDepType<6>::middleRows(data.dlambda_dq,
                                                                 current_row_sol_id);
      //TODO: Sparse
      data.dtau_dq.noalias() -= data.a_partial_da[current_id].transpose() * contact_dlambdac_dq;
      //TODO: remplacer par contact_info::NC
      current_row_sol_id += 6;
    }
    data.ddq_dq.noalias() = -data.Minv*data.dtau_dq;
    //PINOCCHIO_EIGEN_CONST_CAST(MatrixType1,data.ddq_dq).noalias() = data.Minv*data.dtau_dq;
    
  }

  
} // namespace pinocchio

#endif // ifndef __pinocchio_algorithm_contact_dynamics_hxx__
