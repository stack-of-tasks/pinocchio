//
// Copyright (c) 2020 LAAS, INRIA
//

#ifndef __pinocchio_algorithm_contact_dynamics_derivatives_hxx__
#define __pinocchio_algorithm_contact_dynamics_derivatives_hxx__

#include "pinocchio/algorithm/check.hpp"
#include "pinocchio/algorithm/rnea-derivatives.hpp"
#include "pinocchio/algorithm/kinematics-derivatives.hpp"
#include "pinocchio/algorithm/contact-cholesky.hpp"

namespace pinocchio
{


  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  struct ComputeContactDynamicsDerivativesForwardStep
  : public fusion::JointUnaryVisitorBase< ComputeContactDynamicsDerivativesForwardStep<Scalar,Options,JointCollectionTpl> >
  {
    typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
    typedef DataTpl<Scalar,Options,JointCollectionTpl> Data;
    
    typedef boost::fusion::vector<const Model &,
                                  Data &
                                  > ArgsType;
    
    template<typename JointModel>
    static void algo(const JointModelBase<JointModel> & jmodel,
                     JointDataBase<typename JointModel::JointDataDerived> & jdata,
                     const Model & model,
                     Data & data)
    {
      typedef typename Model::JointIndex JointIndex;
      typedef typename Data::Motion Motion;

      const JointIndex & i = jmodel.id();
      const JointIndex & parent = model.parents[i];
      Motion & ov = data.ov[i];
      Motion & oa = data.oa[i];
      Motion & oa_gf = data.oa_gf[i];
      const typename Data::TangentVectorType& a = data.ddq;
      data.a[i] = jdata.S() * jmodel.jointVelocitySelector(a) + jdata.c() + (data.v[i] ^ jdata.v());
      if(parent > 0)
      {
        data.a[i] += data.liMi[i].actInv(data.a[parent]);
      }

      oa = data.oMi[i].act(data.a[i]);
      oa_gf = oa - model.gravity; // add gravity contribution
      
      data.of[i] = data.oinertias[i] * oa_gf + ov.cross(data.oh[i]);
      
      typedef typename SizeDepType<JointModel::NV>::template ColsReturn<typename Data::Matrix6x>::Type ColsBlock;
      ColsBlock J_cols = jmodel.jointCols(data.J);
      ColsBlock dJ_cols = jmodel.jointCols(data.dJ);
      ColsBlock dVdq_cols = jmodel.jointCols(data.dVdq);
      ColsBlock dAdq_cols = jmodel.jointCols(data.dAdq);
      ColsBlock dAdv_cols = jmodel.jointCols(data.dAdv);

      motionSet::motionAction(ov,J_cols,dJ_cols);
      motionSet::motionAction(data.oa_gf[parent],J_cols,dAdq_cols);
      dAdv_cols = dJ_cols;
      if(parent > 0)
      {
        motionSet::motionAction(data.ov[parent],J_cols,dVdq_cols);
        motionSet::motionAction<ADDTO>(data.ov[parent],dVdq_cols,dAdq_cols);
        dAdv_cols.noalias() += dVdq_cols;
      }
      else
      {
        dVdq_cols.setZero();
      }

      // computes variation of inertias
      data.doYcrb[i] = data.oinertias[i].variation(ov);
      typedef ComputeRNEADerivativesForwardStep<Scalar,Options,JointCollectionTpl,typename Data::ConfigVectorType,typename Data::TangentVectorType,typename Data::TangentVectorType> RNEAForwardStepType;
      RNEAForwardStepType::addForceCrossMatrix(data.oh[i],data.doYcrb[i]);
    }
  };

  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
  struct ComputeContactDynamicDerivativesBackwardStep
  : public fusion::JointUnaryVisitorBase<ComputeContactDynamicDerivativesBackwardStep<Scalar,Options,JointCollectionTpl> >
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
      typedef Eigen::Matrix<Scalar,JointModel::NV,6,Options,JointModel::NV==Eigen::Dynamic?6:JointModel::NV,6> MatrixNV6;
      
      const JointIndex & i = jmodel.id();
      const JointIndex & parent = model.parents[i];
      
      // Temporary variables
      typename PINOCCHIO_EIGEN_PLAIN_ROW_MAJOR_TYPE(MatrixNV6) YS (jmodel.nv(),6);
      typename PINOCCHIO_EIGEN_PLAIN_ROW_MAJOR_TYPE(MatrixNV6) StY(jmodel.nv(),6);

      typedef typename SizeDepType<JointModel::NV>::template ColsReturn<typename Data::Matrix6x>::Type ColsBlock;
      
      ColsBlock J_cols = jmodel.jointCols(data.J);
      ColsBlock dVdq_cols = jmodel.jointCols(data.dVdq);
      ColsBlock dAdq_cols = jmodel.jointCols(data.dAdq);
      ColsBlock dAdv_cols = jmodel.jointCols(data.dAdv);
      ColsBlock dFdq_cols = jmodel.jointCols(data.dFdq);
      ColsBlock dFdv_cols = jmodel.jointCols(data.dFdv);

      typename Data::RowMatrixXs& rnea_partial_dq_ = data.dtau_dq;//PINOCCHIO_EIGEN_CONST_CAST(MatrixType1,rnea_partial_dq);
      typename Data::RowMatrixXs& rnea_partial_dv_ = data.dtau_dv;//PINOCCHIO_EIGEN_CONST_CAST(MatrixType2,rnea_partial_dv);

      // dtau/dv
      dFdv_cols.noalias() = data.doYcrb[i] * J_cols;
      motionSet::inertiaAction<ADDTO>(data.oYcrb[i],dAdv_cols,dFdv_cols);

      rnea_partial_dv_.block(jmodel.idx_v(),jmodel.idx_v(),jmodel.nv(),data.nvSubtree[i]).noalias()
      = J_cols.transpose()*data.dFdv.middleCols(jmodel.idx_v(),data.nvSubtree[i]);
      
      // dtau/dq
      if(parent>0)
      {
        dFdq_cols.noalias() = data.doYcrb[i] * dVdq_cols;
        motionSet::inertiaAction<ADDTO>(data.oYcrb[i],dAdq_cols,dFdq_cols);
      }
      else
        motionSet::inertiaAction(data.oYcrb[i],dAdq_cols,dFdq_cols);

      rnea_partial_dq_.block(jmodel.idx_v(),jmodel.idx_v(),jmodel.nv(),data.nvSubtree[i]).noalias()
      = J_cols.transpose()*data.dFdq.middleCols(jmodel.idx_v(),data.nvSubtree[i]);
      
      motionSet::act<ADDTO>(J_cols,data.of[i],dFdq_cols);

      typedef ComputeRNEADerivativesBackwardStep<Scalar,Options,JointCollectionTpl,
                                                 typename Data::RowMatrixXs,
                                                 typename Data::RowMatrixXs,
                                                 typename Data::RowMatrixXs> RNEABackwardStep;
      if(parent > 0)
      {
        RNEABackwardStep::lhsInertiaMult(data.oYcrb[i],J_cols.transpose(),YS);
        StY.noalias() = J_cols.transpose() * data.doYcrb[i];
        for(int j = data.parents_fromRow[(typename Model::Index)jmodel.idx_v()];j >= 0; j = data.parents_fromRow[(typename Model::Index)j])
        {
          rnea_partial_dq_.middleRows(jmodel.idx_v(),jmodel.nv()).col(j).noalias()
          = YS  * data.dAdq.col(j)
          + StY * data.dVdq.col(j);
        }
        for(int j = data.parents_fromRow[(typename Model::Index)jmodel.idx_v()];j >= 0; j = data.parents_fromRow[(typename Model::Index)j])
        {
          rnea_partial_dv_.middleRows(jmodel.idx_v(),jmodel.nv()).col(j).noalias()
          = YS  * data.dAdv.col(j)
          + StY * data.J.col(j);
        }
      }

      if(parent>0)
      {
        data.doYcrb[parent] += data.doYcrb[i];
        data.of[parent] += data.of[i];
      }
      
      // Restore the status of dAdq_cols (remove gravity)
      PINOCCHIO_CHECK_INPUT_ARGUMENT(model.gravity.angular().isZero(), "The gravity must be a pure force vector, no angular part");
      for(Eigen::DenseIndex k =0; k < jmodel.nv(); ++k)
      {
        MotionRef<typename ColsBlock::ColXpr> min(J_cols.col(k));
        MotionRef<typename ColsBlock::ColXpr> mout(dAdq_cols.col(k));
        mout.linear() += model.gravity.linear().cross(min.angular());
      }
    }
  };

  
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, class Allocator,
           class AllocatorData, typename MatrixType1, typename MatrixType2, typename MatrixType3, typename MatrixType4,
           typename MatrixType5, typename MatrixType6>
  inline void computeContactDynamicsDerivatives(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                                                DataTpl<Scalar,Options,JointCollectionTpl> & data,
                                                const std::vector<RigidContactModelTpl<Scalar,Options>,Allocator> & contact_models,
                                                const std::vector<RigidContactDataTpl<Scalar,Options>, AllocatorData> & contact_datas,
                                                const Scalar mu,
                                                const Eigen::MatrixBase<MatrixType1> & ddq_partial_dq,
                                                const Eigen::MatrixBase<MatrixType2> & ddq_partial_dv,
                                                const Eigen::MatrixBase<MatrixType3> & ddq_partial_dtau,
                                                const Eigen::MatrixBase<MatrixType4> & lambda_partial_dq,
                                                const Eigen::MatrixBase<MatrixType5> & lambda_partial_dv,
                                                const Eigen::MatrixBase<MatrixType6> & lambda_partial_dtau)
  {
    const Eigen::DenseIndex& nc = data.contact_chol.constraintDim();
    
    PINOCCHIO_CHECK_INPUT_ARGUMENT(ddq_partial_dq.cols() == model.nv);
    PINOCCHIO_CHECK_INPUT_ARGUMENT(ddq_partial_dq.rows() == model.nv);
    PINOCCHIO_CHECK_INPUT_ARGUMENT(ddq_partial_dv.cols() == model.nv);
    PINOCCHIO_CHECK_INPUT_ARGUMENT(ddq_partial_dv.rows() == model.nv);
    PINOCCHIO_CHECK_INPUT_ARGUMENT(ddq_partial_dtau.cols() == model.nv);
    PINOCCHIO_CHECK_INPUT_ARGUMENT(ddq_partial_dtau.rows() == model.nv);
    PINOCCHIO_CHECK_INPUT_ARGUMENT(lambda_partial_dq.cols() == model.nv);
    PINOCCHIO_CHECK_INPUT_ARGUMENT(lambda_partial_dq.rows() == nc);
    PINOCCHIO_CHECK_INPUT_ARGUMENT(lambda_partial_dv.cols() == model.nv);
    PINOCCHIO_CHECK_INPUT_ARGUMENT(lambda_partial_dv.rows() == nc);
    PINOCCHIO_CHECK_INPUT_ARGUMENT(lambda_partial_dtau.cols() == model.nv);
    PINOCCHIO_CHECK_INPUT_ARGUMENT(lambda_partial_dtau.rows() == nc);
    assert(model.check(data) && "data is not consistent with model.");

    //TODO: mu is currently unused in implementation
    assert(mu >= (Scalar)0 && "mu must be positive.");
    
    typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
    typedef DataTpl<Scalar,Options,JointCollectionTpl> Data;
    
    typedef RigidContactModelTpl<Scalar,Options> RigidContactModel;
    typedef RigidContactDataTpl<Scalar,Options> RigidContactData;
    typedef std::vector<RigidContactModel,Allocator> RigidContactModelVector;
    typedef std::vector<RigidContactData,AllocatorData> RigidContactDataVector;
    typedef typename ModelTpl<Scalar,Options,JointCollectionTpl>::JointIndex JointIndex;

    data.oa_gf[0] = -model.gravity;
    
    typedef ComputeContactDynamicsDerivativesForwardStep<Scalar,Options,JointCollectionTpl> Pass1;
    for(JointIndex i=1; i<(JointIndex) model.njoints; ++i)
    {
      Pass1::run(model.joints[i],data.joints[i],
                 typename Pass1::ArgsType(model,data));
      data.of[i] -= data.oMi[i].act(data.contact_forces[i]);
    }

    typedef ComputeContactDynamicDerivativesBackwardStep<Scalar,Options,JointCollectionTpl> Pass2;
    for(JointIndex i=(JointIndex)(model.njoints-1); i>0; --i)
    {
      Pass2::run(model.joints[i],
                 typename Pass2::ArgsType(model,data));
    }    


    Eigen::DenseIndex current_row_sol_id = 0;
    typename RigidContactDataVector::const_iterator it_d = contact_datas.begin();
    for(typename RigidContactModelVector::const_iterator it = contact_models.begin();
        (it != contact_models.end() && it_d != contact_datas.end()); ++it, ++it_d)
    {
      const RigidContactModel & contact_model = *it;
      const RigidContactData & contact_data = *it_d;

      const typename Model::FrameIndex & frame_id = contact_model.frame_id;
      const typename Model::Frame & frame = model.frames[frame_id];
      const typename Model::JointIndex & joint_id = frame.parent;
      
      //TODO: This is only for size 6. replace with contact_model::NC
      switch(contact_model.type)
      {
      case CONTACT_6D:
      {
        typedef typename SizeDepType<6>::template RowsReturn<typename Data::MatrixXs>::Type RowsBlock;
        //TODO: We don't need all these quantities. Remove those not needed.
        typename Data::Matrix6x& contact_dvc_dq_tmp = data.dFda;
        RowsBlock contact_dac_dq = SizeDepType<6>::middleRows(data.dac_dq,
                                                              current_row_sol_id);
        RowsBlock contact_dac_dv = SizeDepType<6>::middleRows(data.dac_dv,
                                                              current_row_sol_id);
        RowsBlock contact_dac_da = SizeDepType<6>::middleRows(data.dac_da,
                                                              current_row_sol_id);

        getJointAccelerationDerivatives(model, data,
                                        joint_id,
                                        LOCAL,
                                        contact_dvc_dq_tmp,
                                        contact_dac_dq,
                                        contact_dac_dv,
                                        contact_dac_da);

        //TODO: replace with contact_model::nc

        int colRef = nv(model.joints[joint_id])+idx_v(model.joints[joint_id])-1;
        for(Eigen::DenseIndex j=colRef;j>=0;j=data.parents_fromRow[(size_t)j]) {
          contact_dac_dq.template topRows<3>().col(j) +=
            data.v[joint_id].angular().cross(contact_dvc_dq_tmp.template topRows<3>().col(j))
            - data.v[joint_id].linear().cross(contact_dvc_dq_tmp.template bottomRows<3>().col(j));
          contact_dac_dv.template topRows<3>().col(j) +=
            data.v[joint_id].angular().cross(contact_dac_da.template topRows<3>().col(j))
            - data.v[joint_id].linear().cross(contact_dac_da.template bottomRows<3>().col(j));
        }

        //TODO: remplacer par contact_model::NC
        current_row_sol_id += 6;
        break;
      }
      case CONTACT_3D:
      {
        typedef typename SizeDepType<3>::template RowsReturn<typename Data::MatrixXs>::Type RowsBlock;
        //TODO: Specialize for the 3d case.
        //TODO: We don't need all these quantities. Remove those not needed.
        typename Data::Matrix6x& v_partial_dq_tmp = data.dFda;
        typename Data::Matrix6x& a_partial_dq_tmp = data.SDinv;
        typename Data::Matrix6x& a_partial_dv_tmp = data.UDinv;
        typename Data::Matrix6x& a_partial_da_tmp = data.IS;

        getJointAccelerationDerivatives(model, data,
                                        joint_id,
                                        LOCAL,
                                        v_partial_dq_tmp,
                                        a_partial_dq_tmp,
                                        a_partial_dv_tmp,
                                        a_partial_da_tmp);

        //TODO: replace with contact_model::nc
        RowsBlock contact_dac_dq = SizeDepType<3>::middleRows(data.dac_dq,
                                                              current_row_sol_id);
        RowsBlock contact_dac_dv = SizeDepType<3>::middleRows(data.dac_dv,
                                                               current_row_sol_id);
        RowsBlock contact_dac_da = SizeDepType<3>::middleRows(data.dac_da,
                                                                  current_row_sol_id);

        contact_dac_da = a_partial_da_tmp.template topRows<3>();
        contact_dac_dq = a_partial_dq_tmp.template topRows<3>();
        contact_dac_dv = a_partial_dv_tmp.template topRows<3>();
        
        int colRef = nv(model.joints[joint_id])+idx_v(model.joints[joint_id])-1;
        for(Eigen::DenseIndex j=colRef;j>=0;j=data.parents_fromRow[(size_t)j]) {
          contact_dac_dq.template topRows<3>().col(j) +=
            data.v[joint_id].angular().cross(v_partial_dq_tmp.template topRows<3>().col(j))
            - data.v[joint_id].linear().cross(v_partial_dq_tmp.template bottomRows<3>().col(j));
          contact_dac_dv.template topRows<3>().col(j) +=
            data.v[joint_id].angular().cross(a_partial_da_tmp.template topRows<3>().col(j))
            - data.v[joint_id].linear().cross(a_partial_da_tmp.template bottomRows<3>().col(j));
        }
        current_row_sol_id += 3;
        break;
      }
      default:
        assert(false && "must never happen");
        break;
      }
    }
    data.contact_chol.getOperationalSpaceInertiaMatrix(data.osim);
    data.contact_chol.getInverseMassMatrix(data.Minv);

    //Temporary: dlambda_dv stores J*Minv
    //TODO: Sparse
    data.dlambda_dv.noalias() = data.dac_da * data.Minv;
    PINOCCHIO_EIGEN_CONST_CAST(MatrixType6,lambda_partial_dtau).noalias() = -data.osim * data.dlambda_dv; //OUTPUT

    MatrixType3 & ddq_partial_dtau_ = PINOCCHIO_EIGEN_CONST_CAST(MatrixType3,ddq_partial_dtau);
    ddq_partial_dtau_.noalias() = data.dlambda_dv.transpose() * lambda_partial_dtau;
    ddq_partial_dtau_ += data.Minv; //OUTPUT
    
    data.dac_dq.noalias() -= data.dlambda_dv * data.dtau_dq;
    data.dac_dv.noalias() -= data.dlambda_dv * data.dtau_dv;

    PINOCCHIO_EIGEN_CONST_CAST(MatrixType4,lambda_partial_dq).noalias() = -data.osim * data.dac_dq; //OUTPUT
    PINOCCHIO_EIGEN_CONST_CAST(MatrixType5,lambda_partial_dv).noalias() = -data.osim * data.dac_dv; //OUTPUT

    current_row_sol_id = 0;
    for(typename RigidContactModelVector::const_iterator it = contact_models.begin(); it != contact_models.end(); ++it)
    {
      const RigidContactModel & contact_model = *it;
      
      const typename Model::FrameIndex & frame_id = contact_model.frame_id;
      const typename Model::Frame & frame = model.frames[frame_id];
      const typename Model::JointIndex & joint_id = frame.parent;
      
      //TODO: This is only for size 6. replace with contact_model::NC
      switch(contact_model.type)
      {
      case CONTACT_6D:
      {
        typedef typename SizeDepType<6>::template RowsReturn<typename Data::MatrixXs>::Type RowsBlock;
        typedef typename SizeDepType<6>::template RowsReturn<typename Data::MatrixXs>::ConstType ConstRowsBlock;

        //TODO: replace with contact_model::nc
        RowsBlock contact_dac_da = SizeDepType<6>::middleRows(data.dac_da,
                                                                   current_row_sol_id);

        ConstRowsBlock contact_dlambda_dq = SizeDepType<6>::middleRows(lambda_partial_dq,
                                                                  current_row_sol_id);
        ConstRowsBlock contact_dlambda_dv = SizeDepType<6>::middleRows(lambda_partial_dv,
                                                                  current_row_sol_id);
        
        int colRef = nv(model.joints[joint_id])+idx_v(model.joints[joint_id])-1;
        for(Eigen::DenseIndex j=colRef;j>=0;j=data.parents_fromRow[(size_t)j])
        {
          data.dtau_dq.row(j).noalias() -= contact_dac_da.col(j).transpose() * contact_dlambda_dq;
          data.dtau_dv.row(j).noalias() -= contact_dac_da.col(j).transpose() * contact_dlambda_dv;
        }
        current_row_sol_id += 6;
        break;
      }
      case CONTACT_3D:
      {
        typedef typename SizeDepType<3>::template RowsReturn<typename Data::MatrixXs>::Type RowsBlock;
        typedef typename SizeDepType<3>::template RowsReturn<typename Data::MatrixXs>::ConstType ConstRowsBlock;

        //TODO: replace with contact_model::nc
        RowsBlock contact_dac_da = SizeDepType<3>::middleRows(data.dac_da,
                                                                   current_row_sol_id);

        ConstRowsBlock contact_dlambda_dq = SizeDepType<3>::middleRows(lambda_partial_dq,
                                                                  current_row_sol_id);
        ConstRowsBlock contact_dlambda_dv = SizeDepType<3>::middleRows(lambda_partial_dv,
                                                                  current_row_sol_id);
        
        int colRef = nv(model.joints[joint_id])+idx_v(model.joints[joint_id])-1;
        for(Eigen::DenseIndex j=colRef;j>=0;j=data.parents_fromRow[(size_t)j])
        {
          data.dtau_dq.row(j).noalias() -= contact_dac_da.col(j).transpose() * contact_dlambda_dq;
          data.dtau_dv.row(j).noalias() -= contact_dac_da.col(j).transpose() * contact_dlambda_dv;
        }
        current_row_sol_id += 3;
        break;
      }

      default:
        assert(false && "must never happen");
        break;
      }
    }

    PINOCCHIO_EIGEN_CONST_CAST(MatrixType1,ddq_partial_dq).noalias() = -data.Minv*data.dtau_dq; //OUTPUT
    PINOCCHIO_EIGEN_CONST_CAST(MatrixType2,ddq_partial_dv).noalias() = -data.Minv*data.dtau_dv; //OUTPUT
  }
  
  
} // namespace pinocchio

#endif // ifndef __pinocchio_algorithm_contact_dynamics_hxx__
