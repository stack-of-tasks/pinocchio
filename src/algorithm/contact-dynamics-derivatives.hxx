//
// Copyright (c) 2020 CNRS INRIA
//

#ifndef __pinocchio_algorithm_contact_dynamics_derivatives_hxx__
#define __pinocchio_algorithm_contact_dynamics_derivatives_hxx__

#include "pinocchio/algorithm/check.hpp"
#include "pinocchio/algorithm/rnea-derivatives.hpp"
#include "pinocchio/algorithm/kinematics-derivatives.hpp"
#include "pinocchio/algorithm/contact-cholesky.hpp"

namespace pinocchio
{


  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, bool ContactMode>
  struct ComputeContactDynamicsDerivativesForwardStep
    : public fusion::JointUnaryVisitorBase< ComputeContactDynamicsDerivativesForwardStep<Scalar,Options,JointCollectionTpl,ContactMode> >
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
      typedef typename SizeDepType<JointModel::NV>::template ColsReturn<typename Data::Matrix6x>::Type ColsBlock;
      ColsBlock J_cols = jmodel.jointCols(data.J);
      ColsBlock dAdq_cols = jmodel.jointCols(data.dAdq);
      
      if(ContactMode)
      {
        const Motion & ov = data.ov[i];
        ColsBlock dJ_cols = jmodel.jointCols(data.dJ);
        ColsBlock dVdq_cols = jmodel.jointCols(data.dVdq);

        motionSet::motionAction(ov,J_cols,dJ_cols);
        // TODO: make more efficient
        data.v[i] = data.oMi[i].actInv(data.ov[i]);
        if(parent > 0)
        {
          motionSet::motionAction(data.ov[parent],J_cols,dVdq_cols);
        }
        else
          dVdq_cols.setZero();
        // computes variation of inertias
        data.doYcrb[i] = data.oinertias[i].variation(ov);
        typedef ComputeRNEADerivativesForwardStep<Scalar,Options,JointCollectionTpl,typename Data::ConfigVectorType,typename Data::TangentVectorType,typename Data::TangentVectorType> RNEAForwardStepType;
        RNEAForwardStepType::addForceCrossMatrix(data.oh[i],data.doYcrb[i]);
        Motion & oa = data.oa[i];
        Motion & oa_gf = data.oa_gf[i];
        ColsBlock dAdv_cols = jmodel.jointCols(data.dAdv);
        const typename Data::TangentVectorType& a = data.ddq;
        data.a[i] = jdata.S() * jmodel.jointVelocitySelector(a) + jdata.c() + (data.v[i] ^ jdata.v());
        if(parent > 0)
          data.a[i] += data.liMi[i].actInv(data.a[parent]);
        oa = data.oMi[i].act(data.a[i]);
        oa_gf = oa - model.gravity; // add gravity contribution
        data.of[i] = data.oinertias[i] * oa_gf + ov.cross(data.oh[i]);
        motionSet::motionAction(data.oa_gf[parent],J_cols,dAdq_cols);
        dAdv_cols = dJ_cols;
        if(parent > 0)
        {
          motionSet::motionAction<ADDTO>(data.ov[parent],dVdq_cols,dAdq_cols);
          dAdv_cols.noalias() += dVdq_cols;
        }
      }
      else
      {
        Motion & odv = data.oa[i];
        Motion & odvparent = data.oa[parent];
        const typename Data::TangentVectorType& dimpulse = data.ddq;
        //Temporary calculation of J(dq_after)
        odv = J_cols * jmodel.jointVelocitySelector(dimpulse);
        if(parent > 0)
          odv += odvparent;
        motionSet::motionAction(odvparent,J_cols,dAdq_cols);
        data.of[i] = data.oinertias[i] * odv;
      }
    }
  };

  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, bool ContactMode>
  struct ComputeContactDynamicDerivativesBackwardStep
    : public fusion::JointUnaryVisitorBase<ComputeContactDynamicDerivativesBackwardStep<Scalar,Options,JointCollectionTpl,ContactMode> >
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
      typedef typename SizeDepType<JointModel::NV>::template ColsReturn<typename Data::Matrix6x>::Type ColsBlock;
      typedef ComputeRNEADerivativesBackwardStep<Scalar,Options,JointCollectionTpl,
                                                 typename Data::RowMatrixXs,
                                                 typename Data::RowMatrixXs,
                                                 typename Data::RowMatrixXs> RNEABackwardStep;
      
      const JointIndex & i = jmodel.id();
      const JointIndex & parent = model.parents[i];
      ColsBlock J_cols = jmodel.jointCols(data.J);
      ColsBlock dVdq_cols = jmodel.jointCols(data.dVdq);
      ColsBlock dAdq_cols = jmodel.jointCols(data.dAdq);
      ColsBlock dFdq_cols = jmodel.jointCols(data.dFdq);
      typename Data::RowMatrixXs & rnea_partial_dq_ = data.dtau_dq;
      
      // Temporary variables
      typename PINOCCHIO_EIGEN_PLAIN_ROW_MAJOR_TYPE(MatrixNV6) YS (jmodel.nv(),6);
      typename PINOCCHIO_EIGEN_PLAIN_ROW_MAJOR_TYPE(MatrixNV6) StY(jmodel.nv(),6);

      motionSet::inertiaAction(data.oYcrb[i],dAdq_cols,dFdq_cols);
      // dtau/dq
      if(parent>0)
      {
        RNEABackwardStep::lhsInertiaMult(data.oYcrb[i],J_cols.transpose(),YS);
        if(ContactMode)
        {
          dFdq_cols.noalias() += data.doYcrb[i] * dVdq_cols;
          StY.noalias() = J_cols.transpose() * data.doYcrb[i];
          for(int j = data.parents_fromRow[(typename Model::Index)jmodel.idx_v()];j >= 0; j = data.parents_fromRow[(typename Model::Index)j])
            {
              rnea_partial_dq_.middleRows(jmodel.idx_v(),jmodel.nv()).col(j).noalias()
                = YS  * data.dAdq.col(j)
                + StY * data.dVdq.col(j);
            }
        }
        else
        {
          for(int j = data.parents_fromRow[(typename Model::Index)jmodel.idx_v()];j >= 0; j = data.parents_fromRow[(typename Model::Index)j])
            {
              rnea_partial_dq_.middleRows(jmodel.idx_v(),jmodel.nv()).col(j).noalias()
                = YS  * data.dAdq.col(j);
            }
        }
      }

      rnea_partial_dq_.block(jmodel.idx_v(),jmodel.idx_v(),jmodel.nv(),data.nvSubtree[i]).noalias()
      = J_cols.transpose()*data.dFdq.middleCols(jmodel.idx_v(),data.nvSubtree[i]);
      motionSet::act<ADDTO>(J_cols,data.of[i],dFdq_cols);

      if(ContactMode)
      {
        
        ColsBlock dAdv_cols = jmodel.jointCols(data.dAdv);
        ColsBlock dFdv_cols = jmodel.jointCols(data.dFdv); 
        typename Data::RowMatrixXs & rnea_partial_dv_ = data.dtau_dv;       
        dFdv_cols.noalias() = data.doYcrb[i] * J_cols;
        motionSet::inertiaAction<ADDTO>(data.oYcrb[i],dAdv_cols,dFdv_cols);
        //motionSet::act<ADDTO>(J_cols,data.of[i],dFdq_cols);
        
        rnea_partial_dv_.block(jmodel.idx_v(),jmodel.idx_v(),jmodel.nv(),data.nvSubtree[i]).noalias()
          = J_cols.transpose()*data.dFdv.middleCols(jmodel.idx_v(),data.nvSubtree[i]);
        if(parent > 0)
        {
          for(int j = data.parents_fromRow[(typename Model::Index)jmodel.idx_v()];j >= 0; j = data.parents_fromRow[(typename Model::Index)j])
          {
            rnea_partial_dv_.middleRows(jmodel.idx_v(),jmodel.nv()).col(j).noalias()
              = YS  * data.dAdv.col(j)
              + StY * data.J.col(j);
          }
          data.doYcrb[parent] += data.doYcrb[i];
        }
        // Restore the status of dAdq_cols (remove gravity)
        for(Eigen::DenseIndex k =0; k < jmodel.nv(); ++k)
        {
          typedef typename ColsBlock::ColXpr ColType;
          MotionRef<ColType> min(J_cols.col(k));
          MotionRef<ColType> mout(dAdq_cols.col(k));
          mout.linear() += model.gravity.linear().cross(min.angular());
        }
      }
      if (parent > 0)
      {
        data.of[parent] += data.of[i];
      }
    }
  };
  
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, class ContactModelAllocator, class ContactDataAllocator, typename MatrixType1, typename MatrixType2, typename MatrixType3, typename MatrixType4,
           typename MatrixType5, typename MatrixType6>
  inline void computeContactDynamicsDerivatives(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                                                DataTpl<Scalar,Options,JointCollectionTpl> & data,
                                                const std::vector<RigidContactModelTpl<Scalar,Options>,ContactModelAllocator> & contact_models,
                                                std::vector<RigidContactDataTpl<Scalar,Options>,ContactDataAllocator> & contact_data,
                                                const Scalar mu,
                                                const Eigen::MatrixBase<MatrixType1> & ddq_partial_dq,
                                                const Eigen::MatrixBase<MatrixType2> & ddq_partial_dv,
                                                const Eigen::MatrixBase<MatrixType3> & ddq_partial_dtau,
                                                const Eigen::MatrixBase<MatrixType4> & lambda_partial_dq,
                                                const Eigen::MatrixBase<MatrixType5> & lambda_partial_dv,
                                                const Eigen::MatrixBase<MatrixType6> & lambda_partial_dtau)
  {
    const Eigen::DenseIndex & nc = data.contact_chol.constraintDim();
    
    PINOCCHIO_CHECK_INPUT_ARGUMENT(contact_data.size() == contact_models.size(),
                                   "contact_data and contact_models do not have the same size");
    
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
    
    PINOCCHIO_CHECK_INPUT_ARGUMENT(mu >= (Scalar)0 && "mu must be positive.");
    PINOCCHIO_CHECK_INPUT_ARGUMENT(model.gravity.angular().isZero(),
                                   "The gravity must be a pure force vector, no angular part");
    
    assert(model.check(data) && "data is not consistent with model.");
    data.dtau_dq.setZero();
    data.dtau_dv.setZero();
    data.dac_dq.setZero();
    data.dac_dv.setZero();
    
    typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
    typedef DataTpl<Scalar,Options,JointCollectionTpl> Data;
    
    typedef RigidContactModelTpl<Scalar,Options> RigidContactModel;
    typedef RigidContactDataTpl<Scalar,Options> RigidContactData;
    
    typedef typename ModelTpl<Scalar,Options,JointCollectionTpl>::JointIndex JointIndex;

    data.oa_gf[0] = -model.gravity;
    
    typedef ComputeContactDynamicsDerivativesForwardStep<Scalar,Options,JointCollectionTpl,true> Pass1;
    for(JointIndex i=1; i<(JointIndex) model.njoints; ++i)
    {
      Pass1::run(model.joints[i],data.joints[i],
                 typename Pass1::ArgsType(model,data));
    }
    
    // Add the contribution of the external forces. TODO: this should be done at the contact dynamics level.
    for(size_t k = 0; k < contact_models.size(); ++k)
    {
      const RigidContactModel & cmodel = contact_models[k];
      const RigidContactData & cdata = contact_data[k];

      switch(cmodel.reference_frame) {
      case LOCAL:
      {
        
        data.of[model.frames[cmodel.frame_id].parent] -= data.oMf[cmodel.frame_id].act(cdata.contact_force);
        break;
      }
      case WORLD:
      {
        data.of[model.frames[cmodel.frame_id].parent] -= cdata.contact_force;
        break;
      }
      case LOCAL_WORLD_ALIGNED:
      {
        std::cerr<<"BAD CALL> ABORT ABORT";
        break;
      }
      }
    }

    typedef ComputeContactDynamicDerivativesBackwardStep<Scalar,Options,JointCollectionTpl,true> Pass2;
    for(JointIndex i=(JointIndex)(model.njoints-1); i>0; --i)
    {
      Pass2::run(model.joints[i],
                 typename Pass2::ArgsType(model,data));
    }    

    Eigen::DenseIndex current_row_sol_id = 0;
    for(size_t k = 0; k < contact_models.size(); ++k)
    {
      const RigidContactModel & cmodel = contact_models[k];
      const RigidContactData & cdata = contact_data[k];

      const typename Model::FrameIndex & frame_id = cmodel.frame_id;
      const typename Model::Frame & frame = model.frames[frame_id];
      const typename Model::JointIndex joint_id = frame.parent;
      
      //TODO: This is only for size 6. replace with contact_model::NC
      switch(cmodel.type)
      {
        case CONTACT_6D:
        {
          typedef typename SizeDepType<6>::template RowsReturn<typename Data::MatrixXs>::Type RowsBlock;
          //TODO: We don't need all these quantities. Remove those not needed.
          typename Data::Matrix6x & contact_dvc_dq_tmp = data.dFda;
          RowsBlock contact_dac_dq = SizeDepType<6>::middleRows(data.dac_dq,
                                                                current_row_sol_id);
          RowsBlock contact_dac_dv = SizeDepType<6>::middleRows(data.dac_dv,
                                                                current_row_sol_id);
          RowsBlock contact_dac_da = SizeDepType<6>::middleRows(data.dac_da,
                                                                current_row_sol_id);

          switch(cmodel.reference_frame) {
          case WORLD:
          {
            getJointAccelerationDerivatives(model, data,
                                            joint_id,
                                            WORLD,
                                            contact_dvc_dq_tmp,
                                            contact_dac_dq,
                                            contact_dac_dv,
                                            contact_dac_da);
            
            //TODO: replace with contact_model::nc
            int colRef = nv(model.joints[joint_id])+idx_v(model.joints[joint_id])-1;
            for(Eigen::DenseIndex j=colRef;j>=0;j=data.parents_fromRow[(size_t)j])
            {
              contact_dac_dq.template topRows<3>().col(j) +=
                data.ov[joint_id].angular().cross(contact_dvc_dq_tmp.template topRows<3>().col(j))
                - data.ov[joint_id].linear().cross(contact_dvc_dq_tmp.template bottomRows<3>().col(j));
              contact_dac_dv.template topRows<3>().col(j) +=
                data.ov[joint_id].angular().cross(contact_dac_da.template topRows<3>().col(j))
                - data.ov[joint_id].linear().cross(contact_dac_da.template bottomRows<3>().col(j));
            }
            break;
          }
          case LOCAL:
          {
            getJointAccelerationDerivatives(model, data,
                                            joint_id,
                                            LOCAL,
                                            contact_dvc_dq_tmp,
                                            contact_dac_dq,
                                            contact_dac_dv,
                                            contact_dac_da);
            
            //TODO: replace with contact_model::nc
            int colRef = nv(model.joints[joint_id])+idx_v(model.joints[joint_id])-1;
            for(Eigen::DenseIndex j=colRef;j>=0;j=data.parents_fromRow[(size_t)j])
            {
              contact_dac_dq.template topRows<3>().col(j) +=
                data.v[joint_id].angular().cross(contact_dvc_dq_tmp.template topRows<3>().col(j))
                - data.v[joint_id].linear().cross(contact_dvc_dq_tmp.template bottomRows<3>().col(j));
              contact_dac_dv.template topRows<3>().col(j) +=
                data.v[joint_id].angular().cross(contact_dac_da.template topRows<3>().col(j))
                - data.v[joint_id].linear().cross(contact_dac_da.template bottomRows<3>().col(j));            
            }
            break;
          }
          default:
            break;
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
          typename Data::Matrix6x & v_partial_dq_tmp = data.dFda;
          typename Data::Matrix6x & a_partial_dq_tmp = data.SDinv;
          typename Data::Matrix6x & a_partial_dv_tmp = data.UDinv;
          typename Data::Matrix6x & a_partial_da_tmp = data.IS;

          switch(cmodel.reference_frame) {
          case WORLD:
          {
            getJointAccelerationDerivatives(model, data,
                                            joint_id,
                                            WORLD,
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
                data.ov[joint_id].angular().cross(v_partial_dq_tmp.template topRows<3>().col(j))
                - data.ov[joint_id].linear().cross(v_partial_dq_tmp.template bottomRows<3>().col(j));
              contact_dac_dv.template topRows<3>().col(j) +=
                data.ov[joint_id].angular().cross(a_partial_da_tmp.template topRows<3>().col(j))
                - data.ov[joint_id].linear().cross(a_partial_da_tmp.template bottomRows<3>().col(j));
            }
            break;
          }
          case LOCAL:
          {
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
            break;
          }
          default:
            break;
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
    typename Data::MatrixXs & JMinv = data.dlambda_dv;

    JMinv.noalias() = data.dac_da * data.Minv;
    PINOCCHIO_EIGEN_CONST_CAST(MatrixType6,lambda_partial_dtau).noalias() = -data.osim * JMinv; //OUTPUT

    MatrixType3 & ddq_partial_dtau_ = PINOCCHIO_EIGEN_CONST_CAST(MatrixType3,ddq_partial_dtau);
    ddq_partial_dtau_.noalias() = JMinv.transpose() * lambda_partial_dtau;
    ddq_partial_dtau_ += data.Minv; //OUTPUT
    
    data.dac_dq.noalias() -= JMinv * data.dtau_dq;
    data.dac_dv.noalias() -= JMinv * data.dtau_dv;
    
    PINOCCHIO_EIGEN_CONST_CAST(MatrixType4,lambda_partial_dq).noalias() = -data.osim * data.dac_dq; //OUTPUT
    PINOCCHIO_EIGEN_CONST_CAST(MatrixType5,lambda_partial_dv).noalias() = -data.osim * data.dac_dv; //OUTPUT

    current_row_sol_id = 0;
    for(size_t k = 0; k < contact_models.size(); ++k)
    {
      const RigidContactModel & cmodel = contact_models[k];
      const RigidContactData & cdata = contact_data[k];
      
      const typename Model::FrameIndex & frame_id = cmodel.frame_id;
      const typename Model::Frame & frame = model.frames[frame_id];
      const typename Model::JointIndex joint_id = frame.parent;
      
      //TODO: This is only for size 6. replace with contact_model::NC
      switch(cmodel.type)
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

#endif // ifndef __pinocchio_algorithm_contact_dynamics_derivatives_hxx__
