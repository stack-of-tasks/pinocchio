//
// Copyright (c) 2020 CNRS INRIA
//

#ifndef __pinocchio_algorithm_constraint_dynamics_derivatives_hxx__
#define __pinocchio_algorithm_constraint_dynamics_derivatives_hxx__

#include "pinocchio/algorithm/check.hpp"
#include "pinocchio/algorithm/rnea-derivatives.hpp"
#include "pinocchio/algorithm/kinematics-derivatives.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/frames-derivatives.hpp"
#include "pinocchio/algorithm/contact-cholesky.hpp"
#include "pinocchio/algorithm/utils/motion.hpp"

namespace pinocchio
{

  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, bool ContactMode>
  struct ComputeConstraintDynamicsDerivativesForwardStep
  : public fusion::JointUnaryVisitorBase< ComputeConstraintDynamicsDerivativesForwardStep<Scalar,Options,JointCollectionTpl,ContactMode> >
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

      const JointIndex i = jmodel.id();
      const JointIndex parent = model.parents[i];
      
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
      
      const JointIndex i = jmodel.id();
      const JointIndex parent = model.parents[i];
      ColsBlock J_cols = jmodel.jointCols(data.J);
      ColsBlock dVdq_cols = jmodel.jointCols(data.dVdq);
      ColsBlock dAdq_cols = jmodel.jointCols(data.dAdq);
      ColsBlock dFdq_cols = jmodel.jointCols(data.dFdq);
      ColsBlock dFda_cols = jmodel.jointCols(data.dFda);
      
      typename Data::RowMatrixXs & dtau_dq = data.dtau_dq;
      
      // Temporary variables
      typename PINOCCHIO_EIGEN_PLAIN_ROW_MAJOR_TYPE(MatrixNV6) StdY(jmodel.nv(),6);

      motionSet::inertiaAction(data.oYcrb[i],dAdq_cols,dFdq_cols);
      // dtau/dq
      if(parent>0)
      {
        if(ContactMode)
        {
          dFdq_cols.noalias() += data.doYcrb[i] * dVdq_cols;
          StdY.noalias() = J_cols.transpose() * data.doYcrb[i];
          for(int j = data.parents_fromRow[(typename Model::Index)jmodel.idx_v()];j >= 0; j = data.parents_fromRow[(typename Model::Index)j])
            {
              dtau_dq.middleRows(jmodel.idx_v(),jmodel.nv()).col(j).noalias()
                = dFda_cols.transpose() * data.dAdq.col(j)
                + StdY * data.dVdq.col(j);
            }
        }
        else
        {
          for(int j = data.parents_fromRow[(typename Model::Index)jmodel.idx_v()];j >= 0; j = data.parents_fromRow[(typename Model::Index)j])
            {
              dtau_dq.middleRows(jmodel.idx_v(),jmodel.nv()).col(j).noalias()
                = dFda_cols.transpose() * data.dAdq.col(j);
            }
        }
      }

      dtau_dq.block(jmodel.idx_v(),jmodel.idx_v(),jmodel.nv(),data.nvSubtree[i]).noalias()
      = J_cols.transpose()*data.dFdq.middleCols(jmodel.idx_v(),data.nvSubtree[i]);
      motionSet::act<ADDTO>(J_cols,data.of[i],dFdq_cols);

      if(ContactMode)
      {
        ColsBlock dAdv_cols = jmodel.jointCols(data.dAdv);
        ColsBlock dFdv_cols = jmodel.jointCols(data.dFdv);
        
        typename Data::RowMatrixXs & dtau_dv = data.dtau_dv;
        dFdv_cols.noalias() = data.doYcrb[i] * J_cols;
        motionSet::inertiaAction<ADDTO>(data.oYcrb[i],dAdv_cols,dFdv_cols);

        dtau_dv.block(jmodel.idx_v(),jmodel.idx_v(),jmodel.nv(),data.nvSubtree[i]).noalias()
          = J_cols.transpose()*data.dFdv.middleCols(jmodel.idx_v(),data.nvSubtree[i]);
        if(parent > 0)
        {
          for(int j = data.parents_fromRow[(typename Model::Index)jmodel.idx_v()];j >= 0; j = data.parents_fromRow[(typename Model::Index)j])
          {
            dtau_dv.middleRows(jmodel.idx_v(),jmodel.nv()).col(j).noalias()
              = dFda_cols.transpose() * data.dAdv.col(j)
              + StdY * data.J.col(j);
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
      
      if(parent > 0)
        data.of[parent] += data.of[i];
    }
  };
  
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, class ContactModelAllocator, class ContactDataAllocator, typename MatrixType1, typename MatrixType2, typename MatrixType3, typename MatrixType4,
           typename MatrixType5, typename MatrixType6>
  inline void computeConstraintDynamicsDerivatives(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                                                DataTpl<Scalar,Options,JointCollectionTpl> & data,
                                                const std::vector<RigidConstraintModelTpl<Scalar,Options>,ContactModelAllocator> & contact_models,
                                                std::vector<RigidConstraintDataTpl<Scalar,Options>,ContactDataAllocator> & contact_data,
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
    
    PINOCCHIO_CHECK_INPUT_ARGUMENT(check_expression_if_real<Scalar>(
                                              model.gravity.angular()[0] == Scalar(0)
                                              && model.gravity.angular()[1] == Scalar(0)
                                              && model.gravity.angular()[2] == Scalar(0)),
                                  "The gravity must be a pure force vector, no angular part");
    
    assert(model.check(data) && "data is not consistent with model.");
    data.dtau_dq.setZero();
    data.dtau_dv.setZero();
    data.dac_dq.setZero();
    data.dac_dv.setZero();
    
    typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
    typedef DataTpl<Scalar,Options,JointCollectionTpl> Data;
    
    typedef RigidConstraintModelTpl<Scalar,Options> RigidConstraintModel;
    typedef RigidConstraintDataTpl<Scalar,Options> RigidConstraintData;
    
    typedef typename ModelTpl<Scalar,Options,JointCollectionTpl>::JointIndex JointIndex;
    typedef typename Data::SE3 SE3;
    typedef typename Data::Motion Motion;
    typedef typename Data::Force Force;
    typedef typename Data::Vector3 Vector3;
    data.oa_gf[0] = -model.gravity;

    //TODO: Temp variable
    Force of_temp, of_temp2;
    Motion a_temp;
    SE3 se3_temp;
    
    typedef ComputeConstraintDynamicsDerivativesForwardStep<Scalar,Options,JointCollectionTpl,true> Pass1;
    for(JointIndex i=1; i<(JointIndex) model.njoints; ++i)
    {
      Pass1::run(model.joints[i],data.joints[i],
                 typename Pass1::ArgsType(model,data));
    }

    // Add the contribution of the external forces.
    for(size_t k = 0; k < contact_models.size(); ++k)
    {
      const RigidConstraintModel & cmodel = contact_models[k];
      const RigidConstraintData & cdata = contact_data[k];

      // TODO: Temporary variable
      const SE3 & oMc1 = cdata.oMc1;
      Force & of1 = data.of[cmodel.joint1_id];
      const SE3 & oMc2 = cdata.oMc2;
      Force & of2 = data.of[cmodel.joint2_id];

      switch(cmodel.reference_frame) 
      {
      case LOCAL: {
        switch(cmodel.type) {
        case CONTACT_6D: {
          if(cmodel.joint1_id > 0) {
            of1 -= oMc1.act(cdata.contact_force);
          }
          if(cmodel.joint2_id > 0) {
            of_temp = oMc1.act(cdata.contact_force);
            of2 += of_temp;
          }
          break;
        }
        case CONTACT_3D: {
          of_temp.linear().noalias() = oMc1.rotation()*cdata.contact_force.linear();
          
          if(cmodel.joint1_id > 0) {
            of1.linear().noalias() -= of_temp.linear();
            of1.angular().noalias() -= oMc1.translation().cross(of_temp.linear());
          }
          if(cmodel.joint2_id > 0) {
            of2.linear() += of_temp.linear();
            of2.angular().noalias() += oMc2.translation().cross(of_temp.linear());
          }
	  break;
	}
	default: {
	  assert(false && "must never happen");
	  break;
	}
        }
        break;
      }
      case LOCAL_WORLD_ALIGNED: {
        switch(cmodel.type) {
	case CONTACT_6D: {
	  if(cmodel.joint1_id > 0) {
	    of1 -= cdata.contact_force;
	    of1.angular().noalias() -= oMc1.translation().cross(cdata.contact_force.linear());
	  }
	  if(cmodel.joint2_id > 0) {
	    of2 += cdata.contact_force;
	    of2.angular().noalias() += oMc1.translation().cross(cdata.contact_force.linear());
	  }
	  break;
	}
	case CONTACT_3D: {
	  if(cmodel.joint1_id > 0) {
	    of1.linear() -= cdata.contact_force.linear();
	    of1.angular().noalias() -= oMc1.translation().cross(cdata.contact_force.linear());
	  }
	  if(cmodel.joint2_id > 0) {
	    of2.linear() += cdata.contact_force.linear();
	    of2.angular().noalias() += oMc2.translation().cross(cdata.contact_force.linear());
	  }
	  break;
	}
	default: {
	  assert(false && "must never happen");
	  break;
	}
	}
        break;
      }
      default:
        assert(false && "Should never happen");
        break;
      }
    }

    //Backward Pass
    typedef ComputeContactDynamicDerivativesBackwardStep<Scalar,Options,JointCollectionTpl,true> Pass2;
    for(JointIndex i=(JointIndex)(model.njoints-1); i>0; --i)
    {
      Pass2::run(model.joints[i],
                 typename Pass2::ArgsType(model,data));
    }    

    // Compute the contact frame partial derivatives
    typename Data::SE3::Matrix6 Jlog;
    Eigen::DenseIndex current_row_sol_id = 0;
    const Eigen::DenseIndex constraint_dim = data.contact_chol.constraintDim();
    for(size_t k = 0; k < contact_models.size(); ++k)
    {
      const RigidConstraintModel & cmodel = contact_models[k];
      const RigidConstraintData & cdata = contact_data[k];
      typedef typename Data::ContactCholeskyDecomposition ContactCholeskyDecomposition;
      typedef typename ContactCholeskyDecomposition::IndexVector IndexVector;
      typedef typename ContactCholeskyDecomposition::BooleanVector BooleanVector;

      switch(cmodel.type)
      {
        case CONTACT_6D:
        {
          typedef typename SizeDepType<6>::template RowsReturn<typename Data::MatrixXs>::Type RowsBlock;
          RowsBlock contact_dvc_dq = SizeDepType<6>::middleRows(data.dvc_dq,
                                                                current_row_sol_id);
          RowsBlock contact_dac_dq = SizeDepType<6>::middleRows(data.dac_dq,
                                                                current_row_sol_id);
          RowsBlock contact_dac_dv = SizeDepType<6>::middleRows(data.dac_dv,
                                                                current_row_sol_id);
          RowsBlock contact_dac_da = SizeDepType<6>::middleRows(data.dac_da,
                                                                current_row_sol_id);


          if(cmodel.joint1_id >0) {
            getFrameAccelerationDerivatives(model, data,
                                            cmodel.joint1_id,
                                            cmodel.joint1_placement,
                                            cmodel.reference_frame,
                                            contact_dvc_dq,
                                            contact_dac_dq,
                                            contact_dac_dv,
                                            contact_dac_da);
          }

          if(cmodel.joint2_id >0) {
            //TODO: Memory assignment here
            typename Data::Matrix6x j2_dvc_dq(6,model.nv), j2_dac_dq(6,model.nv), j2_dac_dv(6,model.nv), j2_dac_da(6,model.nv);
            j2_dvc_dq.setZero(); j2_dac_dq.setZero(); j2_dac_dv.setZero(); j2_dac_da.setZero();
	    se3_temp = cmodel.joint2_placement * cdata.c1Mc2.inverse();
            getFrameAccelerationDerivatives(model, data,
                                            cmodel.joint2_id,
                                            se3_temp,
                                            cmodel.reference_frame,
                                            j2_dvc_dq,
                                            j2_dac_dq,
                                            j2_dac_dv,
                                            j2_dac_da);

            //TODO: This is only in case reference_frame is LOCAL
            contact_dvc_dq -= j2_dvc_dq;
            contact_dac_dq -= j2_dac_dq;
            contact_dac_dv -= j2_dac_dv;
            contact_dac_da -= j2_dac_da;
          }
          //END TODO
          
          break;
        }
        case CONTACT_3D:
        {
          typedef typename SizeDepType<3>::template RowsReturn<typename Data::MatrixXs>::Type RowsBlock;

          RowsBlock contact_dvc_dq = SizeDepType<3>::middleRows(data.dvc_dq,
                                                                current_row_sol_id);
          RowsBlock contact_dac_dq = SizeDepType<3>::middleRows(data.dac_dq,
                                                                current_row_sol_id);
          RowsBlock contact_dac_dv = SizeDepType<3>::middleRows(data.dac_dv,
                                                                current_row_sol_id);
          RowsBlock contact_dac_da = SizeDepType<3>::middleRows(data.dac_da,
                                                                current_row_sol_id);

          if(cmodel.joint1_id >0) {
            getPointClassicAccelerationDerivatives(model, data,
                                                   cmodel.joint1_id,
                                                   cmodel.joint1_placement,
                                                   cmodel.reference_frame,
                                                   contact_dvc_dq,
                                                   contact_dac_dq,
                                                   contact_dac_dv,
                                                   contact_dac_da);
          }
          if(cmodel.joint2_id >0) {
          
            //TODO: Memory assignment here
            typename Data::Matrix3x j2_dvc_dq(3,model.nv), j2_dac_dq(3,model.nv), j2_dac_dv(3,model.nv), j2_dac_da(3,model.nv);
	    
            j2_dvc_dq.setZero(); j2_dac_dq.setZero(); j2_dac_dv.setZero(); j2_dac_da.setZero();
	    se3_temp.translation() = cmodel.joint2_placement.translation();
	    se3_temp.rotation() = cmodel.joint2_placement.rotation() * cdata.c1Mc2.inverse().rotation();
            getPointClassicAccelerationDerivatives(model, data,
                                                   cmodel.joint2_id,
                                                   se3_temp,
						   cmodel.reference_frame,
                                                   j2_dvc_dq,
                                                   j2_dac_dq,
                                                   j2_dac_dv,
                                                   j2_dac_da);
            
            //TODO: This is only in case reference_frame is LOCAL            
            contact_dvc_dq -= j2_dvc_dq;
            contact_dac_dq -= j2_dac_dq;
            contact_dac_dv -= j2_dac_dv;
            contact_dac_da -= j2_dac_da;
          }
          //END TODO: Memory assignment here          
          break;
        }
        default:
          assert(false && "must never happen");
          break;
      }

      const IndexVector & colwise_sparsity = data.contact_chol.getLoopSparsityPattern(k);
      const BooleanVector& joint2_indexes  = data.contact_chol.getJoint2SparsityPattern(k).tail(model.nv);
      assert(colwise_sparsity.size() > 0 && "Must never happened, the sparsity pattern is empty");

      //Derivative of closed loop kinematic tree
      if(cmodel.joint2_id >0 )
      {
        switch(cmodel.type)
	{
	case CONTACT_6D:
	{
	  //TODO: THIS IS FOR THE LOCAL FRAME ONLY	  
	  const Motion& o_acc_c2 = data.oa[cmodel.joint2_id];
	  typedef typename SizeDepType<6>::template RowsReturn<typename Data::MatrixXs>::Type RowsBlock;
	  const RowsBlock contact_dvc_dq = SizeDepType<6>::middleRows(data.dvc_dq,current_row_sol_id);
	  RowsBlock contact_dac_dq = SizeDepType<6>::middleRows(data.dac_dq,current_row_sol_id);
	  RowsBlock contact_dac_dv = SizeDepType<6>::middleRows(data.dac_dv,current_row_sol_id);
	  const RowsBlock contact_dac_da = SizeDepType<6>::middleRows(data.dac_da,current_row_sol_id);
	  const typename Model::JointIndex joint2_id = cmodel.joint2_id;
	  const Eigen::DenseIndex colRef2 =
	    nv(model.joints[joint2_id])+idx_v(model.joints[joint2_id])-1;

	  switch(cmodel.reference_frame) {
	  case LOCAL: {
	    of_temp = cdata.oMc1.act(cdata.contact_force);
	    break;
	  }
	  case LOCAL_WORLD_ALIGNED: {
	    of_temp = cdata.contact_force;
	    of_temp.angular().noalias() += cdata.oMc1.translation().cross(cdata.contact_force.linear());
	    break;
	  }
	  default: {
	    assert(false && "must never happen");
	    break;
	  }
	  }

	  // d./dq
	  for(Eigen::DenseIndex k = 0; k < colwise_sparsity.size(); ++k)
	  {
	    const Eigen::DenseIndex col_id = colwise_sparsity[k] - constraint_dim;
	    const MotionRef<typename Data::Matrix6x::ColXpr> J_col(data.J.col(col_id));
	    motionSet::motionAction(o_acc_c2, data.J.col(col_id), a_temp.toVector());

	    switch(cmodel.reference_frame) {
	    case LOCAL: {
	      if(joint2_indexes[col_id]) {
		contact_dac_dq.col(col_id).noalias() += cdata.oMc1.actInv(a_temp).toVector();
	      }
	      else {
		contact_dac_dq.col(col_id).noalias() -= cdata.oMc1.actInv(a_temp).toVector();
	      }
	      break;
	    }
	    case LOCAL_WORLD_ALIGNED: {
	      // Do nothing
	      break;
	    }
	    default: {
	      assert(false && "must never happen");
	      break;
	    }
	    }
	    
	    motionSet::act(data.J.col(col_id), of_temp, of_temp2.toVector());
	    for(Eigen::DenseIndex j=colRef2;j>=0;j=data.parents_fromRow[(size_t)j])
	    {
	      if(joint2_indexes[col_id]) {
		data.dtau_dq(j,col_id) -= data.J.col(j).transpose() * of_temp2.toVector();
	      }
	      else {
		data.dtau_dq(j,col_id) += data.J.col(j).transpose() * of_temp2.toVector();
	      }
	    }  
	  }
	  break;
	}
	case CONTACT_3D:
	{

	  typedef typename SizeDepType<3>::template RowsReturn<typename Data::MatrixXs>::Type RowsBlock;
	  RowsBlock contact_dac_dq = SizeDepType<3>::middleRows(data.dac_dq,current_row_sol_id);
	  const typename Model::JointIndex joint2_id = cmodel.joint2_id;
	  const Eigen::DenseIndex colRef2 =
	    nv(model.joints[joint2_id])+idx_v(model.joints[joint2_id])-1;

	  switch(cmodel.reference_frame) {
	  case LOCAL: {
	    of_temp.linear().noalias() = cdata.oMc1.rotation()*cdata.contact_force.linear();
	    const Motion& c2_acc_c2 = getFrameClassicalAcceleration(model, data,
								    cmodel.joint2_id,
								    cmodel.joint2_placement,
								    cmodel.reference_frame);
	    a_temp.angular().noalias() = cdata.oMc2.rotation() * c2_acc_c2.linear();
	    break;
	  }
	  case LOCAL_WORLD_ALIGNED: {
	    of_temp.linear() = cdata.contact_force.linear();
	    break;
	  }
	  default: {
	    assert(false && "must never happen");
	    break;
	  }
	  }

	  // d./dq
	  for(Eigen::DenseIndex k = 0; k < colwise_sparsity.size(); ++k)
	  {
	    const Eigen::DenseIndex col_id = colwise_sparsity[k] - constraint_dim;
	    const MotionRef<typename Data::Matrix6x::ColXpr> J_col(data.J.col(col_id));

	    switch(cmodel.reference_frame) {
	    case LOCAL: {
	      a_temp.linear().noalias() = a_temp.angular().cross(J_col.angular());
	    if(joint2_indexes[col_id]) {
	      contact_dac_dq.col(col_id).noalias() += cdata.oMc1.rotation().transpose() * a_temp.linear();
	    }
	    else {
	      contact_dac_dq.col(col_id).noalias() -= cdata.oMc1.rotation().transpose() * a_temp.linear();
	    }
	      break;
	    }
	    case LOCAL_WORLD_ALIGNED: {
	      // Do nothing
	      break;
	    }
	    default: {
	      assert(false && "must never happen");
	      break;
	    }
	    }

	    of_temp2.linear().noalias() = of_temp.linear().cross(J_col.angular());
	    for(Eigen::DenseIndex j=colRef2;j>=0;j=data.parents_fromRow[(size_t)j])
	    {
	      const MotionRef<typename Data::Matrix6x::ColXpr> J2_col(data.J.col(j));      
	      //Temporary assignment
	      of_temp2.angular().noalias() = J2_col.linear() - cdata.oMc2.translation().cross(J2_col.angular());		
	      if(joint2_indexes[col_id]) {
		data.dtau_dq(j,col_id) += of_temp2.angular().transpose() * of_temp2.linear();
	      }
	      else {
		data.dtau_dq(j,col_id) -= of_temp2.angular().transpose() * of_temp2.linear();
	      }
	    }
	  }
	  break;
	}
	default:
	{
	  assert(false && "must never happen");
	  break;
	}
	}
      }

      // Add the contribution of the corrector
      if(check_expression_if_real<Scalar>(cmodel.corrector.Kp != Scalar(0)))
      {
        Jlog6(cdata.c1Mc2.inverse(),Jlog);

        switch(cmodel.type)
        {
          case CONTACT_6D:
          {
            typedef typename SizeDepType<6>::template RowsReturn<typename Data::MatrixXs>::Type RowsBlock;
            const RowsBlock contact_dvc_dq = SizeDepType<6>::middleRows(data.dvc_dq,current_row_sol_id);
            RowsBlock contact_dac_dq = SizeDepType<6>::middleRows(data.dac_dq,current_row_sol_id);
            RowsBlock contact_dac_dv = SizeDepType<6>::middleRows(data.dac_dv,current_row_sol_id);
            const RowsBlock contact_dac_da = SizeDepType<6>::middleRows(data.dac_da,current_row_sol_id);
            
            // d./dq
            for(Eigen::DenseIndex k = 0; k < colwise_sparsity.size(); ++k)
            {
              const Eigen::DenseIndex row_id = colwise_sparsity[k] - constraint_dim;
              contact_dac_dq.col(row_id).noalias() += cmodel.corrector.Kd * contact_dvc_dq.col(row_id);
              contact_dac_dq.col(row_id).noalias() += cmodel.corrector.Kp * Jlog * contact_dac_da.col(row_id);
            }
            
            // d./dv
            for(Eigen::DenseIndex k = 0; k < colwise_sparsity.size(); ++k)
            {
              const Eigen::DenseIndex row_id = colwise_sparsity[k] - constraint_dim;
              contact_dac_dv.col(row_id).noalias() += cmodel.corrector.Kd * contact_dac_da.col(row_id);
            }
            break;
          }
          case CONTACT_3D:
          {
            typedef typename SizeDepType<3>::template RowsReturn<typename Data::MatrixXs>::Type RowsBlock;
            const RowsBlock contact_dvc_dq = SizeDepType<3>::middleRows(data.dvc_dq,current_row_sol_id);
            RowsBlock contact_dac_dq = SizeDepType<3>::middleRows(data.dac_dq,current_row_sol_id);
            RowsBlock contact_dac_dv = SizeDepType<3>::middleRows(data.dac_dv,current_row_sol_id);
            const RowsBlock contact_dac_da = SizeDepType<3>::middleRows(data.dac_da,current_row_sol_id);
            
            // d./dq
            for(Eigen::DenseIndex k = 0; k < colwise_sparsity.size(); ++k)
            {
              const Eigen::DenseIndex row_id = colwise_sparsity[k] - constraint_dim;
              
              const MotionRef<typename Data::Matrix6x::ColXpr> J_col(data.J.col(row_id));
              contact_dac_dq.col(row_id).noalias() += cmodel.corrector.Kd * contact_dvc_dq.col(row_id);
              contact_dac_dq.col(row_id).noalias() -= cmodel.corrector.Kp * (cdata.oMc1.rotation().transpose()*J_col.angular()).cross(cdata.contact_placement_error.linear());
              contact_dac_dq.col(row_id).noalias() += cmodel.corrector.Kp * contact_dac_da.col(row_id);
            }
            // d./dv
            for(Eigen::DenseIndex k = 0; k < colwise_sparsity.size(); ++k)
            {
              const Eigen::DenseIndex row_id = colwise_sparsity[k] - constraint_dim;
              contact_dac_dv.col(row_id).noalias() += cmodel.corrector.Kd * contact_dac_da.col(row_id);
            }
            break;
          }
          default:
            assert(false && "must never happen");
            break;
        }
      }
      
      current_row_sol_id += cmodel.size();
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
      const RigidConstraintModel & cmodel = contact_models[k];

      const typename Model::JointIndex joint1_id = cmodel.joint1_id;
      const typename Model::JointIndex joint2_id = cmodel.joint2_id;
      const Eigen::DenseIndex colRef = nv(model.joints[joint1_id])
        +idx_v(model.joints[joint1_id])-1;

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

          //TODO: Sparsity in dac_da with loop joints?
          
          data.dtau_dq.noalias() -= contact_dac_da.transpose() * contact_dlambda_dq;
          data.dtau_dv.noalias() -= contact_dac_da.transpose() * contact_dlambda_dv;

          //END TODO
          
          /*
          
          for(Eigen::DenseIndex j=colRef;j>=0;j=data.parents_fromRow[(size_t)j])
          {
            data.dtau_dq.row(j).noalias() -= contact_dac_da.col(j).transpose() * contact_dlambda_dq;
            data.dtau_dv.row(j).noalias() -= contact_dac_da.col(j).transpose() * contact_dlambda_dv;
          }
          */
          break;
        }
        case CONTACT_3D:
        {

          typedef typename SizeDepType<3>::template RowsReturn<typename Data::MatrixXs>::Type RowsBlock;
          typedef typename SizeDepType<3>::template RowsReturn<typename Data::MatrixXs>::ConstType ConstRowsBlock;
          
          RowsBlock contact_dac_da = SizeDepType<3>::middleRows(data.dac_da,
                                                                current_row_sol_id);
          
          ConstRowsBlock contact_dlambda_dq = SizeDepType<3>::middleRows(lambda_partial_dq,
                                                                         current_row_sol_id);
          ConstRowsBlock contact_dlambda_dv = SizeDepType<3>::middleRows(lambda_partial_dv,
                                                                         current_row_sol_id);


          //TODO: Sparsity in dac_da with loop joints?
          
          data.dtau_dq.noalias() -= contact_dac_da.transpose() * contact_dlambda_dq;
          data.dtau_dv.noalias() -= contact_dac_da.transpose() * contact_dlambda_dv;

          //END TODO
          /*


          
          for(Eigen::DenseIndex j=colRef;j>=0;j=data.parents_fromRow[(size_t)j])
          {
            data.dtau_dq.row(j).noalias() -= contact_dac_da.col(j).transpose() * contact_dlambda_dq;
            data.dtau_dv.row(j).noalias() -= contact_dac_da.col(j).transpose() * contact_dlambda_dv;
          }
          */
          break;
        }

        default:
          assert(false && "must never happen");
          break;
      }
      current_row_sol_id += cmodel.size();
    }

    PINOCCHIO_EIGEN_CONST_CAST(MatrixType1,ddq_partial_dq).noalias() = -data.Minv*data.dtau_dq; //OUTPUT
    PINOCCHIO_EIGEN_CONST_CAST(MatrixType2,ddq_partial_dv).noalias() = -data.Minv*data.dtau_dv; //OUTPUT

    MatrixType4& dfc_dq = PINOCCHIO_EIGEN_CONST_CAST(MatrixType4,lambda_partial_dq);
    typedef typename SizeDepType<6>::template RowsReturn<typename Data::MatrixXs>::Type Rows6Block;
    typedef typename SizeDepType<3>::template RowsReturn<typename Data::MatrixXs>::Type Rows3Block;

    current_row_sol_id = 0;
    for(size_t k = 0; k < contact_models.size(); ++k)
    {
      const RigidConstraintModel & cmodel = contact_models[k];
      const RigidConstraintData & cdata = contact_data[k];
      const typename Model::JointIndex joint1_id = cmodel.joint1_id;
      const int colRef = nv(model.joints[joint1_id])+idx_v(model.joints[joint1_id])-1;
      
      switch(cmodel.reference_frame)
      {
      case LOCAL:
        break;
      case LOCAL_WORLD_ALIGNED:
      {
        const Force & of = cdata.contact_force;
        switch(cmodel.type)
        {
        case CONTACT_6D:
        {
          Rows6Block contact_dfc_dq = SizeDepType<6>::middleRows(dfc_dq, current_row_sol_id);
          for(Eigen::DenseIndex j=colRef;j>=0;j=data.parents_fromRow[(size_t)j])
          {
            typedef typename Rows6Block::ColXpr ColType;
            typedef typename Rows6Block::ColXpr ColTypeOut;
	    const MotionRef<typename Data::Matrix6x::ColXpr> J_col(data.J.col(j));
            ForceRef<ColTypeOut> fout(contact_dfc_dq.col(j));
            fout.linear().noalias() += J_col.angular().cross(of.linear());
            fout.angular().noalias() += J_col.angular().cross(of.angular());
          }
          break;
        }
        case CONTACT_3D:
        {
          Rows3Block contact_dfc_dq = SizeDepType<3>::middleRows(dfc_dq, current_row_sol_id);
          for(Eigen::DenseIndex j=colRef;j>=0;j=data.parents_fromRow[(size_t)j])
          {
            typedef typename Data::Matrix6x::ColXpr ColType;
	    const MotionRef<typename Data::Matrix6x::ColXpr> J_col(data.J.col(j));
	    contact_dfc_dq.col(j).noalias() += J_col.angular().cross(of.linear());
          }
          break;
        }
        default:
          assert(false && "must never happen");
          break;
        }
        break;
      }
      default:
        assert(false && "must never happen");
        break;        
      }
      current_row_sol_id += cmodel.size();
    }

  }
  
} // namespace pinocchio

#endif // ifndef __pinocchio_algorithm_constraint_dynamics_derivatives_hxx__
