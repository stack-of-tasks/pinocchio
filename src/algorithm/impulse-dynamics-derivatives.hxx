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

  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, typename Matrix6xOut1, typename Matrix6xOut2>
  struct JointImpulseVelocityDerivativesBackwardStep
  : public fusion::JointUnaryVisitorBase< JointImpulseVelocityDerivativesBackwardStep<Scalar,Options,JointCollectionTpl,Matrix6xOut1,Matrix6xOut2> >
  {
    typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
    typedef DataTpl<Scalar,Options,JointCollectionTpl> Data;
    
    typedef boost::fusion::vector<const Model &,
                                  Data &,
                                  const typename Model::JointIndex &,
                                  const ReferenceFrame &,
                                  Matrix6xOut1 &,
                                  Matrix6xOut2 &
                                  > ArgsType;
    
    template<typename JointModel>
    static void algo(const JointModelBase<JointModel> & jmodel,
                     const Model & model,
                     Data & data,
                     const typename Model::JointIndex & jointId,
                     const ReferenceFrame & rf,
                     const Eigen::MatrixBase<Matrix6xOut1> & v_partial_dq,
                     const Eigen::MatrixBase<Matrix6xOut2> & v_partial_dv)
    {
      typedef typename Model::JointIndex JointIndex;
      typedef typename Data::SE3 SE3;
      typedef typename Data::Motion Motion;
      
      const JointIndex & i = jmodel.id();
      const JointIndex & parent = model.parents[i];
      Motion & vtmp = data.ov[0]; // Temporary variable
      Motion & vtmp2 = data.oa[0]; // Temporary variable
      
      const SE3 & oMlast = data.oMi[jointId];
      const Motion & vlast = data.ov[jointId];
      const Motion & dvlast = data.oa[jointId];

      typedef typename SizeDepType<JointModel::NV>::template ColsReturn<typename Data::Matrix6x>::Type ColsBlock;
      ColsBlock Jcols = jmodel.jointCols(data.J);
      
      typedef typename SizeDepType<JointModel::NV>::template ColsReturn<Matrix6xOut1>::Type ColsBlockOut1;
      Matrix6xOut1 & v_partial_dq_ = PINOCCHIO_EIGEN_CONST_CAST(Matrix6xOut1,v_partial_dq);
      typedef typename SizeDepType<JointModel::NV>::template ColsReturn<Matrix6xOut2>::Type ColsBlockOut2;
      Matrix6xOut2 & v_partial_dv_ = PINOCCHIO_EIGEN_CONST_CAST(Matrix6xOut2,v_partial_dv);
      
      // dvec/dv
      ColsBlockOut2 v_partial_dv_cols = jmodel.jointCols(v_partial_dv_);

      switch(rf)
      {
        case WORLD:
          v_partial_dv_cols = Jcols;
          break;
        case LOCAL_WORLD_ALIGNED:
          details::translateJointJacobian(oMlast,Jcols,v_partial_dv_cols);
          break;
        case LOCAL:
          motionSet::se3ActionInverse(oMlast,Jcols,v_partial_dv_cols);
          break;
        default:
          assert(false && "This must never happened");
      }      

      // dvec/dq
      ColsBlockOut1 v_partial_dq_cols = jmodel.jointCols(v_partial_dq_);

      switch(rf)
      {
        case WORLD:
          if(parent > 0)
          {
            vtmp = data.tmp[0]*(data.ov[parent] - vlast);
            vtmp += data.oa[parent] - dvlast;
          }
          else
          {
            vtmp = -(data.tmp[0]*vlast + dvlast);
          }
          motionSet::motionAction(vtmp,Jcols,v_partial_dq_cols);
          break;
        case LOCAL_WORLD_ALIGNED:
          if(parent > 0)
          {
            vtmp = data.tmp[0]*(data.ov[parent] - vlast);
            vtmp += data.oa[parent] - dvlast;
          }
          else
          {
            vtmp = -(data.tmp[0]*vlast + dvlast);
          }
          vtmp.linear() += vtmp.angular().cross(oMlast.translation());
          motionSet::motionAction(vtmp,v_partial_dv_cols,v_partial_dq_cols);
          break;
        case LOCAL:
          if(parent > 0)
          {
            vtmp2 = data.tmp[0]*data.ov[parent] + data.oa[parent];
            vtmp = oMlast.actInv(vtmp2);
            motionSet::motionAction(vtmp,v_partial_dv_cols,v_partial_dq_cols);
          }
          break;
        default:
          assert(false && "This must never happened");
      } 
    }
  };

  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl, class ContactModelAllocator, class ContactDataAllocator,
           typename MatrixType1, typename MatrixType2, typename MatrixType3, typename MatrixType4>
  inline void computeImpulseDynamicsDerivatives(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                                                DataTpl<Scalar,Options,JointCollectionTpl> & data,
                                                const std::vector<RigidContactModelTpl<Scalar,Options>,ContactModelAllocator> & contact_models,
                                                std::vector<RigidContactDataTpl<Scalar,Options>,ContactDataAllocator> & contact_data,
                                                const Scalar r_coeff,
                                                const Scalar mu,
                                                const Eigen::MatrixBase<MatrixType1> & dvimpulse_partial_dq,
                                                const Eigen::MatrixBase<MatrixType2> & dvimpulse_partial_dv,
                                                const Eigen::MatrixBase<MatrixType3> & impulse_partial_dq,
                                                const Eigen::MatrixBase<MatrixType4> & impulse_partial_dv)
  {
    const Eigen::DenseIndex & nc = data.contact_chol.constraintDim();
    
    PINOCCHIO_CHECK_INPUT_ARGUMENT(contact_data.size() == contact_models.size(),
                                   "contact_data and contact_models do not have the same size");
    
    PINOCCHIO_CHECK_INPUT_ARGUMENT(dvimpulse_partial_dq.cols() == model.nv);
    PINOCCHIO_CHECK_INPUT_ARGUMENT(dvimpulse_partial_dq.rows() == model.nv);
    
    PINOCCHIO_CHECK_INPUT_ARGUMENT(dvimpulse_partial_dv.cols() == model.nv);
    PINOCCHIO_CHECK_INPUT_ARGUMENT(dvimpulse_partial_dv.rows() == model.nv);

    PINOCCHIO_CHECK_INPUT_ARGUMENT(impulse_partial_dq.cols() == model.nv);
    PINOCCHIO_CHECK_INPUT_ARGUMENT(impulse_partial_dq.rows() == nc);
    PINOCCHIO_CHECK_INPUT_ARGUMENT(impulse_partial_dv.cols() == model.nv);
    PINOCCHIO_CHECK_INPUT_ARGUMENT(impulse_partial_dv.rows() == nc);
    
    PINOCCHIO_CHECK_INPUT_ARGUMENT((r_coeff >= Scalar(0)) && (r_coeff <= Scalar(1)) && "mu must be positive.");
    PINOCCHIO_CHECK_INPUT_ARGUMENT(mu >= (Scalar)0 && "mu must be positive.");
    PINOCCHIO_CHECK_INPUT_ARGUMENT(model.gravity.angular().isZero(),
                                   "The gravity must be a pure force vector, no angular part");
    
    assert(model.check(data) && "data is not consistent with model.");
    data.dtau_dq.setZero();
    
    //Save r_coeff in data.tmp
    data.tmp[0] = 1 + r_coeff;
    
    typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
    typedef DataTpl<Scalar,Options,JointCollectionTpl> Data;
    
    typedef RigidContactModelTpl<Scalar,Options> RigidContactModel;
    typedef RigidContactDataTpl<Scalar,Options> RigidContactData;
    
    typedef typename ModelTpl<Scalar,Options,JointCollectionTpl>::JointIndex JointIndex;
    
    typedef ComputeContactDynamicsDerivativesForwardStep<Scalar,Options,JointCollectionTpl,false> Pass1;
    for(JointIndex i=1; i<(JointIndex) model.njoints; ++i)
    {
      Pass1::run(model.joints[i],data.joints[i],
                 typename Pass1::ArgsType(model,data));
    }
    
    // Add the contribution of the impulse. TODO: this should be done at the contact model level.
    for(size_t k = 0; k < contact_models.size(); ++k)
    {
      const RigidContactModel & cmodel = contact_models[k];
      const RigidContactData & cdata = contact_data[k];
      const SE3 & oMc = cdata.oMc1;
      const JointIndex joint1_id = cmodel.joint1_id;
      Force & of = data.of[joint1_id];
      
      switch(cmodel.reference_frame)
      {
        case LOCAL:
        {
          of -= oMc.act(cdata.contact_force);
          break;
        }
        case WORLD:
        {
          of -= cdata.contact_force;
          break;
        }
        case LOCAL_WORLD_ALIGNED:
        {
          of -= cdata.contact_force;
          of.angular().noalias() -= oMc.translation().cross(cdata.contact_force.linear());
          break;
        }
      }
    }

    typedef ComputeContactDynamicDerivativesBackwardStep<Scalar,Options,JointCollectionTpl,false> Pass2;
    for(JointIndex i=(JointIndex)(model.njoints-1); i>0; --i)
    {
      Pass2::run(model.joints[i],
                 typename Pass2::ArgsType(model,data));
    }

    Eigen::DenseIndex current_row_sol_id = 0;
    typedef typename SizeDepType<3>::template RowsReturn<typename Data::MatrixXs>::Type Rows3Block;
    typedef typename SizeDepType<6>::template RowsReturn<typename Data::MatrixXs>::Type Rows6Block;
    for(size_t k = 0; k < contact_models.size(); ++k)
    {
      const RigidContactModel & cmodel = contact_models[k];

      const typename Model::JointIndex joint1_id = cmodel.joint1_id;

      switch(cmodel.type)
      {
        case CONTACT_6D:
        {
          typedef JointImpulseVelocityDerivativesBackwardStep<Scalar,Options,JointCollectionTpl,Rows6Block,Rows6Block> Pass3;
          Rows6Block contact_dvc_dq = SizeDepType<6>::middleRows(data.dvc_dq,
                                                                current_row_sol_id);
          Rows6Block contact_dvc_dv = SizeDepType<6>::middleRows(data.dac_da,
                                                                current_row_sol_id);
          for(JointIndex i = joint1_id; i > 0; i = model.parents[i])
          {
            Pass3::run(model.joints[i],
                       typename Pass3::ArgsType(model,data,
                                                joint1_id,
                                                cmodel.reference_frame,
                                                PINOCCHIO_EIGEN_CONST_CAST(Rows6Block,contact_dvc_dq),
                                                PINOCCHIO_EIGEN_CONST_CAST(Rows6Block,contact_dvc_dv)));
          }
          // Set back ov[0] to a zero value
          data.ov[0].setZero();
          data.oa[0].setZero();
          current_row_sol_id += 6;
          break;
        }
        case CONTACT_3D:
        {
          //TODO: Specialize for the 3d case.
          //TODO: We don't need all these quantities. Remove those not needed.
          typedef JointImpulseVelocityDerivativesBackwardStep<Scalar,Options,JointCollectionTpl,typename Data::Matrix6x,typename Data::Matrix6x> Pass3;
          Rows3Block contact_dvc_dq = SizeDepType<3>::middleRows(data.dvc_dq,
                                                                current_row_sol_id);
          Rows3Block contact_dvc_dv = SizeDepType<3>::middleRows(data.dac_da,
                                                                current_row_sol_id);
          typename Data::Matrix6x & contact_dvc_dq_tmp = data.dFda;
          typename Data::Matrix6x & contact_dvc_dv_tmp= data.IS;
          for(JointIndex i = joint1_id; i > 0; i = model.parents[i])
          {
            Pass3::run(model.joints[i],
                       typename Pass3::ArgsType(model,data,
                                                joint1_id,
                                                cmodel.reference_frame,
                                                PINOCCHIO_EIGEN_CONST_CAST(typename Data::Matrix6x,contact_dvc_dq_tmp),
                                                PINOCCHIO_EIGEN_CONST_CAST(typename Data::Matrix6x,contact_dvc_dv_tmp)));
          }
          // Set back ov[0] to a zero value
          data.ov[0].setZero();
          data.oa[0].setZero();
          contact_dvc_dq = contact_dvc_dq_tmp.template topRows<3>();
          contact_dvc_dv = contact_dvc_dv_tmp.template topRows<3>();
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
    
    //Temporary: dlambda_dtau stores J*Minv
    typename Data::MatrixXs & JMinv = data.dlambda_dtau;
    typename Data::MatrixXs & Jc = data.dac_da;

    JMinv.noalias() = Jc * data.Minv;
    data.dvc_dq.noalias() -= JMinv * data.dtau_dq;

    MatrixType3 & dic_dq = PINOCCHIO_EIGEN_CONST_CAST(MatrixType3,impulse_partial_dq);
    dic_dq.noalias() = -data.osim * data.dvc_dq; //OUTPUT
    //TODO: Implem sparse.
    data.dtau_dq.noalias() -= Jc.transpose()*impulse_partial_dq;
    
    PINOCCHIO_EIGEN_CONST_CAST(MatrixType4,impulse_partial_dv).noalias() = -data.tmp[0]*data.osim*Jc;; //OUTPUT

    PINOCCHIO_EIGEN_CONST_CAST(MatrixType1,dvimpulse_partial_dq).noalias() = -data.Minv*data.dtau_dq; //OUTPUT
    PINOCCHIO_EIGEN_CONST_CAST(MatrixType2,dvimpulse_partial_dv).noalias() = JMinv.transpose()*impulse_partial_dv; //OUTPUT

    //update in world frame a(df/dt) = d(af)/dt + av X af
    
    current_row_sol_id = 0;
    for(size_t k = 0; k < contact_models.size(); ++k)
    {
      const RigidContactModel & cmodel = contact_models[k];
      const RigidContactData & cdata = contact_data[k];
      const typename Model::JointIndex joint1_id = cmodel.joint1_id;
      const int colRef = nv(model.joints[joint1_id])+idx_v(model.joints[joint1_id])-1;
      switch(cmodel.reference_frame)
      {
      case LOCAL:
        break;
      case WORLD:
      {
        const Force& of = cdata.contact_force;
        switch(cmodel.type) {
        case CONTACT_6D:
        {
          Rows6Block contact_dic_dq = SizeDepType<6>::middleRows(dic_dq, current_row_sol_id);
          for(Eigen::DenseIndex j=colRef;j>=0;j=data.parents_fromRow[(size_t)j])
          {
            typedef typename Data::Matrix6x::ColXpr ColType;
            typedef typename Rows6Block::ColXpr ColTypeOut;
            MotionRef<ColType> min(data.J.col(j));
            ForceRef<ColTypeOut> fout(contact_dic_dq.col(j));
            fout += min.cross(of);
          }
          current_row_sol_id += 6;
          break;
        }
        case CONTACT_3D:
        {
          Rows3Block contact_dic_dq = SizeDepType<3>::middleRows(dic_dq, current_row_sol_id);
          for(Eigen::DenseIndex j=colRef;j>=0;j=data.parents_fromRow[(size_t)j])
          {
            typedef typename Data::Matrix6x::ColXpr ColType;
            MotionRef<ColType> min(data.J.col(j));
            contact_dic_dq.col(j) += min.angular().cross(of.linear());
          }
          current_row_sol_id += 3;
          break;
        }
        default:
          assert(false && "must never happen");
          break;
        }
        break;
      }
      case LOCAL_WORLD_ALIGNED:
      {
        const Force& of = cdata.contact_force;
        switch(cmodel.type)
        {
        case CONTACT_6D:
        {
          Rows6Block contact_dvc_dv = SizeDepType<6>::middleRows(data.dac_da,current_row_sol_id);
          Rows6Block contact_dic_dq = SizeDepType<6>::middleRows(dic_dq, current_row_sol_id);
          for(Eigen::DenseIndex j=colRef;j>=0;j=data.parents_fromRow[(size_t)j])
          {
            typedef typename Rows6Block::ColXpr ColType;
            typedef typename Rows6Block::ColXpr ColTypeOut;
            MotionRef<ColType> min(contact_dvc_dv.col(j));
            ForceRef<ColTypeOut> fout(contact_dic_dq.col(j));
            fout.linear().noalias()  += min.angular().cross(of.linear());
            fout.angular().noalias() += min.angular().cross(of.angular());
          }
          current_row_sol_id += 6;
          break;
        }
        case CONTACT_3D:
        {
          Rows3Block contact_dic_dq = SizeDepType<3>::middleRows(dic_dq, current_row_sol_id);
          for(Eigen::DenseIndex j=colRef;j>=0;j=data.parents_fromRow[(size_t)j])
          {
            typedef typename Data::Matrix6x::ColXpr ColType;
            MotionRef<ColType> min(data.J.col(j));
            contact_dic_dq.col(j).noalias() += min.angular().cross(of.linear());
          }
          current_row_sol_id += 3;
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
    }

  }
} // namespace pinocchio

#endif // ifndef __pinocchio_algorithm_impulse_dynamics_derivatives_hxx__