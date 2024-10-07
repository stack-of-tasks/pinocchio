//
// Copyright (c) 2020 CNRS INRIA
//

#ifndef __pinocchio_algorithm_impulse_dynamics_derivatives_hxx__
#define __pinocchio_algorithm_impulse_dynamics_derivatives_hxx__

#include "pinocchio/algorithm/check.hpp"
#include "pinocchio/algorithm/rnea-derivatives.hpp"
#include "pinocchio/algorithm/kinematics-derivatives.hpp"
#include "pinocchio/algorithm/contact-cholesky.hpp"
#include "pinocchio/algorithm/constrained-dynamics-derivatives.hxx"

namespace pinocchio
{

  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    typename Matrix3xOut1,
    typename Matrix3xOut2>
  struct JointImpulseVelocityDerivativesBackwardStep3D
  : public fusion::JointUnaryVisitorBase<JointImpulseVelocityDerivativesBackwardStep3D<
      Scalar,
      Options,
      JointCollectionTpl,
      Matrix3xOut1,
      Matrix3xOut2>>
  {
    typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
    typedef DataTpl<Scalar, Options, JointCollectionTpl> Data;

    typedef boost::fusion::vector<
      const Model &,
      Data &,
      const typename Model::JointIndex &,
      const SE3Tpl<Scalar> &,
      const ReferenceFrame &,
      const Scalar &,
      Matrix3xOut1 &,
      Matrix3xOut2 &>
      ArgsType;

    template<typename JointModel>
    static void algo(
      const JointModelBase<JointModel> & jmodel,
      const Model & model,
      Data & data,
      const typename Model::JointIndex & joint_id,
      const SE3Tpl<Scalar> & placement,
      const ReferenceFrame & rf,
      const Scalar & r_coeff,
      const Eigen::MatrixBase<Matrix3xOut1> & v_partial_dq,
      const Eigen::MatrixBase<Matrix3xOut2> & v_partial_dv)
    {
      typedef typename Model::JointIndex JointIndex;
      typedef typename Data::SE3 SE3;
      typedef typename Data::Motion Motion;

      const JointIndex i = jmodel.id();
      const JointIndex parent = model.parents[i];

      const SE3 oMlast = data.oMi[joint_id] * placement;

      typedef
        typename SizeDepType<JointModel::NV>::template ColsReturn<typename Data::Matrix6x>::Type
          ColsBlock;
      ColsBlock Jcols = jmodel.jointCols(data.J);

      typedef
        typename SizeDepType<JointModel::NV>::template ColsReturn<Matrix3xOut1>::Type ColsBlockOut1;
      Matrix3xOut1 & v_partial_dq_ = PINOCCHIO_EIGEN_CONST_CAST(Matrix3xOut1, v_partial_dq);
      typedef
        typename SizeDepType<JointModel::NV>::template ColsReturn<Matrix3xOut2>::Type ColsBlockOut2;
      Matrix3xOut2 & v_partial_dv_ = PINOCCHIO_EIGEN_CONST_CAST(Matrix3xOut2, v_partial_dv);

      // dvec/dv
      const int nv = jmodel.nv();
      Eigen::Matrix<Scalar, 6, JointModel::NV, Options> v_spatial_partial_dv_cols(6, nv);
      ColsBlockOut2 v_partial_dv_cols = jmodel.jointCols(v_partial_dv_);

      motionSet::se3ActionInverse(oMlast, Jcols, v_spatial_partial_dv_cols);
      v_partial_dv_cols = v_spatial_partial_dv_cols.template middleRows<3>(Motion::LINEAR);

      // dvec/dq
      ColsBlockOut1 v_partial_dq_cols = jmodel.jointCols(v_partial_dq_);

#define FOR_NV() for (Eigen::DenseIndex j = 0; j < nv; ++j)
#define GET_LINEAR(vec6) vec6.template segment<3>(Motion::LINEAR)
#define GET_ANGULAR(vec6) vec6.template segment<3>(Motion::ANGULAR)

      const Scalar factor = Scalar(1) + r_coeff;

      if (parent > 0)
      {
        const Motion vtmp = oMlast.actInv(factor * data.ov[parent] + data.oa[parent]);
        FOR_NV()
        v_partial_dq_cols.col(j).noalias() =
          vtmp.angular().cross(GET_LINEAR(v_spatial_partial_dv_cols.col(j)))
          + vtmp.linear().cross(GET_ANGULAR(v_spatial_partial_dv_cols.col(j)));
      }
      else
        v_partial_dq_cols.setZero();

      if (rf == LOCAL_WORLD_ALIGNED)
      {
        const Motion vtmp = oMlast.actInv(factor * data.ov[joint_id] + data.oa[joint_id]);
        FOR_NV()
        v_partial_dq_cols.col(j) =
          oMlast.rotation()
          * (v_partial_dq_cols.col(j) + GET_ANGULAR(v_spatial_partial_dv_cols.col(j)).cross(vtmp.linear()));
        FOR_NV()
        v_partial_dv_cols.col(j) = oMlast.rotation() * v_partial_dv_cols.col(j);
      }

#undef FOR_NV
#undef GET_LINEAR
#undef GET_ANGULAR
    }
  };

  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    typename Matrix6xOut1,
    typename Matrix6xOut2>
  struct JointImpulseVelocityDerivativesBackwardStep6D
  : public fusion::JointUnaryVisitorBase<JointImpulseVelocityDerivativesBackwardStep6D<
      Scalar,
      Options,
      JointCollectionTpl,
      Matrix6xOut1,
      Matrix6xOut2>>
  {
    typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
    typedef DataTpl<Scalar, Options, JointCollectionTpl> Data;

    typedef boost::fusion::vector<
      const Model &,
      Data &,
      const typename Model::JointIndex &,
      const SE3Tpl<Scalar> &,
      const ReferenceFrame &,
      const Scalar &,
      Matrix6xOut1 &,
      Matrix6xOut2 &>
      ArgsType;

    template<typename JointModel>
    static void algo(
      const JointModelBase<JointModel> & jmodel,
      const Model & model,
      Data & data,
      const typename Model::JointIndex & joint_id,
      const SE3Tpl<Scalar> & placement,
      const ReferenceFrame & rf,
      const Scalar & r_coeff,
      const Eigen::MatrixBase<Matrix6xOut1> & v_partial_dq,
      const Eigen::MatrixBase<Matrix6xOut2> & v_partial_dv)
    {
      typedef typename Model::JointIndex JointIndex;
      typedef typename Data::SE3 SE3;
      typedef typename Data::Motion Motion;

      const JointIndex i = jmodel.id();
      const JointIndex parent = model.parents[i];
      Motion vtmp; // Temporary variables

      const SE3 oMlast = data.oMi[joint_id] * placement;
      const Motion & v_last = data.ov[joint_id];
      const Motion & dv_last = data.oa[joint_id];

      typedef
        typename SizeDepType<JointModel::NV>::template ColsReturn<typename Data::Matrix6x>::Type
          ColsBlock;
      ColsBlock Jcols = jmodel.jointCols(data.J);

      typedef
        typename SizeDepType<JointModel::NV>::template ColsReturn<Matrix6xOut1>::Type ColsBlockOut1;
      Matrix6xOut1 & v_partial_dq_ = PINOCCHIO_EIGEN_CONST_CAST(Matrix6xOut1, v_partial_dq);
      typedef
        typename SizeDepType<JointModel::NV>::template ColsReturn<Matrix6xOut2>::Type ColsBlockOut2;
      Matrix6xOut2 & v_partial_dv_ = PINOCCHIO_EIGEN_CONST_CAST(Matrix6xOut2, v_partial_dv);

      // dvec/dv
      ColsBlockOut2 v_partial_dv_cols = jmodel.jointCols(v_partial_dv_);

      switch (rf)
      {
      case LOCAL_WORLD_ALIGNED:
        details::translateJointJacobian(oMlast, Jcols, v_partial_dv_cols);
        break;
      case LOCAL:
        motionSet::se3ActionInverse(oMlast, Jcols, v_partial_dv_cols);
        break;
      default:
        assert(false && "This must never happened");
      }

      // dvec/dq
      ColsBlockOut1 v_partial_dq_cols = jmodel.jointCols(v_partial_dq_);
      const Scalar factor = Scalar(1) + r_coeff;
      switch (rf)
      {
      case LOCAL_WORLD_ALIGNED:
        if (parent > 0)
        {
          vtmp = factor * (data.ov[parent] - v_last);
          vtmp += data.oa[parent] - dv_last;
        }
        else
        {
          vtmp = -(factor * v_last + dv_last);
        }
        vtmp.linear() += vtmp.angular().cross(oMlast.translation());
        motionSet::motionAction(vtmp, v_partial_dv_cols, v_partial_dq_cols);
        break;
      case LOCAL:
        if (parent > 0)
        {
          vtmp = oMlast.actInv(factor * data.ov[parent] + data.oa[parent]);
          motionSet::motionAction(vtmp, v_partial_dv_cols, v_partial_dq_cols);
        }
        break;
      default:
        assert(false && "This must never happened");
      }
    }
  };

  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    class ConstraintModelAllocator,
    class ConstraintDataAllocator,
    typename MatrixType1,
    typename MatrixType2,
    typename MatrixType3,
    typename MatrixType4>
  inline void computeImpulseDynamicsDerivatives(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const std::vector<RigidConstraintModelTpl<Scalar, Options>, ConstraintModelAllocator> &
      contact_models,
    std::vector<RigidConstraintDataTpl<Scalar, Options>, ConstraintDataAllocator> & contact_data,
    const Scalar r_coeff,
    const ProximalSettingsTpl<Scalar> & settings,
    const Eigen::MatrixBase<MatrixType1> & dvimpulse_partial_dq,
    const Eigen::MatrixBase<MatrixType2> & dvimpulse_partial_dv,
    const Eigen::MatrixBase<MatrixType3> & impulse_partial_dq,
    const Eigen::MatrixBase<MatrixType4> & impulse_partial_dv)
  {
    const Eigen::DenseIndex nc = data.contact_chol.constraintDim();

    PINOCCHIO_CHECK_INPUT_ARGUMENT(
      contact_data.size() == contact_models.size(),
      "contact_data and contact_models do not have the same size");

    PINOCCHIO_CHECK_INPUT_ARGUMENT(dvimpulse_partial_dq.cols() == model.nv);
    PINOCCHIO_CHECK_INPUT_ARGUMENT(dvimpulse_partial_dq.rows() == model.nv);

    PINOCCHIO_CHECK_INPUT_ARGUMENT(dvimpulse_partial_dv.cols() == model.nv);
    PINOCCHIO_CHECK_INPUT_ARGUMENT(dvimpulse_partial_dv.rows() == model.nv);

    PINOCCHIO_CHECK_INPUT_ARGUMENT(impulse_partial_dq.cols() == model.nv);
    PINOCCHIO_CHECK_INPUT_ARGUMENT(impulse_partial_dq.rows() == nc);

    PINOCCHIO_CHECK_INPUT_ARGUMENT(impulse_partial_dv.cols() == model.nv);
    PINOCCHIO_CHECK_INPUT_ARGUMENT(impulse_partial_dv.rows() == nc);

    PINOCCHIO_CHECK_INPUT_ARGUMENT(
      check_expression_if_real<Scalar>(
        model.gravity.angular()[0] == Scalar(0) && model.gravity.angular()[1] == Scalar(0)
        && model.gravity.angular()[2] == Scalar(0)),
      "The gravity must be a pure force vector, no angular part");

    PINOCCHIO_CHECK_INPUT_ARGUMENT(
      check_expression_if_real<Scalar>((r_coeff >= Scalar(0)) && (r_coeff <= Scalar(1)))
      && "coeff of restitution lies between 0 and 1.");
    PINOCCHIO_CHECK_INPUT_ARGUMENT(
      check_expression_if_real<Scalar>(settings.mu >= (Scalar(0))) && "mu must be positive.");

    assert(model.check(data) && "data is not consistent with model.");

    // TODO: User should make sure the internal quantities are reset.
    data.dtau_dq.setZero();

    typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
    typedef DataTpl<Scalar, Options, JointCollectionTpl> Data;
    typedef typename Data::Force Force;

    typedef RigidConstraintModelTpl<Scalar, Options> RigidConstraintModel;
    typedef RigidConstraintDataTpl<Scalar, Options> RigidConstraintData;

    typedef typename ModelTpl<Scalar, Options, JointCollectionTpl>::JointIndex JointIndex;

    typedef ComputeConstraintDynamicsDerivativesForwardStep<
      Scalar, Options, JointCollectionTpl, false>
      Pass1;
    for (JointIndex i = 1; i < (JointIndex)model.njoints; ++i)
    {
      Pass1::run(model.joints[i], data.joints[i], typename Pass1::ArgsType(model, data));
    }

    internal::ContactForceContribution<Scalar>::run(contact_models, data, contact_data);

    typedef ComputeContactDynamicDerivativesBackwardStep<Scalar, Options, JointCollectionTpl, false>
      Pass2;
    for (JointIndex i = (JointIndex)(model.njoints - 1); i > 0; --i)
    {
      Pass2::run(model.joints[i], typename Pass2::ArgsType(model, data));
    }

    Eigen::DenseIndex current_row_sol_id = 0;
    typedef typename SizeDepType<3>::template RowsReturn<typename Data::MatrixXs>::Type Rows3Block;
    typedef typename SizeDepType<6>::template RowsReturn<typename Data::MatrixXs>::Type Rows6Block;
    for (size_t k = 0; k < contact_models.size(); ++k)
    {
      const RigidConstraintModel & cmodel = contact_models[k];

      const typename Model::JointIndex joint1_id = cmodel.joint1_id;

      switch (cmodel.type)
      {
      case CONTACT_6D: {
        typedef JointImpulseVelocityDerivativesBackwardStep6D<
          Scalar, Options, JointCollectionTpl, Rows6Block, Rows6Block>
          Pass3;
        Rows6Block contact_dvc_dq = SizeDepType<6>::middleRows(data.dvc_dq, current_row_sol_id);
        Rows6Block contact_dvc_dv = SizeDepType<6>::middleRows(data.dac_da, current_row_sol_id);
        for (JointIndex i = joint1_id; i > 0; i = model.parents[i])
        {
          Pass3::run(
            model.joints[i],
            typename Pass3::ArgsType(
              model, data, joint1_id, cmodel.joint1_placement, cmodel.reference_frame, r_coeff,
              PINOCCHIO_EIGEN_CONST_CAST(Rows6Block, contact_dvc_dq),
              PINOCCHIO_EIGEN_CONST_CAST(Rows6Block, contact_dvc_dv)));
        }

        break;
      }
      case CONTACT_3D: {
        typedef JointImpulseVelocityDerivativesBackwardStep3D<
          Scalar, Options, JointCollectionTpl, Rows3Block, Rows3Block>
          Pass3;
        Rows3Block contact_dvc_dq = SizeDepType<3>::middleRows(data.dvc_dq, current_row_sol_id);
        Rows3Block contact_dvc_dv = SizeDepType<3>::middleRows(data.dac_da, current_row_sol_id);
        for (JointIndex i = joint1_id; i > 0; i = model.parents[i])
        {
          Pass3::run(
            model.joints[i], typename Pass3::ArgsType(
                               model, data, joint1_id, cmodel.joint1_placement,
                               cmodel.reference_frame, r_coeff, contact_dvc_dq, contact_dvc_dv));
        }
        break;
      }
      default:
        assert(false && "must never happen");
        break;
      }
      current_row_sol_id += cmodel.size();
    }

    data.contact_chol.getOperationalSpaceInertiaMatrix(data.osim);
    data.contact_chol.getInverseMassMatrix(data.Minv);
    // Temporary: dlambda_dtau stores J*Minv
    typename Data::MatrixXs & JMinv = data.dlambda_dtau;
    typename Data::MatrixXs & Jc = data.dac_da;

    JMinv.noalias() = Jc * data.Minv;
    data.dvc_dq.noalias() -= JMinv * data.dtau_dq;

    MatrixType3 & dic_dq = PINOCCHIO_EIGEN_CONST_CAST(MatrixType3, impulse_partial_dq);
    dic_dq.noalias() = -data.osim * data.dvc_dq; // OUTPUT

    // TODO: Implem sparse.
    data.dtau_dq.noalias() -= Jc.transpose() * impulse_partial_dq;

    PINOCCHIO_EIGEN_CONST_CAST(MatrixType4, impulse_partial_dv).noalias() =
      -(Scalar(1) + r_coeff) * data.osim * Jc;
    ; // OUTPUT

    PINOCCHIO_EIGEN_CONST_CAST(MatrixType1, dvimpulse_partial_dq).noalias() =
      -data.Minv * data.dtau_dq; // OUTPUT
    PINOCCHIO_EIGEN_CONST_CAST(MatrixType2, dvimpulse_partial_dv).noalias() =
      JMinv.transpose() * impulse_partial_dv; // OUTPUT

    // update in world frame a(df/dt) = d(af)/dt + av X af

    current_row_sol_id = 0;
    for (size_t k = 0; k < contact_models.size(); ++k)
    {
      const RigidConstraintModel & cmodel = contact_models[k];
      const RigidConstraintData & cdata = contact_data[k];
      const typename Model::JointIndex joint1_id = cmodel.joint1_id;
      const int colRef = nv(model.joints[joint1_id]) + idx_v(model.joints[joint1_id]) - 1;
      switch (cmodel.reference_frame)
      {
      case LOCAL:
        break;
      case LOCAL_WORLD_ALIGNED: {
        const Force & of = cdata.contact_force;
        switch (cmodel.type)
        {
        case CONTACT_6D: {
          Rows6Block contact_dvc_dv = SizeDepType<6>::middleRows(data.dac_da, current_row_sol_id);
          Rows6Block contact_dic_dq = SizeDepType<6>::middleRows(dic_dq, current_row_sol_id);
          for (Eigen::DenseIndex j = colRef; j >= 0; j = data.parents_fromRow[(size_t)j])
          {
            typedef typename Rows6Block::ColXpr ColType;
            typedef typename Rows6Block::ColXpr ColTypeOut;
            MotionRef<ColType> min(contact_dvc_dv.col(j));
            ForceRef<ColTypeOut> fout(contact_dic_dq.col(j));
            fout.linear().noalias() += min.angular().cross(of.linear());
            fout.angular().noalias() += min.angular().cross(of.angular());
          }
          break;
        }
        case CONTACT_3D: {
          Rows3Block contact_dic_dq = SizeDepType<3>::middleRows(dic_dq, current_row_sol_id);
          for (Eigen::DenseIndex j = colRef; j >= 0; j = data.parents_fromRow[(size_t)j])
          {
            typedef typename Data::Matrix6x::ColXpr ColType;
            MotionRef<ColType> min(data.J.col(j));
            contact_dic_dq.col(j).noalias() += min.angular().cross(of.linear());
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

#endif // ifndef __pinocchio_algorithm_impulse_dynamics_derivatives_hxx__
