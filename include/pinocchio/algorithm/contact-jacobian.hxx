//
// Copyright (c) 2021 INRIA
//

#ifndef __pinocchio_algorithm_contact_jacobian_hxx__
#define __pinocchio_algorithm_contact_jacobian_hxx__

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"

namespace pinocchio
{

  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    typename Matrix6or3Like>
  void getConstraintJacobian(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    const DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const RigidConstraintModelTpl<Scalar, Options> & constraint_model,
    RigidConstraintDataTpl<Scalar, Options> & constraint_data,
    const Eigen::MatrixBase<Matrix6or3Like> & J_)
  {
    assert(model.check(data) && "data is not consistent with model.");
    PINOCCHIO_CHECK_ARGUMENT_SIZE(J_.rows(), constraint_model.size());
    PINOCCHIO_CHECK_ARGUMENT_SIZE(J_.cols(), model.nv);

    typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
    typedef typename Model::Motion Motion;
    typedef typename Model::SE3 SE3;
    typedef DataTpl<Scalar, Options, JointCollectionTpl> Data;

    Matrix6or3Like & J = J_.const_cast_derived();

    typedef RigidConstraintModelTpl<Scalar, Options> ConstraintModel;
    const typename ConstraintModel::BooleanVector & colwise_joint1_sparsity =
      constraint_model.colwise_joint1_sparsity;
    const typename ConstraintModel::BooleanVector & colwise_joint2_sparsity =
      constraint_model.colwise_joint2_sparsity;
    const typename ConstraintModel::IndexVector & colwise_span_indexes =
      constraint_model.colwise_span_indexes;

    SE3 & oMc1 = constraint_data.oMc1;
    oMc1 = data.oMi[constraint_model.joint1_id] * constraint_model.joint1_placement;
    SE3 & oMc2 = constraint_data.oMc2;
    oMc2 = data.oMi[constraint_model.joint2_id] * constraint_model.joint2_placement;
    SE3 & c1Mc2 = constraint_data.c1Mc2;
    c1Mc2 = oMc1.actInv(oMc2);

    for (size_t k = 0; k < colwise_span_indexes.size(); ++k)
    {
      const Eigen::DenseIndex col_id = colwise_span_indexes[k];

      const int sign = colwise_joint1_sparsity[col_id] != colwise_joint2_sparsity[col_id]
                         ? colwise_joint1_sparsity[col_id] ? +1 : -1
                         : 0; // specific case for CONTACT_3D

      typedef typename Data::Matrix6x::ConstColXpr ColXprIn;
      const ColXprIn Jcol_in = data.J.col(col_id);
      const MotionRef<const ColXprIn> Jcol_motion_in(Jcol_in);

      typedef typename Matrix6or3Like::ColXpr ColXprOut;
      ColXprOut Jcol_out = J.col(col_id);

      switch (constraint_model.type)
      {
      case CONTACT_3D: {
        switch (constraint_model.reference_frame)
        {
        case WORLD: {
          Jcol_out.noalias() = Jcol_motion_in.linear() * Scalar(sign);
          break;
        }
        case LOCAL: {
          if (sign == 0)
          {
            const Motion Jcol_local1(oMc1.actInv(Jcol_motion_in)); // TODO: simplify computations
            const Motion Jcol_local2(oMc2.actInv(Jcol_motion_in)); // TODO: simplify computations
            Jcol_out.noalias() = Jcol_local1.linear() - c1Mc2.rotation() * Jcol_local2.linear();
          }
          else if (sign == 1)
          {
            const Motion Jcol_local(oMc1.actInv(Jcol_motion_in));
            Jcol_out.noalias() = Jcol_local.linear();
          }
          else // sign == -1
          {
            const Motion Jcol_local(oMc2.actInv(Jcol_motion_in)); // TODO: simplify computations
            Jcol_out.noalias() =
              -c1Mc2.rotation() * Jcol_local.linear(); // TODO: simplify computations
          }
          break;
        }
        case LOCAL_WORLD_ALIGNED: {
          if (sign == 0)
          {
            Jcol_out.noalias() =
              (oMc2.translation() - oMc1.translation()).cross(Jcol_motion_in.angular());
          }
          else
          {
            if (sign == 1)
              Jcol_out.noalias() =
                Jcol_motion_in.linear() - oMc1.translation().cross(Jcol_motion_in.angular());
            else
              Jcol_out.noalias() =
                -Jcol_motion_in.linear() + oMc2.translation().cross(Jcol_motion_in.angular());
          }
          break;
        }
        }
        break;
      }
      case CONTACT_6D: {
        MotionRef<ColXprOut> Jcol_motion_out(Jcol_out);
        assert(check_expression_if_real<Scalar>(sign != 0) && "sign should be equal to +1 or -1.");
        switch (constraint_model.reference_frame)
        {
        case WORLD: {
          Jcol_motion_out = Jcol_motion_in;
          break;
        }
        case LOCAL: {
          Jcol_motion_out = Scalar(sign) * oMc1.actInv(Jcol_motion_in);
          break;
        }
        case LOCAL_WORLD_ALIGNED: {
          Motion Jcol_local_world_aligned(Jcol_motion_in);
          Jcol_local_world_aligned.linear() -=
            oMc1.translation().cross(Jcol_local_world_aligned.angular());
          Jcol_motion_out = Scalar(sign) * Jcol_local_world_aligned;
          break;
        }
        }
        break;
      }

      default:
        break;
      }
    }
  }

  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    typename DynamicMatrixLike,
    class ConstraintModelAllocator,
    class ConstraintDataAllocator>
  void getConstraintsJacobian(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    const DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const std::vector<RigidConstraintModelTpl<Scalar, Options>, ConstraintModelAllocator> &
      constraint_models,
    std::vector<RigidConstraintDataTpl<Scalar, Options>, ConstraintDataAllocator> &
      constraint_datas,
    const Eigen::MatrixBase<DynamicMatrixLike> & J_)
  {
    typedef RigidConstraintModelTpl<Scalar, Options> ContraintModel;
    typedef RigidConstraintDataTpl<Scalar, Options> ContraintData;

    const Eigen::DenseIndex constraint_size = getTotalConstraintSize(constraint_models);
    PINOCCHIO_CHECK_ARGUMENT_SIZE(J_.rows(), constraint_size);
    PINOCCHIO_CHECK_ARGUMENT_SIZE(J_.cols(), model.nv);

    DynamicMatrixLike & J = J_.const_cast_derived();
    Eigen::DenseIndex row_id = 0;
    for (size_t k = 0; k < constraint_models.size(); ++k)
    {
      const ContraintModel & cmodel = constraint_models[k];
      ContraintData & cdata = constraint_datas[k];

      getConstraintJacobian(model, data, cmodel, cdata, J.middleRows(row_id, cmodel.size()));

      row_id += cmodel.size();
    }
  }

} // namespace pinocchio

#endif // ifndef __pinocchio_algorithm_contact_jacobian_hxx__
