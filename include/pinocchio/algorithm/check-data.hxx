//
// Copyright (c) 2025 INRIA
//

#ifndef __pinocchio_algorithm_check_data_hxx__
#define __pinocchio_algorithm_check_data_hxx__

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"

namespace pinocchio
{

  template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
  inline bool checkData(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    const DataTpl<Scalar, Options, JointCollectionTpl> & data)
  {
    typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
    typedef DataTpl<Scalar, Options, JointCollectionTpl> Data;

    typedef typename Model::JointModel JointModel;

#define CHECK_DATA(a)                                                                              \
  if (!(a))                                                                                        \
    return false;

    // TODO JMinvJt,sDUiJt are never explicitly initialized.
    // TODO impulse_c
    // They are not check neither

    CHECK_DATA((int)data.joints.size() == model.njoints);
    CHECK_DATA((int)data.a.size() == model.njoints);
    CHECK_DATA((int)data.a_gf.size() == model.njoints);
    CHECK_DATA((int)data.v.size() == model.njoints);
    CHECK_DATA((int)data.f.size() == model.njoints);
    CHECK_DATA((int)data.oMi.size() == model.njoints);
    CHECK_DATA((int)data.liMi.size() == model.njoints);
    CHECK_DATA((int)data.Ycrb.size() == model.njoints);
    CHECK_DATA((int)data.Yaba.size() == model.njoints);
    CHECK_DATA((int)data.Fcrb.size() == model.njoints);
    for (const typename Data::Matrix6x & F : data.Fcrb)
    {
      CHECK_DATA(F.cols() == model.nv);
    }
    CHECK_DATA((int)data.iMf.size() == model.njoints);
    CHECK_DATA((int)data.iMf.size() == model.njoints);
    CHECK_DATA((int)data.com.size() == model.njoints);
    CHECK_DATA((int)data.vcom.size() == model.njoints);
    CHECK_DATA((int)data.acom.size() == model.njoints);
    CHECK_DATA((int)data.mass.size() == model.njoints);

    CHECK_DATA(data.tau.size() == model.nv);
    CHECK_DATA(data.nle.size() == model.nv);
    CHECK_DATA(data.ddq.size() == model.nv);
    CHECK_DATA(data.u.size() == model.nv);
    CHECK_DATA(data.M.rows() == model.nv);
    CHECK_DATA(data.M.cols() == model.nv);
    CHECK_DATA(data.Ag.cols() == model.nv);
    CHECK_DATA(data.U.cols() == model.nv);
    CHECK_DATA(data.U.rows() == model.nv);
    CHECK_DATA(data.D.size() == model.nv);
    CHECK_DATA(data.tmp.size() >= model.nv);
    CHECK_DATA(data.J.cols() == model.nvExtended);
    CHECK_DATA(data.Jcom.cols() == model.nv);
    CHECK_DATA(data.torque_residual.size() == model.nv);
    CHECK_DATA(data.dq_after.size() == model.nv);
    // CHECK_DATA( data.impulse_c.size()== model.nv );

    CHECK_DATA(data.kinematic_hessians.dimension(0) == 6);
#if EIGEN_VERSION_AT_LEAST(3, 2, 90) && !EIGEN_VERSION_AT_LEAST(3, 2, 93)
    CHECK_DATA(data.kinematic_hessians.dimension(1) == std::max(1, model.nv));
    CHECK_DATA(data.kinematic_hessians.dimension(2) == std::max(1, model.nv));
#else
    CHECK_DATA(data.kinematic_hessians.dimension(1) == model.nv);
    CHECK_DATA(data.kinematic_hessians.dimension(2) == model.nv);
#endif

    CHECK_DATA((int)data.oMf.size() == model.nframes);

    CHECK_DATA((int)data.lastChild.size() == model.njoints);
    CHECK_DATA((int)data.nvSubtree.size() == model.njoints);
    CHECK_DATA((int)data.parents_fromRow.size() == model.nvExtended);
    CHECK_DATA((int)data.idx_v_extended_fromRow.size() == model.nvExtended);
    CHECK_DATA((int)data.nvSubtree_fromRow.size() == model.nvExtended);

    for (JointIndex joint_id = 1; joint_id < (JointIndex)model.njoints; ++joint_id)
    {
      const typename Model::JointModel & jmodel = model.joints[joint_id];

      CHECK_DATA(model.nqs[joint_id] == jmodel.nq());
      CHECK_DATA(model.idx_qs[joint_id] == jmodel.idx_q());
      CHECK_DATA(model.nvs[joint_id] == jmodel.nv());
      CHECK_DATA(model.idx_vs[joint_id] == jmodel.idx_v());
      CHECK_DATA(model.nvExtendeds[joint_id] == jmodel.nvExtended());
      CHECK_DATA(model.idx_vExtendeds[joint_id] == jmodel.idx_vExtended());
    }

    for (JointIndex j = 1; int(j) < model.njoints; ++j)
    {
      // do not check mimic joint, because nv = 0, but it has a subtree
      if (boost::get<JointModelMimicTpl<Scalar, Options, JointCollectionTpl>>(&model.joints[j]))
        continue;
      JointIndex c = (JointIndex)data.lastChild[j];
      CHECK_DATA((int)c < model.njoints);
      int nv = model.joints[j].nv();
      for (JointIndex d = j + 1; d <= c; ++d) // explore all descendant
      {
        CHECK_DATA(model.parents[d] >= j);
        nv += model.joints[d].nv();
      }
      CHECK_DATA(nv == data.nvSubtree[j]);

      for (JointIndex d = c + 1; (int)d < model.njoints; ++d)
        CHECK_DATA((model.parents[d] < j) || (model.parents[d] > c));

      CHECK_DATA(
        data.nvSubtree[j] == data.nvSubtree_fromRow[(size_t)model.joints[j].idx_vExtended()]);

      int row = model.joints[j].idx_vExtended();
      const JointModel & jparent = model.joints[model.parents[j]];
      if (row == 0)
      {
        CHECK_DATA(data.parents_fromRow[(size_t)row] == -1);
      }
      else
      {
        CHECK_DATA(
          jparent.idx_vExtended() + jparent.nvExtended() - 1 == data.parents_fromRow[(size_t)row]);
      }
    }

#undef CHECK_DATA
    return true;
  }

  template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
  inline bool ModelTpl<Scalar, Options, JointCollectionTpl>::check(
    const DataTpl<Scalar, Options, JointCollectionTpl> & data) const
  {
    return checkData(*this, data);
  }

} // namespace pinocchio

#endif // __pinocchio_algorithm_check_data_hxx__
