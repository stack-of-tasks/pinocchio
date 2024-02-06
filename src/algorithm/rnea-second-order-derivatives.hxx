//
// Copyright (c) 2017-2020 CNRS INRIA

#ifndef __pinocchio_algorithm_rnea_second_order_derivatives_hxx__
#define __pinocchio_algorithm_rnea_second_order_derivatives_hxx__

#include "pinocchio/algorithm/check.hpp"
#include "pinocchio/multibody/visitor.hpp"

namespace pinocchio {

template <typename Scalar, int Options,
          template <typename, int> class JointCollectionTpl,
          typename ConfigVectorType, typename TangentVectorType1,
          typename TangentVectorType2>
struct ComputeRNEASecondOrderDerivativesForwardStep
    : public fusion::JointUnaryVisitorBase<ComputeRNEASecondOrderDerivativesForwardStep<
          Scalar, Options, JointCollectionTpl, ConfigVectorType,
          TangentVectorType1, TangentVectorType2>> {
  typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
  typedef DataTpl<Scalar, Options, JointCollectionTpl> Data;

  typedef boost::fusion::vector<const Model &, Data &, const ConfigVectorType &,
                                const TangentVectorType1 &,
                                const TangentVectorType2 &>
      ArgsType;

  template <typename JointModel>
  static void algo(const JointModelBase<JointModel> &jmodel,
                   JointDataBase<typename JointModel::JointDataDerived> &jdata,
                   const Model &model, Data &data,
                   const Eigen::MatrixBase<ConfigVectorType> &q,
                   const Eigen::MatrixBase<TangentVectorType1> &v,
                   const Eigen::MatrixBase<TangentVectorType2> &a) {
    typedef typename Model::JointIndex JointIndex;
    typedef typename Data::Motion Motion;
    typedef typename Data::Inertia Inertia;

    const JointIndex i = jmodel.id();
    const JointIndex parent = model.parents[i];
    Motion &ov = data.ov[i];
    Motion &oa = data.oa[i];
    Motion &vJ = data.v[i];

    jmodel.calc(jdata.derived(), q.derived(), v.derived());

    data.liMi[i] = model.jointPlacements[i] * jdata.M();
    if (parent > 0) {
      data.oMi[i] = data.oMi[parent] * data.liMi[i];
      ov = data.ov[parent];
      oa = data.oa[parent];
    } else {
      data.oMi[i] = data.liMi[i];
      ov.setZero();
      oa = -model.gravity;
    }

    typedef typename SizeDepType<JointModel::NV>::template ColsReturn<
        typename Data::Matrix6x>::Type ColsBlock;
    ColsBlock J_cols = jmodel.jointCols(
        data.J); // data.J has all the phi (in world frame) stacked in columns
    ColsBlock psid_cols =
        jmodel.jointCols(data.psid); // psid_cols is the psi_dot in world frame
    ColsBlock psidd_cols = jmodel.jointCols(
        data.psidd); // psidd_cols is the psi_dotdot in world frame
    ColsBlock dJ_cols =
        jmodel.jointCols(data.dJ); // This here is phi_dot in world frame

    J_cols.noalias() = data.oMi[i].act(
        jdata.S()); // J_cols is just the phi in world frame for a joint
    vJ = data.oMi[i].act(jdata.v());
    motionSet::motionAction(
        ov, J_cols, psid_cols); // This ov here is v(p(i)), psi_dot calcs
    motionSet::motionAction(
        oa, J_cols, psidd_cols); // This oa here is a(p(i)) , psi_dotdot calcs
    motionSet::motionAction<ADDTO>(
        ov, psid_cols,
        psidd_cols); // This ov here is v(p(i)) , psi_dotdot calcs
    ov += vJ;
    oa += (ov ^ vJ) +
          data.oMi[i].act(jdata.S() * jmodel.jointVelocitySelector(a) +
                          jdata.c());
    motionSet::motionAction(
        ov, J_cols, dJ_cols); // This here is phi_dot, here ov used is v(p(i)) +
                              // vJ Composite rigid body inertia
    Inertia &oY = data.oYcrb[i];

    oY = data.oMi[i].act(model.inertias[i]);
    data.oh[i] = oY * ov;

    data.of[i] = oY * oa + oY.vxiv(ov); // f_i in world frame

    data.doYcrb[i] = oY.variation(ov);
    addForceCrossMatrix(data.oh[i], data.doYcrb[i]); // BC{i}
  }
  template <typename ForceDerived, typename M6>
  static void addForceCrossMatrix(const ForceDense<ForceDerived> &f,
                                  const Eigen::MatrixBase<M6> &mout) {
    M6 &mout_ = PINOCCHIO_EIGEN_CONST_CAST(M6, mout);
    addSkew(-f.linear(), mout_.template block<3, 3>(ForceDerived::LINEAR,
                                                    ForceDerived::ANGULAR));
    addSkew(-f.linear(), mout_.template block<3, 3>(ForceDerived::ANGULAR,
                                                    ForceDerived::LINEAR));
    addSkew(-f.angular(), mout_.template block<3, 3>(ForceDerived::ANGULAR,
                                                     ForceDerived::ANGULAR));
  }
};

template <typename Scalar, int Options,
          template <typename, int> class JointCollectionTpl,
          typename Tensor1, typename Tensor2, typename Tensor3,
          typename Tensor4>
struct ComputeRNEASecondOrderDerivativesBackwardStep
    : public fusion::JointUnaryVisitorBase<ComputeRNEASecondOrderDerivativesBackwardStep<
          Scalar, Options, JointCollectionTpl, Tensor1, Tensor2,
          Tensor3, Tensor4>> {
  typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
  typedef DataTpl<Scalar, Options, JointCollectionTpl> Data;

  typedef boost::fusion::vector<const Model &, Data &, const Tensor1 &,
                                const Tensor2 &, const Tensor3 &,
                                const Tensor4 &>
      ArgsType;

  template <typename JointModel>
  static void algo(const JointModelBase<JointModel> &jmodel, const Model &model,
                   Data &data, const Tensor1 &d2tau_dqdq,
                   const Tensor2 &d2tau_dvdv, const Tensor3 &dtau_dqdv,
                   const Tensor3 &dtau_dadq) {
    typedef typename Data::Motion Motion;
    typedef typename Data::Force Force;
    typedef typename Data::Inertia Inertia;
    typedef typename Model::JointIndex JointIndex;
    typedef typename Motion::ActionMatrixType ActionMatrixType;
    typedef typename Data::Matrix6 Matrix6;
    typedef typename Data::Vector6r Vector6r;
    typedef typename Data::Vector6c Vector6c;

    const JointIndex i = jmodel.id();
    const JointIndex parent = model.parents[i];

    const Inertia &oYcrb = data.oYcrb[i];  // IC{i}
    const Matrix6 &oBcrb = data.doYcrb[i]; // BC{i}

    Tensor1 &d2tau_dqdq_ = const_cast<Tensor1 &>(d2tau_dqdq);
    Tensor2 &d2tau_dvdv_ = const_cast<Tensor2 &>(d2tau_dvdv);
    Tensor3 &dtau_dqdv_ = const_cast<Tensor3 &>(dtau_dqdv);
    Tensor4 &dtau_dadq_ = const_cast<Tensor4 &>(dtau_dadq);

    Vector6r u1;
    Vector6r u2;
    Vector6c u3;
    Vector6c u4;
    Vector6c u5;
    Vector6c u6;
    Vector6c u7;
    Vector6c u8;
    Vector6c u9;
    Vector6c u10;
    Vector6r u11;
    Vector6r u12;
    Vector6c u13;

    Matrix6 Bicphii;
    Matrix6 oBicpsidot;

    Scalar p1, p2, p3, p4, p5, p6;

    Matrix6 r0, r1, r2, r3, r4, r5, r6, r7;

    for (int p = 0; p < model.nvs[i]; p++) {
      const Eigen::DenseIndex ip = model.idx_vs[i] + p;

      const MotionRef<typename Data::Matrix6x::ColXpr> S_i = data.J.col(ip);          // S{i}(:,p)
      const ActionMatrixType S_iA = S_i.toActionMatrix(); //(S{i}(:,p) )x matrix
      const MotionRef<typename Data::Matrix6x::ColXpr> psid_dm = data.psid.col(ip);   // psi_dot for p DOF
      const MotionRef<typename Data::Matrix6x::ColXpr> psidd_dm = data.psidd.col(ip); // psi_ddot for p DOF
      const MotionRef<typename Data::Matrix6x::ColXpr> phid_dm = data.dJ.col(ip);     // phi_dot for p DOF

      r1 = Bicphii = oYcrb.variation(S_i);  // S{i}(p)x*IC{i} - IC{i} S{i}(p)x
      oBicpsidot = oYcrb.variation(psid_dm); // new Bicpsidot in world frame

      Force f_tmp = oYcrb * S_i; // IC{i}S{i}(:,p)
      ForceCrossMatrix(f_tmp, r0);                   // cmf_bar(IC{i}S{i}(:,p))
      Bicphii += r0;

      f_tmp = oYcrb * psid_dm; // IC{i}S{i}(:,p)
      addForceCrossMatrix(f_tmp, oBicpsidot); // cmf_bar(IC{i}S{i}(:,p))

      r2.noalias() = 2 * r0 - Bicphii;

      r3.noalias() =
      oBicpsidot - S_iA.transpose() * oBcrb -
          oBcrb * S_iA; // Bicpsidot + S{i}(p)x*BC{i}- BC {i}S{i}(p)x

      // r4
      f_tmp.toVector().noalias() = oBcrb.transpose() * S_i.toVector();
      ForceCrossMatrix(f_tmp, r4); // cmf_bar(BC{i}.'S{i}(:,p))
      // r5
      f_tmp.toVector().noalias() = oBcrb * psid_dm.toVector();
      f_tmp += S_i.cross(data.of[i]);
      motionSet::inertiaAction<ADDTO>(oYcrb, psidd_dm.toVector(), f_tmp.toVector());
      ForceCrossMatrix(
          f_tmp,
          r5); //  cmf_bar(BC{i}psi_dot{i}(:,p)+IC{i}psi_ddot{i}(:,p)+S{i}(:,p)x*f{i})

      // S{i}(:,p)x* IC{i} + r0
      r6 = r0 + oYcrb.vxi(S_i);

      // r7
      f_tmp.toVector().noalias() = oBcrb * S_i.toVector();
      f_tmp += oYcrb * (psid_dm + phid_dm);
      ForceCrossMatrix(f_tmp, r7); // cmf_bar(BC{i}S{i}(:,p) +
                                  // IC{i}(psi_dot{i}(:,p)+phi_dot{i}(:,p)))

      JointIndex j = i;

      while (j > 0) {

        for (int q = 0; q < model.nvs[j]; q++) {
          const Eigen::DenseIndex jq = model.idx_vs[j] + q;

          const MotionRef<typename Data::Matrix6x::ColXpr> S_j = data.J.col(jq);
          const MotionRef<typename Data::Matrix6x::ColXpr> psid_dm = data.psid.col(jq);   // psi_dot{j}(:,q)
          const MotionRef<typename Data::Matrix6x::ColXpr> psidd_dm = data.psidd.col(jq); // psi_ddot{j}(:,q)
          const MotionRef<typename Data::Matrix6x::ColXpr> phid_dm = data.dJ.col(jq);     // phi_dot{j}(:,q)

          u1.noalias() = S_j.toVector().transpose() * r3;
          u2.noalias() = S_j.toVector().transpose() * r1;
          u3.noalias() = r3 * psid_dm.toVector() + r1 * psidd_dm.toVector() + r5 * S_j.toVector();
          u4.noalias()  = r6 * S_j.toVector();
          u5.noalias()  = r2 * psid_dm.toVector();
          u6.noalias()  = Bicphii * psid_dm.toVector();
          u6.noalias()  += r7 * S_j.toVector();
          u7.noalias()  = r3 * S_j.toVector() + r1 * (psid_dm.toVector() + phid_dm.toVector());
          u8.noalias()  = r4 * S_j.toVector();
          u9.noalias()  = r0 * S_j.toVector();
          u10.noalias() = Bicphii * S_j.toVector();
          u11.noalias() = S_j.toVector().transpose() * Bicphii;
          u12.noalias() = psid_dm.toVector().transpose() * Bicphii;
          u13.noalias() = r1 * S_j.toVector();

          JointIndex k = j;

          while (k > 0) {

            for (int r = 0; r < model.nvs[k]; r++) {
              const Eigen::DenseIndex kr = model.idx_vs[k] + r;

              const MotionRef<typename Data::Matrix6x::ColXpr> S_k(data.J.col(kr));
              const MotionRef<typename Data::Matrix6x::ColXpr> psid_dm = data.psid.col(kr);   // psi_dot{k}(:,r)
              const MotionRef<typename Data::Matrix6x::ColXpr> psidd_dm = data.psidd.col(kr); // psi_ddot{k}(:,r)
              const MotionRef<typename Data::Matrix6x::ColXpr> phid_dm = data.dJ.col(kr);     // phi_dot{k}(:,r)

              p1 = u11 * psid_dm.toVector();
              p2 = u9.dot(psidd_dm.toVector());
              p2 += (-u12 + u8.transpose()) * psid_dm.toVector();

              d2tau_dqdq_(ip, jq, kr) = p2;
              dtau_dqdv_(ip, kr, jq) = -p1;

              if (j != i) {
                p3 = -u11 * S_k.toVector();
                p4 = S_k.toVector().dot(u13);
                d2tau_dqdq_(jq, kr, ip) = u1 * psid_dm.toVector();
                d2tau_dqdq_(jq, kr, ip) += u2 * psidd_dm.toVector();
                d2tau_dqdq_(jq, ip, kr) = d2tau_dqdq_(jq, kr, ip);
                dtau_dqdv_(jq, kr, ip) = p1;
                dtau_dqdv_(jq, ip, kr) = u1 * S_k.toVector();
                dtau_dqdv_(jq, ip, kr) += u2 * (psid_dm.toVector() + phid_dm.toVector());
                d2tau_dvdv_(jq, kr, ip) = -p3;
                d2tau_dvdv_(jq, ip, kr) = -p3;
                dtau_dadq_(kr, jq, ip) = p4;
                dtau_dadq_(jq, kr, ip) = p4;
              }

              if (k != j) {
                p3 = -u11 * S_k.toVector();
                p5 = S_k.toVector().dot(u9);
                d2tau_dqdq_(ip, kr, jq) = p2;
                d2tau_dqdq_(kr, ip, jq) = S_k.toVector().dot(u3);
                d2tau_dvdv_(ip, jq, kr) = p3;
                d2tau_dvdv_(ip, kr, jq) = p3;
                dtau_dqdv_(ip, jq, kr) = S_k.toVector().dot(u5 + u8);
                dtau_dqdv_(ip, jq, kr) += u9.dot(psid_dm.toVector() + phid_dm.toVector());
                dtau_dqdv_(kr, jq, ip) = S_k.toVector().dot(u6);
                dtau_dadq_(kr, ip, jq) = p5;
                dtau_dadq_(ip, kr, jq) = p5;
                if (j != i) {
                  p6 = S_k.toVector().dot(u10);
                  d2tau_dqdq_(kr, jq, ip) = d2tau_dqdq_(kr, ip, jq);
                  d2tau_dvdv_(kr, ip, jq) = p6;
                  d2tau_dvdv_(kr, jq, ip) = p6;
                  dtau_dqdv_(kr, ip, jq) = S_k.toVector().dot(u7);

                } else {
                  d2tau_dvdv_(kr, jq, ip) = S_k.toVector().dot(u4);
                }

              } else {
                d2tau_dvdv_(ip, jq, kr) = -u2 * S_k.toVector();
              }
            }

            k = model.parents[k];
          }
        }
        j = model.parents[j];
      }
    }

    if (parent > 0) {
      data.oYcrb[parent] += data.oYcrb[i];
      data.doYcrb[parent] += data.doYcrb[i];
      data.of[parent] += data.of[i];
    }
  }

  // Function for cmf_bar operator
  template <typename ForceDerived, typename M6>
  static void ForceCrossMatrix(const ForceDense<ForceDerived> &f,
                               const Eigen::MatrixBase<M6> &mout) {
    M6 &mout_ = PINOCCHIO_EIGEN_CONST_CAST(M6, mout);
    mout_.template block<3, 3>(ForceDerived::LINEAR, ForceDerived::LINEAR)
        .setZero();
    mout_.template block<3, 3>(ForceDerived::LINEAR, ForceDerived::ANGULAR) =
        mout_.template block<3, 3>(ForceDerived::ANGULAR,
                                   ForceDerived::LINEAR) = skew(-f.linear());
    mout_.template block<3, 3>(ForceDerived::ANGULAR, ForceDerived::ANGULAR) =
        skew(-f.angular());
  }
  template <typename ForceDerived, typename M6>

  static void addForceCrossMatrix(const ForceDense<ForceDerived> &f,
                                  const Eigen::MatrixBase<M6> &mout) {
    M6 &mout_ = PINOCCHIO_EIGEN_CONST_CAST(M6, mout);
    addSkew(-f.linear(), mout_.template block<3, 3>(ForceDerived::LINEAR,
                                                    ForceDerived::ANGULAR));
    addSkew(-f.linear(), mout_.template block<3, 3>(ForceDerived::ANGULAR,
                                                    ForceDerived::LINEAR));
    addSkew(-f.angular(), mout_.template block<3, 3>(ForceDerived::ANGULAR,
                                                     ForceDerived::ANGULAR));
  }
};

template <typename Scalar, int Options,
          template <typename, int> class JointCollectionTpl,
          typename ConfigVectorType, typename TangentVectorType1,
          typename TangentVectorType2, typename Tensor1,
          typename Tensor2, typename Tensor3, typename Tensor4>
inline void ComputeRNEASecondOrderDerivatives(
    const ModelTpl<Scalar, Options, JointCollectionTpl> &model,
    DataTpl<Scalar, Options, JointCollectionTpl> &data,
    const Eigen::MatrixBase<ConfigVectorType> &q,
    const Eigen::MatrixBase<TangentVectorType1> &v,
    const Eigen::MatrixBase<TangentVectorType2> &a, const Tensor1 &d2tau_dqdq,
    const Tensor2 &d2tau_dvdv, const Tensor3 &dtau_dqdv,
    const Tensor4 &dtau_dadq) {
  // Extra safety here
  PINOCCHIO_CHECK_ARGUMENT_SIZE(
      q.size(), model.nq,
      "The joint configuration vector is not of right size");
  PINOCCHIO_CHECK_ARGUMENT_SIZE(
      v.size(), model.nv, "The joint velocity vector is not of right size");
  PINOCCHIO_CHECK_ARGUMENT_SIZE(
      a.size(), model.nv, "The joint acceleration vector is not of right size");
  PINOCCHIO_CHECK_ARGUMENT_SIZE(d2tau_dqdq.dimension(0), model.nv);
  PINOCCHIO_CHECK_ARGUMENT_SIZE(d2tau_dqdq.dimension(1), model.nv);
  PINOCCHIO_CHECK_ARGUMENT_SIZE(d2tau_dqdq.dimension(2), model.nv);
  PINOCCHIO_CHECK_ARGUMENT_SIZE(d2tau_dvdv.dimension(0), model.nv);
  PINOCCHIO_CHECK_ARGUMENT_SIZE(d2tau_dvdv.dimension(1), model.nv);
  PINOCCHIO_CHECK_ARGUMENT_SIZE(d2tau_dvdv.dimension(2), model.nv);
  PINOCCHIO_CHECK_ARGUMENT_SIZE(dtau_dqdv.dimension(0), model.nv);
  PINOCCHIO_CHECK_ARGUMENT_SIZE(dtau_dqdv.dimension(1), model.nv);
  PINOCCHIO_CHECK_ARGUMENT_SIZE(dtau_dqdv.dimension(2), model.nv);
  PINOCCHIO_CHECK_ARGUMENT_SIZE(dtau_dadq.dimension(0), model.nv);
  PINOCCHIO_CHECK_ARGUMENT_SIZE(dtau_dadq.dimension(1), model.nv);
  PINOCCHIO_CHECK_ARGUMENT_SIZE(dtau_dadq.dimension(2), model.nv);
  assert(model.check(data) && "data is not consistent with model.");

  typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
  typedef typename Model::JointIndex JointIndex;

  typedef ComputeRNEASecondOrderDerivativesForwardStep<
      Scalar, Options, JointCollectionTpl, ConfigVectorType, TangentVectorType1,
      TangentVectorType2>
      Pass1;
  for (JointIndex i = 1; i < (JointIndex)model.njoints; ++i) {
    Pass1::run(model.joints[i], data.joints[i],
               typename Pass1::ArgsType(model, data, q.derived(), v.derived(),
                                        a.derived()));
  }

  typedef ComputeRNEASecondOrderDerivativesBackwardStep<
      Scalar, Options, JointCollectionTpl, Tensor1, Tensor2,
      Tensor3, Tensor4>
      Pass2;
  for (JointIndex i = (JointIndex)(model.njoints - 1); i > 0; --i) {
    Pass2::run(model.joints[i],
               typename Pass2::ArgsType(model, data,
                                        const_cast<Tensor1 &>(d2tau_dqdq),
                                        const_cast<Tensor2 &>(d2tau_dvdv),
                                        const_cast<Tensor3 &>(dtau_dqdv),
                                        const_cast<Tensor4 &>(dtau_dadq)));
  }
}

} // namespace pinocchio

#endif // ifndef __pinocchio_algorithm_rnea_second_order_derivatives_hxx__
