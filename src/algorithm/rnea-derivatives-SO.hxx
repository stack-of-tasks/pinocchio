//
// Copyright (c) 2017-2020 CNRS INRIA

#ifndef __pinocchio_RNEADerivativesSO_hxx__
#define __pinocchio_RNEADerivativesSO_hxx__

#include "pinocchio/algorithm/check.hpp"
#include "pinocchio/multibody/visitor.hpp"

namespace pinocchio {

template <typename Scalar, int Options,
          template <typename, int> class JointCollectionTpl,
          typename ConfigVectorType, typename TangentVectorType1,
          typename TangentVectorType2>
struct ComputeRNEADerivativesSOForwardStep
    : public fusion::JointUnaryVisitorBase<ComputeRNEADerivativesSOForwardStep<
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

    const JointIndex &i = jmodel.id();
    const JointIndex &parent = model.parents[i];
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
        data.J); // data.J has all the phi (in ground frame) stacked in columns
    ColsBlock psid_cols =
        jmodel.jointCols(data.psid); // psid_cols is the psi_dot in ground frame
    ColsBlock psidd_cols = jmodel.jointCols(
        data.psidd); // psidd_cols is the psi_dotdot in ground frame
    ColsBlock dJ_cols =
        jmodel.jointCols(data.dJ); // This here is phi_dot in ground frame

    J_cols.noalias() = data.oMi[i].act(
        jdata.S()); // J_cols is just the phi in ground frame for a joint
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

    data.of[i] = oY * oa + oY.vxiv(ov); // f_i in ground frame

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
          typename tensortype1, typename tensortype2, typename tensortype3,
          typename tensortype4>
struct ComputeRNEADerivativesSOBackwardStep
    : public fusion::JointUnaryVisitorBase<ComputeRNEADerivativesSOBackwardStep<
          Scalar, Options, JointCollectionTpl, tensortype1, tensortype2,
          tensortype3, tensortype4>> {
  typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
  typedef DataTpl<Scalar, Options, JointCollectionTpl> Data;

  typedef boost::fusion::vector<const Model &, Data &, const tensortype1 &,
                                const tensortype2 &, const tensortype3 &,
                                const tensortype4 &>
      ArgsType;

  template <typename JointModel>
  static void algo(const JointModelBase<JointModel> &jmodel, const Model &model,
                   Data &data, const tensortype1 &dtau_dq2,
                   const tensortype2 &dtau_dv2, const tensortype3 &dtau_dqdv,
                   const tensortype3 &dtau_dadq) {
    typedef typename Data::Motion Motion;
    typedef typename Data::Force Force;
    typedef typename Data::Inertia Inertia;
    typedef typename Model::JointIndex JointIndex;
    typedef typename Motion::ActionMatrixType ActionMatrixType;
    typedef typename Data::Matrix6x Matrix6x;
    typedef typename Data::Matrix6 Matrix6;
    typedef typename Data::Vector6r Vector6r;
    typedef typename Data::Vector6c Vector6c;

    const JointIndex &i = jmodel.id();
    const JointIndex &parent = model.parents[i];
    JointIndex j, k;
    Eigen::Index joint_idx_j, joint_dofs_j, joint_idx_k, joint_dofs_k;

    typedef typename SizeDepType<JointModel::NV>::template ColsReturn<
        typename Data::Matrix6x>::Type ColsBlock;

    ColsBlock J_cols =
        jmodel.jointCols(data.J); // gives the phi matrix for this joint S{i}
    ColsBlock psid_cols = jmodel.jointCols(
        data.psid); // gives the psi_dot matrix for this joint psi_dot{i}
    ColsBlock psidd_cols = jmodel.jointCols(
        data.psidd); // gives the psid_dot matrix for this joint  psi_ddot{i}
    ColsBlock dJ_cols = jmodel.jointCols(
        data.dJ); // gives the phi_dot for this joint   phi_dot{i}
    ActionMatrixType S_dmA, S_dmAT;

    const Eigen::Index joint_idx = (Eigen::Index)jmodel.idx_v();
    const Eigen::Index joint_dofs =
        (Eigen::Index)jmodel.nv(); // no of joint DOFs

    Inertia &oYcrb = data.oYcrb[i];  // IC{i}
    Matrix6 &oBcrb = data.doYcrb[i]; // BC{i}

    tensortype1 &dtau_dq2_ = const_cast<tensortype1 &>(dtau_dq2);
    tensortype2 &dtau_dv2_ = const_cast<tensortype2 &>(dtau_dv2);
    tensortype3 &dtau_dqdv_ = const_cast<tensortype3 &>(dtau_dqdv);
    tensortype4 &dtau_dadq_ = const_cast<tensortype4 &>(dtau_dadq);

    Motion &S_dm = data.S_dm;
    Vector6c &Sdmv = S_dm.toVector(); // S{i}(:,p) vector
    Motion &psid_dm = data.psid_dm;
    Vector6c &psid_dmv = psid_dm.toVector(); // psid{i}(:,p) vector
    Motion &psidd_dm = data.psidd_dm;
    Vector6c &psidd_dmv = psidd_dm.toVector(); // psidd{i}(:,p) vector
    Motion &phid_dm = data.phid_dm;
    Vector6c &phid_dmv = phid_dm.toVector(); // psidm{i}(:,p) vector
    Force &Ftmp = data.ftmp;
    Vector6c &Ftmpv = Ftmp.toVector();

    Matrix6x &Jcols_j = data.Jcols_j;
    Matrix6x &psid_cols_j = data.psid_cols_j;
    Matrix6x &psidd_cols_j = data.psidd_cols_j;
    Matrix6x &dJ_cols_j = data.dJ_cols_j;
    Matrix6x &Jcols_k = data.Jcols_k;
    Matrix6x &psid_cols_k = data.psid_cols_k;
    Matrix6x &psidd_cols_k = data.psidd_cols_k;
    Matrix6x &dJ_cols_k = data.dJ_cols_k;

    Vector6r &SdmvT = data.tmpv1;
    Vector6r &u1 = data.vecu1;
    Vector6r &u2 = data.vecu2;
    Vector6c &u3 = data.vecu3;
    Vector6c &u4 = data.vecu4;
    Vector6c &u5 = data.vecu5;
    Vector6c &u6 = data.vecu6;
    Vector6c &u7 = data.vecu7;
    Vector6c &u8 = data.vecu8;
    Vector6c &u9 = data.vecu9;
    Vector6c &u10 = data.vecu10;
    Vector6r &u11 = data.vecu11;
    Vector6r &u12 = data.vecu12;
    Vector6c &u13 = data.vecu13;

    Matrix6 &Bicphii = data.tmpoBicphiiIn;
    Matrix6 &oBicpsidot = data.tmpoBicpsidotIn;

    Scalar p1, p2, p3, p4, p5, p6;
    Eigen::Index ip, jq, kr;

    Matrix6 &r0 = data.tmpmr0;
    Matrix6 &r1 = data.tmpr1In;
    Matrix6 &r2 = data.tmpr2In;
    Matrix6 &r3 = data.tmpr3In;
    Matrix6 &r4 = data.tmpr4In;
    Matrix6 &r5 = data.tmpr5In;
    Matrix6 &r6 = data.tmpr6In;
    Matrix6 &r7 = data.tmpr7In;

    for (int p = 0; p < joint_dofs; p++) {
      ip = joint_idx + p;

      S_dm = J_cols.col(p);          // S{i}(:,p)
      S_dmA = S_dm.toActionMatrix(); //(S{i}(:,p) )x matrix
      S_dmAT = S_dmA.transpose();
      psid_dm = psid_cols.col(p);   // psi_dot for p DOF
      psidd_dm = psidd_cols.col(p); // psi_ddot for p DOF
      phid_dm = dJ_cols.col(p);     // phi_dot for p DOF

      Bicphii = oYcrb.variation(S_dm);       // new Bicphii in ground frame
      oBicpsidot = oYcrb.variation(psid_dm); // new Bicpsidot in ground frame

      motionSet::inertiaAction(oYcrb, Sdmv, Ftmpv); // IC{i}S{i}(:,p)
      ForceCrossMatrix(Ftmp, r0);                   // cmf_bar(IC{i}S{i}(:,p))
      Bicphii += r0;

      motionSet::inertiaAction(oYcrb, psid_dmv, Ftmpv); // IC{i}S{i}(:,p)
      addForceCrossMatrix(Ftmp, oBicpsidot); // cmf_bar(IC{i}S{i}(:,p))

      r1 = oYcrb.variation(S_dm); // S{i}(p)x*IC{i} - IC{i} S{i}(p)x
      r2.noalias() = 2 * r0 - Bicphii;
      r3.noalias() =
          oBicpsidot - S_dmAT * oBcrb -
          oBcrb * S_dmA; // Bicpsidot + S{i}(p)x*BC{i}- BC {i}S{i}(p)x

      // r4
      Ftmpv = oBcrb.transpose() * Sdmv;
      ForceCrossMatrix(Ftmp, r4); // cmf_bar(BC{i}.'S{i}(:,p))
      // r5
      Ftmpv = oBcrb * psid_dmv - S_dmAT * data.of[i].toVector();
      motionSet::inertiaAction<ADDTO>(oYcrb, psidd_dmv, Ftmpv);
      ForceCrossMatrix(
          Ftmp,
          r5); //  cmf_bar(BC{i}psi_dot{i}(:,p)+IC{i}psi_ddot{i}(:,p)+S{i}(:,p)x*f{i})

      r6.noalias() =
          -S_dmAT * oYcrb.matrix_impl() + r0; // S{i}(:,p)x* IC{i} + r0
      // r7
      Ftmpv = oBcrb * Sdmv;
      motionSet::inertiaAction<ADDTO>(oYcrb, psid_dmv + phid_dmv, Ftmpv);
      ForceCrossMatrix(Ftmp, r7); // cmf_bar(BC{i}S{i}(:,p) +
                                  // IC{i}(psi_dot{i}(:,p)+phi_dot{i}(:,p)))

      j = i;

      while (j > 0) {
        joint_idx_j = (Eigen::Index)(model.joints[j]).idx_v();
        joint_dofs_j = (Eigen::Index)(model.joints[j]).nv(); // no of joint DOFs
        Jcols_j = (model.joints[j]).jointCols(data.J);       //  S{j}

        psid_cols_j = (model.joints[j]).jointCols(data.psid);   //  psi_dot{j}
        psidd_cols_j = (model.joints[j]).jointCols(data.psidd); //  psi_ddot{j}
        dJ_cols_j = (model.joints[j]).jointCols(data.dJ);       //  phi_dot{j}

        for (int q = 0; q < joint_dofs_j; q++) {
          jq = joint_idx_j + q;
          S_dm = Jcols_j.col(q);          // S{j}(:,q)
          psid_dm = psid_cols_j.col(q);   // psi_dot{j}(:,q)
          psidd_dm = psidd_cols_j.col(q); // psi_ddot{j}(:,q)
          phid_dm = dJ_cols_j.col(q);     // phi_dot{j}(:,q)
          SdmvT = Sdmv.transpose();       // (S{j}(:,q)).'

          u1 = SdmvT * r3;
          u2 = SdmvT * r1;
          u3 = r3 * psid_dmv + r1 * psidd_dmv + r5 * Sdmv;
          u4 = r6 * Sdmv;
          u5 = r2 * psid_dmv;
          u6 = Bicphii * psid_dmv;
          u6 += r7 * Sdmv;
          u7 = r3 * Sdmv + r1 * (psid_dmv + phid_dmv);
          u8 = r4 * Sdmv;
          u9 = r0 * Sdmv;
          u10 = Bicphii * Sdmv;
          u11 = SdmvT * Bicphii;
          u12 = psid_dmv.transpose() * Bicphii;
          u13 = r1 * Sdmv;

          k = j;

          while (k > 0) {
            joint_idx_k = (Eigen::Index)(model.joints[k]).idx_v();
            joint_dofs_k =
                (Eigen::Index)(model.joints[k]).nv();      // no of joint DOFs
            Jcols_k = (model.joints[k]).jointCols(data.J); //  S{k}
            psid_cols_k = (model.joints[k]).jointCols(data.psid); //  psi_dot{k}
            psidd_cols_k =
                (model.joints[k]).jointCols(data.psidd);      //  psi_ddot{k}
            dJ_cols_k = (model.joints[k]).jointCols(data.dJ); //  phi_dot{k}

            for (int r = 0; r < joint_dofs_k; r++) {
              kr = joint_idx_k + r;
              S_dm = Jcols_k.col(r);          // S{k}(:,r)
              psid_dm = psid_cols_k.col(r);   // psi_dot{k}(:,r)
              psidd_dm = psidd_cols_k.col(r); // psi_ddot{k}(:,r)
              phid_dm = dJ_cols_k.col(r);     // phi_dot{k}(:,r)

              p1 = u11 * psid_dmv;
              p2 = u9.dot(psidd_dmv);
              p2 += (-u12 + u8.transpose()) * psid_dmv;

              dtau_dq2_(ip, jq, kr) = p2;
              dtau_dqdv_(ip, kr, jq) = -p1;

              if (j != i) {
                p3 = -u11 * Sdmv;
                p4 = Sdmv.dot(u13);
                dtau_dq2_(jq, kr, ip) = u1 * psid_dmv;
                dtau_dq2_(jq, kr, ip) += u2 * psidd_dmv;
                dtau_dq2_(jq, ip, kr) = dtau_dq2_(jq, kr, ip);
                dtau_dqdv_(jq, kr, ip) = p1;
                dtau_dqdv_(jq, ip, kr) = u1 * Sdmv;
                dtau_dqdv_(jq, ip, kr) += u2 * (psid_dmv + phid_dmv);
                dtau_dv2_(jq, kr, ip) = -p3;
                dtau_dv2_(jq, ip, kr) = -p3;
                dtau_dadq_(kr, jq, ip) = p4;
                dtau_dadq_(jq, kr, ip) = p4;
              }

              if (k != j) {
                p3 = -u11 * Sdmv;
                p5 = Sdmv.dot(u9);
                dtau_dq2_(ip, kr, jq) = p2;
                dtau_dq2_(kr, ip, jq) = Sdmv.dot(u3);
                dtau_dv2_(ip, jq, kr) = p3;
                dtau_dv2_(ip, kr, jq) = p3;
                dtau_dqdv_(ip, jq, kr) = Sdmv.dot(u5 + u8);
                dtau_dqdv_(ip, jq, kr) += u9.dot(psid_dmv + phid_dmv);
                dtau_dqdv_(kr, jq, ip) = Sdmv.dot(u6);
                dtau_dadq_(kr, ip, jq) = p5;
                dtau_dadq_(ip, kr, jq) = p5;
                if (j != i) {
                  p6 = Sdmv.dot(u10);
                  dtau_dq2_(kr, jq, ip) = dtau_dq2_(kr, ip, jq);
                  dtau_dv2_(kr, ip, jq) = p6;
                  dtau_dv2_(kr, jq, ip) = p6;
                  dtau_dqdv_(kr, ip, jq) = Sdmv.dot(u7);

                } else {
                  dtau_dv2_(kr, jq, ip) = Sdmv.dot(u4);
                }

              } else {
                dtau_dv2_(ip, jq, kr) = -u2 * Sdmv;
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
          typename TangentVectorType2, typename tensortype1,
          typename tensortype2, typename tensortype3, typename tensortype4>
inline void computeRNEADerivativesSO(
    const ModelTpl<Scalar, Options, JointCollectionTpl> &model,
    DataTpl<Scalar, Options, JointCollectionTpl> &data,
    const Eigen::MatrixBase<ConfigVectorType> &q,
    const Eigen::MatrixBase<TangentVectorType1> &v,
    const Eigen::MatrixBase<TangentVectorType2> &a, const tensortype1 &dtau_dq2,
    const tensortype2 &dtau_dv2, const tensortype3 &dtau_dqdv,
    const tensortype4 &dtau_dadq) {
  // Extra safety here
  PINOCCHIO_CHECK_ARGUMENT_SIZE(
      q.size(), model.nq,
      "The joint configuration vector is not of right size");
  PINOCCHIO_CHECK_ARGUMENT_SIZE(
      v.size(), model.nv, "The joint velocity vector is not of right size");
  PINOCCHIO_CHECK_ARGUMENT_SIZE(
      a.size(), model.nv, "The joint acceleration vector is not of right size");
  PINOCCHIO_CHECK_ARGUMENT_SIZE(dtau_dq2.dimension(0), model.nv);
  PINOCCHIO_CHECK_ARGUMENT_SIZE(dtau_dq2.dimension(1), model.nv);
  PINOCCHIO_CHECK_ARGUMENT_SIZE(dtau_dq2.dimension(2), model.nv);
  PINOCCHIO_CHECK_ARGUMENT_SIZE(dtau_dv2.dimension(0), model.nv);
  PINOCCHIO_CHECK_ARGUMENT_SIZE(dtau_dv2.dimension(1), model.nv);
  PINOCCHIO_CHECK_ARGUMENT_SIZE(dtau_dv2.dimension(2), model.nv);
  PINOCCHIO_CHECK_ARGUMENT_SIZE(dtau_dqdv.dimension(0), model.nv);
  PINOCCHIO_CHECK_ARGUMENT_SIZE(dtau_dqdv.dimension(1), model.nv);
  PINOCCHIO_CHECK_ARGUMENT_SIZE(dtau_dqdv.dimension(2), model.nv);
  PINOCCHIO_CHECK_ARGUMENT_SIZE(dtau_dadq.dimension(0), model.nv);
  PINOCCHIO_CHECK_ARGUMENT_SIZE(dtau_dadq.dimension(1), model.nv);
  PINOCCHIO_CHECK_ARGUMENT_SIZE(dtau_dadq.dimension(2), model.nv);
  assert(model.check(data) && "data is not consistent with model.");

  typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
  typedef typename Model::JointIndex JointIndex;

  typedef ComputeRNEADerivativesSOForwardStep<
      Scalar, Options, JointCollectionTpl, ConfigVectorType, TangentVectorType1,
      TangentVectorType2>
      Pass1;
  for (JointIndex i = 1; i < (JointIndex)model.njoints; ++i) {
    Pass1::run(model.joints[i], data.joints[i],
               typename Pass1::ArgsType(model, data, q.derived(), v.derived(),
                                        a.derived()));
  }

  typedef ComputeRNEADerivativesSOBackwardStep<
      Scalar, Options, JointCollectionTpl, tensortype1, tensortype2,
      tensortype3, tensortype4>
      Pass2;
  for (JointIndex i = (JointIndex)(model.njoints - 1); i > 0; --i) {
    Pass2::run(model.joints[i],
               typename Pass2::ArgsType(model, data,
                                        const_cast<tensortype1 &>(dtau_dq2),
                                        const_cast<tensortype2 &>(dtau_dv2),
                                        const_cast<tensortype3 &>(dtau_dqdv),
                                        const_cast<tensortype4 &>(dtau_dadq)));
  }
}

} // namespace pinocchio

#endif //
