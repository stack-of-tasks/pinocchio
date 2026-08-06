//
// Copyright (c) 2024-2025 INRIA
//

#pragma once

// IWYU pragma: private, include "pinocchio/algorithm/loop-constrained-aba.hpp"

#ifdef PINOCCHIO_LSP
  #undef PINOCCHIO_LSP
  #include "pinocchio/algorithm/loop-constrained-aba.hpp"
#endif // PINOCCHIO_LSP

namespace pinocchio
{

  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    typename ConfigVectorType,
    typename TangentVectorType>
  struct LCABAForwardStep1
  : public fusion::JointUnaryVisitorBase<
      LCABAForwardStep1<Scalar, Options, JointCollectionTpl, ConfigVectorType, TangentVectorType>>
  {
    typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
    typedef DataTpl<Scalar, Options, JointCollectionTpl> Data;

    typedef boost::fusion::
      vector<const Model &, Data &, const ConfigVectorType &, const TangentVectorType &>
        ArgsType;

    template<typename JointModel>
    static void algo(
      const pinocchio::JointModelBase<JointModel> & jmodel,
      pinocchio::JointDataBase<typename JointModel::JointDataDerived> & jdata,
      const Model & model,
      Data & data,
      const Eigen::MatrixBase<ConfigVectorType> & q,
      const Eigen::MatrixBase<TangentVectorType> & v)
    {
      typedef typename Model::JointIndex JointIndex;
      typedef typename Data::Motion Motion;

      const JointIndex joint_i = jmodel.id();
      Motion & ov = data.ov[joint_i];
      jmodel.calc(jdata.derived(), q.derived(), v.derived());

      const JointIndex parent = model.parents[joint_i];
      data.liMi[joint_i] = model.jointPlacements[joint_i] * jdata.M();
      if (parent > 0)
        data.oMi[joint_i] = data.oMi[parent] * data.liMi[joint_i];
      else
        data.oMi[joint_i] = data.liMi[joint_i];

      jmodel.jointCols(data.J) = data.oMi[joint_i].act(jdata.S());

      ov = data.oMi[joint_i].act(jdata.v());
      if (parent > 0)
        ov += data.ov[parent];

      data.oa_gf[joint_i] = data.oMi[joint_i].act(jdata.c());
      if (parent > 0)
        data.oa_gf[joint_i] += (data.ov[parent] ^ ov);

      data.oinertias[joint_i] = data.oYcrb[joint_i] =
        data.oMi[joint_i].act(model.inertias[joint_i]);
      data.oYaba_augmented[joint_i] = data.oYcrb[joint_i].matrix();

      data.oh[joint_i] = data.oYcrb[joint_i] * ov; // necessary for ABA derivatives
      data.of[joint_i] = ov.cross(data.oh[joint_i]);
    }
  };

  template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
  struct LCABABackwardStep
  : public fusion::JointUnaryVisitorBase<LCABABackwardStep<Scalar, Options, JointCollectionTpl>>
  {
    typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
    typedef DataTpl<Scalar, Options, JointCollectionTpl> Data;

    typedef boost::fusion::vector<const Model &, Data &> ArgsType;

    template<typename JointModel>
    static void algo(
      const JointModelBase<JointModel> & jmodel,
      JointDataBase<typename JointModel::JointDataDerived> & jdata,
      const Model & model,
      Data & data)
    {
      typedef typename JointModel::JointDataDerived JointData;
      typedef typename Model::JointIndex JointIndex;
      typedef typename Data::Force Force;
      typedef typename Data::Vector6 Vector6;
      typedef typename Data::Matrix6 Matrix6;

      typedef std::pair<JointIndex, JointIndex> JointPair;

      const JointIndex joint_i = jmodel.id();
      const JointIndex parent = model.parents[joint_i];
      auto & Ia = data.oYaba_augmented[joint_i];

      const auto Jcols = jmodel.jointCols(data.J);

      Force & fi = data.of[joint_i];

      jmodel.jointVelocitySelector(data.u).noalias() -= Jcols.transpose() * fi.toVector();

      jdata.U().noalias() = Ia * Jcols;
      jdata.StU().noalias() = Jcols.transpose() * jdata.U();

      // Account for the rotor inertia contribution
      jdata.StU().diagonal() += jmodel.jointVelocitySelector(model.armature);

      ::pinocchio::internal::matrix_inversion(jdata.StU(), jdata.Dinv());

      jdata.UDinv().noalias() =
        jdata.U() * jdata.Dinv(); // TODO:check where its used when parent == 0
      if (parent > 0)
      {
        Ia.noalias() -= jdata.UDinv() * jdata.U().transpose();
        data.oYaba_augmented[parent] += Ia;

        fi.toVector().noalias() += Ia * data.oa_gf[joint_i].toVector()
                                   + jdata.UDinv() * jmodel.jointVelocitySelector(data.u);
        data.of[parent] += fi;
      }

      // End of the classic ABA backward pass - beginning of cross-coupling handling
      const auto & neighbours = data.joint_neighbours;
      auto & joint_cross_coupling = data.joint_cross_coupling;
      const auto & joint_neighbours = neighbours[joint_i];

      if (joint_neighbours.size() == 0)
        return; // We can return from this point as this joint has no neighbours

      using Matrix6xNV = std::remove_reference_t<typename JointData::UDTypeRef>;
      using MapMatrix6xNV = Eigen::Map<Matrix6xNV>;
      MapMatrix6xNV mat1_tmp = MapMatrix6xNV(_PINOCCHIO_EIGEN_MAP_ALLOCA(Scalar, 6, jmodel.nv()));
      MapMatrix6xNV mat2_tmp = MapMatrix6xNV(_PINOCCHIO_EIGEN_MAP_ALLOCA(Scalar, 6, jmodel.nv()));

      auto & JDinv = mat1_tmp;
      JDinv.noalias() = Jcols * jdata.Dinv();

      // oL == data.oL[joint_i]
      Matrix6 oL = -JDinv * jdata.U().transpose();
      oL += Matrix6::Identity();

      // a_tmp is a Spatial Acceleration
      Vector6 a_tmp = oL * data.oa_gf[joint_i].toVector();
      a_tmp.noalias() += JDinv * jmodel.jointVelocitySelector(data.u);

      for (size_t j = 0; j < joint_neighbours.size(); j++)
      {
        const JointIndex joint_j = joint_neighbours[j];

        assert(joint_cross_coupling.exists(JointPair(joint_j, joint_i)));
        const auto & crosscoupling_ji = joint_cross_coupling.get(JointPair(joint_j, joint_i));

        auto & crosscoupling_xi_Jcols = mat1_tmp;
        crosscoupling_xi_Jcols.noalias() =
          crosscoupling_ji * Jcols; // Warning: UDinv() is actually edge_ji * J

        auto & crosscoupling_ji_Jcols_Dinv = mat2_tmp;
        crosscoupling_ji_Jcols_Dinv.noalias() = crosscoupling_xi_Jcols * jdata.Dinv();

        data.oYaba_augmented[joint_j].noalias() -=
          crosscoupling_ji_Jcols_Dinv
          * crosscoupling_xi_Jcols.transpose(); // Warning: UDinv() is actually edge_ji * J, U() is
                                                // actually edge_ji * J_cols * Dinv
        data.of[joint_j].toVector().noalias() += crosscoupling_ji * a_tmp;

        const Matrix6 crosscoupling_ji_oL = crosscoupling_ji * oL;
        if (joint_j == parent)
        {
          data.oYaba_augmented[parent].noalias() +=
            crosscoupling_ji_oL + crosscoupling_ji_oL.transpose();
        }
        else
        {
          assert(
            joint_cross_coupling.exists(JointPair(joint_j, parent))
            || joint_cross_coupling.exists(JointPair(parent, joint_j)));

          // In this particular case, the pair (joint_j,parent) might not exist, but (parent,
          // joint_j) will
          if (joint_cross_coupling.exists(JointPair(joint_j, parent)))
          {
            joint_cross_coupling.get({joint_j, parent}).noalias() += crosscoupling_ji_oL;
          }
          else
          {
            joint_cross_coupling.get({parent, joint_j}).noalias() +=
              crosscoupling_ji_oL.transpose();
          }
        }

        for (size_t k = j + 1; k < joint_neighbours.size(); ++k)
        {
          const JointIndex joint_k = joint_neighbours[k];
          assert(joint_j != joint_k && "Must never happen!");

          assert(joint_cross_coupling.exists(JointPair(joint_k, joint_i)));
          const auto & crosscoupling_ki = joint_cross_coupling.get(JointPair(joint_k, joint_i));

          crosscoupling_xi_Jcols.noalias() = crosscoupling_ki * Jcols;

          assert(joint_cross_coupling.exists(JointPair(joint_j, joint_k)));
          auto & crosscoupling_jk = joint_cross_coupling.get(JointPair(joint_j, joint_k));
          crosscoupling_jk.noalias() -=
            crosscoupling_ji_Jcols_Dinv * crosscoupling_xi_Jcols.transpose();
        }
      }
    }
  };

  // A reduced backward sweep that only propagates the affine terms
  template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
  struct LCABAReducedBackwardStep
  : public fusion::JointUnaryVisitorBase<
      LCABAReducedBackwardStep<Scalar, Options, JointCollectionTpl>>
  {
    typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
    typedef DataTpl<Scalar, Options, JointCollectionTpl> Data;

    typedef boost::fusion::vector<const Model &, Data &> ArgsType;

    template<typename JointModel>
    static void algo(
      const JointModelBase<JointModel> & jmodel,
      JointDataBase<typename JointModel::JointDataDerived> & jdata,
      const Model & model,
      Data & data)
    {

      typedef typename Model::JointIndex JointIndex;
      typedef typename Data::Force Force;
      typedef typename Data::Motion Motion;
      typedef typename Motion::Vector6 Vector6;
      typedef std::pair<JointIndex, JointIndex> JointPair;

      const auto & neighbours = data.joint_neighbours;
      auto & joint_cross_coupling = data.joint_cross_coupling;

      const JointIndex joint_i = jmodel.id();
      const JointIndex parent = model.parents[joint_i];

      const auto Jcols = jmodel.jointCols(data.J);

      Force & fi = data.of[joint_i];

      jmodel.jointVelocitySelector(data.u).noalias() = -Jcols.transpose() * fi.toVector();

      jmodel.jointVelocitySelector(data.g).noalias() =
        jdata.Dinv()
        * jmodel.jointVelocitySelector(
          data.u); // Abuse of notation to reuse existing unused data variable

      const Vector6 a_tmp = Jcols * jmodel.jointVelocitySelector(data.g);

      for (const JointIndex joint_j : neighbours[joint_i])
      {
        const auto & coupling_ji = joint_cross_coupling.get(JointPair(joint_j, joint_i));
        data.of[joint_j].toVector().noalias() += coupling_ji * a_tmp;
      }

      if (parent > 0)
      {
        data.of[parent].toVector().noalias() +=
          fi.toVector() + jdata.U() * jmodel.jointVelocitySelector(data.g);
      }
    }
  };

  template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
  struct LCABAForwardStep2
  : public fusion::JointUnaryVisitorBase<LCABAForwardStep2<Scalar, Options, JointCollectionTpl>>
  {
    typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
    typedef DataTpl<Scalar, Options, JointCollectionTpl> Data;

    typedef std::pair<JointIndex, JointIndex> JointPair;
    typedef typename Data::Matrix6 Matrix6;
    typedef typename Data::Force Force;

    typedef boost::fusion::vector<const Model &, Data &> ArgsType;

    template<typename JointModel>
    static void algo(
      const pinocchio::JointModelBase<JointModel> & jmodel,
      pinocchio::JointDataBase<typename JointModel::JointDataDerived> & jdata,
      const Model & model,
      Data & data)
    {
      typedef typename Model::JointIndex JointIndex;
      typedef typename Data::Force Force;
      typedef typename Data::Matrix6x Matrix6x;

      typedef typename SizeDepType<JointModel::NV>::template ColsReturn<Matrix6x>::Type ColBlock;
      ColBlock J_cols = jmodel.jointCols(data.J);

      const JointIndex joint_i = jmodel.id();
      const JointIndex parent = model.parents[joint_i];
      const std::vector<JointIndex> & neighbours = data.joint_neighbours[joint_i];

      data.oa_gf[joint_i] += data.oa_gf[parent]; // does take into account the gravity field

      Force coupling_forces = Force::Zero();
      for (JointIndex joint_j : neighbours)
      {
        const auto & coupling_ji = data.joint_cross_coupling.get(JointPair(joint_j, joint_i));
        coupling_forces.toVector().noalias() +=
          coupling_ji.transpose() * data.oa_gf[joint_j].toVector();
      }

      jmodel.jointVelocitySelector(data.u).noalias() -=
        J_cols.transpose() * coupling_forces.toVector();

      jmodel.jointVelocitySelector(data.ddq).noalias() =
        jdata.Dinv() * jmodel.jointVelocitySelector(data.u)
        - jdata.UDinv().transpose() * data.oa_gf[joint_i].toVector();
      data.oa_gf[joint_i].toVector().noalias() += J_cols * jmodel.jointVelocitySelector(data.ddq);

      // Handle consistent output
      data.oa[joint_i] = data.oa_gf[joint_i]; // + model.gravity;
      // data.of[joint_i] = data.oinertias[joint_i] * data.oa_gf[joint_i] +
      // data.ov[joint_i].cross(data.oh[joint_i]);
    }
  };

  template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
  struct LCABAReducedForwardStep
  : public fusion::JointUnaryVisitorBase<
      LCABAReducedForwardStep<Scalar, Options, JointCollectionTpl>>
  {
    typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
    typedef DataTpl<Scalar, Options, JointCollectionTpl> Data;

    typedef std::pair<JointIndex, JointIndex> JointPair;
    typedef typename Data::Matrix6 Matrix6;
    typedef typename Data::Force Force;

    typedef boost::fusion::vector<const Model &, Data &> ArgsType;

    template<typename JointModel>
    static void algo(
      const pinocchio::JointModelBase<JointModel> & jmodel,
      pinocchio::JointDataBase<typename JointModel::JointDataDerived> & jdata,
      const Model & model,
      Data & data)
    {
      typedef typename Model::JointIndex JointIndex;

      const auto J_cols = jmodel.jointCols(data.J);

      const JointIndex joint_i = jmodel.id();
      const JointIndex parent = model.parents[joint_i];
      const auto & neighbours = data.joint_neighbours[joint_i];

      data.oa_gf[joint_i] = data.oa_gf[parent]; // does take into account the gravity field

      Force & fi = data.of[joint_i];
      for (const JointIndex joint_j : neighbours)
      {
        const auto & coupling_ji = data.joint_cross_coupling.get(JointPair(joint_j, joint_i));
        fi.toVector().noalias() += coupling_ji.transpose() * data.oa_gf[joint_j].toVector();
      }

      jmodel.jointVelocitySelector(data.u).noalias() = -J_cols.transpose() * fi.toVector();

      // Abuse of notation using data.g for storing delta ddq
      jmodel.jointVelocitySelector(data.g).noalias() =
        jdata.Dinv()
        * (jmodel.jointVelocitySelector(data.u) - jdata.U().transpose() * data.oa_gf[joint_i].toVector());
      data.oa_gf[joint_i].toVector().noalias() += J_cols * jmodel.jointVelocitySelector(data.g);

      data.oa[joint_i] += data.oa_gf[joint_i];
    }
  };

  namespace internal
  {
    template<typename ConstraintModel>
    struct LCABAConstraintCalcStep
    {
      typedef typename ConstraintModel::ConstraintData ConstraintData;
      typedef typename ConstraintModel::Scalar Scalar;

      template<typename Model, typename Data>
      static void run(
        const Model & model,
        Data & data,
        const ConstraintModel & cmodel,
        ConstraintData & cdata,
        const Scalar mu);
    };

    template<typename Scalar, int Options>
    struct LCABAConstraintCalcStep<RigidConstraintModelTpl<Scalar, Options>>
    {
      typedef RigidConstraintModelTpl<Scalar, Options> ConstraintModel;
      typedef typename ConstraintModel::ConstraintData ConstraintData;

      template<typename Model, typename Data>
      static void run(
        const Model & model,
        Data & data,
        const ConstraintModel & cmodel,
        ConstraintData & cdata,
        const Scalar mu)
      {
        typedef typename Data::SE3 SE3;
        typedef typename Model::JointIndex JointIndex;
        typedef typename Data::Matrix6 Matrix6;
        typedef typename ConstraintModel::Matrix36 Matrix36;
        typedef typename ConstraintData::Motion Motion;

        cdata.contact_force.setZero();

        cmodel.calc(model, data, cdata);

        SE3 & oMc1 = cdata.oMc1;
        SE3 & oMc2 = cdata.oMc2;
        SE3 & c1Mc2 = cdata.c1Mc2;
        typename ConstraintData::Motion & vc1 = cdata.contact1_velocity;
        typename ConstraintData::Motion & vc2 = cdata.contact2_velocity;
        const JointIndex joint1_id = cmodel.joint1_id;
        const JointIndex joint2_id = cmodel.joint2_id;

        const auto & corrector = cmodel.m_baumgarte_parameters;
        auto & contact_velocity_error = cdata.contact_velocity_error;

        if (joint1_id > 0)
          vc1 = oMc1.actInv(data.ov[joint1_id]);
        else
          vc1.setZero();
        if (joint2_id > 0)
          vc2 = oMc2.actInv(data.ov[joint2_id]);
        else
          vc2.setZero();
        const Motion vc2_in_frame1 = c1Mc2.act(vc2);

        if (cmodel.type == CONTACT_6D)
        {
          cdata.contact_placement_error = -log6(c1Mc2);
          contact_velocity_error = vc1 - vc2_in_frame1;
          const Matrix6 A1 = oMc1.toActionMatrixInverse();
          const Matrix6 A1tA1 = A1.transpose() * A1;
          data.oYaba_augmented[joint1_id].noalias() += mu * A1tA1;

          // Baumgarte
          if (
            check_expression_if_real<Scalar, false>(
              (corrector.Kp == static_cast<Scalar>(0.))
              && (corrector.Kd == static_cast<Scalar>(0.))))
          {
            cdata.contact_acceleration_desired.setZero();
          }
          else
          {
            cdata.contact_acceleration_desired.toVector().noalias() =
              -(corrector.Kd * contact_velocity_error.toVector())
              - (corrector.Kp * cdata.contact_placement_error.toVector());
          }

          cdata.contact_acceleration_desired -= oMc1.actInv(data.oa[joint1_id]);
          cdata.contact_acceleration_desired -= cdata.contact_velocity_error.cross(vc2_in_frame1);

          if (joint2_id > 0)
          {
            cdata.contact_acceleration_desired += oMc1.actInv(data.oa[joint2_id]);

            const Matrix6 A2 =
              -A1; // only for 6D case. also used below for computing A2tA2 and A1tA2
            data.oYaba_augmented[joint2_id].noalias() += mu * A1tA1;
            data.of[joint2_id].toVector().noalias() +=
              A2.transpose()
              * (/*cdata.contact_force.toVector()*/ -mu
                 * cdata.contact_acceleration_desired.toVector());

            assert(
              bool(
                data.joint_cross_coupling.exists({joint1_id, joint2_id})
                || data.joint_cross_coupling.exists({joint2_id, joint1_id}))
              && "Must never happen");

            if (data.joint_cross_coupling.exists({joint1_id, joint2_id}))
              data.joint_cross_coupling.get({joint1_id, joint2_id}) -= mu * A1tA1;
            else
              data.joint_cross_coupling.get({joint2_id, joint1_id}) -= mu * A1tA1;
          }
          else
          {
            cdata.contact_acceleration_desired.toVector().noalias() -=
              A1 * model.gravity.toVector();
          }

          data.of[joint1_id].toVector().noalias() +=
            A1.transpose()
            * (/*cdata.contact_force.toVector()*/ -mu
               * cdata.contact_acceleration_desired.toVector());
        }
        else if (cmodel.type == CONTACT_3D)
        {
          const Matrix36 & A1 = oMc1.toActionMatrixInverse().template topRows<3>();
          data.oYaba_augmented[joint1_id].noalias() += mu * A1.transpose() * A1;

          if (
            check_expression_if_real<Scalar, false>(
              (corrector.Kp == static_cast<Scalar>(0.))
              && (corrector.Kd == static_cast<Scalar>(0.))))
          {
            cdata.contact_acceleration_desired.setZero();
          }
          else
          {
            cdata.contact_acceleration_desired.linear().noalias() =
              -(corrector.Kd * contact_velocity_error.linear())
              - (corrector.Kp * cdata.contact_placement_error.linear());
            cdata.contact_acceleration_desired.angular().setZero();
          }

          cdata.contact_acceleration_desired.linear().noalias() -=
            vc1.angular().cross(vc1.linear());
          if (joint2_id > 0)
          {
            const Matrix36 A2 =
              -c1Mc2.rotation()
              * (oMc2.toActionMatrixInverse().template topRows<3>()); // TODO:remove memalloc

            cdata.contact_acceleration_desired.linear().noalias() +=
              c1Mc2.rotation() * vc2.angular().cross(vc2.linear());
            data.oYaba_augmented[joint2_id].noalias() += mu * A2.transpose() * A2;
            data.of[joint2_id].toVector().noalias() +=
              A2.transpose()
              * (/*cdata.contact_force.toVector()*/ -mu
                 * cdata.contact_acceleration_desired.linear());

            assert(
              bool(
                data.joint_cross_coupling.exists({joint1_id, joint2_id})
                || data.joint_cross_coupling.exists({joint2_id, joint1_id}))
              && "Must never happen");
            if (data.joint_cross_coupling.exists({joint1_id, joint2_id}))
            {
              data.joint_cross_coupling.get({joint1_id, joint2_id}).noalias() +=
                mu * A1.transpose() * A2;
            }
            else
            {
              data.joint_cross_coupling.get({joint2_id, joint1_id}).noalias() +=
                mu * A2.transpose() * A1;
            }
          }
          else
          {
            cdata.contact_acceleration_desired.linear().noalias() -= A1 * model.gravity.toVector();
          }

          data.of[joint1_id].toVector().noalias() +=
            A1.transpose()
            * (/*cdata.contact_force.toVector()*/ -mu
               * cdata.contact_acceleration_desired.linear());
        }
        else
        {
          PINOCCHIO_UNREACHABLE();
        }
      }
    };
  } // namespace internal

  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    typename ConfigVectorType,
    typename TangentVectorType1,
    typename TangentVectorType2,
    class ConstraintModel,
    class ConstraintModelAllocator,
    class ConstraintData,
    class ConstraintDataAllocator>
  inline const typename DataTpl<Scalar, Options, JointCollectionTpl>::TangentVectorType & lcaba(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const Eigen::MatrixBase<ConfigVectorType> & q,
    const Eigen::MatrixBase<TangentVectorType1> & v,
    const Eigen::MatrixBase<TangentVectorType2> & tau,
    const std::vector<ConstraintModel, ConstraintModelAllocator> & constraint_models,
    std::vector<ConstraintData, ConstraintDataAllocator> & constraint_datas,
    ProximalSettingsTpl<Scalar> & settings)
  {

    assert(model.check(data) && "data is not consistent with model.");
    PINOCCHIO_CHECK_ARGUMENT_SIZE(
      q.size(), model.nq, "The joint configuration vector is not of right size");
    PINOCCHIO_CHECK_ARGUMENT_SIZE(
      v.size(), model.nv, "The joint velocity vector is not of right size");
    PINOCCHIO_CHECK_ARGUMENT_SIZE(
      tau.size(), model.nv, "The joint torque vector is not of right size");

    typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
    typedef typename Model::JointIndex JointIndex;
    typedef typename ConstraintModel::Matrix36 Matrix36;
    typedef typename ConstraintData::Force Force;

    data.u = tau;
    data.oa_gf[0] = -model.gravity;
    data.oa[0] = data.oa_gf[0];
    data.of[0].setZero();

    assert(settings.mu > 0 && "lcaba requires mu > 0.");
    const Scalar mu = Scalar(1) / settings.mu;

    for (auto & coupling : data.joint_cross_coupling)
      coupling.setZero();

    typedef LCABAForwardStep1<
      Scalar, Options, JointCollectionTpl, ConfigVectorType, TangentVectorType1>
      Pass1;
    for (JointIndex joint_i = 1; joint_i < (JointIndex)model.njoints; ++joint_i)
    {
      Pass1::run(
        model.joints[joint_i], data.joints[joint_i],
        typename Pass1::ArgsType(model, data, q.derived(), v.derived()));
    }

    for (std::size_t constraint_id = 0; constraint_id < constraint_models.size(); ++constraint_id)
    {
      ConstraintData & cdata = constraint_datas[constraint_id];
      const ConstraintModel & cmodel = constraint_models[constraint_id];

      typedef internal::LCABAConstraintCalcStep<ConstraintModel> CalcStep;
      CalcStep::run(model, data, cmodel, cdata, mu);
    }

    typedef LCABABackwardStep<Scalar, Options, JointCollectionTpl> Pass2;

    const auto & joint_elimination_order = data.joint_elimination_order;

    for (JointIndex joint_i : joint_elimination_order)
    {
      Pass2::run(
        model.joints[joint_i], data.joints[joint_i], typename Pass2::ArgsType(model, data));
    }

    typedef LCABAForwardStep2<Scalar, Options, JointCollectionTpl> Pass3;

    for (int it = int(joint_elimination_order.size()) - 1; it >= 0; --it)
    {
      const JointIndex joint_i = joint_elimination_order[size_t(it)];
      if (data.constraints_supported_dim[joint_i] > 0)
        Pass3::run(
          model.joints[joint_i], data.joints[joint_i], typename Pass3::ArgsType(model, data));
    }

    typedef LCABAReducedBackwardStep<Scalar, Options, JointCollectionTpl> ReducedPass2;
    typedef LCABAReducedForwardStep<Scalar, Options, JointCollectionTpl> ReducedPass3;
    data.g.setZero();
    int iter = 1;
    for (; iter < settings.max_iter; iter++)
    {
      settings.absolute_residual = Scalar(0);
      for (JointIndex j = 1; j < (JointIndex)model.njoints; ++j)
      {
        if (data.constraints_supported_dim[j] > 0)
          data.of[j].setZero();
      }
      for (std::size_t j = 0; j < constraint_models.size(); ++j)
      {
        const ConstraintModel & cmodel = constraint_models[j];
        ConstraintData & cdata = constraint_datas[j];
        const JointIndex joint1_id = cmodel.joint1_id;
        const JointIndex joint2_id = cmodel.joint2_id;
        typename ConstraintData::Motion & contact_acc_err = cdata.contact_acceleration_error;

        if (cmodel.type == CONTACT_6D)
        {
          if (joint2_id > 0)
            contact_acc_err = cdata.oMc1.actInv((data.oa[joint1_id] - data.oa[joint2_id]))
                              - cdata.contact_acceleration_desired;
          else
            contact_acc_err =
              cdata.oMc1.actInv((data.oa[joint1_id])) - cdata.contact_acceleration_desired;

          const Force mu_lambda = Force(mu * contact_acc_err.toVector());
          cdata.contact_force += mu_lambda;

          if (joint1_id > 0)
            data.of[joint1_id] += cdata.oMc1.act(mu_lambda);

          if (joint2_id > 0)
            data.of[joint2_id] -= cdata.oMc1.act(mu_lambda);
        }
        else if (cmodel.type == CONTACT_3D)
        {
          contact_acc_err.linear() = -cdata.contact_acceleration_desired.linear();
          if (joint1_id > 0)
            contact_acc_err.linear() += cdata.oMc1.actInv(data.oa[joint1_id]).linear();
          if (joint2_id > 0)
            contact_acc_err.linear() -=
              cdata.c1Mc2.rotation() * cdata.oMc2.actInv(data.oa[joint2_id]).linear();

          const Force mu_lambda = Force(mu * contact_acc_err.toVector());
          cdata.contact_force.linear() += mu_lambda.linear();

          if (joint1_id > 0)
            data.of[joint1_id] += cdata.oMc1.act(mu_lambda);

          if (joint2_id > 0)
          {
            const Matrix36 A2 =
              -cdata.c1Mc2.rotation() * (cdata.oMc2.toActionMatrixInverse().template topRows<3>());

            data.of[joint2_id].toVector().noalias() += A2.transpose() * mu_lambda.linear();
          }
        }

        const Scalar constraint_residual_norm =
          contact_acc_err.toVector().template lpNorm<Eigen::Infinity>();
        if (settings.absolute_residual < constraint_residual_norm)
          settings.absolute_residual = constraint_residual_norm;
      }

      if (settings.absolute_residual < settings.absolute_accuracy)
        break;

      // reduced backward sweep
      for (JointIndex j : joint_elimination_order)
      {
        if (data.constraints_supported_dim[j] > 0)
          ReducedPass2::run(
            model.joints[j], data.joints[j], typename ReducedPass2::ArgsType(model, data));
      }

      // reduced forward sweep
      data.oa_gf[0].setZero();
      for (int it = int(joint_elimination_order.size()) - 1; it >= 0; it--)
      {
        const JointIndex j = joint_elimination_order[size_t(it)];
        if (data.constraints_supported_dim[j] > 0)
        {
          ReducedPass3::run(
            model.joints[j], data.joints[j], typename ReducedPass3::ArgsType(model, data));
        }
      }
      data.ddq += data.g;
      // related to least-squares residual
      if (data.g.template lpNorm<Eigen::Infinity>() < settings.absolute_accuracy)
        break;
    }
    settings.iter = iter;

    // outward sweep after convergence/timeout for joints not supporting a constraint
    data.oa_gf[0] = -model.gravity;
    for (int it = int(joint_elimination_order.size()) - 1; it >= 0; --it)
    {
      const JointIndex j = joint_elimination_order[size_t(it)];
      if (data.constraints_supported_dim[j] == 0)
        Pass3::run(model.joints[j], data.joints[j], typename Pass3::ArgsType(model, data));
      else
        data.oa_gf[j] = data.oa[j];
    }

    return data.ddq;
  }

} // namespace pinocchio
