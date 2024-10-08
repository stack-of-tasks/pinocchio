//
// Copyright (c) 2020-2024 INRIA CNRS
// Copyright (c) 2023 KU Leuven
//

#ifndef __pinocchio_algorithm_contact_delassus_hxx__
#define __pinocchio_algorithm_contact_delassus_hxx__

#include "pinocchio/algorithm/check.hpp"
#include "pinocchio/algorithm/contact-info.hpp"
#include "pinocchio/algorithm/model.hpp"
#include "pinocchio/multibody/fwd.hpp"

namespace pinocchio
{

  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    typename ConfigVectorType>
  struct ComputeOSIMForwardStep
  : public fusion::JointUnaryVisitorBase<
      ComputeOSIMForwardStep<Scalar, Options, JointCollectionTpl, ConfigVectorType>>
  {
    typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
    typedef DataTpl<Scalar, Options, JointCollectionTpl> Data;

    typedef boost::fusion::vector<const Model &, Data &, const ConfigVectorType &> ArgsType;

    template<typename JointModel>
    static void algo(
      const pinocchio::JointModelBase<JointModel> & jmodel,
      pinocchio::JointDataBase<typename JointModel::JointDataDerived> & jdata,
      const Model & model,
      Data & data,
      const Eigen::MatrixBase<ConfigVectorType> & q)
    {
      typedef typename Model::JointIndex JointIndex;

      const JointIndex i = jmodel.id();
      jmodel.calc(jdata.derived(), q.derived());

      const JointIndex parent = model.parents[i];
      data.liMi[i] = model.jointPlacements[i] * jdata.M();
      if (parent > 0)
        data.oMi[i] = data.oMi[parent] * data.liMi[i];
      else
        data.oMi[i] = data.liMi[i];

      jmodel.jointCols(data.J) = data.oMi[i].act(jdata.S());
      data.oYcrb[i] = data.oMi[i].act(model.inertias[i]);
      data.oYaba[i] = data.oYcrb[i].matrix();
    }
  };

  template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
  struct ComputeOSIMBackwardStep
  : public fusion::JointUnaryVisitorBase<
      ComputeOSIMBackwardStep<Scalar, Options, JointCollectionTpl>>
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
      typedef typename Data::Inertia Inertia;
      typedef typename Data::Matrix6 Matrix6;
      typedef typename Data::Matrix6x Matrix6x;

      const JointIndex i = jmodel.id();
      const JointIndex parent = model.parents[i];
      typename Inertia::Matrix6 & Ia = data.oYaba[i];

      typedef typename SizeDepType<JointModel::NV>::template ColsReturn<Matrix6x>::Type ColBlock;
      ColBlock Jcols = jmodel.jointCols(data.J);

      jdata.U().noalias() = Ia * Jcols;
      jdata.StU().noalias() = Jcols.transpose() * jdata.U();

      // Account for the rotor inertia contribution
      jdata.StU().diagonal() += jmodel.jointVelocitySelector(model.armature);

      internal::PerformStYSInversion<Scalar>::run(jdata.StU(), jdata.Dinv());

      jdata.UDinv().noalias() =
        Jcols
        * jdata.Dinv().transpose(); //@justin can we remove the transpose since Dinv() is symmetric?
      data.oK[i].noalias() = jdata.UDinv() * Jcols.transpose();
      data.oL[i].noalias() = -jdata.UDinv() * jdata.U().transpose();
      data.oL[i] += Matrix6::Identity();

      if (parent > 0)
      {
        jdata.UDinv().noalias() = jdata.U() * jdata.Dinv();
        data.oYaba[parent] += Ia;
        data.oYaba[parent].noalias() -= jdata.UDinv() * jdata.U().transpose();
      }
    }
  };

  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    typename ConfigVectorType,
    class ModelAllocator,
    class DataAllocator,
    typename MatrixType>
  void computeDelassusMatrix(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const Eigen::MatrixBase<ConfigVectorType> & q,
    const std::vector<RigidConstraintModelTpl<Scalar, Options>, ModelAllocator> & contact_models,
    std::vector<RigidConstraintDataTpl<Scalar, Options>, DataAllocator> & contact_data,
    const Eigen::MatrixBase<MatrixType> & delassus_,
    const Scalar mu)
  {
    assert(model.check(data) && "data is not consistent with model.");
    PINOCCHIO_CHECK_ARGUMENT_SIZE(
      q.size(), model.nq, "The joint configuration vector is not of right size");
    PINOCCHIO_CHECK_INPUT_ARGUMENT(
      check_expression_if_real<Scalar>(mu >= Scalar(0)), "mu has to be positive");
    PINOCCHIO_CHECK_ARGUMENT_SIZE(
      contact_models.size(), contact_data.size(), "contact models and data size are not the same");

    MatrixType & delassus = delassus_.const_cast_derived();
    const size_t constraint_total_size = getTotalConstraintSize(contact_models);
    PINOCCHIO_CHECK_ARGUMENT_SIZE(delassus_.rows(), (Eigen::DenseIndex)constraint_total_size);
    PINOCCHIO_CHECK_ARGUMENT_SIZE(delassus_.cols(), (Eigen::DenseIndex)constraint_total_size);

    typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;
    typedef DataTpl<Scalar, Options, JointCollectionTpl> Data;

    typedef typename Model::JointIndex JointIndex;
    typedef typename Model::SE3 SE3;
    typedef typename Model::IndexVector IndexVector;
    typedef RigidConstraintModelTpl<Scalar, Options> RigidConstraintModel;
    typedef RigidConstraintDataTpl<Scalar, Options> RigidConstraintData;

    typedef ComputeOSIMForwardStep<Scalar, Options, JointCollectionTpl, ConfigVectorType> Pass1;
    for (JointIndex i = 1; i < (JointIndex)model.njoints; ++i)
    {
      Pass1::run(
        model.joints[i], data.joints[i], typename Pass1::ArgsType(model, data, q.derived()));
    }

    for (size_t k = 0; k < contact_models.size(); ++k)
    {
      const RigidConstraintModel & cmodel = contact_models[k];
      RigidConstraintData & cdata = contact_data[k];

      const JointIndex joint1_id = cmodel.joint1_id;

      // Compute relative placement between the joint and the contact frame
      SE3 & oMc = cdata.oMc1;
      oMc = data.oMi[joint1_id] * cmodel.joint1_placement; // contact placement

      typedef typename Data::Inertia Inertia;
      typedef typename Inertia::Symmetric3 Symmetric3;

      // Add contact inertia to the joint articulated inertia
      Symmetric3 S(Symmetric3::Zero());
      if (cmodel.type == CONTACT_6D)
        S.setDiagonal(Symmetric3::Vector3::Constant(mu));

      const Inertia contact_inertia(mu, oMc.translation(), S);
      data.oYaba[joint1_id] += contact_inertia.matrix();
    }

    typedef ComputeOSIMBackwardStep<Scalar, Options, JointCollectionTpl> Pass2;
    for (JointIndex i = (JointIndex)model.njoints - 1; i > 0; --i)
    {
      Pass2::run(model.joints[i], data.joints[i], typename Pass2::ArgsType(model, data));
    }

    Eigen::DenseIndex current_row_id = 0;
    for (size_t k = 0; k < contact_models.size(); ++k)
    {
      typedef typename RigidConstraintData::VectorOfMatrix6 VectorOfMatrix6;
      const RigidConstraintModel & cmodel = contact_models[k];
      RigidConstraintData & cdata = contact_data[k];

      const JointIndex joint1_id = cmodel.joint1_id;
      const SE3 & oMc1 = cdata.oMc1;
      const IndexVector & support1 = model.supports[joint1_id];

      {
        VectorOfMatrix6 & propagators = cdata.extended_motion_propagators_joint1;
        VectorOfMatrix6 & lambdas = cdata.lambdas_joint1;
        switch (cmodel.type)
        {
        case CONTACT_3D: {
          oMc1.toActionMatrixInverse(propagators.back());
          propagators.back().template bottomRows<3>().setZero();
          for (size_t j = support1.size() - 1; j > 1; --j)
          {
            lambdas[j].template leftCols<3>().noalias() =
              data.oK[(size_t)support1[j]] * propagators[j].template topRows<3>().transpose();
            propagators[j - 1].template topRows<3>().noalias() =
              propagators[j].template topRows<3>() * data.oL[(size_t)support1[j]];
          }
          lambdas[1].template leftCols<3>().noalias() =
            data.oK[(size_t)support1[1]] * propagators[1].template topRows<3>().transpose();

          for (size_t j = 2; j < support1.size(); ++j)
          {
            lambdas[j].template leftCols<3>().noalias() +=
              data.oL[(size_t)support1[j]] * lambdas[j - 1].template leftCols<3>();
          }
          break;
        }
        case CONTACT_6D: {
          oMc1.toActionMatrixInverse(propagators.back());
          for (size_t j = support1.size() - 1; j > 1; --j)
          {
            lambdas[j].noalias() = data.oK[(size_t)support1[j]] * propagators[j].transpose();
            propagators[j - 1].noalias() = propagators[j] * data.oL[(size_t)support1[j]];
          }
          lambdas[1].noalias() = data.oK[(size_t)support1[1]] * propagators[1].transpose();

          for (size_t j = 2; j < support1.size(); ++j)
          {
            lambdas[j].noalias() += data.oL[(size_t)support1[j]] * lambdas[j - 1];
          }
          break;
        }
        default: {
          assert(false && "must never happened");
          break;
        }
        }
      }

      // Fill the delassus matrix block-wise
      {
        const int size = cmodel.size();

        const VectorOfMatrix6 & propagators = cdata.extended_motion_propagators_joint1;
        const VectorOfMatrix6 & lambdas = cdata.lambdas_joint1;

        Eigen::DenseIndex current_row_id_other = 0;
        for (size_t i = 0; i < k; ++i)
        {
          const RigidConstraintModel & cmodel_other = contact_models[i];
          const RigidConstraintData & cdata_other = contact_data[i];
          const int size_other = cmodel_other.size();
          const IndexVector & support1_other = model.supports[cmodel_other.joint1_id];

          const VectorOfMatrix6 & propagators_other =
            cdata_other.extended_motion_propagators_joint1;

          size_t id_in_support1, id_in_support1_other;
          findCommonAncestor(
            model, joint1_id, cmodel_other.joint1_id, id_in_support1, id_in_support1_other);

          delassus.block(current_row_id_other, current_row_id, size_other, size).noalias() =
            propagators_other[id_in_support1_other].topRows(size_other)
            * lambdas[id_in_support1].leftCols(size);

          current_row_id_other += size_other;
        }

        assert(current_row_id_other == current_row_id && "current row indexes do not match.");
        delassus.block(current_row_id, current_row_id, size, size).noalias() =
          propagators.back().topRows(size) * lambdas.back().leftCols(size);
        current_row_id += size;
      }
    }
    assert(
      current_row_id == delassus.rows()
      && "current row indexes do not the number of rows in the Delassus matrix.");
  }

  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    class Allocator>
  inline void initPvDelassus(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const std::vector<RigidConstraintModelTpl<Scalar, Options>, Allocator> & contact_models)
  {
    std::fill(data.constraints_supported_dim.begin(), data.constraints_supported_dim.end(), 0);
    for (std::size_t i = 0; i < contact_models.size(); ++i)
    {
      const RigidConstraintModelTpl<Scalar, Options> & contact_model = contact_models[i];
      const JointIndex & joint_id = contact_model.joint1_id;
      switch (contact_model.reference_frame)
      {
      case LOCAL:
        if (contact_model.type == CONTACT_6D)
          data.constraints_supported_dim[joint_id] += 6;
        else if (contact_model.type == CONTACT_3D)
          data.constraints_supported_dim[joint_id] += 3;
        else
          assert(false && "Must never happen");
        break;
      case WORLD:
        assert(false && "WORLD not implemented");
        break;
      case LOCAL_WORLD_ALIGNED:
        assert(false && "LOCAL_WORLD_ALIGNED not implemented");
        break;
      default:
        assert(false && "Must never happen");
        break;
      }
      data.accumulation_descendant[joint_id] = joint_id;

      data.constraints_on_joint[joint_id].push_back(i);
    }

    // Running backprop to get the count of constraints
    for (JointIndex i = (JointIndex)model.njoints - 1; i > 0; --i)
    {
      const JointIndex & parent = model.parents[i];
      data.constraints_supported_dim[parent] += data.constraints_supported_dim[i];
      for (std::size_t constraint : data.constraints_on_joint[i])
        data.constraints_supported[i].insert(constraint);
      for (std::size_t constraint : data.constraints_supported[i])
        data.constraints_supported[parent].insert(
          constraint); // For loops, will need to use a set data-structure
    }
    // Assign accumulation descendants for branching nodes
    for (JointIndex i = (JointIndex)model.njoints - 1; i > 0; --i)
    {
      const JointIndex & parent = model.parents[i];
      if (data.constraints_supported_dim[i] > 0)
      {
        data.joints_supporting_constraints.push_back(i);
        data.accumulation_descendant[parent] = data.accumulation_descendant[i];
        if (data.constraints_supported_dim[parent] > data.constraints_supported_dim[i])
        {
          data.accumulation_descendant[parent] = parent;
        }
      }
    }

    for (JointIndex i = 1; i < (JointIndex)model.njoints; i++)
    {
      if (data.accumulation_descendant[i] == i)
        data.accumulation_joints.push_back(i);
    }
    // Assign accumulation ancestors
    for (JointIndex i = (JointIndex)model.njoints - 1; i > 0; --i)
    {
      const JointIndex & parent = model.parents[i];
      if (data.accumulation_descendant[parent] == parent)
      {
        if (data.accumulation_descendant[i] != 0)
          data.accumulation_ancestor[data.accumulation_descendant[i]] = parent;
      }
    }
  }

  template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
  JointIndex findGreatestCommonAncestor(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    const DataTpl<Scalar, Options, JointCollectionTpl> & data,
    JointIndex joint1_id,
    JointIndex joint2_id,
    size_t & index_ancestor_in_support1,
    size_t & index_ancestor_in_support2)
  {
    PINOCCHIO_CHECK_INPUT_ARGUMENT(
      joint1_id < (JointIndex)model.njoints, "joint1_id is not valid.");
    PINOCCHIO_CHECK_INPUT_ARGUMENT(
      joint2_id < (JointIndex)model.njoints, "joint2_id is not valid.");

    if (joint1_id == 0 || joint2_id == 0)
    {
      index_ancestor_in_support1 = index_ancestor_in_support2 = 0;
      return 0;
    }

    index_ancestor_in_support1 = 0;
    index_ancestor_in_support2 = 0;

    if (data.constraints_supported[joint1_id].size() > 1)
      index_ancestor_in_support1++;
    if (data.constraints_supported[joint2_id].size() > 1)
      index_ancestor_in_support2++;

    while (joint1_id != joint2_id)
    {
      if (joint1_id > joint2_id)
      {
        joint1_id = data.accumulation_ancestor[joint1_id];
        index_ancestor_in_support1++;
      }
      else
      {
        joint2_id = data.accumulation_ancestor[joint2_id];
        index_ancestor_in_support2++;
      }
    }

    index_ancestor_in_support1--;
    index_ancestor_in_support2--;

    return joint1_id;
  }

  template<typename Scalar, int Options, template<typename, int> class JointCollectionTpl>
  struct ComputePvDelassusBackwardStep
  : public fusion::JointUnaryVisitorBase<
      ComputeOSIMBackwardStep<Scalar, Options, JointCollectionTpl>>
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
      typedef typename Data::Inertia Inertia;
      typedef typename Data::Matrix6x Matrix6x;

      const JointIndex i = jmodel.id();
      const JointIndex parent = model.parents[i];
      typename Inertia::Matrix6 & Ia = data.oYaba[i];

      typedef typename SizeDepType<JointModel::NV>::template ColsReturn<Matrix6x>::Type ColBlock;
      ColBlock Jcols = jmodel.jointCols(data.J);

      jdata.U().noalias() = Ia * Jcols;
      jdata.StU().noalias() = Jcols.transpose() * jdata.U();

      // Account for the rotor inertia contribution
      jdata.StU().diagonal() += jmodel.jointVelocitySelector(model.armature);

      internal::PerformStYSInversion<Scalar>::run(jdata.StU(), jdata.Dinv());

      jdata.UDinv().noalias() = Jcols * jdata.Dinv().transpose();
      data.oL[i].setIdentity();
      data.oL[i].noalias() -= jdata.UDinv() * jdata.U().transpose();

      if (parent > 0)
      {
        data.oYaba[parent].noalias() = Ia - jdata.UDinv() * jdata.U().transpose();
      }
    }
  };

  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    typename ConfigVectorType,
    class ModelAllocator,
    class DataAllocator,
    typename MatrixType>
  void computePvDelassusMatrix(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const Eigen::MatrixBase<ConfigVectorType> & q,
    const std::vector<RigidConstraintModelTpl<Scalar, Options>, ModelAllocator> & contact_models,
    std::vector<RigidConstraintDataTpl<Scalar, Options>, DataAllocator> & contact_data,
    const Eigen::MatrixBase<MatrixType> & delassus_,
    const Scalar mu)
  {
    assert(model.check(data) && "data is not consistent with model.");
    PINOCCHIO_CHECK_ARGUMENT_SIZE(
      q.size(), model.nq, "The joint configuration vector is not of right size");
    PINOCCHIO_CHECK_INPUT_ARGUMENT(
      check_expression_if_real<Scalar>(mu >= Scalar(0)), "mu has to be positive");
    PINOCCHIO_CHECK_ARGUMENT_SIZE(
      contact_models.size(), contact_data.size(), "contact models and data size are not the same");

    MatrixType & delassus = delassus_.const_cast_derived();
    const size_t constraint_total_size = getTotalConstraintSize(contact_models);
    PINOCCHIO_CHECK_ARGUMENT_SIZE(delassus_.rows(), (Eigen::DenseIndex)constraint_total_size);
    PINOCCHIO_CHECK_ARGUMENT_SIZE(delassus_.cols(), (Eigen::DenseIndex)constraint_total_size);

    typedef DataTpl<Scalar, Options, JointCollectionTpl> Data;

    typename Data::Vector6 scratch_pad_vector = Data::Vector6::Zero();
    typename Data::Vector6 scratch_pad_vector2 = Data::Vector6::Zero();

    typename Data::Matrix6 scratch_pad1;
    typename Data::Matrix6 scratch_pad2;

    typedef ModelTpl<Scalar, Options, JointCollectionTpl> Model;

    typedef typename Model::JointIndex JointIndex;
    typedef typename Model::SE3 SE3;
    typedef RigidConstraintModelTpl<Scalar, Options> RigidConstraintModel;
    typedef RigidConstraintDataTpl<Scalar, Options> RigidConstraintData;

    typedef ComputeOSIMForwardStep<Scalar, Options, JointCollectionTpl, ConfigVectorType> Pass1;
    for (JointIndex i = 1; i < (JointIndex)model.njoints; ++i)
    {
      Pass1::run(
        model.joints[i], data.joints[i], typename Pass1::ArgsType(model, data, q.derived()));
    }

    for (size_t k = 0; k < contact_models.size(); ++k)
    {
      const RigidConstraintModel & cmodel = contact_models[k];
      RigidConstraintData & cdata = contact_data[k];

      const JointIndex joint1_id = cmodel.joint1_id;

      // Compute relative placement between the joint and the contact frame
      SE3 & oMc = cdata.oMc1;
      oMc = data.oMi[joint1_id] * cmodel.joint1_placement; // contact placement

      typedef typename Data::Inertia Inertia;
      typedef typename Inertia::Symmetric3 Symmetric3;

      // Add contact inertia to the joint articulated inertia
      Symmetric3 S(Symmetric3::Zero());
      if (cmodel.type == CONTACT_6D)
        S.setDiagonal(Symmetric3::Vector3::Constant(mu));

      const Inertia contact_inertia(mu, oMc.translation(), S);
      data.oYaba[joint1_id] += contact_inertia.matrix();

      typedef typename RigidConstraintData::VectorOfMatrix6 VectorOfMatrix6;

      VectorOfMatrix6 & propagators = cdata.extended_motion_propagators_joint1;
      switch (cmodel.type)
      {
      case CONTACT_3D: {
        oMc.toActionMatrixInverse(propagators[0]);
        propagators[0].template bottomRows<3>().setZero();
        break;
      }
      case CONTACT_6D: {
        oMc.toActionMatrixInverse(propagators[0]);
        break;
      }

      default: {
        assert(false && "must never happen");
        break;
      }
      }
    }

    // Initialize motion propagator and spatial_inv_inertia at all the accumulation joints
    for (JointIndex i : data.accumulation_joints)
    {

      if (data.constraints_supported[i].size() == 1)
      {
        size_t constraint = data.constraints_on_joint[i][0];
        data.extended_motion_propagator[i] =
          contact_data[constraint].extended_motion_propagators_joint1[0];
      }
      else
      {
        data.extended_motion_propagator[i].setIdentity();
      }
      data.spatial_inv_inertia[i].setZero();
    }

    typedef ComputePvDelassusBackwardStep<Scalar, Options, JointCollectionTpl> Pass2;
    for (JointIndex i = (JointIndex)model.njoints - 1; i > 0; --i)
    {
      Pass2::run(model.joints[i], data.joints[i], typename Pass2::ArgsType(model, data));

      if (data.constraints_supported_dim[i] > 0)
      {
        size_t ad_i = data.accumulation_descendant[i];
        int nv = model.joints[i].nv();
        // propagate the spatial inverse inertia
        if (nv == 1)
        {
          // Optimization for single DoF joints
          if (data.constraints_supported_dim[ad_i] != 3)
          {
            // When propagating 6D constraints
            scratch_pad_vector.noalias() =
              data.extended_motion_propagator[i] * model.joints[i].jointCols(data.J);
            scratch_pad_vector2.noalias() = scratch_pad_vector * data.joints[i].Dinv().coeff(0, 0);
            data.spatial_inv_inertia[ad_i].noalias() +=
              scratch_pad_vector2 * scratch_pad_vector.transpose();
          }
          else
          {
            // Propagating 3D constraints
            scratch_pad_vector.template topRows<3>().noalias() =
              data.extended_motion_propagator[i].template topRows<3>()
              * model.joints[i].jointCols(data.J);
            scratch_pad_vector2.template topRows<3>().noalias() =
              scratch_pad_vector.template topRows<3>() * data.joints[i].Dinv().coeff(0, 0);
            data.spatial_inv_inertia[ad_i].template topLeftCorner<3, 3>().noalias() +=
              scratch_pad_vector2.template topRows<3>()
              * scratch_pad_vector.template topRows<3>().transpose();
          }
        }
        else if (nv == 6)
        {
          scratch_pad1.noalias() =
            data.extended_motion_propagator[i] * model.joints[i].jointCols(data.J);
          scratch_pad2.noalias() = scratch_pad1 * data.joints[i].Dinv();
          data.spatial_inv_inertia[ad_i].noalias() += scratch_pad2 * scratch_pad1.transpose();
        }
        else if (nv > 1)
        {
          // Joints with more than 1 DoF
          scratch_pad1.leftCols(nv).noalias() =
            data.extended_motion_propagator[i] * model.joints[i].jointCols(data.J);
          scratch_pad2.leftCols(nv).noalias() = scratch_pad1.leftCols(nv) * data.joints[i].Dinv();
          data.spatial_inv_inertia[ad_i].noalias() +=
            scratch_pad2.leftCols(nv) * scratch_pad1.leftCols(nv).transpose();
        }
        else
        {
          assert(false && "must never happen");
        }
        // propagate the EMP
        const JointIndex parent = model.parents[i];
        size_t par_ad_i = data.accumulation_descendant[parent];
        if (parent > 0 && par_ad_i == ad_i)
        {

          if (data.constraints_supported_dim[ad_i] != 3)
          {
            data.extended_motion_propagator[parent].noalias() =
              data.extended_motion_propagator[i] * data.oL[i];
          }
          else
          {
            data.extended_motion_propagator[parent].template topRows<3>().noalias() =
              data.extended_motion_propagator[i].template topRows<3>() * data.oL[i];
          }
        }
        else if (par_ad_i != ad_i) // TODO: optimize for 3D constraints
        {
          data.extended_motion_propagator2[ad_i].noalias() =
            data.extended_motion_propagator[i] * data.oL[i];
        }
      }
    }

    for (JointIndex i : data.accumulation_joints)
    {

      size_t an_i = data.accumulation_ancestor[i];
      if (an_i != 0) // TODO: optimize for 3D constraints. Not very important here.
      {
        scratch_pad1.noalias() =
          data.extended_motion_propagator2[i] * data.spatial_inv_inertia[an_i];
        data.spatial_inv_inertia[i].noalias() +=
          scratch_pad1 * data.extended_motion_propagator2[i].transpose();
      }
    }

    typedef typename RigidConstraintData::VectorOfMatrix6 VectorOfMatrix6;
    // compute EMP in the reverse direction
    for (size_t k = 0; k < contact_models.size(); ++k)
    {
      VectorOfMatrix6 & cemp = contact_data[k].extended_motion_propagators_joint1;
      JointIndex curr_joint = contact_models[k].joint1_id;

      JointIndex an_i;

      if (data.constraints_supported[curr_joint].size() == 1)
      {
        cemp[0] = data.extended_motion_propagator2[curr_joint];
        an_i = data.accumulation_ancestor[curr_joint];
      }
      else
      {
        an_i = curr_joint;
      }

      size_t counter = 1;
      while (an_i > 1)
      {
        if (contact_models[k].size() != 3)
          cemp[counter].noalias() = cemp[counter - 1] * data.extended_motion_propagator2[an_i];
        else
          cemp[counter].template topRows<3>().noalias() =
            cemp[counter - 1].template topRows<3>() * data.extended_motion_propagator2[an_i];
        curr_joint = an_i;
        an_i = data.accumulation_ancestor[curr_joint];
        counter++;
      }
    }

    Eigen::DenseIndex current_row_id = 0;
    for (size_t k = 0; k < contact_models.size(); ++k)
    {
      const RigidConstraintModel & cmodel = contact_models[k];
      RigidConstraintData & cdata = contact_data[k];
      const VectorOfMatrix6 & cemp = cdata.extended_motion_propagators_joint1;

      const JointIndex joint1_id = cmodel.joint1_id;

      // Fill the delassus matrix block-wise
      {
        const int size = cmodel.size();

        Eigen::DenseIndex current_row_id_other = 0;
        for (size_t i = 0; i < k; ++i)
        {
          RigidConstraintData & cdata_other = contact_data[i];
          const RigidConstraintModel & cmodel_other = contact_models[i];
          const int size_other = cmodel_other.size();
          const JointIndex joint1_id_other = cmodel_other.joint1_id;

          const VectorOfMatrix6 & cemp_other = cdata_other.extended_motion_propagators_joint1;

          size_t id_in_support1, id_in_support1_other, gca;
          gca = findGreatestCommonAncestor(
            model, data, joint1_id, joint1_id_other, id_in_support1, id_in_support1_other);

          scratch_pad1.noalias() = cemp_other[id_in_support1_other] * data.spatial_inv_inertia[gca];
          if (size == 6)
          {
            if (size_other == 6)
              delassus.template block<6, 6>(current_row_id_other, current_row_id).noalias() =
                scratch_pad1 * cemp[id_in_support1].transpose();
            else if (size_other == 3)
              delassus.template block<3, 6>(current_row_id_other, current_row_id).noalias() =
                scratch_pad1.template topRows<3>()
                * cemp[id_in_support1].template topRows<6>().transpose();
          }
          else if (size == 3)
          {
            if (size_other == 6)
              delassus.template block<6, 3>(current_row_id_other, current_row_id).noalias() =
                scratch_pad1.template topRows<6>()
                * cemp[id_in_support1].template topRows<3>().transpose();
            else if (size_other == 3)
              delassus.template block<3, 3>(current_row_id_other, current_row_id).noalias() =
                scratch_pad1.template topRows<3>()
                * cemp[id_in_support1].template topRows<3>().transpose();
          }
          else
          {
            delassus.block(current_row_id_other, current_row_id, size_other, size).noalias() =
              scratch_pad1.topRows(size_other) * cemp[id_in_support1].topRows(size).transpose();
          }
          current_row_id_other += size_other;
        }

        assert(current_row_id_other == current_row_id && "current row indexes do not match.");
        if (
          data.constraints_supported_dim[joint1_id] > 6
          || data.constraints_supported[joint1_id].size() > 1)
        {
          scratch_pad1.noalias() = cemp[0] * data.spatial_inv_inertia[joint1_id];
          delassus.block(current_row_id, current_row_id, size, size).noalias() =
            scratch_pad1.topRows(size) * cemp[0].topRows(size).transpose();
        }
        else
        {
          if (size == 6)
            delassus.template block<6, 6>(current_row_id, current_row_id) =
              data.spatial_inv_inertia[joint1_id];
          else if (size == 3)
            delassus.template block<3, 3>(current_row_id, current_row_id) =
              data.spatial_inv_inertia[joint1_id].template topLeftCorner<3, 3>();
          else
            delassus.block(current_row_id, current_row_id, size, size) =
              data.spatial_inv_inertia[joint1_id].topLeftCorner(size, size);
        }
        current_row_id += size;
      }
    }
    assert(
      current_row_id == delassus.rows()
      && "current row indexes do not the number of rows in the Delassus matrix.");
  }

  template<
    typename Scalar,
    int Options,
    template<typename, int> class JointCollectionTpl,
    typename ConfigVectorType,
    class ModelAllocator,
    class DataAllocator,
    typename MatrixType>
  void computeDampedDelassusMatrixInverse(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    DataTpl<Scalar, Options, JointCollectionTpl> & data,
    const Eigen::MatrixBase<ConfigVectorType> & q,
    const std::vector<RigidConstraintModelTpl<Scalar, Options>, ModelAllocator> & contact_models,
    std::vector<RigidConstraintDataTpl<Scalar, Options>, DataAllocator> & contact_data,
    const Eigen::MatrixBase<MatrixType> & damped_delassus_inverse_,
    const Scalar mu,
    const bool scaled,
    const bool Pv)
  {
    PINOCCHIO_CHECK_INPUT_ARGUMENT(
      check_expression_if_real<Scalar>(mu >= Eigen::NumTraits<Scalar>::dummy_precision()),
      "mu is too small.");

    const Scalar mu_inv = Scalar(1) / mu;
    MatrixType & damped_delassus_inverse = damped_delassus_inverse_.const_cast_derived();

    const Scalar mu_inv_square = mu_inv * mu_inv;
    assert(
      check_expression_if_real<Scalar>(mu_inv_square != std::numeric_limits<Scalar>::infinity())
      && "mu_inv**2 is equal to infinity.");

    if (Pv)
      computePvDelassusMatrix(
        model, data, q, contact_models, contact_data, damped_delassus_inverse, mu_inv);
    else
      computeDelassusMatrix(
        model, data, q, contact_models, contact_data, damped_delassus_inverse, mu_inv);

    damped_delassus_inverse *= -mu_inv;
    damped_delassus_inverse.diagonal().array() += Scalar(1);
    if (!scaled)
      damped_delassus_inverse *= mu_inv;
  }

} // namespace pinocchio

#endif // ifndef __pinocchio_algorithm_contact_delassus_hxx__
