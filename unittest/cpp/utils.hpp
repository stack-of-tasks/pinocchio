#pragma once

#include "pinocchio/macros.hpp"
#include "pinocchio/multibody.hpp"
#include "pinocchio/multibody/sample-models.hpp"
#include "pinocchio/constraints.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/aba.hpp"

#include <vector>

namespace pinocchio
{
  namespace unittest
  {

    template<typename Scalar, int Options = 0>
    struct ConstrainedHumanoidScene
    {
      typedef ModelTpl<Scalar, Options, JointCollectionDefaultTpl> Model;
      typedef DataTpl<Scalar, Options, JointCollectionDefaultTpl> Data;

      // Use the variant types
      typedef pinocchio::ConstraintModelTpl<Scalar, Options> ConstraintModel;
      typedef pinocchio::ConstraintDataTpl<Scalar, Options> ConstraintData;

      typedef std::vector<ConstraintModel> ConstraintModelVector;
      typedef std::vector<ConstraintData> ConstraintDataVector;

      typedef Eigen::Matrix<Scalar, Eigen::Dynamic, 1> VectorXs;

      Model model;
      Data data;
      Eigen::Matrix<Scalar, Eigen::Dynamic, 1> q;
      ConstraintModelVector constraint_models;
      ConstraintDataVector constraint_datas;

      Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> Minv;
      Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> M;
      Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>
        delassus_matrix_gt; // Dense Delassus J * Minv * J^T
      VectorXs compliance;

      ConstrainedHumanoidScene(
        const bool has_contacts,
        const bool has_joint_limits = false,
        const bool has_joint_friction = false,
        const bool has_point_anchor = false,
        const bool has_frame_anchor = false)
      : model()
      , data()
      {
        pinocchio::buildModels::humanoidRandom(model, true);
        model.lowerPositionLimit.tail(model.nq - 7).fill(-1.);
        model.upperPositionLimit.tail(model.nq - 7).fill(+1.);

        data = Data(model);
        q = pinocchio::neutral(model);

        const std::string RF = "rleg6_joint";
        const std::string LF = "lleg6_joint";
        if (has_contacts)
        {

          if (model.existJointName(RF) && model.existJointName(LF))
          {
            typedef PointContactConstraintModelTpl<Scalar, Options> PointContactConstraintModel;

            const typename Model::JointIndex RF_id = model.getJointId(RF);
            const typename Model::JointIndex LF_id = model.getJointId(LF);

            PointContactConstraintModel cm_RF(model, RF_id, SE3Tpl<Scalar, Options>::Random());
            cm_RF.setCompliance(VectorXs::Random(cm_RF.residualSize()).cwiseAbs());
            constraint_models.push_back(cm_RF);
            constraint_datas.push_back(cm_RF.createData());

            PointContactConstraintModel cm_LF(model, LF_id, SE3Tpl<Scalar, Options>::Random());
            cm_LF.setCompliance(VectorXs::Random(cm_RF.residualSize()).cwiseAbs());
            constraint_models.push_back(cm_LF);
            constraint_datas.push_back(cm_LF.createData());
          }
          else
          {
            PINOCCHIO_THROW(std::runtime_error, "Joint does not exist");
          }
        }

        if (has_point_anchor)
        {
          if (model.existJointName(RF) && model.existJointName(LF))
          {
            typedef PointAnchorConstraintModelTpl<Scalar, Options> PointAnchorConstraintModel;

            const typename Model::JointIndex RF_id = model.getJointId(RF);
            const typename Model::JointIndex LF_id = model.getJointId(LF);

            PointAnchorConstraintModel cm_RF(model, RF_id, SE3Tpl<Scalar, Options>::Random());
            cm_RF.setCompliance(VectorXs::Random(cm_RF.residualSize()).cwiseAbs());
            constraint_models.push_back(cm_RF);
            constraint_datas.push_back(cm_RF.createData());

            PointAnchorConstraintModel cm_LF(model, LF_id, SE3Tpl<Scalar, Options>::Random());
            cm_LF.setCompliance(VectorXs::Random(cm_LF.residualSize()).cwiseAbs());
            constraint_models.push_back(cm_LF);
            constraint_datas.push_back(cm_LF.createData());
          }
          else
          {
            PINOCCHIO_THROW(std::runtime_error, "Joint does not exist");
          }
        }

        if (has_frame_anchor)
        {
          if (model.existJointName(LF))
          {
            typedef PointAnchorConstraintModelTpl<Scalar, Options> FrameAnchorConstraintModel;

            const typename Model::JointIndex RF_id = model.getJointId(RF);
            const typename Model::JointIndex LF_id = model.getJointId(LF);

            FrameAnchorConstraintModel cm_RF(model, RF_id, SE3Tpl<Scalar, Options>::Random());
            cm_RF.setCompliance(VectorXs::Random(cm_RF.residualSize()).cwiseAbs());
            constraint_models.push_back(cm_RF);
            constraint_datas.push_back(cm_RF.createData());

            FrameAnchorConstraintModel cm_LF(model, LF_id, SE3Tpl<Scalar, Options>::Random());
            cm_LF.setCompliance(VectorXs::Random(cm_LF.residualSize()).cwiseAbs());
            constraint_models.push_back(cm_LF);
            constraint_datas.push_back(cm_LF.createData());
          }
          else
          {
            PINOCCHIO_THROW(std::runtime_error, "Joint does not exist");
          }
        }

        if (has_joint_limits)
        {
          typedef JointLimitConstraintModelTpl<Scalar, Options> JointLimitConstraintModel;
          typename Model::IndexVector activable_joint_ids;
          for (typename Model::JointIndex i = 2; i < (typename Model::JointIndex)model.njoints; ++i)
          {
            activable_joint_ids.push_back(i);
          }
          JointLimitConstraintModel cm_limits(model, activable_joint_ids);
          cm_limits.setCompliance(VectorXs::Random(cm_limits.residualSize()).cwiseAbs());
          constraint_models.push_back(cm_limits);
          constraint_datas.push_back(cm_limits.createData());
        }

        if (has_joint_friction)
        {
          typedef JointFrictionConstraintModelTpl<Scalar, Options> JointFrictionConstraintModel;
          typename Model::IndexVector activable_joint_ids;
          for (typename Model::JointIndex i = 2; i < (typename Model::JointIndex)model.njoints; ++i)
          {
            activable_joint_ids.push_back(i);
          }
          JointFrictionConstraintModel cm_friction(model, activable_joint_ids);
          cm_friction.setCompliance(VectorXs::Random(cm_friction.residualSize()).cwiseAbs());
          constraint_models.push_back(cm_friction);
          constraint_datas.push_back(cm_friction.createData());
        }

        computeGroundTruth();
      }

      void computeGroundTruth()
      {
        // 1. Compute M and Minv
        // We use a separate data to ensure clean state
        Data data_gt(model);
        M = crba(model, data_gt, q, Convention::WORLD);
        M.template triangularView<Eigen::StrictlyLower>() =
          M.transpose().template triangularView<Eigen::StrictlyLower>();

        Minv = computeMinverse(model, data_gt, q);
        Minv.template triangularView<Eigen::StrictlyLower>() =
          Minv.transpose().template triangularView<Eigen::StrictlyLower>();

        // 2. Compute Jacobian of constraints
        // Create new constraint datas for GT to avoid side effects
        ConstraintDataVector constraint_datas_gt;
        for (const auto & cm : constraint_models)
        {
          constraint_datas_gt.push_back(cm.createData());
        }

        computeJointJacobians(model, data_gt, q);
        data_gt.q_in = q;
        calc(
          model, data_gt, constraint_models,
          constraint_datas_gt); // update J inside constraint_datas_gt

        const Eigen::Index total_dim = residualSize(constraint_models);

        if (total_dim > 0)
        {
          Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> J(total_dim, model.nv);
          J.setZero();
          getConstraintsJacobian(model, data_gt, constraint_models, constraint_datas_gt, J);
          delassus_matrix_gt = J * Minv * J.transpose();
        }
        else
        {
          delassus_matrix_gt.resize(0, 0);
        }

        // 3. Ensure the main data and constraint_datas are also up-to-date
        computeJointJacobians(model, data, q);
        data.q_in = q;
        calc(model, data, constraint_models, constraint_datas);

        // Compute the constraints compliance vector
        compliance.resize(getTotalConstraintResidualSize(constraint_models));
        retrieveConstraintCompliance(constraint_models, compliance);
      }
    };

  } // namespace unittest
} // namespace pinocchio
