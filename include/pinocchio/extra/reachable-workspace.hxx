//
// Copyright (c) 2016-2023 CNRS INRIA
//

#ifndef __pinocchio_extra_reachable_workspace_hxx__
#define __pinocchio_extra_reachable_workspace_hxx__

#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/check.hpp"
#include "pinocchio/extra/reachable-workspace.hpp"

#include <vector>
#include <cmath>

#include <boost/math/special_functions/factorials.hpp>

namespace pinocchio
{
  template<
    typename Scalar,
    int Options,
    template<typename, int>
    class JointCollectionTpl,
    typename ConfigVectorType>
  void reachableWorkspace(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    const Eigen::MatrixBase<ConfigVectorType> & q0,
    const double time_horizon,
    const int frame_id,
    Eigen::MatrixXd & vertex,
    const ReachableSetParams & params)
  {
    PINOCCHIO_CHECK_ARGUMENT_SIZE(
      q0.size(), model.nq, "The configuration vector is not of the right size");

    auto f = [](
               const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
               DataTpl<Scalar, Options, JointCollectionTpl> & data) -> bool { return true; };

    internal::computeVertex(model, q0, time_horizon, frame_id, f, vertex, params);
  }

  template<
    typename Scalar,
    int Options,
    template<typename, int>
    class JointCollectionTpl,
    typename ConfigVectorType>
  void reachableWorkspaceHull(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    const Eigen::MatrixBase<ConfigVectorType> & q0,
    const double time_horizon,
    const int frame_id,
    ReachableSetResults & res,
    const ReachableSetParams & params)
  {
    reachableWorkspace(model, q0, time_horizon, frame_id, res.vertex, params);
    internal::buildConvexHull(res);
  }

#ifdef PINOCCHIO_WITH_HPP_FCL
  template<
    typename Scalar,
    int Options,
    template<typename, int>
    class JointCollectionTpl,
    typename ConfigVectorType>
  void reachableWorkspaceWithCollisions(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    const GeometryModel & geom_model,
    const Eigen::MatrixBase<ConfigVectorType> & q0,
    const double time_horizon,
    const int frame_id,
    Eigen::MatrixXd & vertex,
    const ReachableSetParams & params)
  {
    using namespace pinocchio::internal;

    PINOCCHIO_CHECK_ARGUMENT_SIZE(
      q0.size(), model.nq, "The configuration vector is not of the right size");

    GeometryData geom_data(geom_model);

    auto f = [&geom_model, &geom_data](
               const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
               DataTpl<Scalar, Options, JointCollectionTpl> & data) -> bool {
      updateGeometryPlacements(model, data, geom_model, geom_data);
      return !pinocchio::computeCollisions(geom_model, geom_data, true);
    };

    internal::computeVertex(model, q0, time_horizon, frame_id, f, vertex, params);
  }

  template<
    typename Scalar,
    int Options,
    template<typename, int>
    class JointCollectionTpl,
    typename ConfigVectorType>
  void reachableWorkspaceWithCollisionsHull(
    const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
    const GeometryModel & geom_model,
    const Eigen::MatrixBase<ConfigVectorType> & q0,
    const double time_horizon,
    const int frame_id,
    ReachableSetResults & res,
    const ReachableSetParams & params)
  {
    reachableWorkspaceWithCollisions(
      model, geom_model, q0, time_horizon, frame_id, res.vertex, params);
    internal::buildConvexHull(res);
  }
#endif // PINOCCHIO_WITH_HPP_FCL

  namespace internal
  {
    template<
      typename Scalar,
      int Options,
      template<typename, int>
      class JointCollectionTpl,
      typename ConfigVectorType,
      class FilterFunction>
    void computeVertex(
      const ModelTpl<Scalar, Options, JointCollectionTpl> & model,
      const Eigen::MatrixBase<ConfigVectorType> & q0,
      const double time_horizon,
      const int frame_id,
      FilterFunction config_filter,
      Eigen::MatrixXd & vertex,
      const ReachableSetParams & params)
    {
      using namespace Eigen;
      using namespace pinocchio;
      using Model = ModelTpl<Scalar, Options, JointCollectionTpl>;
      using Data = typename Model::Data;
      using VectorXs = typename Data::VectorXs;

      Data data(model);

      // // Get limits
      VectorXs dq_max = model.velocityLimit;
      VectorXs dq_min = -model.velocityLimit;

      double time_pin = 1; // pinocchio unit time in s used in integrate and difference
      double time_percent = time_horizon / time_pin;

      VectorXs upper_qv(model.nv);
      difference(model, q0, model.upperPositionLimit, upper_qv);
      VectorXs lower_qv(model.nv);
      difference(model, q0, model.lowerPositionLimit, lower_qv);
      VectorXs dq_ub = dq_max.cwiseMin(upper_qv / time_percent);
      VectorXs dq_lb = dq_min.cwiseMax(lower_qv / time_percent);

      // Combination variable
      int ncomb = static_cast<int>(
        boost::math::factorial<double>(static_cast<unsigned int>(model.nv))
        / boost::math::factorial<double>(static_cast<unsigned int>(params.facet_dims))
        / boost::math::factorial<double>(static_cast<unsigned int>(model.nv - params.facet_dims)));

      // Permutation Variable
      int perm_size = model.nv - params.facet_dims;
      VectorXd temp(2);
      temp << 1, 0;
      int n_ps = static_cast<int>(std::pow(temp.size(), perm_size));

      // Rng variable
      VectorXd x_rng =
        VectorXd::LinSpaced(params.n_samples, 0, params.n_samples - 1) / params.n_samples;
      int n_rng = static_cast<int>(std::pow(x_rng.size(), params.facet_dims));

      // Vertex resizing
      vertex.resize(3, n_ps * n_rng * ncomb);
      int c_vertex = 0;

      // Indices for loops
      VectorXi indicesPerm = VectorXi::Zero(perm_size);
      VectorXi indicesRng = VectorXi::Zero(params.facet_dims);

      // All vectors and matrices needed
      VectorXi comb = VectorXi::Zero(params.facet_dims);

      VectorXs DQ_ub_ind = VectorXs::Zero(perm_size);
      VectorXs DQ_lb_ind = VectorXs::Zero(perm_size);
      VectorXd perm = VectorXd::Zero(perm_size);
      VectorXd ones = VectorXd::Constant(perm_size, 1.);
      VectorXd resultsPerm = VectorXd::Zero(perm_size);

      VectorXd diff_dq = VectorXd::Zero(params.facet_dims);
      VectorXd reduced_dqlb = VectorXd::Zero(params.facet_dims);
      VectorXd combRng = VectorXd::Zero(params.facet_dims);
      VectorXd resultsRng = VectorXd::Zero(params.facet_dims);

      VectorXs qv = VectorXs::Zero(model.nv);
      VectorXs q = VectorXs::Zero(model.nq);
      for (int c = 0; c < ncomb; c++)
      {
        internal::generateCombination(model.nv, params.facet_dims, comb);
        // Generate all the matrices linked to the joint combination
        int count_true = 0;
        int count_false = 0;
        for (int k = 0; k < model.nv; k++)
        {
          if (k == comb[count_false])
          {
            diff_dq(count_false) = dq_ub(comb[count_false]) - dq_lb(comb[count_false]);
            reduced_dqlb(count_false) = dq_lb(comb[count_false]);
            if (count_false < params.facet_dims - 1)
              count_false++;
          }
          else
          {
            DQ_ub_ind(count_true) = dq_ub(k);
            DQ_lb_ind(count_true) = dq_lb(k);
            if (count_true < perm_size - 1)
              count_true++;
          }
        }
        for (int p = 0; p < n_ps; p++)
        {
          internal::productCombination(temp, perm_size, indicesPerm, perm);
          resultsPerm.noalias() = perm.cwiseProduct(DQ_ub_ind);
          ones.noalias() -= perm;
          resultsPerm.noalias() += ones.cwiseProduct(DQ_lb_ind);
          ones.setConstant(1.);
          for (int ng = 0; ng < n_rng; ng++)
          {
            internal::productCombination(x_rng, params.facet_dims, indicesRng, combRng);
            resultsRng.noalias() = diff_dq.asDiagonal() * combRng;
            resultsRng.noalias() += reduced_dqlb;

            // Compute Configuration Vector
            computeJointVel(resultsRng, resultsPerm, comb, qv);
            pinocchio::integrate(model, q0, qv * time_percent, q);
            framesForwardKinematics(model, data, q);
            // Store operational position as a point for hull computation
            if (config_filter(model, data))
            {
              vertex.col(c_vertex) = data.oMf[frame_id].translation();
              c_vertex++;
            }
          }
        }
      }
      vertex.conservativeResize(3, c_vertex);
    }

    void computeJointVel(
      const Eigen::VectorXd & res1,
      const Eigen::VectorXd & res2,
      const Eigen::VectorXi & comb,
      Eigen::VectorXd & qv)
    {
      int c_1 = 0;
      int c_2 = 0;
      for (int k = 0; k < static_cast<int>(qv.size()); k++)
      {
        if (k == comb[c_1])
        {
          qv(k) = res1(c_1);
          if (c_1 < res1.size() - 1)
            c_1++;
        }
        else
        {
          qv(k) = res2(c_2);
          if (c_2 < res2.size() - 1)
            c_2++;
        }
      }
    }

    void generateCombination(const int n, const int k, Eigen::VectorXi & indices)
    {
      int j = k - 1;
      while (j >= 0 && indices(j) == n - k + j)
        j--;

      if (j < 0)
        return;

      indices(j)++;
      for (int i = j + 1; i < k; i++)
        indices(i) = indices(i - 1) + 1;

      // Ensure each element is strictly greater than the previous one to avoid repetition
      for (int i = 1; i < k; i++)
        if (indices(i) <= indices(i - 1))
          indices(i) = indices(i - 1) + 1;
    }

    void productCombination(
      const Eigen::VectorXd & element,
      const int repeat,
      Eigen::VectorXi & indices,
      Eigen::VectorXd & combination)
    {
      for (int i = 0; i < repeat; ++i)
        combination(i) = element(indices(i));
      // Increment indices
      for (int i = repeat - 1; i >= 0; --i)
      {
        if (indices(i) < element.size() - 1)
        {
          indices(i)++;
          break;
        }
        else
        {
          indices(i) = 0;
        }
      }
    }
  } // namespace internal
} // namespace pinocchio

#endif // ifndef __pinocchio_extra_reachable_workspace_hxx__
