//
// Copyright (c) 2016-2023 CNRS INRIA
//

#ifndef __pinocchio_extra_reachable_workspace_hpp__
#define __pinocchio_extra_reachable_workspace_hpp__

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/multibody/geometry.hpp"
#include "pinocchio/algorithm/geometry.hpp"
#include "pinocchio/extra/config.hpp"

#ifdef PINOCCHIO_WITH_HPP_FCL
  #include "pinocchio/collision/collision.hpp"
#endif // PINOCCHIO_WITH_HPP_FCL

#include <Eigen/Core>

namespace pinocchio
{
  /// @brief Structure containing the return value for the reachable algorithm
  /// \param vertex Matrix where all vertex coordinates will be stored.
  /// \param faces Matrix where index of vertices linked to each facets will be stored.
  struct ReachableSetResults
  {
    Eigen::MatrixXd vertex;
    Eigen::MatrixXi faces;
  };

  /// @brief Parameters for the reachable space algorithm
  /// \param n_samples The number of samples to use for the discretization of the joint velocity
  /// space. The higher the number of samples, the more accurate the reachable set will be, however
  /// the longer the computation time will be \param facet_dims The dimension of the facet that will
  /// be sampled. Between 0 and the number of DOF of the robot.  The higher the number of samples,
  /// the more accurate the reachable set will be, however the longer the computation time will be
  /// \param convex Whether to make the reachable set convex or not. If set to True, the reachable
  /// set will be convex, if False the reachable set will be non-convex.
  struct ReachableSetParams
  {
    int n_samples = 5;
    int facet_dims = 3;
  };

  /// \brief Computes the reachable workspace on a fixed time horizon. For more information, please
  /// see https://gitlab.inria.fr/auctus-team/people/antunskuric/pycapacity
  ///
  /// \tparam JointCollection Collection of Joint types.
  /// \tparam ConfigVectorType Type of the joint configuration vector.

  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] q The initial joint configuration vector (dim model.nq).
  /// \param[in] frame_id Index of the frame for which the workspace should be computed.
  /// \param[in] time_horizon: time horizon for which the polytope will be computed (in seconds)
  /// \param[in] params parameters of the algorithm

  /// \param[out] points inside of the reachable workspace
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
    const ReachableSetParams & params = ReachableSetParams());

  /// \brief Computes the convex Hull reachable workspace on a fixed time horizon. For more
  /// information, please see https://gitlab.inria.fr/auctus-team/people/antunskuric/pycapacity
  ///
  /// \tparam JointCollection Collection of Joint types.
  /// \tparam ConfigVectorType Type of the joint configuration vector.

  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] q The initial joint configuration vector (dim model.nq).
  /// \param[in] frame_id Index of the frame for which the workspace should be computed.
  /// \param[in] time_horizon: time horizon for which the polytope will be computed (in seconds)
  /// \param[in] params parameters of the algorithm

  /// \param[out] res Results of algorithm
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
    const ReachableSetParams & params = ReachableSetParams());

#ifdef PINOCCHIO_WITH_HPP_FCL
  /// \brief Computes the reachable workspace with respect to a geometry model on a fixed time
  /// horizon. For more information, please see
  /// https://gitlab.inria.fr/auctus-team/people/antunskuric/pycapacity
  ///
  /// \tparam JointCollection Collection of Joint types.
  /// \tparam ConfigVectorType Type of the joint configuration vector.

  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] q The initial joint configuration vector (dim model.nq).
  /// \param[in] frame_id Index of the frame for which the workspace should be computed.
  /// \param[in] time_horizon: time horizon for which the polytope will be computed (in seconds)
  /// \param[in] params parameters of the algorithm

  /// \param[out] points inside of the reachable workspace
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
    const ReachableSetParams & params = ReachableSetParams());

  /// \brief Computes the convex Hull of the reachable workspace with respect to a geometry model on
  /// a fixed time horizon. Make sure that reachable workspace takes into account collisions with
  /// environment. For more information, please see
  /// https://gitlab.inria.fr/auctus-team/people/antunskuric/pycapacity
  ///
  /// \tparam JointCollection Collection of Joint types.
  /// \tparam ConfigVectorType Type of the joint configuration vector.

  /// \param[in] model The model structure of the rigid body system.
  /// \param[in] geom_model: Geometry model, to take into accout collision with the environment
  /// \param[in] q The initial joint configuration vector (dim model.nq).
  /// \param[in] frame_id Index of the frame for which the workspace should be computed.
  /// \param[in] time_horizon: time horizon for which the polytope will be computed (in seconds)
  /// \param[in] params parameters of the algorithm

  /// \param[out] res Results of algorithm
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
    const ReachableSetParams & params = ReachableSetParams());
#endif // PINOCCHIO_WITH_HPP_FCL

  namespace internal
  {
    /// \brief Samples points to create the reachable workspace that will respect mechanical limits
    /// of the model as well as the time horizon
    ///
    /// \tparam JointCollection Collection of Joint types.
    /// \tparam ConfigVectorType Type of the joint configuration vector.
    /// \tparam FilterFunction Function template use to filter points in the workspace. Prototype :
    /// f(model, data) -> bool
    ///
    /// \param[in] model The model structure of the rigid body system.
    /// \param[in] geom_model: Geometry model associated with the model
    /// \param[in] q The initial joint configuration vector (dim model.nq).
    /// \param[in] time_horizon: time horizon for which the polytope will be computed (in seconds)
    /// \param[in] frame_id Index of the frame for which the workspace should be computed.
    /// \param[in] params parameters of the algorithm

    /// \param[out] vertex Results of algorithm
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
      const ReachableSetParams & params = ReachableSetParams());

    /// \brief Computes the convex hull using qhull associated with the vertex stored in res
    /// \param[out] res Contain both the points and the faces of the convex hull
    PINOCCHIO_EXTRA_DLLAPI void buildConvexHull(ReachableSetResults & res);

    /// \brief Computes the joint configuration associated with the permutation results stored in
    /// res1 and res2 \param[in] res1 First permutation result \param[in] res2 Second permutation
    /// result \param[in] comb Vector of active joints in this configuration

    /// \param[out] qv Joint Velocity result
    void computeJointVel(
      const Eigen::VectorXd & res1,
      const Eigen::VectorXd & res2,
      const Eigen::VectorXi & comb,
      Eigen::VectorXd & qv);

    /// \brief Return a subsequence of length k of elements from range 0 to n. Inspired by
    /// https://docs.python.org/3/library/itertools.html#itertools.combinations. Indices table will
    /// hold the results \param[in] n Max range of element \param[in] k length of subsequences

    /// \param[out] indices results of the combination
    void generateCombination(const int n, const int k, Eigen::VectorXi & indices);

    /// \brief Cartesian product of input element with itself. Number of repetition is specified
    /// with repeat argument. Inspired by
    /// https://docs.python.org/3/library/itertools.html#itertools.product \param[in] element Vector
    /// for which the cartesian product is needed. \param[in] repeat Number of repetition \param[in]
    /// indices Vector of indexes of which element will be repeated (will be changed during function
    /// call)

    /// \param[out] combination Cartesian Product associated with the indices
    void productCombination(
      const Eigen::VectorXd & element,
      const int repeat,
      Eigen::VectorXi & indices,
      Eigen::VectorXd & combination);
  } // namespace internal
} // namespace pinocchio

/* --- Details -------------------------------------------------------------------- */
#include "pinocchio/extra/reachable-workspace.hxx"

#endif // ifndef __pinocchio_extra_reachable_workspace_hpp__
