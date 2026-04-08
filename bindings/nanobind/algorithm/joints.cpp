// Copyright (c) 2026 INRIA
//
// This file is a port of: bindings/python/algorithm/expose-joints.cpp

#include "pinocchio/bindings/python-nb/fwd.hpp"

#include "pinocchio/algorithm/joint-configuration.hpp"

#include <nanobind/eigen/dense.h>

PINOCCHIO_PYTHON_NAMESPACE_BEGIN
using namespace nb::literals;

void exposeJointsAlgo(nb::module_ m)
{
  using ConstMatrixRef = Eigen::Ref<const MatrixXs>;

  m.def(
    "integrate",
    [](const Model & model, ConstVectorRef q, ConstVectorRef v) -> VectorXs {
      return integrate(model, q, v);
    },
    "model"_a, "q"_a, "v"_a,
    "Integrate the joint configuration vector q with a tangent vector v during one unit time.\n"
    "This is the canonical integrator of a Configuration Space composed of Lie groups, such as "
    "most robots.\n\n"
    "Parameters:\n"
    "\t model: model of the kinematic tree\n"
    "\t q: the joint configuration vector (size model.nq)\n"
    "\t v: the joint velocity vector (size model.nv)");

  m.def(
    "dIntegrate",
    [](const Model & model, ConstVectorRef q, ConstVectorRef v) -> nb::tuple {
      MatrixXs J0(MatrixXs::Zero(model.nv, model.nv));
      MatrixXs J1(MatrixXs::Zero(model.nv, model.nv));
      dIntegrate(model, q, v, J0, ARG0);
      dIntegrate(model, q, v, J1, ARG1);
      return nb::make_tuple(std::move(J0), std::move(J1));
    },
    "model"_a, "q"_a, "v"_a,
    "Computes the partial derivatives of the integrate function with respect to the first "
    "and the second argument, and returns the two Jacobians as a tuple.\n\n"
    "Parameters:\n"
    "\t model: model of the kinematic tree\n"
    "\t q: the joint configuration vector (size model.nq)\n"
    "\t v: the joint velocity vector (size model.nv)");

  m.def(
    "dIntegrate",
    [](const Model & model, ConstVectorRef q, ConstVectorRef v, ArgumentPosition arg) -> MatrixXs {
      MatrixXs J(MatrixXs::Zero(model.nv, model.nv));
      dIntegrate(model, q, v, J, arg);
      return J;
    },
    "model"_a, "q"_a, "v"_a, "argument_position"_a,
    "Computes the partial derivatives of the integrate function with respect to the first "
    "(arg == ARG0) or the second argument (arg == ARG1).\n\n"
    "Parameters:\n"
    "\t model: model of the kinematic tree\n"
    "\t q: the joint configuration vector (size model.nq)\n"
    "\t v: the joint velocity vector (size model.nv)\n"
    "\t argument_position: either pinocchio.ArgumentPosition.ARG0 or "
    "pinocchio.ArgumentPosition.ARG1, depending on the desired Jacobian value.");

  m.def(
    "dIntegrateTransport",
    [](
      const Model & model, ConstVectorRef q, ConstVectorRef v, ConstMatrixRef Jin,
      ArgumentPosition arg) -> MatrixXs {
      MatrixXs Jout(MatrixXs::Zero(model.nv, Jin.cols()));
      dIntegrateTransport(model, q, v, Jin, Jout, arg);
      return Jout;
    },
    "model"_a, "q"_a, "v"_a, "Jin"_a, "argument_position"_a,
    "Takes a matrix expressed at q (+) v and uses parallel transport to express it in the "
    "tangent space at q. "
    "This operation does the product of the matrix by the Jacobian of the integration "
    "operation, but more efficiently.\n"
    "Parameters:\n"
    "\t model: model of the kinematic tree\n"
    "\t q: the joint configuration vector (size model.nq)\n"
    "\t v: the joint velocity vector (size model.nv)\n"
    "\t Jin: the input matrix (row size model.nv)\n"
    "\t argument_position: either pinocchio.ArgumentPosition.ARG0 (q) or "
    "pinocchio.ArgumentPosition.ARG1 (v), depending on the desired Jacobian value.");

  m.def(
    "interpolate",
    [](const Model & model, ConstVectorRef q1, ConstVectorRef q2, Scalar u) -> VectorXs {
      return interpolate(model, q1, q2, u);
    },
    "model"_a, "q1"_a, "q2"_a, "alpha"_a,
    "Interpolate between two given joint configuration vectors q1 and q2.\n\n"
    "Parameters:\n"
    "\t model: model of the kinematic tree\n"
    "\t q1: the initial joint configuration vector (size model.nq)\n"
    "\t q2: the terminal joint configuration vector (size model.nq)\n"
    "\t alpha: the interpolation coefficient in [0,1]");

  m.def(
    "difference",
    [](const Model & model, ConstVectorRef q1, ConstVectorRef q2) -> VectorXs {
      return difference(model, q1, q2);
    },
    "model"_a, "q1"_a, "q2"_a,
    "Difference between two joint configuration vectors, i.e. the tangent vector that must be "
    "integrated during one unit time to go from q1 to q2.\n\n"
    "Parameters:\n"
    "\t model: model of the kinematic tree\n"
    "\t q1: the initial joint configuration vector (size model.nq)\n"
    "\t q2: the terminal joint configuration vector (size model.nq)");

  m.def(
    "squaredDistance",
    [](const Model & model, ConstVectorRef q1, ConstVectorRef q2) -> VectorXs {
      return squaredDistance(model, q1, q2);
    },
    "model"_a, "q1"_a, "q2"_a,
    "Squared distance vector between two joint configuration vectors.\n\n"
    "Parameters:\n"
    "\t model: model of the kinematic tree\n"
    "\t q1: the initial joint configuration vector (size model.nq)\n"
    "\t q2: the terminal joint configuration vector (size model.nq)");

  m.def(
    "distance",
    [](const Model & model, ConstVectorRef q1, ConstVectorRef q2) -> Scalar {
      return distance(model, q1, q2);
    },
    "model"_a, "q1"_a, "q2"_a,
    "Distance between two joint configuration vectors.\n\n"
    "Parameters:\n"
    "\t model: model of the kinematic tree\n"
    "\t q1: the initial joint configuration vector (size model.nq)\n"
    "\t q2: the terminal joint configuration vector (size model.nq)");

  m.def(
    "dDifference",
    [](const Model & model, ConstVectorRef q1, ConstVectorRef q2) -> nb::tuple {
      MatrixXs J0(MatrixXs::Zero(model.nv, model.nv));
      MatrixXs J1(MatrixXs::Zero(model.nv, model.nv));
      dDifference(model, q1, q2, J0, ARG0);
      dDifference(model, q1, q2, J1, ARG1);
      return nb::make_tuple(std::move(J0), std::move(J1));
    },
    "model"_a, "q1"_a, "q2"_a,
    "Computes the partial derivatives of the difference function with respect to the first "
    "and the second argument, and returns the two Jacobians as a tuple.\n\n"
    "Parameters:\n"
    "\t model: model of the kinematic tree\n"
    "\t q1: the initial joint configuration vector (size model.nq)\n"
    "\t q2: the terminal joint configuration vector (size model.nq)");

  m.def(
    "dDifference",
    [](
      const Model & model, ConstVectorRef q1, ConstVectorRef q2, ArgumentPosition arg) -> MatrixXs {
      MatrixXs J(MatrixXs::Zero(model.nv, model.nv));
      dDifference(model, q1, q2, J, arg);
      return J;
    },
    "model"_a, "q1"_a, "q2"_a, "argument_position"_a,
    "Computes the partial derivatives of the difference function with respect to the first "
    "(arg == ARG0) or the second argument (arg == ARG1).\n\n"
    "Parameters:\n"
    "\t model: model of the kinematic tree\n"
    "\t q1: the initial joint configuration vector (size model.nq)\n"
    "\t q2: the terminal joint configuration vector (size model.nq)\n"
    "\t argument_position: either pinocchio.ArgumentPosition.ARG0 or "
    "pinocchio.ArgumentPosition.ARG1, depending on the desired Jacobian value.");

  m.def(
    "tangentMap",
    [](const Model & model, ConstVectorRef q) -> MatrixXs {
      MatrixXs TM(MatrixXs::Zero(model.nq, model.nv));
      tangentMap(model, q, TM, SETTO);
      return TM;
    },
    "model"_a, "q"_a,
    "Computes the tangent map in configuration q that maps a small variation expressed in the "
    "Lie algebra as a small variation in the parametric space.\n\n"
    "Parameters:\n"
    "\t model: model of the kinematic tree\n"
    "\t q: the joint configuration vector (size model.nq)");

  m.def(
    "compactTangentMap",
    [](const Model & model, ConstVectorRef q) -> MatrixXs {
      MatrixXs TMc(MatrixXs::Zero(model.nq, MAX_JOINT_NV));
      std::vector<Model::JointIndex> joint_ids;
      for (Model::JointIndex i = 1; i < (Model::JointIndex)model.njoints; ++i)
        joint_ids.push_back(i);
      compactTangentMap(model, joint_ids, q, TMc);
      return TMc;
    },
    "model"_a, "q"_a,
    "Computes the tangent map in configuration q that maps a small variation expressed in the "
    "Lie algebra as a small variation in the parametric space. Stores the result in a compact "
    "manner that can be exploited using getTangentToConfigurationSparsitySegment.\n\n"
    "Parameters:\n"
    "\t model: model of the kinematic tree\n"
    "\t q: the joint configuration vector (size model.nq)");

  m.def(
    "tangentMapProduct",
    [](const Model & model, ConstVectorRef q, ConstMatrixRef mat_in) -> MatrixXs {
      MatrixXs mat_out(MatrixXs::Zero(model.nq, mat_in.cols()));
      tangentMapProduct(model, q, mat_in, mat_out, SETTO);
      return mat_out;
    },
    "model"_a, "q"_a, "mat_in"_a,
    "Apply the tangent map to a matrix mat_in.\n\n"
    "Parameters:\n"
    "\t model: model of the kinematic tree\n"
    "\t q: the joint configuration vector (size model.nq)\n"
    "\t mat_in: a matrix (size model.nv, ncols)");

  m.def(
    "tangentMapTransposeProduct",
    [](const Model & model, ConstVectorRef q, ConstMatrixRef mat_in) -> MatrixXs {
      MatrixXs mat_out(MatrixXs::Zero(model.nv, mat_in.cols()));
      tangentMapTransposeProduct(model, q, mat_in, mat_out, SETTO);
      return mat_out;
    },
    "model"_a, "q"_a, "mat_in"_a,
    "Apply the transpose of the tangent map to a matrix mat_in.\n\n"
    "Parameters:\n"
    "\t model: model of the kinematic tree\n"
    "\t q: the joint configuration vector (size model.nq)\n"
    "\t mat_in: a matrix (size model.nq, ncols)");

  m.def(
    "randomConfiguration",
    [](const Model & model) -> VectorXs { return randomConfiguration(model); }, "model"_a,
    "Generate a random configuration in the bounds given by the lower and upper limits "
    "contained in the model.\n\n"
    "Parameters:\n"
    "\t model: model of the kinematic tree");

  m.def(
    "randomConfiguration",
    [](const Model & model, ConstVectorRef lower_bound, ConstVectorRef upper_bound) -> VectorXs {
      return randomConfiguration(model, lower_bound, upper_bound);
    },
    "model"_a, "lower_bound"_a, "upper_bound"_a,
    "Generate a random configuration in the bounds given by the joint lower and upper limits "
    "arguments.\n\n"
    "Parameters:\n"
    "\t model: model of the kinematic tree\n"
    "\t lower_bound: the lower bound on the joint configuration vectors (size model.nq)\n"
    "\t upper_bound: the upper bound on the joint configuration vectors (size model.nq)");

  m.def(
    "neutral", [](const Model & model) -> VectorXs { return neutral(model); }, "model"_a,
    "Returns the neutral configuration vector associated to the model.\n\n"
    "Parameters:\n"
    "\t model: model of the kinematic tree");

  m.def(
    "normalize",
    [](const Model & model, ConstVectorRef q) -> VectorXs {
      VectorXs qcopy(q);
      normalize(model, qcopy);
      return qcopy;
    },
    "model"_a, "q"_a,
    "Returns the normalized configuration.\n"
    "For instance, when the configuration vector contains quaternion values, it may be "
    "required to renormalize these components to keep orthonormal rotation values.\n\n"
    "Parameters:\n"
    "\t model: model of the kinematic tree\n"
    "\t q: a joint configuration vector to normalize (size model.nq)");

  m.def(
    "lieGroup", [](const Model & model) -> auto { return lieGroup(model); }, "model"_a,
    "Returns the Lie group associated to the model. It is the cartesian product of the lie "
    "groups of all its joints.\n\n"
    "Parameters:\n"
    "\tmodel: model of the kinematic tree");

  m.def(
    "getTangentToConfigurationSparsitySegment",
    [](const Model & model) -> nb::tuple {
      std::vector<int> nvs;
      std::vector<int> idx_vs;
      std::vector<Model::JointIndex> joint_ids;
      for (Model::JointIndex i = 1; i < (Model::JointIndex)model.njoints; ++i)
        joint_ids.push_back(i);
      getTangentToConfigurationSparsitySegment(model, joint_ids, nvs, idx_vs);
      return nb::make_tuple(std::move(nvs), std::move(idx_vs));
    },
    "model"_a,
    "Returns two vectors that give for each q_i the associated idx_v and nv of the joint for "
    "which q_i is a configuration component.\n\n"
    "Parameters:\n"
    "\t model: model of the kinematic tree");

  static const Scalar dummy_precision = Eigen::NumTraits<Scalar>::dummy_precision();

  m.def(
    "isSameConfiguration",
    [](const Model & model, ConstVectorRef q1, ConstVectorRef q2, Scalar prec) -> bool {
      return isSameConfiguration(model, q1, q2, prec);
    },
    "model"_a, "q1"_a, "q2"_a, "prec"_a,
    "Return true if two configurations are equivalent within the given precision.\n\n"
    "Parameters:\n"
    "\t model: model of the kinematic tree\n"
    "\t q1: a joint configuration vector (size model.nq)\n"
    "\t q2: a joint configuration vector (size model.nq)\n"
    "\t prec: requested accuracy for the comparison");

  m.def(
    "isNormalized",
    [](const Model & model, ConstVectorRef q, Scalar prec) -> bool {
      return isNormalized(model, q, prec);
    },
    "model"_a, "q"_a, "prec"_a = dummy_precision,
    "Check whether a configuration vector is normalized within the given precision.\n\n"
    "Parameters:\n"
    "\t model: model of the kinematic tree\n"
    "\t q: a joint configuration vector (size model.nq)\n"
    "\t prec: requested accuracy for the check");
}
PINOCCHIO_PYTHON_NAMESPACE_END;
