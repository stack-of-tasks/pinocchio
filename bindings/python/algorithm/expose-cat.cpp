//
// Copyright (c) 2015-2021 CNRS INRIA
//

#include "pinocchio/bindings/python/algorithm/algorithms.hpp"
#include "pinocchio/algorithm/compute-all-terms.hpp"
#include "pinocchio/bindings/python/utils/model-checker.hpp"

namespace pinocchio
{
  namespace python
  {
    static void computeAllTerms_proxy(
      const context::Model & model,
      context::Data & data,
      const context::VectorXs & q,
      const context::VectorXs & v)
    {
      data.M.fill(context::Scalar(0));
      computeAllTerms(model, data, q, v);
      data.M.triangularView<Eigen::StrictlyLower>() =
        data.M.transpose().triangularView<Eigen::StrictlyLower>();
    }

    void exposeCAT()
    {
      bp::def(
        "computeAllTerms", computeAllTerms_proxy, bp::args("model", "data", "q", "v"),
        "Compute all the terms M, non linear effects, center of mass quantities, centroidal "
        "quantities and Jacobians in"
        "in the same loop and store the results in data.\n"
        "This algorithm is equivalent to calling:\n"
        "\t- forwardKinematics\n"
        "\t- crba\n"
        "\t- nonLinearEffects\n"
        "\t- computeJointJacobians\n"
        "\t- centerOfMass\n"
        "\t- jacobianCenterOfMass\n"
        "\t- ccrba\n"
        "\t- computeKineticEnergy\n"
        "\t- computePotentialEnergy\n"
        "\t- computeGeneralizedGravity\n"
        "Parameters:\n"
        "\tmodel: model of the kinematic tree\n"
        "\tdata: data related to the model\n"
        "\tq: the joint configuration vector (size model.nq)\n"
        "\tv: the joint velocity vector (size model.nv)\n",
        mimic_not_supported_function<>(0));
    }
  } // namespace python
} // namespace pinocchio
