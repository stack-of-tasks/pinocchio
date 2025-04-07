//
// Copyright (c) 2015-2021 CNRS INRIA
//

#include "pinocchio/bindings/python/algorithm/algorithms.hpp"
#include "pinocchio/bindings/python/utils/namespace.hpp"

#include "pinocchio/algorithm/aba.hpp"
#include "pinocchio/bindings/python/utils/eigen.hpp"
#include "pinocchio/bindings/python/utils/model-checker.hpp"

namespace pinocchio
{
  namespace python
  {

    const context::Data::RowMatrixXs & computeMinverse_proxy(
      const context::Model & model, context::Data & data, const context::VectorXs & q)
    {
      computeMinverse(model, data, q);
      make_symmetric(data.Minv);
      return data.Minv;
    }

    const context::Data::RowMatrixXs &
    computeMinverse_min_proxy(const context::Model & model, context::Data & data)
    {
      pinocchio::computeMinverse(model, data);
      make_symmetric(data.Minv);
      return data.Minv;
    }

    void exposeABA()
    {
      typedef context::Scalar Scalar;
      typedef context::VectorXs VectorXs;
      enum
      {
        Options = context::Options
      };

      bp::def(
        "computeMinverse", &computeMinverse_proxy, bp::args("model", "data", "q"),
        "Computes the inverse of the joint space inertia matrix using an extension of the "
        "Articulated Body algorithm.\n"
        "The result is stored in data.Minv.\n"
        "Parameters:\n"
        "\t model: Model of the kinematic tree\n"
        "\t data: Data related to the kinematic tree\n"
        "\t q: joint configuration (size model.nq)",
        mimic_not_supported_function<bp::return_value_policy<bp::return_by_value>>(0));

      bp::def(
        "aba", &aba<Scalar, Options, JointCollectionDefaultTpl, VectorXs, VectorXs, VectorXs>,
        (bp::arg("model"), bp::arg("data"), bp::arg("q"), bp::arg("v"), bp::arg("tau"),
         bp::arg("convention") = pinocchio::Convention::LOCAL),
        "Compute ABA, store the result in data.ddq and return it.\n"
        "Parameters:\n"
        "\t model: Model of the kinematic tree\n"
        "\t data: Data related to the kinematic tree\n"
        "\t q: joint configuration (size model.nq)\n"
        "\t tau: joint velocity (size model.nv)\n"
        "\t v: joint torque (size model.nv)"
        "\t convention: Convention to use",
        mimic_not_supported_function<bp::return_value_policy<bp::return_by_value>>(0));

      bp::def(
        "aba",
        &aba<
          Scalar, Options, JointCollectionDefaultTpl, VectorXs, VectorXs, VectorXs, context::Force>,
        (bp::arg("model"), bp::arg("data"), bp::arg("q"), bp::arg("v"), bp::arg("tau"),
         bp::arg("fext"), bp::arg("convention") = pinocchio::Convention::LOCAL),
        "Compute ABA with external forces, store the result in data.ddq and return it.\n"
        "Parameters:\n"
        "\t model: Model of the kinematic tree\n"
        "\t data: Data related to the kinematic tree\n"
        "\t q: joint configuration (size model.nq)\n"
        "\t v: joint velocity (size model.nv)\n"
        "\t tau: joint torque (size model.nv)\n"
        "\t fext: vector of external forces expressed in the local frame of the joint (size "
        "model.njoints)"
        "\t convention: Convention to use",
        mimic_not_supported_function<bp::return_value_policy<bp::return_by_value>>(0));

      bp::def(
        "computeMinverse", &computeMinverse_min_proxy, bp::args("model", "data"),
        "Computes the inverse of the joint space inertia matrix using an extension of the "
        "Articulated Body algorithm.\n"
        "The result is stored in data.Minv.\n"
        "Remarks: pinocchio.aba should have been called first.\n"
        "Parameters:\n"
        "\t model: Model of the kinematic tree\n"
        "\t data: Data related to the kinematic tree",
        mimic_not_supported_function<bp::return_value_policy<bp::return_by_value>>(0));
    }

  } // namespace python
} // namespace pinocchio
