// Copyright (c) 2026 INRIA

#pragma once

#include "../fwd.hpp"
#include "pinocchio/multibody/liegroup.hpp"

#include <nanobind/stl/string.h>
#include <nanobind/eigen/dense.h>

PINOCCHIO_PYTHON_NAMESPACE_BEGIN

namespace
{
  using nb::literals::operator""_a;
}

/// Static helper methods wrapping LieGroupType member functions that use
/// Eigen template arguments
template<class LieGroupType>
struct LieGroupWrapperTpl
{
  using ConfigVectorType = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;
  using TangentVectorType = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;
  using JacobianMatrixType = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>;

  static ConfigVectorType
  integrate(const LieGroupType & lg, const ConfigVectorType & q, const TangentVectorType & v)
  {
    return lg.integrate(q, v);
  }

  static ConfigVectorType interpolate(
    const LieGroupType & lg,
    const ConfigVectorType & q0,
    const ConfigVectorType & q1,
    const Scalar & u)
  {
    return lg.interpolate(q0, q1, u);
  }

  static TangentVectorType
  difference(const LieGroupType & lg, const ConfigVectorType & q0, const ConfigVectorType & q1)
  {
    return lg.difference(q0, q1);
  }

  static JacobianMatrixType dDifference1(
    const LieGroupType & lg,
    const ConfigVectorType & q0,
    const ConfigVectorType & q1,
    const ArgumentPosition arg)
  {
    JacobianMatrixType J(lg.nv(), lg.nv());
    lg.dDifference(q0, q1, J, arg);
    return J;
  }

  static JacobianMatrixType dDifference2(
    const LieGroupType & lg,
    const ConfigVectorType & q0,
    const ConfigVectorType & q1,
    const ArgumentPosition arg,
    const JacobianMatrixType & Jin,
    int self)
  {
    JacobianMatrixType J(Jin.rows(), Jin.cols());
    switch (arg)
    {
    case ARG0:
      lg.template dDifference<ARG0>(q0, q1, Jin, self, J, SETTO);
      break;
    case ARG1:
      lg.template dDifference<ARG1>(q0, q1, Jin, self, J, SETTO);
      break;
    default:
      throw std::invalid_argument("arg must be either ARG0 or ARG1");
    }
    return J;
  }

  static JacobianMatrixType dDifference3(
    const LieGroupType & lg,
    const ConfigVectorType & q0,
    const ConfigVectorType & q1,
    const ArgumentPosition arg,
    int self,
    const JacobianMatrixType & Jin)
  {
    JacobianMatrixType J(Jin.rows(), Jin.cols());
    switch (arg)
    {
    case ARG0:
      lg.template dDifference<ARG0>(q0, q1, self, Jin, J, SETTO);
      break;
    case ARG1:
      lg.template dDifference<ARG1>(q0, q1, self, Jin, J, SETTO);
      break;
    default:
      throw std::invalid_argument("arg must be either ARG0 or ARG1");
    }
    return J;
  }

  static JacobianMatrixType dIntegrate(
    const LieGroupType & lg,
    const ConfigVectorType & q,
    const TangentVectorType & v,
    const ArgumentPosition arg)
  {
    JacobianMatrixType J(lg.nv(), lg.nv());
    lg.dIntegrate(q, v, J, arg);
    return J;
  }

  static JacobianMatrixType
  dIntegrate_dq1(const LieGroupType & lg, const ConfigVectorType & q, const TangentVectorType & v)
  {
    JacobianMatrixType J(lg.nv(), lg.nv());
    lg.dIntegrate_dq(q, v, J);
    return J;
  }

  static JacobianMatrixType dIntegrate_dq2(
    const LieGroupType & lg,
    const ConfigVectorType & q,
    const TangentVectorType & v,
    const JacobianMatrixType & Jin,
    int self)
  {
    JacobianMatrixType J(Jin.rows(), lg.nv());
    lg.dIntegrate_dq(q, v, Jin, self, J, SETTO);
    return J;
  }

  static JacobianMatrixType dIntegrate_dq3(
    const LieGroupType & lg,
    const ConfigVectorType & q,
    const TangentVectorType & v,
    int self,
    const JacobianMatrixType & Jin)
  {
    JacobianMatrixType J(lg.nv(), Jin.cols());
    lg.dIntegrate_dq(q, v, self, Jin, J, SETTO);
    return J;
  }

  static JacobianMatrixType
  dIntegrate_dv1(const LieGroupType & lg, const ConfigVectorType & q, const TangentVectorType & v)
  {
    JacobianMatrixType J(lg.nv(), lg.nv());
    lg.dIntegrate_dv(q, v, J);
    return J;
  }

  static JacobianMatrixType dIntegrate_dv2(
    const LieGroupType & lg,
    const ConfigVectorType & q,
    const TangentVectorType & v,
    const JacobianMatrixType & Jin,
    int self)
  {
    JacobianMatrixType J(Jin.rows(), lg.nv());
    lg.dIntegrate_dv(q, v, Jin, self, J, SETTO);
    return J;
  }

  static JacobianMatrixType dIntegrate_dv3(
    const LieGroupType & lg,
    const ConfigVectorType & q,
    const TangentVectorType & v,
    int self,
    const JacobianMatrixType & Jin)
  {
    JacobianMatrixType J(lg.nv(), Jin.cols());
    lg.dIntegrate_dv(q, v, self, Jin, J, SETTO);
    return J;
  }

  static JacobianMatrixType dIntegrateTransport_proxy(
    const LieGroupType & lg,
    const ConfigVectorType & q,
    const TangentVectorType & v,
    const JacobianMatrixType & J,
    const ArgumentPosition arg)
  {
    JacobianMatrixType Jout(lg.nv(), J.cols());
    lg.dIntegrateTransport(q, v, J, Jout, arg);
    return Jout;
  }

  // second proxy for numpy 1D arrays because nanobind is less permissive than eigenpy
  static TangentVectorType dIntegrateTransport_proxy_vec(
    const LieGroupType & lg,
    const ConfigVectorType & q,
    const TangentVectorType & v,
    Eigen::Ref<const TangentVectorType> vin,
    const ArgumentPosition arg)
  {
    TangentVectorType vout(vin.size());
    lg.dIntegrateTransport(q, v, vin, vout, arg);
    return vout;
  }

  static JacobianMatrixType tangentMap(const LieGroupType & lg, const ConfigVectorType & q)
  {
    JacobianMatrixType TM(lg.nq(), lg.nv());
    lg.tangentMap(q, TM, SETTO);
    return TM;
  }

  static JacobianMatrixType tangentMapProduct(
    const LieGroupType & lg, const ConfigVectorType & q, const JacobianMatrixType & Min)
  {
    JacobianMatrixType Mout(lg.nq(), Min.cols());
    lg.tangentMapProduct(q, Min, Mout, SETTO);
    return Mout;
  }

  static JacobianMatrixType tangentMapTransposeProduct(
    const LieGroupType & lg, const ConfigVectorType & q, const JacobianMatrixType & Min)
  {
    JacobianMatrixType Mout(lg.nv(), Min.cols());
    lg.tangentMapTransposeProduct(q, Min, Mout, SETTO);
    return Mout;
  }
};

// Template function to expose specific Lie groups.
// In main bindings, used only to expose the generic AKA Cartesian product Lie group.
// Can be reused for finer-grained bindings.
template<class LieGroupType>
void exposeLieGroup(nb::module_ m, const char * name)
{
  using LGW = LieGroupWrapperTpl<LieGroupType>;
  using ConfigVectorType = typename LGW::ConfigVectorType;

  nb::class_<LieGroupType>(m, name)
    .def(nb::init<>(), "Default constructor.")
    .def("integrate", &LGW::integrate, "q"_a, "v"_a, "Integrate a tangent vector v at q.")
    .def(
      "dIntegrate", &LGW::dIntegrate, "q"_a, "v"_a, "arg"_a,
      "Compute the Jacobian of the integrate operation.")
    .def(
      "dIntegrate_dq", &LGW::dIntegrate_dq1, "q"_a, "v"_a,
      "Compute the Jacobian of integrate with respect to q.")
    .def(
      "dIntegrate_dq", &LGW::dIntegrate_dq2, "q"_a, "v"_a, "Jin"_a, "self"_a,
      "Compute the product Jin * J where J = dIntegrate_dq.")
    .def(
      "dIntegrate_dq", &LGW::dIntegrate_dq3, "q"_a, "v"_a, "self"_a, "Jin"_a,
      "Compute the product J * Jin where J = dIntegrate_dq.")
    .def(
      "dIntegrate_dv", &LGW::dIntegrate_dv1, "q"_a, "v"_a,
      "Compute the Jacobian of integrate with respect to v.")
    .def(
      "dIntegrate_dv", &LGW::dIntegrate_dv2, "q"_a, "v"_a, "Jin"_a, "self"_a,
      "Compute the product Jin * J where J = dIntegrate_dv.")
    .def(
      "dIntegrate_dv", &LGW::dIntegrate_dv3, "q"_a, "v"_a, "self"_a, "Jin"_a,
      "Compute the product J * Jin where J = dIntegrate_dv.")
    .def(
      "dIntegrateTransport", &LGW::dIntegrateTransport_proxy, "q"_a, "v"_a, "J"_a, "arg"_a,
      "Transport a matrix J through the integrate operation.")
    .def(
      "dIntegrateTransport", &LGW::dIntegrateTransport_proxy_vec, "q"_a, "v"_a, "vin"_a, "arg"_a,
      "Transport a vector v through the integrate operation.")
    .def(
      "difference", &LGW::difference, "q0"_a, "q1"_a, "Compute the difference between q0 and q1.")
    .def(
      "dDifference", &LGW::dDifference1, "q0"_a, "q1"_a, "arg"_a,
      "Compute the Jacobian of the difference operation.")
    .def(
      "dDifference", &LGW::dDifference2, "q0"_a, "q1"_a, "arg"_a, "Jin"_a, "self"_a,
      "Compute the product Jin * J where J = dDifference.")
    .def(
      "dDifference", &LGW::dDifference3, "q0"_a, "q1"_a, "arg"_a, "self"_a, "Jin"_a,
      "Compute the product J * Jin where J = dDifference.")
    .def("tangentMap", &LGW::tangentMap, "q"_a, "Compute the tangent map at q.")
    .def(
      "tangentMapProduct", &LGW::tangentMapProduct, "q"_a, "Min"_a,
      "Compute the product TM(q) * Min where TM is the tangent map.")
    .def(
      "tangentMapTransposeProduct", &LGW::tangentMapTransposeProduct, "q"_a, "Min"_a,
      "Compute the product TM(q).T * Min where TM is the tangent map.")
    .def("interpolate", &LGW::interpolate, "q0"_a, "q1"_a, "u"_a, "Interpolate between q0 and q1.")
    .def(
      "random",
      static_cast<typename LieGroupType::ConfigVector_t (LieGroupType::*)() const>(
        &LieGroupType::random),
      "Sample a random configuration.")
    .def(
      "randomConfiguration",
      [](const LieGroupType & lg, const ConfigVectorType & lower, const ConfigVectorType & upper) {
        return lg.template randomConfiguration<ConfigVectorType, ConfigVectorType>(lower, upper);
      },
      "lower_pos_limit"_a, "upper_pos_limit"_a,
      "Sample a random configuration within the given bounds.")
    .def(
      "distance",
      [](const LieGroupType & lg, const ConfigVectorType & q0, const ConfigVectorType & q1) {
        return lg.template distance<ConfigVectorType, ConfigVectorType>(q0, q1);
      },
      "q0"_a, "q1"_a, "Compute the distance between q0 and q1.")
    .def(
      "squaredDistance",
      [](const LieGroupType & lg, const ConfigVectorType & q0, const ConfigVectorType & q1) {
        return lg.template squaredDistance<ConfigVectorType, ConfigVectorType>(q0, q1);
      },
      "q0"_a, "q1"_a, "Compute the squared distance between q0 and q1.")
    .def(
      "normalize", [](const LieGroupType & lg, ConfigVectorType & q) { lg.normalize(q); }, "q"_a,
      "Normalize the configuration vector q in place.")
    .def_prop_ro("name", &LieGroupType::name, "Name of the Lie group.")
    .def_prop_ro("neutral", &LieGroupType::neutral, "Neutral element of the Lie group.")
    .def_prop_ro("nq", &LieGroupType::nq, "Dimension of the configuration space.")
    .def_prop_ro("nv", &LieGroupType::nv, "Dimension of the tangent space.")
    .def(
      "__mul__", [](const LieGroupType & self, const LieGroupType & other) { return self * other; })
    .def(
      "__imul__",
      [](LieGroupType & self, const LieGroupType & other) -> LieGroupType & {
        return self *= other;
      },
      nb::rv_policy::reference_internal)
    .def("__eq__", [](const LieGroupType & self, const LieGroupType & other) {
      return self == other;
    });
}

PINOCCHIO_PYTHON_NAMESPACE_END
