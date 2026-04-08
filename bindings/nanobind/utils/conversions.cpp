// Copyright (c) 2019-2021 CNRS, 2019-2026 INRIA
// Port of file: bindings/python/utils/conversions.cpp

#include "pinocchio/bindings/python-nb/fwd.hpp"
#include "pinocchio/bindings/python-nb/spatial/se3.hpp"

#include <nanobind/eigen/dense.h>
#include <nanobind/stl/array.h>
#include <nanobind/stl/tuple.h>

PINOCCHIO_PYTHON_NAMESPACE_BEGIN

using namespace nb::literals;

using Vector7s = Eigen::Matrix<Scalar, 7, 1>;
using QuatMap = Eigen::Map<SE3::Quaternion>;
using QuatConstMap = Eigen::Map<const SE3::Quaternion>;

static Vector7s SE3ToXYZQUAT(const SE3 & M)
{
  Vector7s res;
  res.head<3>() = M.translation();
  QuatMap(res.tail<4>().data()) = M.rotation();
  return res;
}

static nb::tuple SE3ToXYZQUATtuple(const SE3 & M)
{
  const SE3::Quaternion q(M.rotation());
  return nb::make_tuple(
    M.translation()(0), M.translation()(1), M.translation()(2), q.x(), q.y(), q.z(), q.w());
}

static SE3 XYZQUATToSE3(Eigen::Ref<const Vector7s> v)
{
  QuatConstMap q(v.template tail<4>().data());
  return SE3(q.matrix(), v.template head<3>());
}

using Tuple7s = std::tuple<Scalar, Scalar, Scalar, Scalar, Scalar, Scalar, Scalar>;

static SE3 XYZQUATToSE3fromTuple(const Tuple7s & v)
{
  SE3::Quaternion q(std::get<6>(v), std::get<3>(v), std::get<4>(v), std::get<5>(v));
  SE3::Vector3 t(std::get<0>(v), std::get<1>(v), std::get<2>(v));
  return SE3(q.matrix(), t);
}

static SE3 XYZQUATToSE3fromArray(const std::array<Scalar, 7> & v)
{
  QuatConstMap q(v.data() + 3);
  return SE3(q.matrix(), Eigen::Map<const SE3::Vector3>(v.data()));
}

void exposeConversions(nb::module_ m)
{
  m.def("SE3ToXYZQUAT", SE3ToXYZQUAT, "M"_a, "Convert the input SE3 object to a numpy array.");
  m.def(
    "SE3ToXYZQUATtuple", SE3ToXYZQUATtuple, "M"_a,
    "Convert the input SE3 object to a 7D tuple of floats (X,Y,Z,x,y,z,w).");
  const char * doc2 =
    "Reverse function of SE3ToXYZQUAT: convert [X,Y,Z,x,y,z,w] to an SE3 element.";
  m.def("XYZQUATToSE3", XYZQUATToSE3, "array"_a, doc2);
  m.def("XYZQUATToSE3", XYZQUATToSE3fromTuple, "tuple"_a, doc2);
  m.def("XYZQUATToSE3", XYZQUATToSE3fromArray, "array"_a, doc2);
}

PINOCCHIO_PYTHON_NAMESPACE_END
