//
// Copyright (c) 2015-2019 CNRS INRIA
// Copyright (c) 2015 Wandercraft, 86 rue de Paris 91400 Orsay, France.
//

#include "pinocchio/bindings/python/fwd.hpp"
#include <boost/python.hpp>
#include "pinocchio/spatial/skew.hpp"

namespace pinocchio
{
  namespace python
  {
    namespace bp = boost::python;

    Eigen::Matrix3d skew_proxy(const Eigen::Vector3d & v)
    {
      return pinocchio::skew(v);
    }

    Eigen::Vector3d unSkew_proxy(const Eigen::Matrix3d & M)
    {
      return pinocchio::unSkew(M);
    }

    void exposeSkew()
    {

      bp::def("skew",&skew_proxy,
              bp::arg("Angular velocity (vector of size 3)"),
              "Computes the skew representation of a given 3D vector, "
              "i.e. the antisymmetric matrix representation of the cross product operator.");

      bp::def("unSkew",&unSkew_proxy,
              bp::arg("M: a 3x3 matrix."),
              "Inverse of skew operator. From a given skew-symmetric matrix M "
              "of dimension 3x3, it extracts the supporting vector, i.e. the entries of M."
              "Mathematically speacking, it computes v such that M x = cross(x, v).");
    }

  } // namespace python
} // namespace pinocchio
