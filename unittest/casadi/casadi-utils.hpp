//
// Copyright (c) 2021 Inria
//
// Utils for tests, e.g. conversion between Eigen matrix
// and DM without copy.
#include "pinocchio/autodiff/casadi.hpp"
#include "pinocchio/autodiff/casadi-algo.hpp"

/// Without copy
template<typename Derived>
casadi::DM eigenToDM(const Eigen::MatrixBase<Derived> & x)
{
  typedef Eigen::Map<Derived> Map_t;
  std::vector<double> x_vec((size_t)x.size());
  Map_t(x_vec.data(), x.rows(), x.cols()) = x;
  casadi::DM out(x_vec);
  return reshape(out, x.rows(), x.cols());
}

template<typename Derived>
casadi::DM SE3toCasadiDM(const pinocchio::SE3Base<Derived> & M)
{
  typedef pinocchio::SE3Base<Derived> SE3;
  typedef typename Derived::Scalar Scalar;
  typename SE3::HomogeneousMatrixType M_mat = M.toHomogeneousMatrix();
  std::vector<Scalar> flat_M_vec(M_mat.data(), M_mat.data() + M_mat.size());
  casadi::DM out{flat_M_vec};
  return reshape(out, 4, 4);
}
