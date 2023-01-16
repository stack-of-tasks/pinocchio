//
// Copyright (c) 2018 CNRS
// created on 1/15/23
// This version is only for ACCURACY tests for FO/SO partials (using CasADi) of
// RNEA w.r.t anaytical algorithms
/*
 * Comments
 * 1. FO partials of RNEA using analytical method                    -- done
 * 2 SO partials of RNEA using finite-difference                     -- done
 * 3. Added RNEA_SO_v9

 */

#include "pinocchio/algorithm/aba.hpp"
#include "pinocchio/algorithm/center-of-mass.hpp"
#include "pinocchio/algorithm/centroidal.hpp"
#include "pinocchio/algorithm/cholesky.hpp"
#include "pinocchio/algorithm/compute-all-terms.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/rnea-derivatives-SO.hpp"
#include "pinocchio/algorithm/rnea-derivatives.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/codegen/code-generator-algo.hpp"
#include "pinocchio/container/aligned-vector.hpp"
#include "pinocchio/parsers/sample-models.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/utils/tensor_utils.hpp"
#include "pinocchio/utils/timer.hpp"
#include <dlfcn.h>
#include <iostream>

//#include "pinocchio/algorithm/rnea-derivatives.hpp"

using namespace std;
using namespace pinocchio;

int main(int argc, const char **argv) {
  using namespace Eigen;

  Model model;
  bool with_ff;

  std::string model_name;
  std::cout << "Enter the model name " << std::endl;
  std::cin >> model_name;
  cout << "with_ff = " << endl;
  cin >> with_ff;
  std::string filename =
      "../models" + std::string("/") + model_name + std::string(".urdf");
  if (with_ff)
    pinocchio::urdf::buildModel(filename, JointModelFreeFlyer(), model);
  else
    pinocchio::urdf::buildModel(filename, model);

  if (with_ff) {
    model_name += std::string("_f");
  }
  cout << "model.nv = " << model.nv << endl;
  cout << "model.nq = " << model.nq << endl;
  cout << "model.njoints = " << model.njoints << endl;

  Data data(model);

  // Data for error check
  Data::Tensor3x temp1_diff(model.nv, model.nv, model.nv);
  double temp_q_SO, temp_v_SO, temp_qv_SO, temp_qa_SO;

  // Sample random configuration
  VectorXd qmax = Eigen::VectorXd::Ones(model.nq);
  typedef Model::ConfigVectorType ConfigVector;
  typedef Model::TangentVectorType TangentVector;
  ConfigVector q(model.nq);
  q = pinocchio::randomConfiguration(model, -qmax, qmax);
  TangentVector v(TangentVector::Random(model.nv));
  TangentVector a(TangentVector::Random(model.nv));

  //------------------------------------------------//
  // FO partials of RNEA ---------------------------//
  //------------------------------------------------//

  Eigen::MatrixXd rnea_partial_dq(model.nv, model.nv);
  rnea_partial_dq.setZero();
  Eigen::MatrixXd rnea_partial_dv(model.nv, model.nv);
  rnea_partial_dv.setZero();
  Eigen::MatrixXd rnea_partial_da(model.nv, model.nv);
  rnea_partial_da.setZero();

  pinocchio::computeRNEADerivatives(model, data, q, v, a, rnea_partial_dq,
                                    rnea_partial_dv, rnea_partial_da);
  rnea_partial_da.triangularView<Eigen::StrictlyLower>() =
      rnea_partial_da.transpose().triangularView<Eigen::StrictlyLower>();

  //------------------------------------------------//
  // SO partials of RNEA ---------------------------//
  //------------------------------------------------//
  Data::Tensor3x dtau2_dq_ana(model.nv, model.nv, model.nv);
  Data::Tensor3x dtau2_dv_ana(model.nv, model.nv, model.nv);
  Data::Tensor3x dtau2_dqv_ana(model.nv, model.nv, model.nv);
  Data::Tensor3x dtau_dadq_ana(model.nv, model.nv, model.nv);
  dtau2_dq_ana.setZero();
  dtau2_dv_ana.setZero();
  dtau2_dqv_ana.setZero();
  dtau_dadq_ana.setZero();

  computeRNEADerivativesSO(model, data, q, v, a, dtau2_dq_ana, dtau2_dv_ana,
                           dtau2_dqv_ana, dtau_dadq_ana);
  //   dtau2_dq_ana = data.d2tau_dq;
  //   dtau2_dv_ana = data.d2tau_dv;
  //   dtau2_dqv_ana = data.d2tau_dqdv;
  //   dtau_dadq_ana = data.d2tau_dadq;

  //---------------------------------------------------//
  // SO partials of ID using Finite difference --------//
  //---------------------------------------------------//

  PINOCCHIO_EIGEN_PLAIN_ROW_MAJOR_TYPE(MatrixXd)
  drnea_dq(MatrixXd::Zero(model.nv, model.nv));
  PINOCCHIO_EIGEN_PLAIN_ROW_MAJOR_TYPE(MatrixXd)
  drnea_dv(MatrixXd::Zero(model.nv, model.nv));
  MatrixXd drnea_da(MatrixXd::Zero(model.nv, model.nv));

  computeRNEADerivatives(model, data, q, v, a, drnea_dq, drnea_dv, drnea_da);

  // perturbed variables here

  PINOCCHIO_EIGEN_PLAIN_ROW_MAJOR_TYPE(MatrixXd)
  drnea_dq_plus(MatrixXd::Zero(model.nv, model.nv));
  PINOCCHIO_EIGEN_PLAIN_ROW_MAJOR_TYPE(MatrixXd)
  drnea_dv_plus(MatrixXd::Zero(model.nv, model.nv));
  MatrixXd drnea_da_plus(MatrixXd::Zero(model.nv, model.nv));

  Data::Tensor3x dtau2_dq(model.nv, model.nv, model.nv);
  Data::Tensor3x dtau2_dqd(model.nv, model.nv, model.nv);
  Data::Tensor3x dtau2_qv(model.nv, model.nv, model.nv);
  Data::Tensor3x dtau2_qa(model.nv, model.nv, model.nv);

  VectorXd v_eps(VectorXd::Zero(model.nv));
  VectorXd a_eps(VectorXd::Zero(model.nv));
  VectorXd q_plus(model.nq);
  VectorXd qd_plus(model.nv);
  VectorXd a_plus(model.nv);

  MatrixXd temp_mat1(MatrixXd::Zero(model.nv, model.nv));
  MatrixXd temp_mat2(MatrixXd::Zero(model.nv, model.nv));
  MatrixXd temp_mat3(MatrixXd::Zero(model.nv, model.nv));

  double alpha = 1e-7; // performs well

  // Partial wrt q
  for (int k = 0; k < model.nv; ++k) {
    v_eps[k] += alpha;
    q_plus = integrate(
        model, q,
        v_eps); // This is used to add the v_eps to q in the k^th direction
    computeRNEADerivatives(model, data, q_plus, v, a, drnea_dq_plus,
                           drnea_dv_plus, drnea_da_plus);
    temp_mat1 = (drnea_dq_plus - drnea_dq) / alpha;
    temp_mat2 =
        (drnea_da_plus - drnea_da) / alpha; // MSO partial of dtau_dq wrt a
    temp_mat2.triangularView<Eigen::StrictlyLower>() =
        temp_mat2.transpose().triangularView<Eigen::StrictlyLower>();
    hess_assign_fd_v1(dtau2_dq, temp_mat1, model.nv, k);
    hess_assign_fd_v1(dtau2_qa, temp_mat2, model.nv, k);
    v_eps[k] -= alpha;
  }

  // Partial wrt qd
  for (int k = 0; k < model.nv; ++k) {
    v_eps[k] += alpha;
    qd_plus =
        v + v_eps; // This is used to add the v_eps to q in the k^th direction
    computeRNEADerivatives(model, data, q, qd_plus, a, drnea_dq_plus,
                           drnea_dv_plus, drnea_da_plus);
    temp_mat1 = (drnea_dv_plus - drnea_dv) / alpha; // SO partial wrt qdot
    temp_mat2 =
        (drnea_dq_plus - drnea_dq) / alpha; // MSO partial of dtau_dq wrt qdot
    hess_assign_fd_v1(dtau2_dqd, temp_mat1, model.nv, k);
    hess_assign_fd_v1(dtau2_qv, temp_mat2, model.nv, k);
    v_eps[k] -= alpha;
  }

  // difference variables
  Data::Tensor3x temptens1(model.nv, model.nv, model.nv);
  Data::Tensor3x temptens2(model.nv, model.nv, model.nv);
  Data::Tensor3x temptens3(model.nv, model.nv, model.nv);
  Data::Tensor3x temptens4(model.nv, model.nv, model.nv);

  temptens1 = dtau2_dq_ana - dtau2_dq;
  temptens2 = dtau2_dv_ana - dtau2_dqd;
  temptens3 = dtau2_dqv_ana - dtau2_qv;
  temptens4 = dtau_dadq_ana - dtau2_qa;

  temp_q_SO = get_tens_diff_norm(dtau2_dq_ana, dtau2_dq, model.nv);
  temp_v_SO = get_tens_diff_norm(dtau2_dv_ana, dtau2_dqd, model.nv);
  temp_qv_SO = get_tens_diff_norm(dtau2_dqv_ana, dtau2_qv, model.nv);
  temp_qa_SO = get_tens_diff_norm(dtau_dadq_ana, dtau2_qa, model.nv);

  std::cout << "---------------------------------------------------------------"
               "-------------------"
            << std::endl;
  std::cout << "Difference in the SO partial w.r.t q for FD with Ana max val"
            << (temptens1.abs()).maximum() << std::endl;
  std::cout << "Difference in the SO partial w.r.t q for FD with Ana norm"
            << temp_q_SO << std::endl;

  std::cout << "---------------------------------------------------------------"
               "-------------------"
            << std::endl;
  std::cout << "Difference in the SO partial w.r.t v for FD with Ana max val"
            << (temptens2.abs()).maximum() << std::endl;
  std::cout << "Difference in the SO partial w.r.t v for FD with Ana norm"
            << temp_v_SO << std::endl;
  std::cout << "---------------------------------------------------------------"
               "-------------------"
            << std::endl;
  std::cout << "Difference in the SO partial w.r.t q,v for FD with Ana max val"
            << (temptens3.abs()).maximum() << std::endl;
  std::cout << "Difference in the SO partial w.r.t q,v for FD with Ana norm"
            << temp_qv_SO << std::endl;
  std::cout << "---------------------------------------------------------------"
               "-------------------"
            << std::endl;
  std::cout << "Difference in the SO partial w.r.t a,q for FD with Ana max val"
            << (temptens4.abs()).maximum() << std::endl;
  std::cout << "Difference in the SO partial w.r.t q,v for FD with Ana norm"
            << temp_qa_SO << std::endl;

  return 0;
}