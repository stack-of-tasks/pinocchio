//
// Copyright (c) 2019 INRIA
//

#include "pinocchio/spatial/se3.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/cholesky.hpp"
#include "pinocchio/algorithm/contact-info.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/contact-cholesky.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/parsers/sample-models.hpp"

#include <iostream>

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

namespace pinocchio
{
  namespace cholesky
  {
    template<typename Scalar, int Options>
    struct ContactCholeskyDecompositionAccessorTpl
    : public ContactCholeskyDecompositionTpl<Scalar,Options>
    {
      typedef ContactCholeskyDecompositionTpl<Scalar,Options> Base;
      typedef typename Base::IndexVector IndexVector;
      typedef typename Base::BooleanVector BooleanVector;
      
      ContactCholeskyDecompositionAccessorTpl(const Base & other)
      : Base(other)
      {}
      
      const IndexVector & getParents_fromRow() const
      { return this->parents_fromRow; }
      const IndexVector & getLastChild() const
      { return this->last_child; }
      const IndexVector & getNvSubtree_fromRow() const
      { return this->nv_subtree_fromRow; }
      const std::vector<BooleanVector> & getExtented_parents_fromRow() const
      { return this->extented_parents_fromRow; }
    };
    
    typedef ContactCholeskyDecompositionAccessorTpl<double,0> ContactCholeskyDecompositionAccessor;
  }
}

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

BOOST_AUTO_TEST_CASE(contact_cholesky_simple)
{
  using namespace Eigen;
  using namespace pinocchio;
  using namespace pinocchio::cholesky;
  
  pinocchio::Model model;
  pinocchio::buildModels::humanoidRandom(model,true);
  pinocchio::Data data_ref(model);
  
  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill(1.);
  VectorXd q = randomConfiguration(model);
  crba(model,data_ref,q);
  
  pinocchio::cholesky::decompose(model,data_ref);
  data_ref.M.triangularView<Eigen::StrictlyLower>() =
  data_ref.M.triangularView<Eigen::StrictlyUpper>().transpose();
  
  ContactCholeskyDecomposition contact_chol_decomposition;
  const PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(ContactInfo) contact_info_empty;
  contact_chol_decomposition.allocate(model,contact_info_empty);
  
  BOOST_CHECK(contact_chol_decomposition.D.size() == model.nv);
  BOOST_CHECK(contact_chol_decomposition.Dinv.size() == model.nv);
  BOOST_CHECK(contact_chol_decomposition.U.rows() == model.nv);
  BOOST_CHECK(contact_chol_decomposition.U.cols() == model.nv);
  BOOST_CHECK(contact_chol_decomposition.dim() == model.nv);
  BOOST_CHECK(contact_chol_decomposition.U.diagonal().isOnes());
  
  Data data(model); crba(model,data,q);
  data.M.triangularView<Eigen::StrictlyLower>() =
  data.M.triangularView<Eigen::StrictlyUpper>().transpose();
  
  contact_chol_decomposition.compute(model,data,contact_info_empty);
  
  BOOST_CHECK(data.M.isApprox(data_ref.M));
  BOOST_CHECK(data_ref.D.isApprox(contact_chol_decomposition.D.tail(model.nv)));
  BOOST_CHECK(data_ref.Dinv.isApprox(contact_chol_decomposition.Dinv.tail(model.nv)));
  BOOST_CHECK(data_ref.U.isApprox(contact_chol_decomposition.U.bottomRightCorner(model.nv,model.nv)));
  
  ContactCholeskyDecompositionAccessor access(contact_chol_decomposition);
  for(Eigen::DenseIndex k = 0; k < model.nv; ++k)
  {
    BOOST_CHECK(access.getParents_fromRow()[k] == data.parents_fromRow[(size_t)k]);
  }
    
  for(Eigen::DenseIndex k = 0; k < model.njoints; ++k)
  {
    BOOST_CHECK(access.getLastChild()[k] == data.lastChild[(size_t)k]);
  }
  
  for(Eigen::DenseIndex k = 0; k < model.nv; ++k)
  {
    BOOST_CHECK(access.getNvSubtree_fromRow()[k] == data.nvSubtree_fromRow[(size_t)k]);
  }
  
  // Test basic operation
  VectorXd v_in(VectorXd::Random(model.nv));
  MatrixXd mat_in(MatrixXd::Random(contact_chol_decomposition.dim(),20));
  
  // Test Uv
  VectorXd Uv_op_res(v_in), Uv_op_ref(v_in);
  
  contact_chol_decomposition.Uv(Uv_op_res);
  pinocchio::cholesky::Uv(model,data_ref,Uv_op_ref);
  
  BOOST_CHECK(Uv_op_res.isApprox(Uv_op_ref));
  
  MatrixXd Uv_mat_op_res(mat_in), Uv_mat_op_ref(mat_in);
  
  contact_chol_decomposition.Uv(Uv_mat_op_res);
  pinocchio::cholesky::Uv(model,data_ref,Uv_mat_op_ref);
  
  BOOST_CHECK(Uv_mat_op_res.isApprox(Uv_mat_op_ref));

  // Test Utv
  VectorXd Utv_op_res(v_in), Utv_op_ref(v_in);
  
  contact_chol_decomposition.Utv(Utv_op_res);
  pinocchio::cholesky::Utv(model,data_ref,Utv_op_ref);
  
  BOOST_CHECK(Utv_op_res.isApprox(Utv_op_ref));
  
  MatrixXd Utv_mat_op_res(mat_in), Utv_mat_op_ref(mat_in);
  
  contact_chol_decomposition.Utv(Utv_mat_op_res);
  pinocchio::cholesky::Utv(model,data_ref,Utv_mat_op_ref);
  
  BOOST_CHECK(Utv_mat_op_res.isApprox(Utv_mat_op_ref));
  
  // Test Uiv
  VectorXd Uiv_op_res(v_in), Uiv_op_ref(v_in);
  
  contact_chol_decomposition.Uiv(Uiv_op_res);
  pinocchio::cholesky::Uiv(model,data_ref,Uiv_op_ref);

  BOOST_CHECK(Uiv_op_res.isApprox(Uiv_op_ref));
  
  MatrixXd Uiv_mat_op_res(mat_in), Uiv_mat_op_ref(mat_in);
  
  contact_chol_decomposition.Uiv(Uiv_mat_op_res);
  pinocchio::cholesky::Uiv(model,data_ref,Uiv_mat_op_ref);
  
  BOOST_CHECK(Uiv_mat_op_res.isApprox(Uiv_mat_op_ref));
  
  // Test Utiv
  VectorXd Utiv_op_res(v_in), Utiv_op_ref(v_in);
  
  contact_chol_decomposition.Utiv(Utiv_op_res);
  pinocchio::cholesky::Utiv(model,data_ref,Utiv_op_ref);
  
  BOOST_CHECK(Utiv_op_res.isApprox(Utiv_op_ref));
  
  MatrixXd Utiv_mat_op_res(mat_in), Utiv_mat_op_ref(mat_in);
  
  contact_chol_decomposition.Utiv(Utiv_mat_op_res);
  pinocchio::cholesky::Utiv(model,data_ref,Utiv_mat_op_ref);
  
  BOOST_CHECK(Utiv_mat_op_res.isApprox(Utiv_mat_op_ref));
  
  // SolveInPlace
  VectorXd sol(v_in);
  contact_chol_decomposition.solveInPlace(sol);
  
  VectorXd sol_ref(data.M.inverse()*v_in);
  
  BOOST_CHECK(sol.isApprox(sol_ref));
  
  MatrixXd sol_mat(mat_in);
  contact_chol_decomposition.solveInPlace(sol_mat);
  
  MatrixXd sol_mat_ref(data.M.inverse()*mat_in);
  
  BOOST_CHECK(sol_mat.isApprox(sol_mat_ref));
}

BOOST_AUTO_TEST_CASE(contact_cholesky_contact6D)
{
  using namespace Eigen;
  using namespace pinocchio;
  using namespace pinocchio::cholesky;
  
  pinocchio::Model model;
  pinocchio::buildModels::humanoidRandom(model,true);
  pinocchio::Data data_ref(model);
  
  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill(1.);
  VectorXd q = randomConfiguration(model);
  
  const std::string RF = "rleg6_joint";
  const std::string LF = "lleg6_joint";
  
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(ContactInfo) contact_infos;
  ContactInfo ci_RF(CONTACT_6D,model.getFrameId(RF));
  contact_infos.push_back(ci_RF);
  ContactInfo ci_LF(CONTACT_6D,model.getFrameId(LF));
  contact_infos.push_back(ci_LF);

  // Compute Mass Matrix
  crba(model,data_ref,q);
  data_ref.M.triangularView<Eigen::StrictlyLower>() =
  data_ref.M.triangularView<Eigen::StrictlyUpper>().transpose();
  
  // Compute Cholesky decomposition
  pinocchio::cholesky::decompose(model,data_ref);

  // Compute Jacobians
  Data::Matrix6x J_RF(6,model.nv), J_LF(6,model.nv);
  J_RF.setZero();
  getJointJacobian(model, data_ref, model.getJointId(RF), WORLD, J_RF);
  J_LF.setZero();
  getJointJacobian(model, data_ref, model.getJointId(LF), WORLD, J_LF);

  const int constraint_dim = 12;
  const int total_dim = model.nv + constraint_dim;

  Data::MatrixXs H(total_dim,total_dim); H.setZero();
  H.bottomRightCorner(model.nv, model.nv) = data_ref.M;
  H.middleRows<6>(0).rightCols(model.nv) = J_RF;
  H.middleRows<6>(6).rightCols(model.nv) = J_LF;
  
  H.triangularView<Eigen::StrictlyLower>() =
  H.triangularView<Eigen::StrictlyUpper>().transpose();

  Data data(model); crba(model,data,q);
  ContactCholeskyDecomposition contact_chol_decomposition;
  contact_chol_decomposition.allocate(model, contact_infos);
  
  ContactCholeskyDecompositionAccessor access(contact_chol_decomposition);
  for(Eigen::DenseIndex k = 0; k < model.nv; ++k)
  {
    if(data.parents_fromRow[(size_t)k] == -1)
      BOOST_CHECK(access.getParents_fromRow()[k+constraint_dim] == -1);
    else
      BOOST_CHECK(access.getParents_fromRow()[k+constraint_dim] == data.parents_fromRow[(size_t)k]+constraint_dim);
  }
  
  contact_chol_decomposition.compute(model,data,contact_infos);
  
  data.M.triangularView<Eigen::StrictlyLower>() =
  data.M.triangularView<Eigen::StrictlyUpper>().transpose();
  
  BOOST_CHECK(data_ref.D.isApprox(contact_chol_decomposition.D.tail(model.nv)));
  BOOST_CHECK(data_ref.Dinv.isApprox(contact_chol_decomposition.Dinv.tail(model.nv)));
  BOOST_CHECK(data_ref.U.isApprox(contact_chol_decomposition.U.bottomRightCorner(model.nv,model.nv)));
  
  Data::MatrixXs M_recomposed =
    contact_chol_decomposition.U.bottomRightCorner(model.nv,model.nv)
  * contact_chol_decomposition.D.tail(model.nv).asDiagonal()
  * contact_chol_decomposition.U.bottomRightCorner(model.nv,model.nv).transpose();
  BOOST_CHECK(M_recomposed.isApprox(data.M));
  
  Data::MatrixXs H_recomposed = contact_chol_decomposition.U * contact_chol_decomposition.D.asDiagonal() * contact_chol_decomposition.U.transpose();
  
  BOOST_CHECK(H_recomposed.bottomRightCorner(model.nv,model.nv).isApprox(data.M));
  BOOST_CHECK(H_recomposed.topRightCorner(constraint_dim,model.nv).isApprox(H.topRightCorner(constraint_dim,model.nv)));
  BOOST_CHECK(H_recomposed.isApprox(H));
  
  ContactCholeskyDecomposition contact_chol_decomposition_mu;
  contact_chol_decomposition_mu.allocate(model, contact_infos);
  contact_chol_decomposition_mu.compute(model,data,contact_infos,0.);
  
  BOOST_CHECK(contact_chol_decomposition_mu.D.isApprox(contact_chol_decomposition.D));
  BOOST_CHECK(contact_chol_decomposition_mu.Dinv.isApprox(contact_chol_decomposition.Dinv));
  BOOST_CHECK(contact_chol_decomposition_mu.U.isApprox(contact_chol_decomposition.U));
  
  const double mu = 0.1;
  contact_chol_decomposition_mu.compute(model,data,contact_infos,mu);
  Data::MatrixXs H_mu(H);
  H_mu.topLeftCorner(constraint_dim, constraint_dim).diagonal().fill(-mu);
  
  Data::MatrixXs H_recomposed_mu
  = contact_chol_decomposition_mu.U
  * contact_chol_decomposition_mu.D.asDiagonal()
  * contact_chol_decomposition_mu.U.transpose();
  
  BOOST_CHECK(H_recomposed_mu.isApprox(H_mu));
  
  // Test basic operation
  VectorXd v_in(VectorXd::Random(contact_chol_decomposition.dim()));
  MatrixXd mat_in(MatrixXd::Random(contact_chol_decomposition.dim(),20));

  // Test Uv
  VectorXd Uv_op_res(v_in), Uv_op_ref(v_in);

  contact_chol_decomposition.Uv(Uv_op_res);
  Uv_op_ref.noalias() = contact_chol_decomposition.U * v_in;

  BOOST_CHECK(Uv_op_res.isApprox(Uv_op_ref));
  
  MatrixXd Uv_mat_op_res(mat_in), Uv_mat_op_ref(mat_in);
  
  contact_chol_decomposition.Uv(Uv_mat_op_res);
  Uv_mat_op_ref.noalias() = contact_chol_decomposition.U * mat_in;
  
  BOOST_CHECK(Uv_mat_op_res.isApprox(Uv_mat_op_ref));
  
  // Test Utv
  VectorXd Utv_op_res(v_in), Utv_op_ref(v_in);
  
  contact_chol_decomposition.Utv(Utv_op_res);
  Utv_op_ref.noalias() = contact_chol_decomposition.U.transpose() * v_in;
  
  BOOST_CHECK(Utv_op_res.isApprox(Utv_op_ref));
  
  MatrixXd Utv_mat_op_res(mat_in), Utv_mat_op_ref(mat_in);
  
  contact_chol_decomposition.Utv(Utv_mat_op_res);
  Utv_mat_op_ref.noalias() = contact_chol_decomposition.U.transpose() * mat_in;
  
  BOOST_CHECK(Utv_mat_op_res.isApprox(Utv_mat_op_ref));
  
  // Test Uiv
  VectorXd Uiv_op_res(v_in), Uiv_op_ref(v_in);
  
  contact_chol_decomposition.Uiv(Uiv_op_res);
  Uiv_op_ref.noalias() = contact_chol_decomposition.U.inverse() * v_in;
  
  BOOST_CHECK(Uiv_op_res.isApprox(Uiv_op_ref));
  
  MatrixXd Uiv_mat_op_res(mat_in), Uiv_mat_op_ref(mat_in);
  
  contact_chol_decomposition.Uiv(Uiv_mat_op_res);
  Uiv_mat_op_ref.noalias() = contact_chol_decomposition.U.inverse() * mat_in;
  
  BOOST_CHECK(Uiv_mat_op_res.isApprox(Uiv_mat_op_ref));
  
  // Test Utiv
  VectorXd Utiv_op_res(v_in), Utiv_op_ref(v_in);
  
  contact_chol_decomposition.Utiv(Utiv_op_res);
  Utiv_op_ref.noalias() = contact_chol_decomposition.U.inverse().transpose() * v_in;
  
  BOOST_CHECK(Utiv_op_res.isApprox(Utiv_op_ref));
  
  MatrixXd Utiv_mat_op_res(mat_in), Utiv_mat_op_ref(mat_in);
  
  contact_chol_decomposition.Utiv(Utiv_mat_op_res);
  Utiv_mat_op_ref.noalias() = contact_chol_decomposition.U.inverse().transpose() * mat_in;
  
  BOOST_CHECK(Utiv_mat_op_res.isApprox(Utiv_mat_op_ref));
  
  // SolveInPlace
  VectorXd sol(v_in);
  contact_chol_decomposition.solveInPlace(sol);
  
  VectorXd sol_ref(H.inverse()*v_in);
  
  BOOST_CHECK(sol.isApprox(sol_ref));
  
  MatrixXd sol_mat(mat_in), sol_mat_ref(mat_in);
  
  contact_chol_decomposition.solveInPlace(sol_mat);
  sol_mat_ref.noalias() = H.inverse() * mat_in;
  
  BOOST_CHECK(sol_mat.isApprox(sol_mat_ref));
}

BOOST_AUTO_TEST_CASE(contact_cholesky_contact3D_6D)
{
  using namespace Eigen;
  using namespace pinocchio;
  using namespace pinocchio::cholesky;
  
  pinocchio::Model model;
  pinocchio::buildModels::humanoidRandom(model,true);
  pinocchio::Data data_ref(model);
  
  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill(1.);
  VectorXd q = randomConfiguration(model);
  
  const std::string RF = "rleg6_joint";
  const std::string LF = "lleg6_joint";
  
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(ContactInfo) contact_infos;
  ContactInfo ci_RF(CONTACT_6D,model.getFrameId(RF));
  contact_infos.push_back(ci_RF);
  ContactInfo ci_LF(CONTACT_3D,model.getFrameId(LF));
  contact_infos.push_back(ci_LF);
  
  // Compute Mass Matrix
  crba(model,data_ref,q);
  data_ref.M.triangularView<Eigen::StrictlyLower>() =
  data_ref.M.triangularView<Eigen::StrictlyUpper>().transpose();
  
  // Compute Cholesky decomposition
  pinocchio::cholesky::decompose(model,data_ref);
  
  // Compute Jacobians
  Data::Matrix6x J_RF(6,model.nv), J_LF(6,model.nv);
  J_RF.setZero();
  getJointJacobian(model, data_ref, model.getJointId(RF), WORLD, J_RF);
  J_LF.setZero();
  getJointJacobian(model, data_ref, model.getJointId(LF), WORLD, J_LF);
  
  const int constraint_dim = 9;
  const int total_dim = model.nv + constraint_dim;
  
  Data::MatrixXs H(total_dim,total_dim); H.setZero();
  H.bottomRightCorner(model.nv, model.nv) = data_ref.M;
  H.middleRows<6>(0).rightCols(model.nv) = J_RF;
  H.middleRows<3>(6).rightCols(model.nv) = J_LF.middleRows<3>(Motion::LINEAR);
  
  H.triangularView<Eigen::StrictlyLower>() =
  H.triangularView<Eigen::StrictlyUpper>().transpose();
  
  Data data(model); crba(model,data,q);
  ContactCholeskyDecomposition contact_chol_decomposition;
  contact_chol_decomposition.allocate(model, contact_infos);
  
  ContactCholeskyDecompositionAccessor access(contact_chol_decomposition);
  for(Eigen::DenseIndex k = 0; k < model.nv; ++k)
  {
    if(data.parents_fromRow[(size_t)k] == -1)
      BOOST_CHECK(access.getParents_fromRow()[k+constraint_dim] == -1);
    else
      BOOST_CHECK(access.getParents_fromRow()[k+constraint_dim] == data.parents_fromRow[(size_t)k]+constraint_dim);
  }
  
  contact_chol_decomposition.compute(model,data,contact_infos);
  
  data.M.triangularView<Eigen::StrictlyLower>() =
  data.M.triangularView<Eigen::StrictlyUpper>().transpose();
  
  BOOST_CHECK(data_ref.D.isApprox(contact_chol_decomposition.D.tail(model.nv)));
  BOOST_CHECK(data_ref.Dinv.isApprox(contact_chol_decomposition.Dinv.tail(model.nv)));
  BOOST_CHECK(data_ref.U.isApprox(contact_chol_decomposition.U.bottomRightCorner(model.nv,model.nv)));
  
  Data::MatrixXs M_recomposed =
  contact_chol_decomposition.U.bottomRightCorner(model.nv,model.nv)
  * contact_chol_decomposition.D.tail(model.nv).asDiagonal()
  * contact_chol_decomposition.U.bottomRightCorner(model.nv,model.nv).transpose();
  BOOST_CHECK(M_recomposed.isApprox(data.M));
  
  Data::MatrixXs H_recomposed = contact_chol_decomposition.U * contact_chol_decomposition.D.asDiagonal() * contact_chol_decomposition.U.transpose();
  
  BOOST_CHECK(H_recomposed.bottomRightCorner(model.nv,model.nv).isApprox(data.M));
  BOOST_CHECK(H_recomposed.topRightCorner(constraint_dim,model.nv).isApprox(H.topRightCorner(constraint_dim,model.nv)));
  BOOST_CHECK(H_recomposed.isApprox(H));
  
  // Test basic operation
  VectorXd v_in(VectorXd::Random(contact_chol_decomposition.dim()));
  MatrixXd mat_in(MatrixXd::Random(contact_chol_decomposition.dim(),20));
  
  // Test Uv
  VectorXd Uv_op_res(v_in), Uv_op_ref(v_in);
  
  contact_chol_decomposition.Uv(Uv_op_res);
  Uv_op_ref.noalias() = contact_chol_decomposition.U * v_in;
  
  BOOST_CHECK(Uv_op_res.isApprox(Uv_op_ref));
  
  MatrixXd Uv_mat_op_res(mat_in), Uv_mat_op_ref(mat_in);
  
  contact_chol_decomposition.Uv(Uv_mat_op_res);
  Uv_mat_op_ref.noalias() = contact_chol_decomposition.U * mat_in;
  
  BOOST_CHECK(Uv_mat_op_res.isApprox(Uv_mat_op_ref));
  
  // Test Utv
  VectorXd Utv_op_res(v_in), Utv_op_ref(v_in);
  
  contact_chol_decomposition.Utv(Utv_op_res);
  Utv_op_ref.noalias() = contact_chol_decomposition.U.transpose() * v_in;
  
  BOOST_CHECK(Utv_op_res.isApprox(Utv_op_ref));
  
  MatrixXd Utv_mat_op_res(mat_in), Utv_mat_op_ref(mat_in);
  
  contact_chol_decomposition.Utv(Utv_mat_op_res);
  Utv_mat_op_ref.noalias() = contact_chol_decomposition.U.transpose() * mat_in;
  
  BOOST_CHECK(Utv_mat_op_res.isApprox(Utv_mat_op_ref));
  
  // Test Uiv
  VectorXd Uiv_op_res(v_in), Uiv_op_ref(v_in);
  
  contact_chol_decomposition.Uiv(Uiv_op_res);
  Uiv_op_ref.noalias() = contact_chol_decomposition.U.inverse() * v_in;
  
  BOOST_CHECK(Uiv_op_res.isApprox(Uiv_op_ref));
  
  MatrixXd Uiv_mat_op_res(mat_in), Uiv_mat_op_ref(mat_in);
  
  contact_chol_decomposition.Uiv(Uiv_mat_op_res);
  Uiv_mat_op_ref.noalias() = contact_chol_decomposition.U.inverse() * mat_in;
  
  BOOST_CHECK(Uiv_mat_op_res.isApprox(Uiv_mat_op_ref));
  
  // Test Utiv
  VectorXd Utiv_op_res(v_in), Utiv_op_ref(v_in);
  
  contact_chol_decomposition.Utiv(Utiv_op_res);
  Utiv_op_ref.noalias() = contact_chol_decomposition.U.inverse().transpose() * v_in;
  
  BOOST_CHECK(Utiv_op_res.isApprox(Utiv_op_ref));
  
  MatrixXd Utiv_mat_op_res(mat_in), Utiv_mat_op_ref(mat_in);
  
  contact_chol_decomposition.Utiv(Utiv_mat_op_res);
  Utiv_mat_op_ref.noalias() = contact_chol_decomposition.U.inverse().transpose() * mat_in;
  
  BOOST_CHECK(Utiv_mat_op_res.isApprox(Utiv_mat_op_ref));
  
  // SolveInPlace
  VectorXd sol(v_in);
  contact_chol_decomposition.solveInPlace(sol);
  
  VectorXd sol_ref(H.inverse()*v_in);
  
  BOOST_CHECK(sol.isApprox(sol_ref));
  
  MatrixXd sol_mat(mat_in), sol_mat_ref(mat_in);
  
  contact_chol_decomposition.solveInPlace(sol_mat);
  sol_mat_ref.noalias() = H.inverse() * mat_in;
  
  BOOST_CHECK(sol_mat.isApprox(sol_mat_ref));
}

BOOST_AUTO_TEST_SUITE_END()
