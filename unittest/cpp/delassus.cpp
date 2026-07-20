//
// Copyright (c) 2023-2024 INRIA CNRS
// Copyright (c) 2023 KU Leuven
//

#define BOOST_TEST_MODULE delassus

#include "pinocchio/multibody/sample-models.hpp"
#include "pinocchio/constraints.hpp"

#include "pinocchio/algorithm/constraint-cholesky.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/delassus.hpp"
#include "pinocchio/algorithm/compute-all-terms.hpp"

#include <boost/utility/binary.hpp>

#include <boost/test/unit_test.hpp>

namespace pinocchio
{
  template<typename Scalar, int Options>
  struct ConstraintCholeskyDecompositionAccessorTpl
  : public ConstraintCholeskyDecompositionTpl<Scalar, Options>
  {
    typedef ConstraintCholeskyDecompositionTpl<Scalar, Options> Base;
    using typename Base::BooleanVector;
    using typename Base::EigenIndexVector;

    ConstraintCholeskyDecompositionAccessorTpl(const Base & other)
    : Base(other)
    {
    }

    const EigenIndexVector & getParents_fromRow() const
    {
      return this->parents_fromRow;
    }
    const EigenIndexVector & getNvSubtree_fromRow() const
    {
      return this->nv_subtree_fromRow;
    }
    const std::vector<BooleanVector> & getJoint1_indexes() const
    {
      return this->joint1_indexes;
    }
    const std::vector<BooleanVector> & getJoint2_indexes() const
    {
      return this->joint2_indexes;
    }
  };

  typedef ConstraintCholeskyDecompositionAccessorTpl<double, 0>
    ConstraintCholeskyDecompositionAccessor;
} // namespace pinocchio

using namespace pinocchio;

double mu = 1e-4;

BOOST_AUTO_TEST_CASE(contact_6D)
{
  using namespace Eigen;
  pinocchio::Model model;
  pinocchio::buildModels::humanoidRandom(model, true);
  pinocchio::Data data(model);

  const std::string RA = "rleg6_joint";
  RigidConstraintModel ci_RA_6D(CONTACT_6D, model, model.getJointId(RA), LOCAL);
  std::vector<RigidConstraintModel> contact_models;
  std::vector<RigidConstraintData> contact_data;
  contact_models.push_back(ci_RA_6D);
  contact_data.push_back(RigidConstraintData(ci_RA_6D));

  VectorXd q = randomConfiguration(model);
  VectorXd v = Eigen::VectorXd::Random(model.nv);
  data.q_in = q;
  data.v_in = v;
  pinocchio::computeJointJacobians(model, data, q);
  pinocchio::calc(model, data, contact_models, contact_data);

  pinocchio::Data::ConstraintCholeskyDecomposition constraint_chol(
    model, data, contact_models, contact_data);
  MatrixXd H_inverse(constraint_chol.size(), constraint_chol.size());

  initPvDelassus(model, data, contact_models); // Allocate memory

  for (int i = 0; i < 10; i++)
  {
    computeAllTerms(model, data, q, v);
    data.q_in = q;
    data.v_in = v;
    pinocchio::calc(model, data, contact_models, contact_data);
    constraint_chol.compute(model, data, contact_models, contact_data, mu);
    constraint_chol.inverse(H_inverse);

    Eigen::MatrixXd dampedDelassusInverse;
    dampedDelassusInverse.resize(constraint_chol.constraintDim(), constraint_chol.constraintDim());

    Eigen::MatrixXd dampedDelassusInverse2;
    dampedDelassusInverse2.resize(constraint_chol.constraintDim(), constraint_chol.constraintDim());

    dampedDelassusInverse2 =
      -H_inverse.topLeftCorner(constraint_chol.constraintDim(), constraint_chol.constraintDim());
    computeDampedDelassusMatrixInverse(
      model, data, q, contact_models, contact_data, dampedDelassusInverse, mu);
    dampedDelassusInverse.triangularView<StrictlyLower>() =
      dampedDelassusInverse.triangularView<StrictlyUpper>().transpose();
    BOOST_CHECK(dampedDelassusInverse2.isApprox(dampedDelassusInverse, 1e-10));

    computeDampedDelassusMatrixInverse(
      model, data, q, contact_models, contact_data, dampedDelassusInverse, mu, false, false);
    dampedDelassusInverse.triangularView<StrictlyLower>() =
      dampedDelassusInverse.triangularView<StrictlyUpper>().transpose();
    BOOST_CHECK(dampedDelassusInverse.isApprox(
      -H_inverse.topLeftCorner(constraint_chol.constraintDim(), constraint_chol.constraintDim()),
      1e-9));

    q = randomConfiguration(model);
    v = Eigen::VectorXd::Random(model.nv);
  }
}

BOOST_AUTO_TEST_CASE(contact_6D6D)
{
  using namespace Eigen;
  pinocchio::Model model;
  pinocchio::buildModels::humanoidRandom(model, true);
  pinocchio::Data data(model);

  const std::string RA = "rleg6_joint";
  const std::string LA = "lleg6_joint";
  RigidConstraintModel ci_LA_6D(CONTACT_6D, model, model.getJointId(LA), LOCAL);
  RigidConstraintModel ci_RA_6D(CONTACT_6D, model, model.getJointId(RA), LOCAL);
  std::vector<RigidConstraintModel> contact_models;
  std::vector<RigidConstraintData> contact_data;
  contact_models.push_back(ci_RA_6D);
  contact_data.push_back(RigidConstraintData(ci_RA_6D));
  contact_models.push_back(ci_LA_6D);
  contact_data.push_back(RigidConstraintData(ci_LA_6D));

  VectorXd q = randomConfiguration(model);
  VectorXd v = Eigen::VectorXd::Random(model.nv);
  data.q_in = q;
  data.v_in = v;
  pinocchio::computeJointJacobians(model, data, q);
  pinocchio::calc(model, data, contact_models, contact_data);

  pinocchio::Data::ConstraintCholeskyDecomposition constraint_chol(
    model, data, contact_models, contact_data);
  MatrixXd H_inverse(constraint_chol.size(), constraint_chol.size());

  computeAllTerms(model, data, q, v);
  constraint_chol.compute(model, data, contact_models, contact_data, mu);
  constraint_chol.inverse(H_inverse);

  Data::MatrixXs dampedDelassusInverse;
  dampedDelassusInverse.resize(constraint_chol.constraintDim(), constraint_chol.constraintDim());

  initPvDelassus(model, data, contact_models); // Allocate memory
  computeDampedDelassusMatrixInverse(
    model, data, q, contact_models, contact_data, dampedDelassusInverse, mu);
  dampedDelassusInverse.triangularView<StrictlyLower>() =
    dampedDelassusInverse.triangularView<StrictlyUpper>().transpose();
  BOOST_CHECK(dampedDelassusInverse.isApprox(
    -H_inverse.topLeftCorner(constraint_chol.constraintDim(), constraint_chol.constraintDim()),
    1e-10));

  computeDampedDelassusMatrixInverse(
    model, data, q, contact_models, contact_data, dampedDelassusInverse, mu, false, false);
  dampedDelassusInverse.triangularView<StrictlyLower>() =
    dampedDelassusInverse.triangularView<StrictlyUpper>().transpose();
  BOOST_CHECK(dampedDelassusInverse.isApprox(
    -H_inverse.topLeftCorner(constraint_chol.constraintDim(), constraint_chol.constraintDim()),
    1e-10));
}

BOOST_AUTO_TEST_CASE(contact_6D4)
{
  using namespace Eigen;
  pinocchio::Model model;
  pinocchio::buildModels::humanoidRandom(model, true);
  pinocchio::Data data(model);

  const std::string RF = "rleg6_joint";
  const std::string LF = "lleg6_joint";
  const std::string RA = "rarm6_joint";
  const std::string LA = "larm6_joint";
  RigidConstraintModel ci_LA_6D(CONTACT_6D, model, model.getJointId(LA), LOCAL);
  RigidConstraintModel ci_RA_6D(CONTACT_6D, model, model.getJointId(RA), LOCAL);
  RigidConstraintModel ci_LF_6D(CONTACT_6D, model, model.getJointId(LF), LOCAL);
  RigidConstraintModel ci_RF_6D(CONTACT_6D, model, model.getJointId(RF), LOCAL);
  std::vector<RigidConstraintModel> contact_models;
  std::vector<RigidConstraintData> contact_data;
  contact_models.push_back(ci_RA_6D);
  contact_data.push_back(RigidConstraintData(ci_RA_6D));
  contact_models.push_back(ci_LA_6D);
  contact_data.push_back(RigidConstraintData(ci_LA_6D));
  contact_models.push_back(ci_RF_6D);
  contact_data.push_back(RigidConstraintData(ci_RF_6D));
  contact_models.push_back(ci_LF_6D);
  contact_data.push_back(RigidConstraintData(ci_LF_6D));

  VectorXd q = randomConfiguration(model);
  VectorXd v = Eigen::VectorXd::Random(model.nv);
  pinocchio::computeJointJacobians(model, data, q);
  data.q_in = q;
  pinocchio::calc(model, data, contact_models, contact_data);

  pinocchio::Data::ConstraintCholeskyDecomposition constraint_chol(
    model, data, contact_models, contact_data);
  MatrixXd H_inverse(constraint_chol.size(), constraint_chol.size());

  computeAllTerms(model, data, q, v);
  constraint_chol.compute(model, data, contact_models, contact_data, mu);
  constraint_chol.inverse(H_inverse);

  Data::MatrixXs dampedDelassusInverse;
  dampedDelassusInverse.resize(constraint_chol.constraintDim(), constraint_chol.constraintDim());

  initPvDelassus(model, data, contact_models); // Allocate memory
  computeDampedDelassusMatrixInverse(
    model, data, q, contact_models, contact_data, dampedDelassusInverse, mu);
  dampedDelassusInverse.triangularView<StrictlyLower>() =
    dampedDelassusInverse.triangularView<StrictlyUpper>().transpose();
  BOOST_CHECK(dampedDelassusInverse.isApprox(
    -H_inverse.topLeftCorner(constraint_chol.constraintDim(), constraint_chol.constraintDim()),
    1e-10));

  computeDampedDelassusMatrixInverse(
    model, data, q, contact_models, contact_data, dampedDelassusInverse, mu, false, false);
  dampedDelassusInverse.triangularView<StrictlyLower>() =
    dampedDelassusInverse.triangularView<StrictlyUpper>().transpose();
  BOOST_CHECK(dampedDelassusInverse.isApprox(
    -H_inverse.topLeftCorner(constraint_chol.constraintDim(), constraint_chol.constraintDim()),
    1e-10));
}

BOOST_AUTO_TEST_CASE(contact_6D_repeated)
{
  using namespace Eigen;
  pinocchio::Model model;
  pinocchio::buildModels::humanoidRandom(model, true);
  pinocchio::Data data(model);

  const std::string RA = "rleg6_joint";
  RigidConstraintModel ci_RA_6D(CONTACT_6D, model, model.getJointId(RA), LOCAL);
  std::vector<RigidConstraintModel> contact_models;
  std::vector<RigidConstraintData> contact_data;
  contact_models.push_back(ci_RA_6D);
  contact_data.push_back(RigidConstraintData(ci_RA_6D));
  contact_models.push_back(ci_RA_6D);
  contact_data.push_back(RigidConstraintData(ci_RA_6D));

  VectorXd q = randomConfiguration(model);
  VectorXd v = Eigen::VectorXd::Random(model.nv);
  pinocchio::computeJointJacobians(model, data, q);
  data.q_in = q;
  pinocchio::calc(model, data, contact_models, contact_data);

  pinocchio::Data::ConstraintCholeskyDecomposition constraint_chol(
    model, data, contact_models, contact_data);
  MatrixXd H_inverse(constraint_chol.size(), constraint_chol.size());

  computeAllTerms(model, data, q, v);
  constraint_chol.compute(model, data, contact_models, contact_data, mu);
  constraint_chol.inverse(H_inverse);

  Data::MatrixXs dampedDelassusInverse;
  dampedDelassusInverse.resize(constraint_chol.constraintDim(), constraint_chol.constraintDim());

  initPvDelassus(model, data, contact_models); // Allocate memory
  computeDampedDelassusMatrixInverse(
    model, data, q, contact_models, contact_data, dampedDelassusInverse, mu);
  dampedDelassusInverse.triangularView<StrictlyLower>() =
    dampedDelassusInverse.triangularView<StrictlyUpper>().transpose();
  BOOST_CHECK(dampedDelassusInverse.isApprox(
    -H_inverse.topLeftCorner(constraint_chol.constraintDim(), constraint_chol.constraintDim()),
    1e-10));

  computeDampedDelassusMatrixInverse(
    model, data, q, contact_models, contact_data, dampedDelassusInverse, mu, false, false);
  dampedDelassusInverse.triangularView<StrictlyLower>() =
    dampedDelassusInverse.triangularView<StrictlyUpper>().transpose();
  BOOST_CHECK(dampedDelassusInverse.isApprox(
    -H_inverse.topLeftCorner(constraint_chol.constraintDim(), constraint_chol.constraintDim()),
    1e-10));
}

BOOST_AUTO_TEST_CASE(contact_6D_repeated_6D3)
{
  using namespace Eigen;
  pinocchio::Model model;
  pinocchio::buildModels::humanoidRandom(model, true);
  pinocchio::Data data(model);

  const std::string RF = "rleg6_joint";
  const std::string LF = "lleg6_joint";
  const std::string RA = "rarm6_joint";
  const std::string LA = "larm6_joint";
  RigidConstraintModel ci_LA_6D(CONTACT_6D, model, model.getJointId(LA), LOCAL);
  RigidConstraintModel ci_RA_6D(CONTACT_6D, model, model.getJointId(RA), LOCAL);
  RigidConstraintModel ci_LF_6D(CONTACT_6D, model, model.getJointId(LF), LOCAL);
  RigidConstraintModel ci_RF_6D(CONTACT_6D, model, model.getJointId(RF), LOCAL);
  std::vector<RigidConstraintModel> contact_models;
  std::vector<RigidConstraintData> contact_data;
  contact_models.push_back(ci_RA_6D);
  contact_data.push_back(RigidConstraintData(ci_RA_6D));
  contact_models.push_back(ci_RA_6D);
  contact_data.push_back(RigidConstraintData(ci_RA_6D));

  contact_models.push_back(ci_LA_6D);
  contact_data.push_back(RigidConstraintData(ci_LA_6D));

  VectorXd q = randomConfiguration(model);
  VectorXd v = Eigen::VectorXd::Random(model.nv);
  pinocchio::computeJointJacobians(model, data, q);
  data.q_in = q;
  pinocchio::calc(model, data, contact_models, contact_data);

  pinocchio::Data::ConstraintCholeskyDecomposition constraint_chol(
    model, data, contact_models, contact_data);
  MatrixXd H_inverse(constraint_chol.size(), constraint_chol.size());

  computeAllTerms(model, data, q, v);
  constraint_chol.compute(model, data, contact_models, contact_data, mu);
  constraint_chol.inverse(H_inverse);

  Data::MatrixXs dampedDelassusInverse;
  dampedDelassusInverse.resize(constraint_chol.constraintDim(), constraint_chol.constraintDim());

  initPvDelassus(model, data, contact_models); // Allocate memory
  computeDampedDelassusMatrixInverse(
    model, data, q, contact_models, contact_data, dampedDelassusInverse, mu);
  dampedDelassusInverse.triangularView<StrictlyLower>() =
    dampedDelassusInverse.triangularView<StrictlyUpper>().transpose();
  BOOST_CHECK(dampedDelassusInverse.isApprox(
    -H_inverse.topLeftCorner(constraint_chol.constraintDim(), constraint_chol.constraintDim()),
    1e-10));

  computeDampedDelassusMatrixInverse(
    model, data, q, contact_models, contact_data, dampedDelassusInverse, mu, false, false);
  dampedDelassusInverse.triangularView<StrictlyLower>() =
    dampedDelassusInverse.triangularView<StrictlyUpper>().transpose();
  BOOST_CHECK(dampedDelassusInverse.isApprox(
    -H_inverse.topLeftCorner(constraint_chol.constraintDim(), constraint_chol.constraintDim()),
    1e-10));
}

BOOST_AUTO_TEST_CASE(contact_3D)
{
  using namespace Eigen;
  pinocchio::Model model;
  pinocchio::buildModels::humanoidRandom(model, true);
  pinocchio::Data data(model);

  const std::string RA = "rleg6_joint";
  RigidConstraintModel ci_RA_3D(CONTACT_3D, model, model.getJointId(RA), LOCAL);
  std::vector<RigidConstraintModel> contact_models;
  std::vector<RigidConstraintData> contact_data;
  contact_models.push_back(ci_RA_3D);
  contact_data.push_back(RigidConstraintData(ci_RA_3D));

  VectorXd q = randomConfiguration(model);
  VectorXd v = Eigen::VectorXd::Random(model.nv);
  pinocchio::computeJointJacobians(model, data, q);
  data.q_in = q;
  pinocchio::calc(model, data, contact_models, contact_data);

  pinocchio::Data::ConstraintCholeskyDecomposition constraint_chol(
    model, data, contact_models, contact_data);
  MatrixXd H_inverse(constraint_chol.size(), constraint_chol.size());

  computeAllTerms(model, data, q, v);
  constraint_chol.compute(model, data, contact_models, contact_data, mu);
  constraint_chol.inverse(H_inverse);

  Data::MatrixXs dampedDelassusInverse;
  dampedDelassusInverse.resize(constraint_chol.constraintDim(), constraint_chol.constraintDim());

  initPvDelassus(model, data, contact_models); // Allocate memory
  computeDampedDelassusMatrixInverse(
    model, data, q, contact_models, contact_data, dampedDelassusInverse, mu);
  dampedDelassusInverse.triangularView<StrictlyLower>() =
    dampedDelassusInverse.triangularView<StrictlyUpper>().transpose();
  BOOST_CHECK(dampedDelassusInverse.isApprox(
    -H_inverse.topLeftCorner(constraint_chol.constraintDim(), constraint_chol.constraintDim()),
    1e-10));

  computeDampedDelassusMatrixInverse(
    model, data, q, contact_models, contact_data, dampedDelassusInverse, mu, false, false);
  dampedDelassusInverse.triangularView<StrictlyLower>() =
    dampedDelassusInverse.triangularView<StrictlyUpper>().transpose();
  BOOST_CHECK(dampedDelassusInverse.isApprox(
    -H_inverse.topLeftCorner(constraint_chol.constraintDim(), constraint_chol.constraintDim()),
    1e-7));
}

BOOST_AUTO_TEST_CASE(contact_3D3D)
{
  using namespace Eigen;
  pinocchio::Model model;
  pinocchio::buildModels::humanoidRandom(model, true);
  pinocchio::Data data(model);

  const std::string RA = "rleg6_joint";
  const std::string LA = "lleg6_joint";
  RigidConstraintModel ci_LA_3D(CONTACT_3D, model, model.getJointId(LA), LOCAL);
  RigidConstraintModel ci_RA_3D(CONTACT_3D, model, model.getJointId(RA), LOCAL);
  std::vector<RigidConstraintModel> contact_models;
  std::vector<RigidConstraintData> contact_data;
  contact_models.push_back(ci_RA_3D);
  contact_data.push_back(RigidConstraintData(ci_RA_3D));
  contact_models.push_back(ci_LA_3D);
  contact_data.push_back(RigidConstraintData(ci_LA_3D));

  VectorXd q = randomConfiguration(model);
  VectorXd v = Eigen::VectorXd::Random(model.nv);
  pinocchio::computeJointJacobians(model, data, q);
  data.q_in = q;
  pinocchio::calc(model, data, contact_models, contact_data);

  pinocchio::Data::ConstraintCholeskyDecomposition constraint_chol(
    model, data, contact_models, contact_data);
  MatrixXd H_inverse(constraint_chol.size(), constraint_chol.size());

  computeAllTerms(model, data, q, v);
  constraint_chol.compute(model, data, contact_models, contact_data, mu);
  constraint_chol.inverse(H_inverse);

  Data::MatrixXs dampedDelassusInverse;
  dampedDelassusInverse.resize(constraint_chol.constraintDim(), constraint_chol.constraintDim());

  initPvDelassus(model, data, contact_models); // Allocate memory
  computeDampedDelassusMatrixInverse(
    model, data, q, contact_models, contact_data, dampedDelassusInverse, mu);
  dampedDelassusInverse.triangularView<StrictlyLower>() =
    dampedDelassusInverse.triangularView<StrictlyUpper>().transpose();
  BOOST_CHECK(dampedDelassusInverse.isApprox(
    -H_inverse.topLeftCorner(constraint_chol.constraintDim(), constraint_chol.constraintDim()),
    1e-10));

  computeDampedDelassusMatrixInverse(
    model, data, q, contact_models, contact_data, dampedDelassusInverse, mu, false, false);
  dampedDelassusInverse.triangularView<StrictlyLower>() =
    dampedDelassusInverse.triangularView<StrictlyUpper>().transpose();
  BOOST_CHECK(dampedDelassusInverse.isApprox(
    -H_inverse.topLeftCorner(constraint_chol.constraintDim(), constraint_chol.constraintDim()),
    1e-7));
}

BOOST_AUTO_TEST_CASE(contact_3D4)
{
  using namespace Eigen;
  pinocchio::Model model;
  pinocchio::buildModels::humanoidRandom(model, true);
  pinocchio::Data data(model);

  const std::string RF = "rleg6_joint";
  const std::string LF = "lleg6_joint";
  const std::string RA = "rarm6_joint";
  const std::string LA = "larm6_joint";
  RigidConstraintModel ci_LA_3D(CONTACT_3D, model, model.getJointId(LA), LOCAL);
  RigidConstraintModel ci_RA_3D(CONTACT_3D, model, model.getJointId(RA), LOCAL);
  RigidConstraintModel ci_LF_3D(CONTACT_3D, model, model.getJointId(LF), LOCAL);
  RigidConstraintModel ci_RF_3D(CONTACT_3D, model, model.getJointId(RF), LOCAL);
  std::vector<RigidConstraintModel> contact_models;
  std::vector<RigidConstraintData> contact_data;
  contact_models.push_back(ci_RA_3D);
  contact_data.push_back(RigidConstraintData(ci_RA_3D));
  contact_models.push_back(ci_LA_3D);
  contact_data.push_back(RigidConstraintData(ci_LA_3D));
  contact_models.push_back(ci_RF_3D);
  contact_data.push_back(RigidConstraintData(ci_RF_3D));
  contact_models.push_back(ci_LF_3D);
  contact_data.push_back(RigidConstraintData(ci_LF_3D));

  VectorXd q = randomConfiguration(model);
  VectorXd v = Eigen::VectorXd::Random(model.nv);
  pinocchio::computeJointJacobians(model, data, q);
  data.q_in = q;
  pinocchio::calc(model, data, contact_models, contact_data);

  pinocchio::Data::ConstraintCholeskyDecomposition constraint_chol(
    model, data, contact_models, contact_data);
  MatrixXd H_inverse(constraint_chol.size(), constraint_chol.size());

  computeAllTerms(model, data, q, v);
  constraint_chol.compute(model, data, contact_models, contact_data, mu);
  constraint_chol.inverse(H_inverse);

  Data::MatrixXs dampedDelassusInverse;
  dampedDelassusInverse.resize(constraint_chol.constraintDim(), constraint_chol.constraintDim());

  initPvDelassus(model, data, contact_models); // Allocate memory
  computeDampedDelassusMatrixInverse(
    model, data, q, contact_models, contact_data, dampedDelassusInverse, mu);
  dampedDelassusInverse.triangularView<StrictlyLower>() =
    dampedDelassusInverse.triangularView<StrictlyUpper>().transpose();
  BOOST_CHECK(dampedDelassusInverse.isApprox(
    -H_inverse.topLeftCorner(constraint_chol.constraintDim(), constraint_chol.constraintDim()),
    1e-10));

  computeDampedDelassusMatrixInverse(
    model, data, q, contact_models, contact_data, dampedDelassusInverse, mu, false, false);
  dampedDelassusInverse.triangularView<StrictlyLower>() =
    dampedDelassusInverse.triangularView<StrictlyUpper>().transpose();
  BOOST_CHECK(dampedDelassusInverse.isApprox(
    -H_inverse.topLeftCorner(constraint_chol.constraintDim(), constraint_chol.constraintDim()),
    1e-7));
}

BOOST_AUTO_TEST_CASE(contact_3D_repeated)
{
  using namespace Eigen;
  pinocchio::Model model;
  pinocchio::buildModels::humanoidRandom(model, true);
  pinocchio::Data data(model);

  double mu = 1e-3;
  const std::string RA = "rleg6_joint";
  RigidConstraintModel ci_RA_3D(CONTACT_3D, model, model.getJointId(RA), LOCAL);
  std::vector<RigidConstraintModel> contact_models;
  std::vector<RigidConstraintData> contact_data;
  contact_models.push_back(ci_RA_3D);
  contact_data.push_back(RigidConstraintData(ci_RA_3D));
  contact_models.push_back(ci_RA_3D);
  contact_data.push_back(RigidConstraintData(ci_RA_3D));

  VectorXd q = randomConfiguration(model);
  VectorXd v = Eigen::VectorXd::Random(model.nv);
  pinocchio::computeJointJacobians(model, data, q);
  data.q_in = q;
  pinocchio::calc(model, data, contact_models, contact_data);

  pinocchio::Data::ConstraintCholeskyDecomposition constraint_chol(
    model, data, contact_models, contact_data);
  MatrixXd H_inverse(constraint_chol.size(), constraint_chol.size());

  computeAllTerms(model, data, q, v);
  constraint_chol.compute(model, data, contact_models, contact_data, mu);
  constraint_chol.inverse(H_inverse);

  Data::MatrixXs dampedDelassusInverse;
  dampedDelassusInverse.resize(constraint_chol.constraintDim(), constraint_chol.constraintDim());

  initPvDelassus(model, data, contact_models); // Allocate memory
  computeDampedDelassusMatrixInverse(
    model, data, q, contact_models, contact_data, dampedDelassusInverse, mu);
  dampedDelassusInverse.triangularView<StrictlyLower>() =
    dampedDelassusInverse.triangularView<StrictlyUpper>().transpose();
  BOOST_CHECK(dampedDelassusInverse.isApprox(
    -H_inverse.topLeftCorner(constraint_chol.constraintDim(), constraint_chol.constraintDim()),
    1e-10));

  computeDampedDelassusMatrixInverse(
    model, data, q, contact_models, contact_data, dampedDelassusInverse, mu, false, false);
  dampedDelassusInverse.triangularView<StrictlyLower>() =
    dampedDelassusInverse.triangularView<StrictlyUpper>().transpose();
  BOOST_CHECK(dampedDelassusInverse.isApprox(
    -H_inverse.topLeftCorner(constraint_chol.constraintDim(), constraint_chol.constraintDim()),
    1e-7));
}

BOOST_AUTO_TEST_CASE(contact_3D_repeated4)
{
  using namespace Eigen;
  pinocchio::Model model;
  pinocchio::buildModels::humanoidRandom(model, true);
  pinocchio::Data data(model);

  const std::string RA = "rleg6_joint";
  RigidConstraintModel ci_RA_3D(CONTACT_3D, model, model.getJointId(RA), LOCAL);
  std::vector<RigidConstraintModel> contact_models;
  std::vector<RigidConstraintData> contact_data;
  contact_models.push_back(ci_RA_3D);
  contact_data.push_back(RigidConstraintData(ci_RA_3D));
  contact_models.push_back(ci_RA_3D);
  contact_data.push_back(RigidConstraintData(ci_RA_3D));
  contact_models.push_back(ci_RA_3D);
  contact_data.push_back(RigidConstraintData(ci_RA_3D));
  contact_models.push_back(ci_RA_3D);
  contact_data.push_back(RigidConstraintData(ci_RA_3D));

  VectorXd q = randomConfiguration(model);
  VectorXd v = Eigen::VectorXd::Random(model.nv);
  pinocchio::computeJointJacobians(model, data, q);
  data.q_in = q;
  pinocchio::calc(model, data, contact_models, contact_data);

  pinocchio::Data::ConstraintCholeskyDecomposition constraint_chol(
    model, data, contact_models, contact_data);
  MatrixXd H_inverse(constraint_chol.size(), constraint_chol.size());

  computeAllTerms(model, data, q, v);
  constraint_chol.compute(model, data, contact_models, contact_data, mu);
  constraint_chol.inverse(H_inverse);

  Data::MatrixXs dampedDelassusInverse;
  dampedDelassusInverse.resize(constraint_chol.constraintDim(), constraint_chol.constraintDim());

  initPvDelassus(model, data, contact_models); // Allocate memory
  computeDampedDelassusMatrixInverse(
    model, data, q, contact_models, contact_data, dampedDelassusInverse, mu);
  dampedDelassusInverse.triangularView<StrictlyLower>() =
    dampedDelassusInverse.triangularView<StrictlyUpper>().transpose();
  BOOST_CHECK(dampedDelassusInverse.isApprox(
    -H_inverse.topLeftCorner(constraint_chol.constraintDim(), constraint_chol.constraintDim()),
    1e-10));

  computeDampedDelassusMatrixInverse(
    model, data, q, contact_models, contact_data, dampedDelassusInverse, mu, false, false);
  dampedDelassusInverse.triangularView<StrictlyLower>() =
    dampedDelassusInverse.triangularView<StrictlyUpper>().transpose();
  BOOST_CHECK(dampedDelassusInverse.isApprox(
    -H_inverse.topLeftCorner(constraint_chol.constraintDim(), constraint_chol.constraintDim()),
    1e-10));
}

BOOST_AUTO_TEST_CASE(contact_3D_repeated4_6D4)
{
  using namespace Eigen;
  pinocchio::Model model;
  pinocchio::buildModels::humanoidRandom(model, true);
  pinocchio::Data data(model);

  const std::string RA = "rleg6_joint";
  RigidConstraintModel ci_RA_3D(CONTACT_3D, model, model.getJointId(RA), LOCAL);
  std::vector<RigidConstraintModel> contact_models;
  std::vector<RigidConstraintData> contact_data;
  contact_models.push_back(ci_RA_3D);
  contact_data.push_back(RigidConstraintData(ci_RA_3D));
  contact_models.push_back(ci_RA_3D);
  contact_data.push_back(RigidConstraintData(ci_RA_3D));
  contact_models.push_back(ci_RA_3D);
  contact_data.push_back(RigidConstraintData(ci_RA_3D));
  contact_models.push_back(ci_RA_3D);
  contact_data.push_back(RigidConstraintData(ci_RA_3D));

  const std::string RF = "rleg6_joint";
  const std::string LF = "lleg6_joint";
  const std::string LA = "larm6_joint";
  RigidConstraintModel ci_LA_6D(CONTACT_6D, model, model.getJointId(LA), LOCAL);
  RigidConstraintModel ci_RA_6D(CONTACT_6D, model, model.getJointId(RA), LOCAL);
  RigidConstraintModel ci_LF_6D(CONTACT_6D, model, model.getJointId(LF), LOCAL);
  RigidConstraintModel ci_RF_6D(CONTACT_6D, model, model.getJointId(RF), LOCAL);
  contact_models.push_back(ci_RA_6D);
  contact_data.push_back(RigidConstraintData(ci_RA_6D));
  contact_models.push_back(ci_LA_6D);
  contact_data.push_back(RigidConstraintData(ci_LA_6D));
  contact_models.push_back(ci_RF_6D);
  contact_data.push_back(RigidConstraintData(ci_RF_6D));
  contact_models.push_back(ci_LF_6D);
  contact_data.push_back(RigidConstraintData(ci_LF_6D));

  VectorXd q = randomConfiguration(model);
  VectorXd v = Eigen::VectorXd::Random(model.nv);
  pinocchio::computeJointJacobians(model, data, q);
  data.q_in = q;
  pinocchio::calc(model, data, contact_models, contact_data);

  pinocchio::Data::ConstraintCholeskyDecomposition constraint_chol(
    model, data, contact_models, contact_data);
  MatrixXd H_inverse(constraint_chol.size(), constraint_chol.size());

  computeAllTerms(model, data, q, v);
  constraint_chol.compute(model, data, contact_models, contact_data, mu);
  constraint_chol.inverse(H_inverse);

  Data::MatrixXs dampedDelassusInverse;
  dampedDelassusInverse.resize(constraint_chol.constraintDim(), constraint_chol.constraintDim());

  initPvDelassus(model, data, contact_models); // Allocate memory
  computeDampedDelassusMatrixInverse(
    model, data, q, contact_models, contact_data, dampedDelassusInverse, mu);
  dampedDelassusInverse.triangularView<StrictlyLower>() =
    dampedDelassusInverse.triangularView<StrictlyUpper>().transpose();
  BOOST_CHECK(dampedDelassusInverse.isApprox(
    -H_inverse.topLeftCorner(constraint_chol.constraintDim(), constraint_chol.constraintDim()),
    1e-10));

  computeDampedDelassusMatrixInverse(
    model, data, q, contact_models, contact_data, dampedDelassusInverse, mu, false, false);
  dampedDelassusInverse.triangularView<StrictlyLower>() =
    dampedDelassusInverse.triangularView<StrictlyUpper>().transpose();
  BOOST_CHECK(dampedDelassusInverse.isApprox(
    -H_inverse.topLeftCorner(constraint_chol.constraintDim(), constraint_chol.constraintDim()),
    1e-7));
}

BOOST_AUTO_TEST_CASE(contact_3D_ancestors)
{
  using namespace Eigen;
  pinocchio::Model model;
  pinocchio::buildModels::humanoidRandom(model, true);
  pinocchio::Data data(model);

  double mu = 1e-3;
  const std::string RA = "rleg6_joint";
  const std::string LA = "rleg4_joint";
  RigidConstraintModel ci_LA_3D(CONTACT_3D, model, model.getJointId(LA), LOCAL);
  RigidConstraintModel ci_RA_3D(CONTACT_3D, model, model.getJointId(RA), LOCAL);
  std::vector<RigidConstraintModel> contact_models;
  std::vector<RigidConstraintData> contact_data;
  contact_models.push_back(ci_RA_3D);
  contact_data.push_back(RigidConstraintData(ci_RA_3D));
  contact_models.push_back(ci_LA_3D);
  contact_data.push_back(RigidConstraintData(ci_LA_3D));

  VectorXd v = Eigen::VectorXd::Random(model.nv);
  VectorXd q = randomConfiguration(model);
  pinocchio::computeJointJacobians(model, data, q);
  data.q_in = q;
  pinocchio::calc(model, data, contact_models, contact_data);

  pinocchio::Data::ConstraintCholeskyDecomposition constraint_chol(
    model, data, contact_models, contact_data);
  MatrixXd H_inverse(constraint_chol.size(), constraint_chol.size());

  computeAllTerms(model, data, q, v);
  constraint_chol.compute(model, data, contact_models, contact_data, mu);
  constraint_chol.inverse(H_inverse);

  initPvDelassus(model, data, contact_models); // Allocate memory

  for (int i = 0; i < 2; i++)
  {
    Eigen::MatrixXd dampedDelassusInverse;
    dampedDelassusInverse.resize(constraint_chol.constraintDim(), constraint_chol.constraintDim());
    dampedDelassusInverse.setZero();

    computeAllTerms(model, data, q, v);
    data.q_in = q;
    data.v_in = v;
    pinocchio::calc(model, data, contact_models, contact_data);
    constraint_chol.compute(model, data, contact_models, contact_data, mu);
    constraint_chol.inverse(H_inverse);

    computeDampedDelassusMatrixInverse(
      model, data, q, contact_models, contact_data, dampedDelassusInverse, mu);
    dampedDelassusInverse.triangularView<StrictlyLower>() =
      dampedDelassusInverse.triangularView<StrictlyUpper>().transpose();
    BOOST_CHECK(dampedDelassusInverse.isApprox(
      -H_inverse.topLeftCorner(constraint_chol.constraintDim(), constraint_chol.constraintDim()),
      1e-10));

    computeDampedDelassusMatrixInverse(
      model, data, q, contact_models, contact_data, dampedDelassusInverse, mu, false, false);
    dampedDelassusInverse.triangularView<StrictlyLower>() =
      dampedDelassusInverse.triangularView<StrictlyUpper>().transpose();
    BOOST_CHECK(dampedDelassusInverse.isApprox(
      -H_inverse.topLeftCorner(constraint_chol.constraintDim(), constraint_chol.constraintDim()),
      1e-10));

    q = randomConfiguration(model);
    v = Eigen::VectorXd::Random(model.nv);
  }
}

BOOST_AUTO_TEST_CASE(contact_3D_6D_ancestor)
{
  using namespace Eigen;
  pinocchio::Model model;
  pinocchio::buildModels::humanoidRandom(model, true);
  pinocchio::Data data(model);

  const std::string RA = "rleg6_joint";
  const std::string LA = "rleg4_joint";
  RigidConstraintModel ci_LA_6D(CONTACT_6D, model, model.getJointId(LA), LOCAL);
  RigidConstraintModel ci_RA_3D(CONTACT_3D, model, model.getJointId(RA), LOCAL);
  std::vector<RigidConstraintModel> contact_models;
  std::vector<RigidConstraintData> contact_data;
  contact_models.push_back(ci_RA_3D);
  contact_data.push_back(RigidConstraintData(ci_RA_3D));
  contact_models.push_back(ci_LA_6D);
  contact_data.push_back(RigidConstraintData(ci_LA_6D));

  VectorXd q = randomConfiguration(model);
  VectorXd v = Eigen::VectorXd::Random(model.nv);
  pinocchio::computeJointJacobians(model, data, q);
  data.q_in = q;
  pinocchio::calc(model, data, contact_models, contact_data);

  pinocchio::Data::ConstraintCholeskyDecomposition constraint_chol(
    model, data, contact_models, contact_data);
  MatrixXd H_inverse(constraint_chol.size(), constraint_chol.size());

  computeAllTerms(model, data, q, v);
  constraint_chol.compute(model, data, contact_models, contact_data, mu);
  constraint_chol.inverse(H_inverse);

  Data::MatrixXs dampedDelassusInverse;
  dampedDelassusInverse.resize(constraint_chol.constraintDim(), constraint_chol.constraintDim());

  initPvDelassus(model, data, contact_models); // Allocate memory
  computeDampedDelassusMatrixInverse(
    model, data, q, contact_models, contact_data, dampedDelassusInverse, mu);
  dampedDelassusInverse.triangularView<StrictlyLower>() =
    dampedDelassusInverse.triangularView<StrictlyUpper>().transpose();
  BOOST_CHECK(dampedDelassusInverse.isApprox(
    -H_inverse.topLeftCorner(constraint_chol.constraintDim(), constraint_chol.constraintDim()),
    1e-10));

  computeDampedDelassusMatrixInverse(
    model, data, q, contact_models, contact_data, dampedDelassusInverse, mu, false, false);
  dampedDelassusInverse.triangularView<StrictlyLower>() =
    dampedDelassusInverse.triangularView<StrictlyUpper>().transpose();
  BOOST_CHECK(dampedDelassusInverse.isApprox(
    -H_inverse.topLeftCorner(constraint_chol.constraintDim(), constraint_chol.constraintDim()),
    1e-7));
}

BOOST_AUTO_TEST_CASE(contact_3D_6D_ancestor_6D4)
{
  using namespace Eigen;
  pinocchio::Model model;
  pinocchio::buildModels::humanoidRandom(model, true);
  pinocchio::Data data(model);

  const std::string RA = "rleg6_joint";
  const std::string LA = "rleg4_joint";
  RigidConstraintModel ci_LA_6D(CONTACT_6D, model, model.getJointId(LA), LOCAL);
  RigidConstraintModel ci_RA_3D(CONTACT_3D, model, model.getJointId(RA), LOCAL);
  std::vector<RigidConstraintModel> contact_models;
  std::vector<RigidConstraintData> contact_data;
  contact_models.push_back(ci_RA_3D);
  contact_data.push_back(RigidConstraintData(ci_RA_3D));
  contact_models.push_back(ci_LA_6D);
  contact_data.push_back(RigidConstraintData(ci_LA_6D));

  const std::string RF = "rleg6_joint";
  const std::string LF = "lleg6_joint";
  RigidConstraintModel ci_LF_6D(CONTACT_6D, model, model.getJointId(LF), LOCAL);
  RigidConstraintModel ci_RF_6D(CONTACT_6D, model, model.getJointId(RF), LOCAL);
  contact_models.push_back(ci_RF_6D);
  contact_data.push_back(RigidConstraintData(ci_RF_6D));
  contact_models.push_back(ci_LF_6D);
  contact_data.push_back(RigidConstraintData(ci_LF_6D));

  VectorXd q = randomConfiguration(model);
  VectorXd v = Eigen::VectorXd::Random(model.nv);
  pinocchio::computeJointJacobians(model, data, q);
  data.q_in = q;
  pinocchio::calc(model, data, contact_models, contact_data);

  pinocchio::Data::ConstraintCholeskyDecomposition constraint_chol(
    model, data, contact_models, contact_data);
  MatrixXd H_inverse(constraint_chol.size(), constraint_chol.size());

  computeAllTerms(model, data, q, v);
  constraint_chol.compute(model, data, contact_models, contact_data, mu);
  constraint_chol.inverse(H_inverse);

  Data::MatrixXs dampedDelassusInverse;
  dampedDelassusInverse.resize(constraint_chol.constraintDim(), constraint_chol.constraintDim());

  initPvDelassus(model, data, contact_models); // Allocate memory
  computeDampedDelassusMatrixInverse(
    model, data, q, contact_models, contact_data, dampedDelassusInverse, mu);
  dampedDelassusInverse.triangularView<StrictlyLower>() =
    dampedDelassusInverse.triangularView<StrictlyUpper>().transpose();
  BOOST_CHECK(dampedDelassusInverse.isApprox(
    -H_inverse.topLeftCorner(constraint_chol.constraintDim(), constraint_chol.constraintDim()),
    1e-10));

  computeDampedDelassusMatrixInverse(
    model, data, q, contact_models, contact_data, dampedDelassusInverse, mu, false, false);
  dampedDelassusInverse.triangularView<StrictlyLower>() =
    dampedDelassusInverse.triangularView<StrictlyUpper>().transpose();
  BOOST_CHECK(dampedDelassusInverse.isApprox(
    -H_inverse.topLeftCorner(constraint_chol.constraintDim(), constraint_chol.constraintDim()),
    1e-7));
}
