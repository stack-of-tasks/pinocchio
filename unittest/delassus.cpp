//
// Copyright (c) 2023-2024 INRIA CNRS
// Copyright (c) 2023 KU Leuven
//

#include "pinocchio/algorithm/contact-info.hpp"
#include "pinocchio/algorithm/contact-dynamics.hpp"
#include "pinocchio/algorithm/contact-cholesky.hxx"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/multibody/sample-models.hpp"
#include "pinocchio/algorithm/delassus.hpp"

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

namespace pinocchio
{
  template<typename Scalar, int Options>
  struct ContactCholeskyDecompositionAccessorTpl
  : public ContactCholeskyDecompositionTpl<Scalar, Options>
  {
    typedef ContactCholeskyDecompositionTpl<Scalar, Options> Base;
    typedef typename Base::IndexVector IndexVector;
    typedef typename Base::BooleanVector BooleanVector;

    ContactCholeskyDecompositionAccessorTpl(const Base & other)
    : Base(other)
    {
    }

    const IndexVector & getParents_fromRow() const
    {
      return this->parents_fromRow;
    }
    const IndexVector & getLastChild() const
    {
      return this->last_child;
    }
    const IndexVector & getNvSubtree_fromRow() const
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

  typedef ContactCholeskyDecompositionAccessorTpl<double, 0> ContactCholeskyDecompositionAccessor;
} // namespace pinocchio

using namespace pinocchio;

double mu = 1e-4;
BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

BOOST_AUTO_TEST_CASE(contact_6D)
{
  using namespace Eigen;
  pinocchio::Model model;
  pinocchio::buildModels::humanoidRandom(model, true);
  pinocchio::Data data(model);

  const std::string RA = "rleg6_joint";
  RigidConstraintModel ci_RA_6D(CONTACT_6D, model, model.getJointId(RA), LOCAL);
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) contact_models;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintData) contact_data;
  contact_models.push_back(ci_RA_6D);
  contact_data.push_back(RigidConstraintData(ci_RA_6D));

  pinocchio::Data::ContactCholeskyDecomposition contact_chol(model, contact_models);

  MatrixXd H_inverse(contact_chol.size(), contact_chol.size());
  VectorXd q = randomConfiguration(model);
  VectorXd v = Eigen::VectorXd::Random(model.nv);

  initPvDelassus(model, data, contact_models); // Allocate memory

  for (int i = 0; i < 10; i++)
  {
    computeAllTerms(model, data, q, v);
    contact_chol.compute(model, data, contact_models, contact_data, mu);
    contact_chol.inverse(H_inverse);

    Eigen::MatrixXd dampedDelassusInverse;
    dampedDelassusInverse.resize(contact_chol.constraintDim(), contact_chol.constraintDim());

    Eigen::MatrixXd dampedDelassusInverse2;
    dampedDelassusInverse2.resize(contact_chol.constraintDim(), contact_chol.constraintDim());

    dampedDelassusInverse2 =
      -H_inverse.topLeftCorner(contact_chol.constraintDim(), contact_chol.constraintDim());
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
      -H_inverse.topLeftCorner(contact_chol.constraintDim(), contact_chol.constraintDim()), 1e-10));

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
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) contact_models;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintData) contact_data;
  contact_models.push_back(ci_RA_6D);
  contact_data.push_back(RigidConstraintData(ci_RA_6D));
  contact_models.push_back(ci_LA_6D);
  contact_data.push_back(RigidConstraintData(ci_LA_6D));

  pinocchio::Data::ContactCholeskyDecomposition contact_chol(model, contact_models);

  MatrixXd H_inverse(contact_chol.size(), contact_chol.size());
  VectorXd q = randomConfiguration(model);
  VectorXd v = Eigen::VectorXd::Random(model.nv);

  computeAllTerms(model, data, q, v);
  contact_chol.compute(model, data, contact_models, contact_data, mu);
  contact_chol.inverse(H_inverse);

  Data::MatrixXs dampedDelassusInverse;
  dampedDelassusInverse.resize(contact_chol.constraintDim(), contact_chol.constraintDim());

  initPvDelassus(model, data, contact_models); // Allocate memory
  computeDampedDelassusMatrixInverse(
    model, data, q, contact_models, contact_data, dampedDelassusInverse, mu);
  dampedDelassusInverse.triangularView<StrictlyLower>() =
    dampedDelassusInverse.triangularView<StrictlyUpper>().transpose();
  BOOST_CHECK(dampedDelassusInverse.isApprox(
    -H_inverse.topLeftCorner(contact_chol.constraintDim(), contact_chol.constraintDim()), 1e-11));

  computeDampedDelassusMatrixInverse(
    model, data, q, contact_models, contact_data, dampedDelassusInverse, mu, false, false);
  dampedDelassusInverse.triangularView<StrictlyLower>() =
    dampedDelassusInverse.triangularView<StrictlyUpper>().transpose();
  BOOST_CHECK(dampedDelassusInverse.isApprox(
    -H_inverse.topLeftCorner(contact_chol.constraintDim(), contact_chol.constraintDim()), 1e-11));
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
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) contact_models;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintData) contact_data;
  contact_models.push_back(ci_RA_6D);
  contact_data.push_back(RigidConstraintData(ci_RA_6D));
  contact_models.push_back(ci_LA_6D);
  contact_data.push_back(RigidConstraintData(ci_LA_6D));
  contact_models.push_back(ci_RF_6D);
  contact_data.push_back(RigidConstraintData(ci_RF_6D));
  contact_models.push_back(ci_LF_6D);
  contact_data.push_back(RigidConstraintData(ci_LF_6D));

  pinocchio::Data::ContactCholeskyDecomposition contact_chol(model, contact_models);

  MatrixXd H_inverse(contact_chol.size(), contact_chol.size());
  VectorXd q = randomConfiguration(model);
  VectorXd v = Eigen::VectorXd::Random(model.nv);

  computeAllTerms(model, data, q, v);
  contact_chol.compute(model, data, contact_models, contact_data, mu);
  contact_chol.inverse(H_inverse);

  Data::MatrixXs dampedDelassusInverse;
  dampedDelassusInverse.resize(contact_chol.constraintDim(), contact_chol.constraintDim());

  initPvDelassus(model, data, contact_models); // Allocate memory
  computeDampedDelassusMatrixInverse(
    model, data, q, contact_models, contact_data, dampedDelassusInverse, mu);
  dampedDelassusInverse.triangularView<StrictlyLower>() =
    dampedDelassusInverse.triangularView<StrictlyUpper>().transpose();
  BOOST_CHECK(dampedDelassusInverse.isApprox(
    -H_inverse.topLeftCorner(contact_chol.constraintDim(), contact_chol.constraintDim()), 1e-11));

  computeDampedDelassusMatrixInverse(
    model, data, q, contact_models, contact_data, dampedDelassusInverse, mu, false, false);
  dampedDelassusInverse.triangularView<StrictlyLower>() =
    dampedDelassusInverse.triangularView<StrictlyUpper>().transpose();
  BOOST_CHECK(dampedDelassusInverse.isApprox(
    -H_inverse.topLeftCorner(contact_chol.constraintDim(), contact_chol.constraintDim()), 1e-10));
}

BOOST_AUTO_TEST_CASE(contact_6D_repeated)
{
  using namespace Eigen;
  pinocchio::Model model;
  pinocchio::buildModels::humanoidRandom(model, true);
  pinocchio::Data data(model);

  const std::string RA = "rleg6_joint";
  RigidConstraintModel ci_RA_6D(CONTACT_6D, model, model.getJointId(RA), LOCAL);
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) contact_models;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintData) contact_data;
  contact_models.push_back(ci_RA_6D);
  contact_data.push_back(RigidConstraintData(ci_RA_6D));
  contact_models.push_back(ci_RA_6D);
  contact_data.push_back(RigidConstraintData(ci_RA_6D));

  pinocchio::Data::ContactCholeskyDecomposition contact_chol(model, contact_models);

  MatrixXd H_inverse(contact_chol.size(), contact_chol.size());
  VectorXd q = randomConfiguration(model);
  VectorXd v = Eigen::VectorXd::Random(model.nv);

  computeAllTerms(model, data, q, v);
  contact_chol.compute(model, data, contact_models, contact_data, mu);
  contact_chol.inverse(H_inverse);

  Data::MatrixXs dampedDelassusInverse;
  dampedDelassusInverse.resize(contact_chol.constraintDim(), contact_chol.constraintDim());

  initPvDelassus(model, data, contact_models); // Allocate memory
  computeDampedDelassusMatrixInverse(
    model, data, q, contact_models, contact_data, dampedDelassusInverse, mu);
  dampedDelassusInverse.triangularView<StrictlyLower>() =
    dampedDelassusInverse.triangularView<StrictlyUpper>().transpose();
  BOOST_CHECK(dampedDelassusInverse.isApprox(
    -H_inverse.topLeftCorner(contact_chol.constraintDim(), contact_chol.constraintDim()), 1e-11));

  computeDampedDelassusMatrixInverse(
    model, data, q, contact_models, contact_data, dampedDelassusInverse, mu, false, false);
  dampedDelassusInverse.triangularView<StrictlyLower>() =
    dampedDelassusInverse.triangularView<StrictlyUpper>().transpose();
  BOOST_CHECK(dampedDelassusInverse.isApprox(
    -H_inverse.topLeftCorner(contact_chol.constraintDim(), contact_chol.constraintDim()), 1e-11));
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
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) contact_models;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintData) contact_data;
  contact_models.push_back(ci_RA_6D);
  contact_data.push_back(RigidConstraintData(ci_RA_6D));
  contact_models.push_back(ci_RA_6D);
  contact_data.push_back(RigidConstraintData(ci_RA_6D));

  contact_models.push_back(ci_LA_6D);
  contact_data.push_back(RigidConstraintData(ci_LA_6D));

  pinocchio::Data::ContactCholeskyDecomposition contact_chol(model, contact_models);

  MatrixXd H_inverse(contact_chol.size(), contact_chol.size());
  VectorXd q = randomConfiguration(model);
  VectorXd v = Eigen::VectorXd::Random(model.nv);

  computeAllTerms(model, data, q, v);
  contact_chol.compute(model, data, contact_models, contact_data, mu);
  contact_chol.inverse(H_inverse);

  Data::MatrixXs dampedDelassusInverse;
  dampedDelassusInverse.resize(contact_chol.constraintDim(), contact_chol.constraintDim());

  initPvDelassus(model, data, contact_models); // Allocate memory
  computeDampedDelassusMatrixInverse(
    model, data, q, contact_models, contact_data, dampedDelassusInverse, mu);
  dampedDelassusInverse.triangularView<StrictlyLower>() =
    dampedDelassusInverse.triangularView<StrictlyUpper>().transpose();
  BOOST_CHECK(dampedDelassusInverse.isApprox(
    -H_inverse.topLeftCorner(contact_chol.constraintDim(), contact_chol.constraintDim()), 1e-11));

  computeDampedDelassusMatrixInverse(
    model, data, q, contact_models, contact_data, dampedDelassusInverse, mu, false, false);
  dampedDelassusInverse.triangularView<StrictlyLower>() =
    dampedDelassusInverse.triangularView<StrictlyUpper>().transpose();
  BOOST_CHECK(dampedDelassusInverse.isApprox(
    -H_inverse.topLeftCorner(contact_chol.constraintDim(), contact_chol.constraintDim()), 1e-11));
}

BOOST_AUTO_TEST_CASE(contact_3D)
{
  using namespace Eigen;
  pinocchio::Model model;
  pinocchio::buildModels::humanoidRandom(model, true);
  pinocchio::Data data(model);

  const std::string RA = "rleg6_joint";
  RigidConstraintModel ci_RA_3D(CONTACT_3D, model, model.getJointId(RA), LOCAL);
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) contact_models;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintData) contact_data;
  contact_models.push_back(ci_RA_3D);
  contact_data.push_back(RigidConstraintData(ci_RA_3D));

  pinocchio::Data::ContactCholeskyDecomposition contact_chol(model, contact_models);

  MatrixXd H_inverse(contact_chol.size(), contact_chol.size());
  VectorXd q = randomConfiguration(model);
  VectorXd v = Eigen::VectorXd::Random(model.nv);

  computeAllTerms(model, data, q, v);
  contact_chol.compute(model, data, contact_models, contact_data, mu);
  contact_chol.inverse(H_inverse);

  Data::MatrixXs dampedDelassusInverse;
  dampedDelassusInverse.resize(contact_chol.constraintDim(), contact_chol.constraintDim());

  initPvDelassus(model, data, contact_models); // Allocate memory
  computeDampedDelassusMatrixInverse(
    model, data, q, contact_models, contact_data, dampedDelassusInverse, mu);
  dampedDelassusInverse.triangularView<StrictlyLower>() =
    dampedDelassusInverse.triangularView<StrictlyUpper>().transpose();
  BOOST_CHECK(dampedDelassusInverse.isApprox(
    -H_inverse.topLeftCorner(contact_chol.constraintDim(), contact_chol.constraintDim()), 1e-11));

  computeDampedDelassusMatrixInverse(
    model, data, q, contact_models, contact_data, dampedDelassusInverse, mu, false, false);
  dampedDelassusInverse.triangularView<StrictlyLower>() =
    dampedDelassusInverse.triangularView<StrictlyUpper>().transpose();
  BOOST_CHECK(dampedDelassusInverse.isApprox(
    -H_inverse.topLeftCorner(contact_chol.constraintDim(), contact_chol.constraintDim()), 1e-7));
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
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) contact_models;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintData) contact_data;
  contact_models.push_back(ci_RA_3D);
  contact_data.push_back(RigidConstraintData(ci_RA_3D));
  contact_models.push_back(ci_LA_3D);
  contact_data.push_back(RigidConstraintData(ci_LA_3D));

  pinocchio::Data::ContactCholeskyDecomposition contact_chol(model, contact_models);

  MatrixXd H_inverse(contact_chol.size(), contact_chol.size());
  VectorXd q = randomConfiguration(model);
  VectorXd v = Eigen::VectorXd::Random(model.nv);

  computeAllTerms(model, data, q, v);
  contact_chol.compute(model, data, contact_models, contact_data, mu);
  contact_chol.inverse(H_inverse);

  Data::MatrixXs dampedDelassusInverse;
  dampedDelassusInverse.resize(contact_chol.constraintDim(), contact_chol.constraintDim());

  initPvDelassus(model, data, contact_models); // Allocate memory
  computeDampedDelassusMatrixInverse(
    model, data, q, contact_models, contact_data, dampedDelassusInverse, mu);
  dampedDelassusInverse.triangularView<StrictlyLower>() =
    dampedDelassusInverse.triangularView<StrictlyUpper>().transpose();
  BOOST_CHECK(dampedDelassusInverse.isApprox(
    -H_inverse.topLeftCorner(contact_chol.constraintDim(), contact_chol.constraintDim()), 1e-11));

  computeDampedDelassusMatrixInverse(
    model, data, q, contact_models, contact_data, dampedDelassusInverse, mu, false, false);
  dampedDelassusInverse.triangularView<StrictlyLower>() =
    dampedDelassusInverse.triangularView<StrictlyUpper>().transpose();
  BOOST_CHECK(dampedDelassusInverse.isApprox(
    -H_inverse.topLeftCorner(contact_chol.constraintDim(), contact_chol.constraintDim()), 1e-7));
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
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) contact_models;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintData) contact_data;
  contact_models.push_back(ci_RA_3D);
  contact_data.push_back(RigidConstraintData(ci_RA_3D));
  contact_models.push_back(ci_LA_3D);
  contact_data.push_back(RigidConstraintData(ci_LA_3D));
  contact_models.push_back(ci_RF_3D);
  contact_data.push_back(RigidConstraintData(ci_RF_3D));
  contact_models.push_back(ci_LF_3D);
  contact_data.push_back(RigidConstraintData(ci_LF_3D));

  pinocchio::Data::ContactCholeskyDecomposition contact_chol(model, contact_models);

  MatrixXd H_inverse(contact_chol.size(), contact_chol.size());
  VectorXd q = randomConfiguration(model);
  VectorXd v = Eigen::VectorXd::Random(model.nv);

  computeAllTerms(model, data, q, v);
  contact_chol.compute(model, data, contact_models, contact_data, mu);
  contact_chol.inverse(H_inverse);

  Data::MatrixXs dampedDelassusInverse;
  dampedDelassusInverse.resize(contact_chol.constraintDim(), contact_chol.constraintDim());

  initPvDelassus(model, data, contact_models); // Allocate memory
  computeDampedDelassusMatrixInverse(
    model, data, q, contact_models, contact_data, dampedDelassusInverse, mu);
  dampedDelassusInverse.triangularView<StrictlyLower>() =
    dampedDelassusInverse.triangularView<StrictlyUpper>().transpose();
  BOOST_CHECK(dampedDelassusInverse.isApprox(
    -H_inverse.topLeftCorner(contact_chol.constraintDim(), contact_chol.constraintDim()), 1e-11));

  computeDampedDelassusMatrixInverse(
    model, data, q, contact_models, contact_data, dampedDelassusInverse, mu, false, false);
  dampedDelassusInverse.triangularView<StrictlyLower>() =
    dampedDelassusInverse.triangularView<StrictlyUpper>().transpose();
  BOOST_CHECK(dampedDelassusInverse.isApprox(
    -H_inverse.topLeftCorner(contact_chol.constraintDim(), contact_chol.constraintDim()), 1e-7));
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
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) contact_models;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintData) contact_data;
  contact_models.push_back(ci_RA_3D);
  contact_data.push_back(RigidConstraintData(ci_RA_3D));
  contact_models.push_back(ci_RA_3D);
  contact_data.push_back(RigidConstraintData(ci_RA_3D));

  pinocchio::Data::ContactCholeskyDecomposition contact_chol(model, contact_models);

  MatrixXd H_inverse(contact_chol.size(), contact_chol.size());
  VectorXd q = randomConfiguration(model);
  VectorXd v = Eigen::VectorXd::Random(model.nv);

  computeAllTerms(model, data, q, v);
  contact_chol.compute(model, data, contact_models, contact_data, mu);
  contact_chol.inverse(H_inverse);

  Data::MatrixXs dampedDelassusInverse;
  dampedDelassusInverse.resize(contact_chol.constraintDim(), contact_chol.constraintDim());

  initPvDelassus(model, data, contact_models); // Allocate memory
  computeDampedDelassusMatrixInverse(
    model, data, q, contact_models, contact_data, dampedDelassusInverse, mu);
  dampedDelassusInverse.triangularView<StrictlyLower>() =
    dampedDelassusInverse.triangularView<StrictlyUpper>().transpose();
  BOOST_CHECK(dampedDelassusInverse.isApprox(
    -H_inverse.topLeftCorner(contact_chol.constraintDim(), contact_chol.constraintDim()), 1e-11));

  computeDampedDelassusMatrixInverse(
    model, data, q, contact_models, contact_data, dampedDelassusInverse, mu, false, false);
  dampedDelassusInverse.triangularView<StrictlyLower>() =
    dampedDelassusInverse.triangularView<StrictlyUpper>().transpose();
  BOOST_CHECK(dampedDelassusInverse.isApprox(
    -H_inverse.topLeftCorner(contact_chol.constraintDim(), contact_chol.constraintDim()), 1e-7));
}

BOOST_AUTO_TEST_CASE(contact_3D_repeated4)
{
  using namespace Eigen;
  pinocchio::Model model;
  pinocchio::buildModels::humanoidRandom(model, true);
  pinocchio::Data data(model);

  const std::string RA = "rleg6_joint";
  RigidConstraintModel ci_RA_3D(CONTACT_3D, model, model.getJointId(RA), LOCAL);
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) contact_models;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintData) contact_data;
  contact_models.push_back(ci_RA_3D);
  contact_data.push_back(RigidConstraintData(ci_RA_3D));
  contact_models.push_back(ci_RA_3D);
  contact_data.push_back(RigidConstraintData(ci_RA_3D));
  contact_models.push_back(ci_RA_3D);
  contact_data.push_back(RigidConstraintData(ci_RA_3D));
  contact_models.push_back(ci_RA_3D);
  contact_data.push_back(RigidConstraintData(ci_RA_3D));

  pinocchio::Data::ContactCholeskyDecomposition contact_chol(model, contact_models);

  MatrixXd H_inverse(contact_chol.size(), contact_chol.size());
  VectorXd q = randomConfiguration(model);
  VectorXd v = Eigen::VectorXd::Random(model.nv);

  computeAllTerms(model, data, q, v);
  contact_chol.compute(model, data, contact_models, contact_data, mu);
  contact_chol.inverse(H_inverse);

  Data::MatrixXs dampedDelassusInverse;
  dampedDelassusInverse.resize(contact_chol.constraintDim(), contact_chol.constraintDim());

  initPvDelassus(model, data, contact_models); // Allocate memory
  computeDampedDelassusMatrixInverse(
    model, data, q, contact_models, contact_data, dampedDelassusInverse, mu);
  dampedDelassusInverse.triangularView<StrictlyLower>() =
    dampedDelassusInverse.triangularView<StrictlyUpper>().transpose();
  BOOST_CHECK(dampedDelassusInverse.isApprox(
    -H_inverse.topLeftCorner(contact_chol.constraintDim(), contact_chol.constraintDim()), 1e-10));

  computeDampedDelassusMatrixInverse(
    model, data, q, contact_models, contact_data, dampedDelassusInverse, mu, false, false);
  dampedDelassusInverse.triangularView<StrictlyLower>() =
    dampedDelassusInverse.triangularView<StrictlyUpper>().transpose();
  BOOST_CHECK(dampedDelassusInverse.isApprox(
    -H_inverse.topLeftCorner(contact_chol.constraintDim(), contact_chol.constraintDim()), 1e-10));
}

BOOST_AUTO_TEST_CASE(contact_3D_repeated4_6D4)
{
  using namespace Eigen;
  pinocchio::Model model;
  pinocchio::buildModels::humanoidRandom(model, true);
  pinocchio::Data data(model);

  const std::string RA = "rleg6_joint";
  RigidConstraintModel ci_RA_3D(CONTACT_3D, model, model.getJointId(RA), LOCAL);
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) contact_models;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintData) contact_data;
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

  pinocchio::Data::ContactCholeskyDecomposition contact_chol(model, contact_models);

  MatrixXd H_inverse(contact_chol.size(), contact_chol.size());
  VectorXd q = randomConfiguration(model);
  VectorXd v = Eigen::VectorXd::Random(model.nv);

  computeAllTerms(model, data, q, v);
  contact_chol.compute(model, data, contact_models, contact_data, mu);
  contact_chol.inverse(H_inverse);

  Data::MatrixXs dampedDelassusInverse;
  dampedDelassusInverse.resize(contact_chol.constraintDim(), contact_chol.constraintDim());

  initPvDelassus(model, data, contact_models); // Allocate memory
  computeDampedDelassusMatrixInverse(
    model, data, q, contact_models, contact_data, dampedDelassusInverse, mu);
  dampedDelassusInverse.triangularView<StrictlyLower>() =
    dampedDelassusInverse.triangularView<StrictlyUpper>().transpose();
  BOOST_CHECK(dampedDelassusInverse.isApprox(
    -H_inverse.topLeftCorner(contact_chol.constraintDim(), contact_chol.constraintDim()), 1e-11));

  computeDampedDelassusMatrixInverse(
    model, data, q, contact_models, contact_data, dampedDelassusInverse, mu, false, false);
  dampedDelassusInverse.triangularView<StrictlyLower>() =
    dampedDelassusInverse.triangularView<StrictlyUpper>().transpose();
  BOOST_CHECK(dampedDelassusInverse.isApprox(
    -H_inverse.topLeftCorner(contact_chol.constraintDim(), contact_chol.constraintDim()), 1e-7));
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
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) contact_models;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintData) contact_data;
  contact_models.push_back(ci_RA_3D);
  contact_data.push_back(RigidConstraintData(ci_RA_3D));
  contact_models.push_back(ci_LA_3D);
  contact_data.push_back(RigidConstraintData(ci_LA_3D));

  pinocchio::Data::ContactCholeskyDecomposition contact_chol(model, contact_models);

  MatrixXd H_inverse(contact_chol.size(), contact_chol.size());

  VectorXd v = Eigen::VectorXd::Random(model.nv);
  VectorXd q = randomConfiguration(model);

  computeAllTerms(model, data, q, v);
  contact_chol.compute(model, data, contact_models, contact_data, mu);
  contact_chol.inverse(H_inverse);

  initPvDelassus(model, data, contact_models); // Allocate memory

  for (int i = 0; i < 2; i++)
  {
    Eigen::MatrixXd dampedDelassusInverse;
    dampedDelassusInverse.resize(contact_chol.constraintDim(), contact_chol.constraintDim());
    dampedDelassusInverse.setZero();

    computeAllTerms(model, data, q, v);
    contact_chol.compute(model, data, contact_models, contact_data, mu);
    contact_chol.inverse(H_inverse);

    computeDampedDelassusMatrixInverse(
      model, data, q, contact_models, contact_data, dampedDelassusInverse, mu);
    dampedDelassusInverse.triangularView<StrictlyLower>() =
      dampedDelassusInverse.triangularView<StrictlyUpper>().transpose();
    BOOST_CHECK(dampedDelassusInverse.isApprox(
      -H_inverse.topLeftCorner(contact_chol.constraintDim(), contact_chol.constraintDim()), 1e-10));

    computeDampedDelassusMatrixInverse(
      model, data, q, contact_models, contact_data, dampedDelassusInverse, mu, false, false);
    dampedDelassusInverse.triangularView<StrictlyLower>() =
      dampedDelassusInverse.triangularView<StrictlyUpper>().transpose();
    BOOST_CHECK(dampedDelassusInverse.isApprox(
      -H_inverse.topLeftCorner(contact_chol.constraintDim(), contact_chol.constraintDim()), 1e-10));

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
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) contact_models;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintData) contact_data;
  contact_models.push_back(ci_RA_3D);
  contact_data.push_back(RigidConstraintData(ci_RA_3D));
  contact_models.push_back(ci_LA_6D);
  contact_data.push_back(RigidConstraintData(ci_LA_6D));

  pinocchio::Data::ContactCholeskyDecomposition contact_chol(model, contact_models);

  MatrixXd H_inverse(contact_chol.size(), contact_chol.size());
  VectorXd q = randomConfiguration(model);
  VectorXd v = Eigen::VectorXd::Random(model.nv);

  computeAllTerms(model, data, q, v);
  contact_chol.compute(model, data, contact_models, contact_data, mu);
  contact_chol.inverse(H_inverse);

  Data::MatrixXs dampedDelassusInverse;
  dampedDelassusInverse.resize(contact_chol.constraintDim(), contact_chol.constraintDim());

  initPvDelassus(model, data, contact_models); // Allocate memory
  computeDampedDelassusMatrixInverse(
    model, data, q, contact_models, contact_data, dampedDelassusInverse, mu);
  dampedDelassusInverse.triangularView<StrictlyLower>() =
    dampedDelassusInverse.triangularView<StrictlyUpper>().transpose();
  BOOST_CHECK(dampedDelassusInverse.isApprox(
    -H_inverse.topLeftCorner(contact_chol.constraintDim(), contact_chol.constraintDim()), 1e-11));

  computeDampedDelassusMatrixInverse(
    model, data, q, contact_models, contact_data, dampedDelassusInverse, mu, false, false);
  dampedDelassusInverse.triangularView<StrictlyLower>() =
    dampedDelassusInverse.triangularView<StrictlyUpper>().transpose();
  BOOST_CHECK(dampedDelassusInverse.isApprox(
    -H_inverse.topLeftCorner(contact_chol.constraintDim(), contact_chol.constraintDim()), 1e-7));
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
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintModel) contact_models;
  PINOCCHIO_STD_VECTOR_WITH_EIGEN_ALLOCATOR(RigidConstraintData) contact_data;
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

  pinocchio::Data::ContactCholeskyDecomposition contact_chol(model, contact_models);

  MatrixXd H_inverse(contact_chol.size(), contact_chol.size());
  VectorXd q = randomConfiguration(model);
  VectorXd v = Eigen::VectorXd::Random(model.nv);

  computeAllTerms(model, data, q, v);
  contact_chol.compute(model, data, contact_models, contact_data, mu);
  contact_chol.inverse(H_inverse);

  Data::MatrixXs dampedDelassusInverse;
  dampedDelassusInverse.resize(contact_chol.constraintDim(), contact_chol.constraintDim());

  initPvDelassus(model, data, contact_models); // Allocate memory
  computeDampedDelassusMatrixInverse(
    model, data, q, contact_models, contact_data, dampedDelassusInverse, mu);
  dampedDelassusInverse.triangularView<StrictlyLower>() =
    dampedDelassusInverse.triangularView<StrictlyUpper>().transpose();
  BOOST_CHECK(dampedDelassusInverse.isApprox(
    -H_inverse.topLeftCorner(contact_chol.constraintDim(), contact_chol.constraintDim()), 1e-11));

  computeDampedDelassusMatrixInverse(
    model, data, q, contact_models, contact_data, dampedDelassusInverse, mu, false, false);
  dampedDelassusInverse.triangularView<StrictlyLower>() =
    dampedDelassusInverse.triangularView<StrictlyUpper>().transpose();
  BOOST_CHECK(dampedDelassusInverse.isApprox(
    -H_inverse.topLeftCorner(contact_chol.constraintDim(), contact_chol.constraintDim()), 1e-7));
}

BOOST_AUTO_TEST_SUITE_END()
