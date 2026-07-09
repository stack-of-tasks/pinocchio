//
// Copyright (c) 2023-2024 INRIA
//

#define BOOST_TEST_MODULE constraint_variants

#include "pinocchio/multibody.hpp"
#include "pinocchio/constraints.hpp"
#include "pinocchio/multibody/sample-models.hpp"

#include "constraints/init_constraints.hpp"

#include <boost/utility/binary.hpp>

#include <boost/test/unit_test.hpp>

using namespace pinocchio;
using namespace Eigen;

BOOST_AUTO_TEST_CASE(constraint_variants)
{
  Model model;
  buildModels::humanoidRandom(model, true);

  Data data(model);

  PointContactConstraintModel rcm = init_constraint<PointContactConstraintModel>(model);
  PointContactConstraintData rcd(rcm);

  ConstraintModel::ConstraintModelVariant constraint_model_variant = rcm;
  ConstraintModel constraint_model(rcm);
  ConstraintModel constraint_model_equal = rcm;

  ConstraintData constraint_data = rcm.createData();
}

BOOST_AUTO_TEST_CASE(constraint_visitors)
{
  Model model;
  buildModels::humanoidRandom(model, true);

  Data data(model);

  PointContactConstraintModel rcm = init_constraint<PointContactConstraintModel>(model);
  PointContactConstraintData rcd(rcm);

  BOOST_CHECK(ConstraintData(rcd) == ConstraintData(rcd));
  BOOST_CHECK(ConstraintData(rcd) == rcd);

  ConstraintModel constraint_model(rcm);
  ConstraintData constraint_data(rcd);

  // Test size
  {
    BOOST_CHECK(
      constraint_model.residualSize(MaximalSelection()) == rcm.residualSize(MaximalSelection()));
    BOOST_CHECK(constraint_model.residualSize() == rcm.residualSize());
  }

  // Test create data visitor
  {
    PointContactConstraintData rcd(rcm);
    ConstraintData constraint_data = visitors::createData(constraint_model);
    constraint_data = rcd;
    BOOST_CHECK(constraint_data == rcd);
  }

  // Test calc visitor
  {
    ConstraintData constraint_data1(rcm.createData());
    visitors::calc(constraint_model, model, data, constraint_data1);
    rcm.calc(model, data, rcd);
    //    BOOST_CHECK(rcd == constraint_data1);
    ConstraintData constraint_data2(rcm.createData());
    constraint_model.calc(model, data, constraint_data2);
    //    BOOST_CHECK(rcd == constraint_data2);
  }

  // Test jacobian visitor
  {
    ConstraintData constraint_data(rcm.createData());
    Data::MatrixXs jacobian_matrix1 = Data::MatrixXs::Zero(rcm.residualSize(), model.nv),
                   jacobian_matrix2 = Data::MatrixXs::Zero(rcm.residualSize(), model.nv),
                   jacobian_matrix_ref = Data::MatrixXs::Zero(rcm.residualSize(), model.nv);
    rcm.jacobian(model, data, rcd, jacobian_matrix_ref);
    visitors::jacobian(constraint_model, model, data, constraint_data, jacobian_matrix1);
    BOOST_CHECK(jacobian_matrix1 == jacobian_matrix_ref);
    constraint_model.jacobian(model, data, constraint_data, jacobian_matrix2);
    BOOST_CHECK(jacobian_matrix2 == jacobian_matrix_ref);
  }

  // Test getRowIndexes
  {
    ConstraintData constraint_data(rcm.createData());
    rcm.calc(model, data, rcd);
    PointContactConstraintModel::EigenIndexVector cm_indexes, rcm_indexes;
    for (Eigen::Index row_id = 0; row_id < constraint_model.residualSize(); ++row_id)
    {
      constraint_model.getRowIndexes(model, data, constraint_data, row_id, cm_indexes);
      rcm.getRowIndexes(model, data, rcd, row_id, rcm_indexes);
      BOOST_CHECK(cm_indexes == rcm_indexes);
    }
  }

  // Test getRowSparsityPattern
  {
    ConstraintData constraint_data(rcm.createData());
    PointContactConstraintModel::BooleanVector cm_sparsity, rcm_sparsity;
    for (Eigen::Index row_id = 0; row_id < constraint_model.residualSize(); ++row_id)
    {
      constraint_model.getRowSparsityPattern(model, data, constraint_data, row_id, cm_sparsity);
      rcm.getRowSparsityPattern(model, data, rcd, row_id, rcm_sparsity);
      BOOST_CHECK(cm_sparsity == rcm_sparsity);
    }
  }

  // Test jacobianMatrixProduct
  {
    const Eigen::Index num_cols = 20;
    ConstraintData constraint_data(rcm.createData());
    const Data::MatrixXs input_matrix = Data::MatrixXs::Random(model.nv, num_cols);
    Data::MatrixXs output_matrix1(rcm.residualSize(), num_cols),
      output_matrix2(rcm.residualSize(), num_cols), output_matrix_ref(rcm.residualSize(), num_cols);
    rcm.jacobianMatrixProduct(model, data, rcd, input_matrix, output_matrix_ref);
    visitors::jacobianMatrixProduct(
      constraint_model, model, data, constraint_data, input_matrix, output_matrix1);
    BOOST_CHECK(output_matrix1 == output_matrix_ref);
    constraint_model.jacobianMatrixProduct(
      model, data, constraint_data, input_matrix, output_matrix2);
    BOOST_CHECK(output_matrix2 == output_matrix_ref);
  }

  // Test jacobianTransposeMatrixProduct
  {
    const Eigen::Index num_cols = 20;
    ConstraintData constraint_data(rcm.createData());
    const Data::MatrixXs input_matrix = Data::MatrixXs::Random(rcm.residualSize(), num_cols);
    Data::MatrixXs output_matrix1(model.nv, num_cols), output_matrix2(model.nv, num_cols),
      output_matrix_ref(model.nv, num_cols);
    rcm.jacobianTransposeMatrixProduct(model, data, rcd, input_matrix, output_matrix_ref);
    visitors::jacobianTransposeMatrixProduct(
      constraint_model, model, data, constraint_data, input_matrix, output_matrix1);
    BOOST_CHECK(output_matrix1 == output_matrix_ref);
    constraint_model.jacobianTransposeMatrixProduct(
      model, data, constraint_data, input_matrix, output_matrix2);
    BOOST_CHECK(output_matrix2 == output_matrix_ref);
  }
}
