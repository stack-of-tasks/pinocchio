//
// Copyright (c) 2019-2020 INRIA
//

#include "pinocchio/multibody/data.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/sample-models.hpp"

#include "pinocchio/algorithm/check.hpp"

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

#include "utils/model-generator.hpp"

using namespace pinocchio;

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

BOOST_AUTO_TEST_CASE(test_empty_model)
{
  Model empty_model;
  Data empty_data(empty_model);

  BOOST_CHECK(empty_model.check(empty_data));
}

BOOST_AUTO_TEST_CASE(test_data_start_idx_v_fromRow)
{
  Model model;
  buildModels::humanoidRandom(model);

  Data data(model);

  for (Model::JointIndex joint_id = 1; joint_id < (Model::JointIndex)model.njoints; ++joint_id)
  {
    const int nv_joint = model.joints[joint_id].nv();
    const int idx_joint = model.joints[joint_id].idx_v();

    for (int k = 0; k < nv_joint; ++k)
    {
      BOOST_CHECK(data.start_idx_v_fromRow[(size_t)(idx_joint + k)] == idx_joint);
      BOOST_CHECK(data.end_idx_v_fromRow[(size_t)(idx_joint + k)] == idx_joint + nv_joint - 1);
    }
  }
}

BOOST_AUTO_TEST_CASE(test_data_supports_fromRow)
{
  Model model;
  buildModels::humanoidRandom(model);

  Data data(model);

  for (size_t k = 0; k < (size_t)model.nv; ++k)
  {
    const std::vector<int> & support = data.supports_fromRow[k];
    const int parent_id = data.parents_fromRow[k];

    if (parent_id >= 0)
    {
      const std::vector<int> & support_parent = data.supports_fromRow[(size_t)parent_id];
      BOOST_CHECK(support.size() == support_parent.size() + 1);
      for (size_t j = 0; j < support_parent.size(); ++j)
      {
        BOOST_CHECK(support[j] == support_parent[j]);
      }
    }

    BOOST_CHECK(support.back() == (int)k);
  }
}

BOOST_AUTO_TEST_CASE(test_data_mimic_idx_vExtended_to_idx_v_fromRow)
{
  for (int i = 0; i < pinocchio::MimicTestCases::N_CASES; i++)
  {
    const pinocchio::MimicTestCases mimic_test_case(i);
    const pinocchio::Model & model_mimic = mimic_test_case.model_mimic;
    const pinocchio::Model & model_full = mimic_test_case.model_full;

    Data data_mimic(model_mimic);
    Data data_full(model_full);

    for (size_t joint_id = 1; joint_id < model_mimic.njoints; joint_id++)
    {
      const int idx_vj = model_mimic.joints[joint_id].idx_v();
      const int idx_vExtended_j = model_mimic.joints[joint_id].idx_vExtended();
      const int nvExtended_j = model_mimic.joints[joint_id].nvExtended();
      for (int v = 0; v < nvExtended_j; v++)
      {
        BOOST_CHECK(
          data_mimic.idx_vExtended_to_idx_v_fromRow[size_t(idx_vExtended_j + v)] == idx_vj + v);
        BOOST_CHECK(
          data_full.idx_vExtended_to_idx_v_fromRow[size_t(idx_vExtended_j + v)]
          == idx_vExtended_j + v);
      }
    }
  }
}

BOOST_AUTO_TEST_CASE(test_data_mimic_mimic_parents_fromRow)
{
  for (int i = 0; i < pinocchio::MimicTestCases::N_CASES; i++)
  {
    const pinocchio::MimicTestCases mimic_test_case(i);
    const pinocchio::Model & model_mimic = mimic_test_case.model_mimic;

    Data data_mimic(model_mimic);

    for (size_t joint_id = 1; joint_id < model_mimic.njoints; joint_id++)
    {
      const int idx_vExtended_j = model_mimic.joints[joint_id].idx_vExtended();
      const int nvExtended_j = model_mimic.joints[joint_id].nvExtended();

      // If the parent from row is not the universe, it should be either mimic or non mimic - not
      // both
      const bool parent_is_universe = (data_mimic.parents_fromRow[idx_vExtended_j] == -1);
      const bool parent_is_mimic =
        (data_mimic.mimic_parents_fromRow[idx_vExtended_j]
         == data_mimic.parents_fromRow[idx_vExtended_j]);
      const bool parent_is_not_mimic =
        (data_mimic.non_mimic_parents_fromRow[idx_vExtended_j]
         == data_mimic.parents_fromRow[idx_vExtended_j]);
      BOOST_CHECK(parent_is_universe || (parent_is_mimic != parent_is_not_mimic));

      for (int v = 1; v < nvExtended_j; v++)
      {
        BOOST_CHECK(
          data_mimic.mimic_parents_fromRow[size_t(idx_vExtended_j + v)] == idx_vExtended_j + v - 1);
      }
    }
  }
}

BOOST_AUTO_TEST_CASE(test_copy_and_equal_op)
{
  Model model;
  buildModels::humanoidRandom(model);

  Data data(model);
  Data data_copy = data;

  BOOST_CHECK(data == data);
  BOOST_CHECK(data == data_copy);

  data_copy.oMi[0].setRandom();
  BOOST_CHECK(data != data_copy);
}

BOOST_AUTO_TEST_CASE(test_container_aligned_vector)
{
  Model model;
  buildModels::humanoidRandom(model);

  Data data(model);

  container::aligned_vector<Data::Force> & f = data.f;
  data.f[0].setRandom();

  BOOST_CHECK(data.f[0] == f[0]);
}

BOOST_AUTO_TEST_CASE(test_std_vector_of_Data)
{
  Model model;
  buildModels::humanoidRandom(model);

  PINOCCHIO_ALIGNED_STD_VECTOR(Data) datas;
  for (size_t k = 0; k < 20; ++k)
    datas.push_back(Data(model));
}

BOOST_AUTO_TEST_CASE(test_mimic_subtree)
{
  Model model;
  buildModels::manipulator(model);
  // Direct parent/Child
  std::vector<pinocchio::JointIndex> mimicked = {model.getJointId("shoulder1_joint")};
  std::vector<pinocchio::JointIndex> mimicking = {model.getJointId("shoulder2_joint")};

  const std::vector<double> ratio = {2.5};
  const std::vector<double> offset = {0.75};
  pinocchio::Model model_mimic;
  pinocchio::buildMimicModel(model, mimicked, mimicking, ratio, offset, model_mimic);

  Data data_mimic(model_mimic);
  Data data(model);

  // No mimic so should not be filled
  BOOST_CHECK(data.mimic_subtree_joint.size() == 0);

  // it's a linear model with RX and RY so the joint after shoulder2 is not a mimic and is in its
  // subtree
  BOOST_CHECK(data_mimic.mimic_subtree_joint[0] == model_mimic.getJointId("shoulder3_joint"));

  // Test when mimic is terminal
  Model man_mimic;
  buildModels::manipulator(man_mimic, true);

  Data data_man_mimic(man_mimic);
  BOOST_CHECK(data_man_mimic.mimic_subtree_joint[0] == 0);
}

BOOST_AUTO_TEST_SUITE_END()
