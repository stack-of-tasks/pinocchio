//
// Copyright (c) 2019-2020 INRIA
//

#include "pinocchio/multibody/data.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/parsers/sample-models.hpp"

#include "pinocchio/algorithm/check.hpp"

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

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
    
    for(Model::JointIndex joint_id = 1; joint_id < (Model::JointIndex)model.njoints; ++joint_id)
    {
      const int nv_joint = model.joints[joint_id].nv();
      const int idx_joint = model.joints[joint_id].idx_v();
      
      for(int k = 0; k < nv_joint; ++k)
      {
        BOOST_CHECK(data.start_idx_v_fromRow[(size_t)(idx_joint+k)] == idx_joint);
        BOOST_CHECK(data.end_idx_v_fromRow[(size_t)(idx_joint+k)] == idx_joint+nv_joint-1);
      }
    }
  }

  BOOST_AUTO_TEST_CASE(test_data_supports_fromRow)
  {
    Model model;
    buildModels::humanoidRandom(model);
    
    Data data(model);
    
    for(size_t k = 0; k < (size_t)model.nv; ++k)
    {
      const std::vector<int> & support = data.supports_fromRow[k];
      const int parent_id = data.parents_fromRow[k];
      
      if(parent_id >= 0)
      {
        const std::vector<int> & support_parent = data.supports_fromRow[(size_t)parent_id];
        BOOST_CHECK(support.size() == support_parent.size()+1);
        for(size_t j = 0; j < support_parent.size(); ++j)
        {
          BOOST_CHECK(support[j] == support_parent[j]);
        }
      }
      
      BOOST_CHECK(support.back() == (int)k);
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
    for(size_t k = 0; k < 20; ++k)
      datas.push_back(Data(model));
  }

BOOST_AUTO_TEST_SUITE_END()
