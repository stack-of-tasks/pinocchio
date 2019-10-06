//
// Copyright (c) 2019 INRIA
//

#include "pinocchio/multibody/data.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/parsers/sample-models.hpp"

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

using namespace pinocchio;

BOOST_AUTO_TEST_SUITE ( BOOST_TEST_MODULE )

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

BOOST_AUTO_TEST_SUITE_END()
