//
// Copyright (c) 2021-2022 INRIA
//

#include "pinocchio/spatial/fwd.hpp"
#include "pinocchio/algorithm/parallel/aba.hpp"
#include "pinocchio/algorithm/aba.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/multibody/sample-models.hpp"
#include "pinocchio/utils/timer.hpp"

#include <iostream>

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

using namespace pinocchio;

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

BOOST_AUTO_TEST_CASE(test_parallel_aba)
{
  pinocchio::Model model;
  buildModels::humanoidRandom(model);
  Data data_ref(model);

  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill(1.);

  const Eigen::DenseIndex batch_size = 128;
  const size_t num_threads = (size_t)omp_get_max_threads();

  Eigen::MatrixXd q(model.nq, batch_size);
  Eigen::MatrixXd v(model.nv, batch_size);
  Eigen::MatrixXd tau(model.nv, batch_size);
  Eigen::MatrixXd a(model.nv, batch_size);
  Eigen::MatrixXd a_ref(model.nv, batch_size);

  for (Eigen::DenseIndex i = 0; i < batch_size; ++i)
  {
    q.col(i) = randomConfiguration(model);
    v.col(i) = Eigen::VectorXd::Random(model.nv);
    tau.col(i) = Eigen::VectorXd::Random(model.nv);
  }

  ModelPool pool(model);
  abaInParallel(num_threads, pool, q, v, tau, a);

  for (Eigen::DenseIndex i = 0; i < batch_size; ++i)
  {
    a_ref.col(i) = aba(model, data_ref, q.col(i), v.col(i), tau.col(i), Convention::WORLD);
  }

  BOOST_CHECK(a == a_ref);
}

BOOST_AUTO_TEST_SUITE_END()
