//
// Copyright (c) 2021-2022 INRIA
//

#define BOOST_TEST_MODULE parallel_rnea

#include "pinocchio/spatial.hpp"
#include "pinocchio/algorithm/parallel/rnea.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/multibody/sample-models.hpp"

#include <boost/utility/binary.hpp>

#include <boost/test/unit_test.hpp>

using namespace pinocchio;

BOOST_AUTO_TEST_CASE(test_parallel_rnea)
{
  pinocchio::Model model;
  buildModels::humanoidRandom(model);
  Data data_ref(model);

  model.lowerPositionLimit.head<3>().fill(-1.);
  model.upperPositionLimit.head<3>().fill(1.);

  const Eigen::Index batch_size = 128;
  const size_t num_thread = (size_t)omp_get_max_threads();

  Eigen::MatrixXd q(model.nq, batch_size);
  Eigen::MatrixXd v(model.nv, batch_size);
  Eigen::MatrixXd a(model.nv, batch_size);
  Eigen::MatrixXd tau(model.nv, batch_size);
  Eigen::MatrixXd tau_ref(model.nv, batch_size);

  for (Eigen::Index i = 0; i < batch_size; ++i)
  {
    q.col(i) = randomConfiguration(model);
    v.col(i) = Eigen::VectorXd::Random(model.nv);
    a.col(i) = Eigen::VectorXd::Random(model.nv);
  }

  ModelPool pool(model, num_thread);
  rneaInParallel(num_thread, pool, q, v, a, tau);

  for (Eigen::Index i = 0; i < batch_size; ++i)
  {
    tau_ref.col(i) = rnea(model, data_ref, q.col(i), v.col(i), a.col(i));
  }

  BOOST_CHECK(tau == tau_ref);
}
