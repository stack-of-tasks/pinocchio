#include "pinocchio/multibody/sample-models.hpp"

#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/aba.hpp"

#include <iostream>

int main(int /*argc*/, char ** /*argv*/)
{
  using namespace pinocchio;

  pinocchio::Model model;
  pinocchio::buildModels::humanoidRandom(model);

  // Build data related to model
  Data data(model);

  // Sample a random joint configuration as well as random joint velocity and torque
  Eigen::VectorXd q = randomConfiguration(model);
  Eigen::VectorXd v = Eigen::VectorXd::Random(model.nv);
  Eigen::VectorXd tau = Eigen::VectorXd::Random(model.nv);

  // Computes the forward dynamics (ABA)
  aba(model, data, q, v, tau, Convention::WORLD);

  // Get access to the joint acceleration
  std::cout << "Joint acceleration: " << data.ddq << std::endl;
  return 0;
}
