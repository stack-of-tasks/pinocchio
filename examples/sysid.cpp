#include "pinocchio/parsers/urdf.hpp"

#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics-derivatives.hpp"
#include "pinocchio/algorithm/regressor.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/aba.hpp"
#include "pinocchio/algorithm/energy.hpp"

#include <iostream>

// PINOCCHIO_MODEL_DIR is defined by the CMake but you can define your own directory here.
#ifndef PINOCCHIO_MODEL_DIR
  #define PINOCCHIO_MODEL_DIR "path_to_the_model_dir"
#endif
int main(int argc, char ** argv)
{
  using namespace pinocchio;

  // You should change here to set up your own URDF file or just pass it as an argument of this
  // example.
  const std::string urdf_filename =
    (argc <= 1) ? PINOCCHIO_MODEL_DIR
                    + std::string("/example-robot-data/robots/ur_description/urdf/ur5_robot.urdf")
                : argv[1];

  // Load the URDF model
  Model model;
  pinocchio::urdf::buildModel(urdf_filename, model);

  // Build a data related to model
  Data data(model);

  // In this example we explore some of the system identification tools provided by Pinocchio.
  // We start by defining a vector of dynamical parameters of our dynamic model.
  Eigen::VectorXd dyn_parameters = Eigen::VectorXd::Zero(model.nv * 10);
  for (JointIndex jnt_idx = 1; jnt_idx < model.njoints; ++jnt_idx)
  {
    // We can set the inertial parameters of the joints
    dyn_parameters.segment<10>((jnt_idx - 1) * 10) = model.inertias[jnt_idx].toDynamicParameters();
  }

  {
    // Sample a random joint configuration as well as random joint velocity and acceleration
    Eigen::VectorXd q = randomConfiguration(model);
    Eigen::VectorXd v = Eigen::VectorXd::Random(model.nv);
    Eigen::VectorXd a = Eigen::VectorXd::Random(model.nv);

    // Some of the dynamics quantities can be parametrized linearly with respect to the dynamical
    // parameters. For instance, in RNEA algorithm, the resulting joint torques can be expressed as
    // a $Y(q, v, a) \cdot \theta = \tau$ where $Y(q, v, a)$ is a so-called joint-torque regressor.
    auto jointTorqueRegressor = computeJointTorqueRegressor(model, data, q, v, a);
    auto regressorTau = jointTorqueRegressor * dyn_parameters;
    auto rneaTau = rnea(model, data, q, v, a);
    // The two torques should be equal
    assert((regressorTau - rneaTau).isZero(1e-12));

    // However, in the real-world scenario, measuring acceleration accurately is almost impossible.
    // Instead, we can use other quantities such as energy or momentum to compute how well the
    // parameters fit the data.
    // Let's start with energy parametrization
    auto kineticEnergyRegressor = computeKineticEnergyRegressor(model, data, q, v);
    auto potentialEnergyRegressor = computePotentialEnergyRegressor(model, data, q);
    auto regressorEnergy = kineticEnergyRegressor + potentialEnergyRegressor;
    auto energy = computeKineticEnergy(model, data, q, v) + computePotentialEnergy(model, data, q);

    // The energy should be equal
    assert(std::abs((regressorEnergy * dyn_parameters - energy)) < 1e-12);
  }

  // However, the logical question is how we can compute the energy (which uses default parameters).
  // We recall that the power is time derivative of energy. Therefore, we can use
  // the torque and velocity to compute the mechanical power and integrate on horizon to get the
  // energy. Let's reset the configuration and simulate the system with the sine wave of joint
  // torques;
  {
    auto torque_fn = [&model](const double & t) -> Eigen::VectorXd {
      return Eigen::VectorXd::Ones(model.nv) * std::sin(t);
    };

    // Reset the configuration
    Eigen::VectorXd q = randomConfiguration(model);
    Eigen::VectorXd v = Eigen::VectorXd::Zero(model.nv);
    // Perform simulation for 1000 steps with dt=1e-3
    double dt = 2e-4;
    const int N = 1000;
    Eigen::MatrixXd history_q = Eigen::MatrixXd::Zero(model.nq, N);
    Eigen::MatrixXd history_v = Eigen::MatrixXd::Zero(model.nv, N);

    for (int i = 0; i < N; ++i)
    {
      auto tau = torque_fn(i * dt);
      auto a = aba(model, data, q, v, tau);
      // simple integration
      v += a * dt;
      q = integrate(model, q, v * dt);

      history_q.col(i) = q;
      history_v.col(i) = v;
    }

    // Now we can compute the difference in energy between the initial and final states
    // using the regressor and integrate the power.
    auto regEnergyFn = [&model, &data, &dyn_parameters](
                         const Eigen::VectorXd & q, const Eigen::VectorXd & v) -> double {
      auto kineticEnergyRegressor = computeKineticEnergyRegressor(model, data, q, v);
      auto potentialEnergyRegressor = computePotentialEnergyRegressor(model, data, q);
      auto regressorEnergy = kineticEnergyRegressor + potentialEnergyRegressor;
      return regressorEnergy * dyn_parameters;
    };

    // Compute the energy difference
    auto energy_diff = regEnergyFn(history_q.col(N - 1), history_v.col(N - 1))
                       - regEnergyFn(history_q.col(0), history_v.col(0));

    // Compute the power integral
    double power_integral = 0;
    for (int i = 0; i < N; ++i)
    {
      power_integral += torque_fn(i * dt).dot(history_v.col(i)) * dt;
    }

    // The energy difference should be close to the power integral
    assert(
      std::abs(energy_diff - power_integral)
      < 1e-2); // the tolerance is high due numerical integration
  }

  // Another concept we can approach is the momentum.
  // Momentum can be defined as $H = M(q) \cdot v$ where $M(q)$ is the mass inertia matrix or $H =
  // Y_H(q, v) \pi$ in regressor form. On the other hand, one can show that $\dot_H = \tau + C(q,
  // v)^T v - g(q)$ where $C(q, v)$ is the Coriolis matrix and $g(q)$ is the gravity vector.
  // Fortunately, C(q, v)^T v can be also expressed in regressor form.
  {
    Eigen::VectorXd q = randomConfiguration(model);
    Eigen::VectorXd v = Eigen::VectorXd::Random(model.nv);
    auto tau = Eigen::VectorXd::Random(model.nv);
    const double dt = 1e-3;

    auto regressors1 = computeIndirectRegressors(model, data, q, v);
    // compute the momentum using regressor form
    Eigen::VectorXd H1 = regressors1.first * dyn_parameters;

    // integrate forward
    auto v_next = v + aba(model, data, q, v, tau) * dt;
    auto q_next = integrate(model, q, v_next * dt);
    auto regressors2 = computeIndirectRegressors(model, data, q_next, v_next);
    // compute the momentum using regressor form
    Eigen::VectorXd H2 = regressors2.first * dyn_parameters;

    // compute the numerical momentum difference
    Eigen::VectorXd numericalMomentumDiff = (H2 - H1);

    // Compare the C^T v term
    Eigen::VectorXd CTv_regressor = regressors1.second * dyn_parameters;
    Eigen::VectorXd CTv = computeCoriolisMatrix(model, data, q, v).transpose() * v;
    assert((CTv_regressor - CTv).isZero(1e-12));

    // Compare the gravity term
    Eigen::VectorXd g_regressor = computePotentialEnergyRegressor(model, data, q);
    Eigen::VectorXd g = computeGeneralizedGravity(model, data, q);
    assert((g_regressor - g).isZero(1e-12));

    // find analytical momentum derivative
    Eigen::VectorXd analyticalMomentumDot = CTv + tau - g;

    // Verify that the numerical momentum difference is close to the analytical momentum derivative
    assert((numericalMomentumDiff - analyticalMomentumDot * dt).isZero(1e-5));
  }
}
