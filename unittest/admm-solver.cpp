//
// Copyright (c) 2024 INRIA
//

#define BOOST_TEST_MODULE admm_solver

#include "pinocchio/algorithm/constraint-cholesky.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/constraints.hpp"
#include "pinocchio/algorithm/solvers/admm-solver.hpp"
#include "pinocchio/algorithm/aba.hpp"
#include "pinocchio/algorithm/crba.hpp"

#include <boost/utility/binary.hpp>

#include <boost/test/unit_test.hpp>

using namespace pinocchio;

template<typename _ConstraintModel>
struct TestBoxTpl
{
  typedef _ConstraintModel ConstraintModel;

  typedef typename ConstraintModel::ConstraintData ConstraintData;

  TestBoxTpl(const Model & model, const std::vector<ConstraintModel> & constraint_models)
  : model(model)
  , data(model)
  , constraint_models(constraint_models)
  , v_next(Eigen::VectorXd::Zero(model.nv))
  {
    for (const auto & cm : constraint_models)
    {
      constraint_datas.push_back(cm.createData());
    }

    const Eigen::Index constraint_size = getTotalConstraintResidualSize(constraint_models);
    impulse_solution = velocity_solution = Eigen::VectorXd::Zero(constraint_size);
  }

  void operator()(
    const Eigen::VectorXd & q0,
    const Eigen::VectorXd & v0,
    const Eigen::VectorXd & tau0,
    const Force & fext,
    const double dt,
    const bool test_warmstart = false)
  {
    std::vector<Force> external_forces(size_t(model.njoints), Force::Zero());
    external_forces[1] = fext;

    const Eigen::VectorXd v_free =
      v0 + dt * aba(model, data, q0, v0, tau0, external_forces, Convention::WORLD);
    data.q_in = q0;
    data.v_in = v0;
    data.tau_in = tau0;
    calc(model, data, constraint_models, constraint_datas);

    // cholesky of the Delassus matrix
    crba(model, data, q0, Convention::WORLD);
    ConstraintCholeskyDecomposition chol(model, data, constraint_models, constraint_datas);
    chol.rebuild(model, data, constraint_models, constraint_datas);
    chol.compute(model, data, constraint_models, constraint_datas, 1e-10);

    const Eigen::MatrixXd delassus_matrix_plain =
      chol.getDelassusOperatorCholeskyExpression().matrix();
    auto G_expression = chol.getDelassusOperatorCholeskyExpression();

    // construct constraint drift g
    Eigen::MatrixXd constraint_jacobian(delassus_matrix_plain.rows(), model.nv);
    constraint_jacobian.setZero();
    getConstraintsJacobian(model, data, constraint_models, constraint_datas, constraint_jacobian);
    const Eigen::VectorXd g = constraint_jacobian * v_free;

    // optional compliance
    Eigen::VectorXd compliance = Eigen::VectorXd::Zero(g.size());
    G_expression.updateCompliance(compliance);

    // Configure the member ADMM solver
    ADMMConstraintSolver admm_solver;
    BOOST_CHECK(admm_solver.isValid() == false);
    ADMMSolverSettings admm_settings; // default settings
    admm_settings.max_iterations = 10000;
    admm_settings.absolute_feasibility_tol = 1e-10;
    admm_settings.relative_feasibility_tol = 1e-12;
    admm_settings.absolute_complementarity_tol = 1e-10;
    admm_settings.relative_complementarity_tol = 1e-12;
    admm_settings.lanczos_size = static_cast<std::size_t>(g.size());
    admm_settings.rho_init = std::nullopt;
    admm_settings.admm_update_rule =
      ADMMUpdateRule::SPECTRAL; // test if eigenvalue computation mechanism works well
    admm_settings.admm_proximal_rule = ADMMProximalRule::MANUAL;
    admm_settings.mu_prox = 1e-6;
    admm_settings.stat_record = true;
    admm_settings.solve_ncp = true;
    ADMMSolverResult admm_result;
    admm_result.setConstraintImpulseGuess(impulse_solution);
    admm_result.clearConstraintVelocityGuess();

    has_converged = admm_solver.solve(
      G_expression, g, constraint_models, constraint_datas, admm_settings, admm_result);
    BOOST_CHECK(admm_result.problem_size == static_cast<std::size_t>(G_expression.rows()));
    BOOST_CHECK(admm_solver.isValid() == true);
    BOOST_CHECK(admm_result.isValid() == true);
    admm_result.retrieveConstraintImpulses(impulse_solution);

    if (test_warmstart)
    {
      admm_result.setConstraintImpulseGuess(impulse_solution);
      admm_settings.warmstart_rho_with_previous_result = true;
      has_converged =
        has_converged
        && admm_solver.solve(
          G_expression, g, constraint_models, constraint_datas, admm_settings, admm_result);
      admm_result.retrieveConstraintImpulses(impulse_solution);
    }

    admm_result.retrieveConstraintVelocities(velocity_solution);
    n_iter = admm_result.iterations;
    const Eigen::VectorXd tau_ext = constraint_jacobian.transpose() * impulse_solution / dt;

    v_next =
      v0
      + dt * aba(model, data, q0, v0, (tau0 + tau_ext).eval(), external_forces, Convention::WORLD);
  }

  Model model;
  Data data;
  std::vector<ConstraintModel> constraint_models;
  std::vector<ConstraintData> constraint_datas;
  Eigen::VectorXd v_next;

  Eigen::VectorXd impulse_solution, velocity_solution;
  bool has_converged;
  std::size_t n_iter;
};

BOOST_AUTO_TEST_CASE(ball)
{
  Model model;
  model.addJoint(0, JointModelFreeFlyer(), SE3::Identity(), "free_flyer");

  const double ball_dim = 1;
  const double ball_mass = 10;
  const Inertia ball_inertia = Inertia::FromSphere(ball_mass, ball_dim);

  model.appendBodyToJoint(1, ball_inertia);

  BOOST_CHECK(model.check(model.createData()));

  Eigen::VectorXd q0 = neutral(model);
  q0.const_cast_derived()[2] += ball_dim / 2;
  const Eigen::VectorXd v0 = Eigen::VectorXd::Zero(model.nv);
  const Eigen::VectorXd tau0 = Eigen::VectorXd::Zero(model.nv);

  const double dt = 1e-3;

  typedef PointContactConstraintModel ConstraintModel;
  typedef TestBoxTpl<ConstraintModel> TestBox;
  std::vector<ConstraintModel> constraint_models;

  const double friction_value = 0.4;
  {
    const SE3 local_placement_ball(SE3::Matrix3::Identity(), SE3::Vector3(0, 0, -ball_dim));
    ConstraintModel cm(model, 0, SE3::Identity(), 1, local_placement_ball);
    cm.setFriction(friction_value);
    constraint_models.push_back(cm);
  }

  // Test static motion with zero external force
  {
    const Force fext = Force::Zero();

    TestBox test(model, constraint_models);
    test(q0, v0, tau0, fext, dt, false);

    BOOST_CHECK(test.has_converged == true);
    BOOST_CHECK(test.velocity_solution.isZero(2e-10));
    const Force::Vector3 f_tot_ref = -ball_mass * Model::gravity981 - fext.linear();
    Force::Vector3 f_tot = test.impulse_solution.head(3) / dt;
    BOOST_CHECK(f_tot.isApprox(f_tot_ref, 1e-8));
    BOOST_CHECK(test.v_next.isZero(2e-10));

    // Test warmstart
    test(q0, v0, tau0, fext, dt, true);
    BOOST_CHECK(test.has_converged == true);
    BOOST_CHECK(test.velocity_solution.isZero(2e-10));
    f_tot = test.impulse_solution.head(3) / dt;
    BOOST_CHECK(f_tot.isApprox(f_tot_ref, 1e-8));
    BOOST_CHECK(test.v_next.isZero(2e-10));
    BOOST_CHECK(test.n_iter == 0);
  }
}

void buildStackOfCubesModel(
  std::vector<double> masses,
  ::pinocchio::Model & model,
  std::vector<PointContactConstraintModel> & constraint_models)
{
  const SE3::Vector3 box_dims = SE3::Vector3::Ones();
  const int n_cubes = (int)masses.size();

  for (int i = 0; i < n_cubes; i++)
  {
    const double box_mass = masses[(std::size_t)i];
    const Inertia box_inertia = Inertia::FromBox(box_mass, box_dims[0], box_dims[1], box_dims[2]);
    JointIndex joint_id =
      model.addJoint(0, JointModelFreeFlyer(), SE3::Identity(), "free_flyer_" + std::to_string(i));
    model.appendBodyToJoint(joint_id, box_inertia);
  }

  const double friction_value = 0.4;
  for (int i = 0; i < n_cubes; i++)
  {
    const SE3 local_placement_box_1(
      SE3::Matrix3::Identity(), 0.5 * SE3::Vector3(box_dims[0], box_dims[1], box_dims[2]));
    const SE3 local_placement_box_2(
      SE3::Matrix3::Identity(), 0.5 * SE3::Vector3(box_dims[0], box_dims[1], -box_dims[2]));
    SE3::Matrix3 rot = SE3::Matrix3::Identity();
    for (int j = 0; j < 4; ++j)
    {
      const SE3 local_placement_1(
        SE3::Matrix3::Identity(), rot * local_placement_box_1.translation());
      const SE3 local_placement_2(
        SE3::Matrix3::Identity(), rot * local_placement_box_2.translation());
      PointContactConstraintModel cm(
        model, (JointIndex)i, local_placement_1, (JointIndex)i + 1, local_placement_2);
      cm.setFriction(friction_value);
      constraint_models.push_back(cm);
      rot = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()).toRotationMatrix() * rot;
    }
  }
}

Eigen::Vector3d computeFtotOfFirstBoxInStackOfBoxes(const Eigen::VectorXd & contact_forces)
{
  // we make the assumption that each box has contact constraints at each corner
  PINOCCHIO_THROW_IF(
    contact_forces.size() < 3 * 4, std::logic_error, "Invalid number of contact forces");

  Eigen::Vector3d f_tot = Eigen::Vector3d::Zero();
  for (int k = 0; k < 4; ++k)
  {
    f_tot += contact_forces.segment(3 * k, 3);
  }
  return f_tot;
}

BOOST_AUTO_TEST_CASE(box)
{
  Model model;
  typedef PointContactConstraintModel ConstraintModel;
  typedef TestBoxTpl<ConstraintModel> TestBox;
  std::vector<ConstraintModel> constraint_models;
  const double box_mass = 1e1;
  const std::vector<double> masses = {box_mass};

  const SE3::Vector3 box_dims = SE3::Vector3::Ones();
  buildStackOfCubesModel(masses, model, constraint_models);

  const int num_tests =
#ifdef NDEBUG
    1000
#else
    10
#endif
    ;

  BOOST_CHECK(model.check(model.createData()));

  Eigen::VectorXd q0 = neutral(model);
  q0[2] += box_dims[2] / 2;
  const Eigen::VectorXd v0 = Eigen::VectorXd::Zero(model.nv);
  const Eigen::VectorXd tau0 = Eigen::VectorXd::Zero(model.nv);

  const double dt = 1e-3;

  // Test static motion with zero external force
  {
    const Force fext = Force::Zero();

    TestBox test(model, constraint_models);
    test(q0, v0, tau0, fext, dt, false);

    BOOST_CHECK(test.has_converged == true);
    BOOST_CHECK(test.velocity_solution.isZero(2e-10));
    const Force::Vector3 f_tot_ref = -box_mass * Model::gravity981 - fext.linear();
    const Force::Vector3 f_tot = computeFtotOfFirstBoxInStackOfBoxes(test.impulse_solution / dt);
    BOOST_CHECK(f_tot.isApprox(f_tot_ref, 1e-8));
    BOOST_CHECK(test.v_next.isZero(2e-10));
  }

  const double friction_value = 0.4;
  const double f_sliding = friction_value * Model::gravity981.norm() * box_mass;

  // Test static motion with small external force
  for (int k = 0; k < num_tests; ++k)
  {
    const double scaling = 0.9;
    Force fext = Force::Zero();
    fext.linear().head<2>().setRandom().normalize();
    fext.linear() *= scaling * f_sliding;

    TestBox test(model, constraint_models);
    test(q0, v0, tau0, fext, dt, false);

    BOOST_CHECK(test.has_converged == true);
    BOOST_CHECK(test.velocity_solution.isZero(1e-8));
    const Force::Vector3 f_tot_ref = -box_mass * Model::gravity981 - fext.linear();
    const Force::Vector3 f_tot = computeFtotOfFirstBoxInStackOfBoxes(test.impulse_solution / dt);
    BOOST_CHECK(f_tot.isApprox(f_tot_ref, 1e-6));
    BOOST_CHECK(test.v_next.isZero(1e-8));
  }

  // Test slidding motion
  for (int k = 0; k < num_tests; ++k)
  {
    const double scaling = 1.1;
    Force fext = Force::Zero();
    fext.linear().head<2>().setRandom().normalize();
    fext.linear() *= scaling * f_sliding;

    TestBox test(model, constraint_models);
    test(q0, v0, tau0, fext, dt, false);

    BOOST_CHECK(test.has_converged == true);
    const Force::Vector3 f_tot_ref = -box_mass * Model::gravity981 - 1 / scaling * fext.linear();
    const Force::Vector3 f_tot = computeFtotOfFirstBoxInStackOfBoxes(test.impulse_solution / dt);
    BOOST_CHECK(f_tot.isApprox(f_tot_ref, 1e-6));
    BOOST_CHECK(
      math::fabs(Motion(test.v_next).linear().norm() - (f_sliding * 0.1 / box_mass * dt)) <= 1e-6);
    BOOST_CHECK(Motion(test.v_next).angular().isZero(1e-6));
  }
}

BOOST_AUTO_TEST_CASE(stack_of_boxes)
{
  const int n_cubes = 10;
  const double conditionning = 1e6;
  const double mass_factor = std::pow(conditionning, 1. / (n_cubes - 1));
  std::vector<double> masses;
  double mass_tot = 0;
  for (int i = 0; i < n_cubes; i++)
  {
    const double box_mass = 1e-3 * std::pow(mass_factor, i);
    masses.push_back(box_mass);
    mass_tot += box_mass;
  }

  Model model;
  typedef PointContactConstraintModel ConstraintModel;
  typedef TestBoxTpl<ConstraintModel> TestBox;
  std::vector<ConstraintModel> constraint_models;

  buildStackOfCubesModel(masses, model, constraint_models);
  BOOST_CHECK(model.check(model.createData()));

  const SE3::Vector3 box_dims = SE3::Vector3::Ones();

  Eigen::VectorXd q0 = neutral(model);
  for (int i = 0; i < n_cubes; i++)
  {
    q0[7 * i + 2] = i * box_dims[2];
    q0[7 * i + 2] += box_dims[2] / 2;
  }
  const Eigen::VectorXd v0 = Eigen::VectorXd::Zero(model.nv);
  const Eigen::VectorXd tau0 = Eigen::VectorXd::Zero(model.nv);

  const double dt = 1e-3;

  // Test static motion with zero external force
  {
    const Force fext = Force::Zero();

    TestBox test(model, constraint_models);
    test(q0, v0, tau0, fext, dt, false);

    BOOST_CHECK(test.has_converged == true);
    BOOST_CHECK(test.velocity_solution.isZero(2e-10));
    // We check the total force applied on the bottom box of the stack
    const Force::Vector3 f_tot_ref = -mass_tot * Model::gravity981;
    const Force::Vector3 f_tot = computeFtotOfFirstBoxInStackOfBoxes(test.impulse_solution / dt);
    BOOST_CHECK(f_tot.isApprox(f_tot_ref, 1e-6));
    BOOST_CHECK(test.v_next.isZero(1e-8));
  }
}

BOOST_AUTO_TEST_CASE(point_anchor_box)
{
  Model model;
  model.addJoint(0, JointModelFreeFlyer(), SE3::Identity(), "free_flyer");

  const int num_tests =
#ifdef NDEBUG
    100000
#else
    100
#endif
    ;

  const SE3::Vector3 box_dims = SE3::Vector3::Ones();
  const double box_mass = 10;
  const Inertia box_inertia = Inertia::FromBox(box_mass, box_dims[0], box_dims[1], box_dims[2]);

  model.appendBodyToJoint(1, box_inertia);

  BOOST_CHECK(model.check(model.createData()));

  Eigen::VectorXd q0 = neutral(model);
  q0.const_cast_derived()[2] += box_dims[2] / 2;
  const Eigen::VectorXd v0 = Eigen::VectorXd::Zero(model.nv);
  const Eigen::VectorXd tau0 = Eigen::VectorXd::Zero(model.nv);

  const double dt = 1e-3;

  typedef PointAnchorConstraintModel ConstraintModel;
  typedef TestBoxTpl<ConstraintModel> TestBox;
  std::vector<ConstraintModel> constraint_models;

  {
    const SE3 local_placement_box(
      SE3::Matrix3::Identity(), 0.5 * SE3::Vector3(box_dims[0], box_dims[1], -box_dims[2]));
    SE3::Matrix3 rot = SE3::Matrix3::Identity();
    for (int i = 0; i < 4; ++i)
    {
      const SE3 local_placement(SE3::Matrix3::Identity(), rot * local_placement_box.translation());
      ConstraintModel cm(model, 0, SE3::Identity(), 1, local_placement);
      constraint_models.push_back(cm);
      rot = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()).toRotationMatrix() * rot;
    }
  }

  // Test static motion with zero external force
  {
    const Force fext = Force::Zero();

    TestBox test(model, constraint_models);
    test(q0, v0, tau0, fext, dt);

    BOOST_CHECK(test.has_converged == true);
    BOOST_CHECK(test.velocity_solution.isZero(2e-10));
    const Force::Vector3 f_tot_ref = -box_mass * Model::gravity981;
    Force::Vector3 f_tot = Force::Vector3::Zero();
    for (int k = 0; k < 4; ++k)
    {
      f_tot += test.impulse_solution.segment(3 * k, 3);
    }
    f_tot /= dt;
    BOOST_CHECK(f_tot.isApprox(f_tot_ref, 1e-6));
    BOOST_CHECK(test.v_next.isZero(2e-10));
  }

  for (int k = 0; k < num_tests; ++k)
  {
    Force fext = Force::Zero();
    fext.linear().setRandom();

    TestBox test(model, constraint_models);
    test(q0, v0, tau0, fext, dt);

    BOOST_CHECK(test.has_converged == true);
    BOOST_CHECK(test.velocity_solution.isZero(1e-8));
    const Force::Vector3 f_tot_ref = -box_mass * Model::gravity981 - fext.linear();
    Force::Vector3 f_tot = Force::Vector3::Zero();
    for (int k = 0; k < 4; ++k)
    {
      f_tot += test.impulse_solution.segment(3 * k, 3);
    }
    f_tot /= dt;
    BOOST_CHECK(f_tot.isApprox(f_tot_ref, 1e-6));
    BOOST_CHECK(test.v_next.isZero(1e-8));
  }
}

BOOST_AUTO_TEST_CASE(dry_friction_box)
{
  Model model;
  model.addJoint(0, JointModelFreeFlyer(), SE3::Identity(), "free_flyer");

  const SE3::Vector3 box_dims = SE3::Vector3::Ones();
  const double box_mass = 10;
  const Inertia box_inertia = Inertia::FromBox(box_mass, box_dims[0], box_dims[1], box_dims[2]);

  model.appendBodyToJoint(1, box_inertia);
  model.gravity.setZero();
  Data data(model);

  Eigen::VectorXd q0 = neutral(model);
  q0.const_cast_derived()[2] += box_dims[2] / 2;
  const Eigen::VectorXd v0 = Eigen::VectorXd::Zero(model.nv);
  const Eigen::VectorXd tau0 = Eigen::VectorXd::Zero(model.nv);

  const double dt = 1e-3;

  typedef JointFrictionConstraintModel ConstraintModel;
  typedef ConstraintModel::ConstraintData ConstraintData;
  std::vector<ConstraintModel> constraint_models;
  std::vector<ConstraintData> constraint_datas;

  ConstraintModel dry_friction_free_flyer(model, ConstraintModel::JointIndexVector(1, 1));
  constraint_models.push_back(dry_friction_free_flyer);

  for (const auto & cm : constraint_models)
    constraint_datas.push_back(cm.createData());

  constraint_models[0].setFrictionLowerLimit(Eigen::VectorXd::Constant(6, -1.));
  constraint_models[0].setFrictionUpperLimit(Eigen::VectorXd::Constant(6, +1.));
  const auto box_set = constraint_models[0].set(constraint_datas[0]);

  const Eigen::VectorXd v_free = v0 + dt * aba(model, data, q0, v0, tau0, Convention::WORLD);

  // Cholesky of the Delassus matrix
  crba(model, data, q0, Convention::WORLD);
  data.q_in = q0;
  calc(model, data, constraint_models, constraint_datas);
  ConstraintCholeskyDecomposition chol(model, data, constraint_models, constraint_datas);
  chol.rebuild(model, data, constraint_models, constraint_datas);
  chol.compute(model, data, constraint_models, constraint_datas, 1e-10);

  auto G_expression = chol.getDelassusOperatorCholeskyExpression();
  const Eigen::MatrixXd delassus_matrix_plain =
    chol.getDelassusOperatorCholeskyExpression().matrix();
  const auto & G = delassus_matrix_plain;
  //    std::cout << "G:\n" << delassus_matrix_plain << std::endl;

  Eigen::MatrixXd constraint_jacobian(dry_friction_free_flyer.residualSize(), model.nv);
  constraint_jacobian.setZero();
  getConstraintsJacobian(model, data, constraint_models, constraint_datas, constraint_jacobian);

  const Eigen::VectorXd g = constraint_jacobian * v_free;

  Eigen::VectorXd compliance = Eigen::VectorXd::Zero(g.size());
  G_expression.updateCompliance(compliance);
  Eigen::VectorXd velocity_solution(Eigen::VectorXd::Zero(g.size()));
  Eigen::VectorXd impulse_solution(Eigen::VectorXd::Zero(g.size()));

  ADMMConstraintSolver admm_solver(std::size_t(delassus_matrix_plain.rows()));
  ADMMSolverSettings admm_settings;
  admm_settings.absolute_feasibility_tol = 1e-13;
  admm_settings.relative_feasibility_tol = 1e-14;
  admm_settings.absolute_complementarity_tol = 1e-13;
  admm_settings.relative_complementarity_tol = 1e-14;
  ADMMSolverResult admm_result;
  admm_result.setConstraintImpulseGuess(impulse_solution);

  const bool has_converged = admm_solver.solve(
    G_expression, g, constraint_models, constraint_datas, admm_settings, admm_result);
  admm_result.retrieveConstraintImpulses(impulse_solution);
  BOOST_CHECK(has_converged);

  velocity_solution = G * impulse_solution + g;

  BOOST_CHECK(std::fabs(impulse_solution.dot(velocity_solution)) <= 1e-8);
  BOOST_CHECK(impulse_solution.isZero());

  typedef TestBoxTpl<ConstraintModel> TestBox;

  // Test static motion with zero external force
  {
    TestBox test(model, constraint_models);
    test(q0, v0, tau0, Force::Zero(), dt);

    BOOST_CHECK(test.has_converged == true);
    BOOST_CHECK(test.velocity_solution.isZero(2e-10));
    BOOST_CHECK(test.v_next.isZero(2e-10));
    BOOST_CHECK(box_set.isInside(test.impulse_solution));
  }

  for (int i = 0; i < 6; ++i)
  {
    TestBox test(model, constraint_models);
    test(q0, v0, tau0 + 2 * Force::Vector6::Unit(i) / dt, Force::Zero(), dt);

    //    std::cout << "test.velocity_solution: " << test.velocity_solution.transpose() <<
    //    std::endl;
    BOOST_CHECK(test.has_converged == true);
    BOOST_CHECK(!test.impulse_solution.isZero(2e-10));
    BOOST_CHECK(!test.v_next.isZero(2e-10));
    BOOST_CHECK(box_set.isInside(test.impulse_solution));
    BOOST_CHECK(std::fabs(test.impulse_solution[i] - box_set.lb[i]) < 1e-8);
  }

  // Sign reversed
  for (int i = 0; i < 6; ++i)
  {
    TestBox test(model, constraint_models);
    test(q0, v0, tau0 - 2 * Force::Vector6::Unit(i) / dt, Force::Zero(), dt);

    BOOST_CHECK(test.has_converged == true);
    BOOST_CHECK(!test.velocity_solution.isZero(2e-10));
    BOOST_CHECK(!test.v_next.isZero(2e-10));
    BOOST_CHECK(box_set.isInside(test.impulse_solution));
    BOOST_CHECK(std::fabs(test.impulse_solution[i] - box_set.ub[i]) < 1e-8);
  }
}

BOOST_AUTO_TEST_CASE(joint_limit_slider)
{
  Model model;
  model.addJoint(0, JointModelPX(), SE3::Identity(), "slider");
  model.lowerPositionLimit[0] = 0.;

  const SE3::Vector3 box_dims = SE3::Vector3::Ones();
  const double box_mass = 10;
  const Inertia box_inertia = Inertia::FromBox(box_mass, box_dims[0], box_dims[1], box_dims[2]);

  model.appendBodyToJoint(1, box_inertia);
  model.gravity.setZero();
  Data data(model);

  Eigen::VectorXd q0 = Eigen::VectorXd::Zero(model.nq);
  const Eigen::VectorXd v0 = Eigen::VectorXd::Zero(model.nv);
  const Eigen::VectorXd tau_push_against_lower_bound = -Eigen::VectorXd::Ones(model.nv);

  const double dt = 1e-3;

  typedef JointLimitConstraintModel ConstraintModel;
  typedef ConstraintModel::ConstraintData ConstraintData;
  std::vector<ConstraintModel> constraint_models;
  std::vector<ConstraintData> constraint_datas;

  ConstraintModel joint_limit_constraint_model(model, ConstraintModel::JointIndexVector(1, 1));
  joint_limit_constraint_model.makeSelectionFilteredByLimitProximity(q0);
  constraint_models.push_back(joint_limit_constraint_model);

  for (const auto & cm : constraint_models)
    constraint_datas.push_back(cm.createData());

  const Eigen::VectorXd v_free_against_lower_bound =
    v0 + dt * aba(model, data, q0, v0, tau_push_against_lower_bound, Convention::WORLD);
  const Eigen::VectorXd v_free_move_away =
    v0 + dt * aba(model, data, q0, v0, -tau_push_against_lower_bound, Convention::WORLD);

  // Cholesky of the Delassus matrix
  crba(model, data, q0, Convention::WORLD);
  data.q_in = q0;
  auto & cmodel = constraint_models[0];
  auto & cdata = constraint_datas[0];
  cmodel.calc(model, data, cdata);
  ConstraintCholeskyDecomposition chol(model, data, constraint_models, constraint_datas);
  chol.rebuild(model, data, constraint_models, constraint_datas);
  chol.compute(model, data, constraint_models, constraint_datas, 1e-10);

  auto G_expression = chol.getDelassusOperatorCholeskyExpression();
  const auto G_plain = G_expression.matrix();
  const Eigen::MatrixXd delassus_matrix_plain = G_expression.matrix();

  Eigen::MatrixXd constraint_jacobian(cmodel.residualSize(), model.nv);
  constraint_jacobian.setZero();
  getConstraintsJacobian(model, data, constraint_models, constraint_datas, constraint_jacobian);

  // External torques push the slider against the lower bound
  {
    const Eigen::VectorXd g_against_lower_bound = constraint_jacobian * v_free_against_lower_bound;
    const Eigen::VectorXd g_tilde_against_lower_bound =
      g_against_lower_bound + cdata.constraint_residual / dt;

    Eigen::VectorXd compliance = Eigen::VectorXd::Zero(g_tilde_against_lower_bound.size());
    G_expression.updateCompliance(compliance);

    Eigen::VectorXd velocity_solution = Eigen::VectorXd::Zero(cmodel.residualSize());
    Eigen::VectorXd impulse_solution = Eigen::VectorXd::Zero(cmodel.residualSize());

    ADMMConstraintSolver admm_solver(std::size_t(delassus_matrix_plain.rows()));
    ADMMSolverSettings admm_settings;
    admm_settings.absolute_feasibility_tol = 1e-13;
    admm_settings.relative_feasibility_tol = 1e-14;
    admm_settings.absolute_complementarity_tol = 1e-13;
    admm_settings.relative_complementarity_tol = 1e-14;
    ADMMSolverResult admm_result;
    admm_result.setConstraintImpulseGuess(impulse_solution);

    const bool has_converged = admm_solver.solve(
      G_expression, g_tilde_against_lower_bound, constraint_models, constraint_datas, admm_settings,
      admm_result);
    admm_result.retrieveConstraintImpulses(impulse_solution);
    BOOST_CHECK(has_converged);

    velocity_solution = G_plain * impulse_solution + g_against_lower_bound;
    Eigen::VectorXd velocity_solution2;
    admm_result.retrieveConstraintVelocities(velocity_solution2);

    BOOST_CHECK(std::fabs(impulse_solution.dot(velocity_solution)) <= 1e-8);
    BOOST_CHECK(velocity_solution.isZero(1e-6));
    BOOST_CHECK(velocity_solution2.isZero(1e-6));

    BOOST_CHECK(
      (tau_push_against_lower_bound + constraint_jacobian.transpose() * impulse_solution / dt)
        .isZero(1e-6));
  }

  // External torques push the slider away from the lower bound
  {
    const Eigen::VectorXd g_move_away = constraint_jacobian * v_free_move_away;
    const Eigen::VectorXd g_tilde_move_away = g_move_away + cdata.constraint_residual / dt;

    Eigen::VectorXd velocity_solution = Eigen::VectorXd::Zero(cmodel.residualSize());
    Eigen::VectorXd impulse_solution = Eigen::VectorXd::Zero(cmodel.residualSize());

    Eigen::VectorXd compliance = Eigen::VectorXd::Zero(g_tilde_move_away.size());
    G_expression.updateCompliance(compliance);

    ADMMConstraintSolver admm_solver(std::size_t(delassus_matrix_plain.rows()));
    ADMMSolverSettings admm_settings;
    admm_settings.absolute_feasibility_tol = 1e-13;
    admm_settings.relative_feasibility_tol = 1e-14;
    admm_settings.absolute_complementarity_tol = 1e-13;
    admm_settings.relative_complementarity_tol = 1e-14;
    ADMMSolverResult admm_result;
    admm_result.setConstraintImpulseGuess(impulse_solution);

    const bool has_converged = admm_solver.solve(
      G_expression, g_tilde_move_away, constraint_models, constraint_datas, admm_settings,
      admm_result);
    admm_result.retrieveConstraintImpulses(impulse_solution);
    BOOST_CHECK(has_converged);

    velocity_solution = G_plain * impulse_solution + g_move_away;
    Eigen::VectorXd velocity_solution2;
    admm_result.retrieveConstraintVelocities(velocity_solution2);

    BOOST_CHECK(std::fabs(impulse_solution.dot(velocity_solution)) <= 1e-8);
    BOOST_CHECK(impulse_solution.isZero());
    BOOST_CHECK(velocity_solution.isApprox(g_move_away));
  }
}

BOOST_AUTO_TEST_CASE(joint_limit_revolute_xyz)
{
  Model model;
  JointIndex joint_id_x = model.addJoint(0, JointModelRX(), SE3::Identity(), "revolute_x");
  JointIndex joint_id_y = model.addJoint(joint_id_x, JointModelRY(), SE3::Identity(), "revolute_y");
  JointIndex joint_id_z = model.addJoint(joint_id_y, JointModelRZ(), SE3::Identity(), "revolute_z");

  const SE3::Vector3 small_box_dims = SE3::Vector3::Ones() * 1e-3;
  const double small_box_mass = 1e-6;
  const Inertia small_box_inertia =
    Inertia::FromBox(small_box_mass, small_box_dims[0], small_box_dims[1], small_box_dims[2]);

  const SE3::Vector3 box_dims = SE3::Vector3::Ones();
  const double box_mass = 10;
  const Inertia box_inertia = Inertia::FromBox(box_mass, box_dims[0], box_dims[1], box_dims[2]);

  model.appendBodyToJoint(joint_id_x, small_box_inertia);
  model.appendBodyToJoint(joint_id_y, small_box_inertia);
  model.appendBodyToJoint(joint_id_z, box_inertia);
  model.gravity.setZero();
  model.lowerPositionLimit[0] = 0.;
  model.lowerPositionLimit[1] = 0.;
  model.lowerPositionLimit[2] = 0.;
  Data data(model);

  Eigen::VectorXd q0 = Eigen::VectorXd::Zero(model.nq);
  const Eigen::VectorXd v0 = Eigen::VectorXd::Zero(model.nv);
  const Eigen::VectorXd tau_push_against_lower_bound = -Eigen::VectorXd::Ones(model.nv);

  const double dt = 1e-3;

  typedef JointLimitConstraintModel ConstraintModel;
  typedef ConstraintModel::ConstraintData ConstraintData;
  std::vector<ConstraintModel> constraint_models;
  std::vector<ConstraintData> constraint_datas;
  ConstraintModel::JointIndexVector active_joints = {joint_id_x, joint_id_y, joint_id_z};

  ConstraintModel joint_limit_constraint_model(model, active_joints);
  joint_limit_constraint_model.makeSelectionFilteredByLimitProximity(q0);
  constraint_models.push_back(joint_limit_constraint_model);

  for (const auto & cm : constraint_models)
    constraint_datas.push_back(cm.createData());

  const Eigen::VectorXd v_free_against_lower_bound =
    v0 + dt * aba(model, data, q0, v0, tau_push_against_lower_bound, Convention::WORLD);
  const Eigen::VectorXd v_free_move_away =
    v0 + dt * aba(model, data, q0, v0, -tau_push_against_lower_bound, Convention::WORLD);

  // Cholesky of the Delassus matrix
  crba(model, data, q0, Convention::WORLD);
  data.q_in = q0;
  auto & cmodel = constraint_models[0];
  auto & cdata = constraint_datas[0];
  cmodel.calc(model, data, cdata);
  ConstraintCholeskyDecomposition chol(model, data, constraint_models, constraint_datas);
  chol.rebuild(model, data, constraint_models, constraint_datas);
  chol.compute(model, data, constraint_models, constraint_datas, 1e-10);

  auto G_expression = chol.getDelassusOperatorCholeskyExpression();
  const auto G_plain = G_expression.matrix();
  const Eigen::MatrixXd delassus_matrix_plain = G_expression.matrix();

  Eigen::MatrixXd constraint_jacobian(cmodel.residualSize(), model.nv);
  constraint_jacobian.setZero();
  getConstraintsJacobian(model, data, constraint_models, constraint_datas, constraint_jacobian);

  // External torques push the slider against the lower bound
  {
    const Eigen::VectorXd g_against_lower_bound = constraint_jacobian * v_free_against_lower_bound;
    const Eigen::VectorXd g_tilde_against_lower_bound =
      g_against_lower_bound + cdata.constraint_residual / dt;

    Eigen::VectorXd velocity_solution = Eigen::VectorXd::Zero(cmodel.residualSize());
    Eigen::VectorXd impulse_solution = Eigen::VectorXd::Zero(cmodel.residualSize());

    Eigen::VectorXd compliance = Eigen::VectorXd::Zero(g_tilde_against_lower_bound.size());
    G_expression.updateCompliance(compliance);

    ADMMConstraintSolver admm_solver(std::size_t(delassus_matrix_plain.rows()));
    ADMMSolverSettings admm_settings;
    admm_settings.absolute_feasibility_tol = 1e-13;
    admm_settings.relative_feasibility_tol = 1e-14;
    admm_settings.absolute_complementarity_tol = 1e-13;
    admm_settings.relative_complementarity_tol = 1e-14;
    ADMMSolverResult admm_result;
    admm_result.setConstraintImpulseGuess(impulse_solution);

    const bool has_converged = admm_solver.solve(
      G_expression, g_tilde_against_lower_bound, constraint_models, constraint_datas, admm_settings,
      admm_result);
    admm_result.retrieveConstraintImpulses(impulse_solution);
    BOOST_CHECK(has_converged);

    velocity_solution = G_plain * impulse_solution + g_against_lower_bound;
    Eigen::VectorXd velocity_solution2;
    admm_result.retrieveConstraintVelocities(velocity_solution2);

    BOOST_CHECK(std::fabs(impulse_solution.dot(velocity_solution)) <= 1e-8);
    BOOST_CHECK(velocity_solution.isZero(1e-6));
    BOOST_CHECK(velocity_solution2.isZero(1e-6));

    // std::cout << "tau_push_against_lower_bound:   " << tau_push_against_lower_bound << std::endl;
    // std::cout << "constraint_jacobian.transpose() * impulse_solution:   "
    //           << constraint_jacobian.transpose() * impulse_solution << std::endl;
    // std::cout << "impulse_solution:   " << impulse_solution << std::endl;

    BOOST_CHECK(
      (tau_push_against_lower_bound + constraint_jacobian.transpose() * impulse_solution / dt)
        .isZero(1e-6));
  }

  // External torques push the slider away from the lower bound
  {
    const Eigen::VectorXd g_move_away = constraint_jacobian * v_free_move_away;
    const Eigen::VectorXd g_tilde_move_away = g_move_away + cdata.constraint_residual / dt;

    Eigen::VectorXd velocity_solution = Eigen::VectorXd::Zero(cmodel.residualSize());
    Eigen::VectorXd impulse_solution = Eigen::VectorXd::Zero(cmodel.residualSize());

    Eigen::VectorXd compliance = Eigen::VectorXd::Zero(g_tilde_move_away.size());
    G_expression.updateCompliance(compliance);

    ADMMConstraintSolver admm_solver(std::size_t(delassus_matrix_plain.rows()));
    ADMMSolverSettings admm_settings;
    admm_settings.absolute_feasibility_tol = 1e-13;
    admm_settings.relative_feasibility_tol = 1e-14;
    admm_settings.absolute_complementarity_tol = 1e-13;
    admm_settings.relative_complementarity_tol = 1e-14;
    ADMMSolverResult admm_result;
    admm_result.setConstraintImpulseGuess(impulse_solution);

    const bool has_converged = admm_solver.solve(
      G_expression, g_tilde_move_away, constraint_models, constraint_datas, admm_settings,
      admm_result);
    admm_result.retrieveConstraintImpulses(impulse_solution);
    BOOST_CHECK(has_converged);

    velocity_solution = G_plain * impulse_solution + g_move_away;
    Eigen::VectorXd velocity_solution2;
    admm_result.retrieveConstraintVelocities(velocity_solution2);

    BOOST_CHECK(std::fabs(impulse_solution.dot(velocity_solution)) <= 1e-8);
    BOOST_CHECK(impulse_solution.isZero());
    BOOST_CHECK(velocity_solution.isApprox(g_move_away));
  }
}

BOOST_AUTO_TEST_CASE(joint_limit_slider_xyz)
{
  Model model;
  JointIndex joint_id_x = model.addJoint(0, JointModelPX(), SE3::Identity(), "slider_x");
  JointIndex joint_id_y = model.addJoint(joint_id_x, JointModelPY(), SE3::Identity(), "slider_y");
  JointIndex joint_id_z = model.addJoint(joint_id_y, JointModelPZ(), SE3::Identity(), "slider_z");

  const SE3::Vector3 small_box_dims = SE3::Vector3::Ones() * 1e-3;
  const double small_box_mass = 1e-6;
  const Inertia small_box_inertia =
    Inertia::FromBox(small_box_mass, small_box_dims[0], small_box_dims[1], small_box_dims[2]);

  const SE3::Vector3 box_dims = SE3::Vector3::Ones();
  const double box_mass = 10;
  const Inertia box_inertia = Inertia::FromBox(box_mass, box_dims[0], box_dims[1], box_dims[2]);

  model.appendBodyToJoint(joint_id_x, small_box_inertia);
  model.appendBodyToJoint(joint_id_y, small_box_inertia);
  model.appendBodyToJoint(joint_id_z, box_inertia);
  model.gravity.setZero();
  model.lowerPositionLimit[0] = 0.;
  model.lowerPositionLimit[1] = 0.;
  model.lowerPositionLimit[2] = 0.;
  Data data(model);

  Eigen::VectorXd q0 = Eigen::VectorXd::Zero(model.nq);
  const Eigen::VectorXd v0 = Eigen::VectorXd::Zero(model.nv);
  const Eigen::VectorXd tau_push_against_lower_bound = -Eigen::VectorXd::Ones(model.nv);

  const double dt = 1e-3;

  typedef JointLimitConstraintModel ConstraintModel;
  typedef ConstraintModel::ConstraintData ConstraintData;
  std::vector<ConstraintModel> constraint_models;
  std::vector<ConstraintData> constraint_datas;
  ConstraintModel::JointIndexVector active_joints = {joint_id_x, joint_id_y, joint_id_z};

  ConstraintModel joint_limit_constraint_model(model, active_joints);
  joint_limit_constraint_model.makeSelectionFilteredByLimitProximity(q0);
  constraint_models.push_back(joint_limit_constraint_model);

  for (const auto & cm : constraint_models)
    constraint_datas.push_back(cm.createData());

  const Eigen::VectorXd v_free_against_lower_bound =
    v0 + dt * aba(model, data, q0, v0, tau_push_against_lower_bound, Convention::WORLD);
  const Eigen::VectorXd v_free_move_away =
    v0 + dt * aba(model, data, q0, v0, -tau_push_against_lower_bound, Convention::WORLD);

  // Cholesky of the Delassus matrix
  crba(model, data, q0, Convention::WORLD);
  data.q_in = q0;
  auto & cmodel = constraint_models[0];
  auto & cdata = constraint_datas[0];
  cmodel.calc(model, data, cdata);
  ConstraintCholeskyDecomposition chol(model, data, constraint_models, constraint_datas);
  chol.rebuild(model, data, constraint_models, constraint_datas);
  chol.compute(model, data, constraint_models, constraint_datas, 1e-10);

  auto G_expression = chol.getDelassusOperatorCholeskyExpression();
  const auto G_plain = G_expression.matrix();
  const Eigen::MatrixXd delassus_matrix_plain = G_expression.matrix();

  Eigen::MatrixXd constraint_jacobian(cmodel.residualSize(), model.nv);
  constraint_jacobian.setZero();
  getConstraintsJacobian(model, data, constraint_models, constraint_datas, constraint_jacobian);

  // External torques push the slider against the lower bound
  {
    const Eigen::VectorXd g_against_lower_bound = constraint_jacobian * v_free_against_lower_bound;
    const Eigen::VectorXd g_tilde_against_lower_bound =
      g_against_lower_bound + cdata.constraint_residual / dt;

    Eigen::VectorXd velocity_solution = Eigen::VectorXd::Zero(cmodel.residualSize());
    Eigen::VectorXd impulse_solution = Eigen::VectorXd::Zero(cmodel.residualSize());

    Eigen::VectorXd compliance = Eigen::VectorXd::Zero(g_tilde_against_lower_bound.size());
    G_expression.updateCompliance(compliance);

    ADMMConstraintSolver admm_solver(std::size_t(delassus_matrix_plain.rows()));
    ADMMSolverSettings admm_settings;
    admm_settings.absolute_feasibility_tol = 1e-13;
    admm_settings.relative_feasibility_tol = 1e-14;
    admm_settings.absolute_complementarity_tol = 1e-13;
    admm_settings.relative_complementarity_tol = 1e-14;
    ADMMSolverResult admm_result;
    admm_result.setConstraintImpulseGuess(impulse_solution);

    const bool has_converged = admm_solver.solve(
      G_expression, g_tilde_against_lower_bound, constraint_models, constraint_datas, admm_settings,
      admm_result);
    admm_result.retrieveConstraintImpulses(impulse_solution);
    BOOST_CHECK(has_converged);

    velocity_solution = G_plain * impulse_solution + g_against_lower_bound;
    Eigen::VectorXd velocity_solution2;
    admm_result.retrieveConstraintVelocities(velocity_solution2);

    BOOST_CHECK(std::fabs(impulse_solution.dot(velocity_solution)) <= 1e-8);
    BOOST_CHECK(velocity_solution.isZero(1e-6));
    BOOST_CHECK(velocity_solution2.isZero(1e-6));

    // std::cout << "tau_push_against_lower_bound:   " << tau_push_against_lower_bound << std::endl;
    // std::cout << "constraint_jacobian.transpose() * impulse_solution:   "
    //           << constraint_jacobian.transpose() * impulse_solution << std::endl;
    // std::cout << "impulse_solution:   " << impulse_solution << std::endl;

    BOOST_CHECK(
      (tau_push_against_lower_bound + constraint_jacobian.transpose() * impulse_solution / dt)
        .isZero(1e-6));
  }

  // External torques push the slider away from the lower bound
  {
    const Eigen::VectorXd g_move_away = constraint_jacobian * v_free_move_away;
    const Eigen::VectorXd g_tilde_move_away = g_move_away + cdata.constraint_residual / dt;

    Eigen::VectorXd velocity_solution = Eigen::VectorXd::Zero(cmodel.residualSize());
    Eigen::VectorXd impulse_solution = Eigen::VectorXd::Zero(cmodel.residualSize());

    Eigen::VectorXd compliance = Eigen::VectorXd::Zero(g_tilde_move_away.size());
    G_expression.updateCompliance(compliance);

    ADMMConstraintSolver admm_solver(std::size_t(delassus_matrix_plain.rows()));
    ADMMSolverSettings admm_settings;
    admm_settings.absolute_feasibility_tol = 1e-13;
    admm_settings.relative_feasibility_tol = 1e-14;
    admm_settings.absolute_complementarity_tol = 1e-13;
    admm_settings.relative_complementarity_tol = 1e-14;
    ADMMSolverResult admm_result;
    admm_result.setConstraintImpulseGuess(impulse_solution);

    const bool has_converged = admm_solver.solve(
      G_expression, g_tilde_move_away, constraint_models, constraint_datas, admm_settings,
      admm_result);
    admm_result.retrieveConstraintImpulses(impulse_solution);
    BOOST_CHECK(has_converged);

    velocity_solution = G_plain * impulse_solution + g_move_away;
    Eigen::VectorXd velocity_solution2;
    admm_result.retrieveConstraintVelocities(velocity_solution2);

    BOOST_CHECK(std::fabs(impulse_solution.dot(velocity_solution)) <= 1e-8);
    BOOST_CHECK(impulse_solution.isZero());
    BOOST_CHECK(velocity_solution.isApprox(g_move_away));
  }
}

BOOST_AUTO_TEST_CASE(joint_limit_translation)
{
  // We test limits for a joint with nq>1
  Model model;
  model.addJoint(0, JointModelTranslation(), SE3::Identity(), "translation");
  model.lowerPositionLimit = Eigen::VectorXd::Ones(model.nq) * -10000;
  model.lowerPositionLimit[2] = 0;
  model.upperPositionLimit = Eigen::VectorXd::Ones(model.nq) * 10000;

  const SE3::Vector3 box_dims = SE3::Vector3::Ones();
  const double box_mass = 10;
  const Inertia box_inertia = Inertia::FromBox(box_mass, box_dims[0], box_dims[1], box_dims[2]);

  model.appendBodyToJoint(1, box_inertia);
  Data data(model);

  Eigen::VectorXd q0 = neutral(model);
  const Eigen::VectorXd v0 = Eigen::VectorXd::Zero(model.nv);
  Eigen::VectorXd tau_gravity = Eigen::VectorXd::Zero(model.nv);
  tau_gravity(2) = 9.81 * box_mass;

  const double dt = 1e-3;

  typedef JointLimitConstraintModel ConstraintModel;
  typedef ConstraintModel::ConstraintData ConstraintData;
  std::vector<ConstraintModel> constraint_models;
  std::vector<ConstraintData> constraint_datas;

  ConstraintModel joint_limit_constraint_model(model, ConstraintModel::JointIndexVector(1, 1));
  joint_limit_constraint_model.makeSelectionFilteredByLimitProximity(q0);
  constraint_models.push_back(joint_limit_constraint_model);

  for (const auto & cm : constraint_models)
    constraint_datas.push_back(cm.createData());

  const Eigen::VectorXd v_free_against_lower_bound =
    v0 + dt * aba(model, data, q0, v0, Eigen::VectorXd::Zero(model.nv), Convention::WORLD);
  const Eigen::VectorXd v_free_move_away =
    v0 + dt * aba(model, data, q0, v0, tau_gravity, Convention::WORLD);

  // Cholesky of the Delassus matrix
  crba(model, data, q0, Convention::WORLD);
  data.q_in = q0;
  auto & cmodel = constraint_models[0];
  auto & cdata = constraint_datas[0];
  cmodel.calc(model, data, cdata);
  ConstraintCholeskyDecomposition chol(model, data, constraint_models, constraint_datas);
  chol.rebuild(model, data, constraint_models, constraint_datas);
  chol.compute(model, data, constraint_models, constraint_datas, 1e-10);

  auto G_expression = chol.getDelassusOperatorCholeskyExpression();
  const auto G_plain = G_expression.matrix();
  const Eigen::MatrixXd delassus_matrix_plain = G_expression.matrix();

  Eigen::MatrixXd constraint_jacobian(cmodel.residualSize(), model.nv);
  constraint_jacobian.setZero();
  getConstraintsJacobian(model, data, constraint_models, constraint_datas, constraint_jacobian);

  // Gravity pushes the freeflyer against the lower bound
  {
    const Eigen::VectorXd g_against_lower_bound = constraint_jacobian * v_free_against_lower_bound;
    const Eigen::VectorXd g_tilde_against_lower_bound =
      g_against_lower_bound + cdata.constraint_residual / dt;

    Eigen::VectorXd constraint_velocity = Eigen::VectorXd::Zero(cmodel.residualSize());
    Eigen::VectorXd impulse_solution = Eigen::VectorXd::Zero(cmodel.residualSize());

    Eigen::VectorXd compliance = Eigen::VectorXd::Zero(g_tilde_against_lower_bound.size());
    G_expression.updateCompliance(compliance);

    ADMMConstraintSolver admm_solver(std::size_t(delassus_matrix_plain.rows()));
    ADMMSolverSettings admm_settings;
    admm_settings.absolute_feasibility_tol = 1e-13;
    admm_settings.relative_feasibility_tol = 1e-14;
    admm_settings.absolute_complementarity_tol = 1e-13;
    admm_settings.relative_complementarity_tol = 1e-14;
    ADMMSolverResult admm_result;
    admm_result.setConstraintImpulseGuess(impulse_solution);

    const bool has_converged = admm_solver.solve(
      G_expression, g_tilde_against_lower_bound, constraint_models, constraint_datas, admm_settings,
      admm_result);
    admm_result.retrieveConstraintImpulses(impulse_solution);
    BOOST_CHECK(has_converged);

    constraint_velocity = G_plain * impulse_solution + g_against_lower_bound;
    constraint_velocity /= dt;
    Eigen::VectorXd velocity_solution;
    admm_result.retrieveConstraintVelocities(velocity_solution);

    BOOST_CHECK(std::fabs(impulse_solution.dot(velocity_solution)) <= 1e-8);
    BOOST_CHECK(constraint_velocity.isZero(1e-6));
    BOOST_CHECK((velocity_solution - (G_plain * impulse_solution + g_tilde_against_lower_bound))
                  .isZero(1e-6));

    BOOST_CHECK(
      (-tau_gravity + constraint_jacobian.transpose() * impulse_solution / dt).isZero(1e-6));
  }

  // External torques compensate the gravity to push the freeflyer away from the lower bound
  {
    const Eigen::VectorXd g_move_away = constraint_jacobian * v_free_move_away;
    const Eigen::VectorXd g_tilde_move_away = g_move_away + cdata.constraint_residual / dt;

    Eigen::VectorXd velocity_solution = Eigen::VectorXd::Zero(cmodel.residualSize());
    Eigen::VectorXd impulse_solution = Eigen::VectorXd::Zero(cmodel.residualSize());

    Eigen::VectorXd compliance = Eigen::VectorXd::Zero(g_tilde_move_away.size());
    G_expression.updateCompliance(compliance);

    ADMMConstraintSolver admm_solver(std::size_t(delassus_matrix_plain.rows()));
    ADMMSolverSettings admm_settings;
    admm_settings.absolute_feasibility_tol = 1e-13;
    admm_settings.relative_feasibility_tol = 1e-14;
    admm_settings.absolute_complementarity_tol = 1e-13;
    admm_settings.relative_complementarity_tol = 1e-14;
    ADMMSolverResult admm_result;
    admm_result.setConstraintImpulseGuess(impulse_solution);

    const bool has_converged = admm_solver.solve(
      G_expression, g_tilde_move_away, constraint_models, constraint_datas, admm_settings,
      admm_result);
    admm_result.retrieveConstraintImpulses(impulse_solution);
    BOOST_CHECK(has_converged);

    velocity_solution = G_plain * impulse_solution + g_move_away;
    Eigen::VectorXd velocity_solution2;
    admm_result.retrieveConstraintVelocities(velocity_solution2);

    BOOST_CHECK(std::fabs(impulse_solution.dot(velocity_solution)) <= 1e-8);
    BOOST_CHECK(impulse_solution.isZero());
    BOOST_CHECK(velocity_solution.isApprox(g_move_away));
  }
}

BOOST_AUTO_TEST_CASE(joint_limit_freeflyer)
{
  // We test limits for a joint with nq>1
  Model model;
  model.addJoint(0, JointModelFreeFlyer(), SE3::Identity(), "freeflyer");
  model.lowerPositionLimit = Eigen::VectorXd::Ones(model.nq) * -10000;
  model.lowerPositionLimit[2] = 0;
  model.upperPositionLimit = Eigen::VectorXd::Ones(model.nq) * 10000;

  const SE3::Vector3 box_dims = SE3::Vector3::Ones();
  const double box_mass = 10;
  const Inertia box_inertia = Inertia::FromBox(box_mass, box_dims[0], box_dims[1], box_dims[2]);

  model.appendBodyToJoint(1, box_inertia);
  Data data(model);

  Eigen::VectorXd q0 = neutral(model);
  const Eigen::VectorXd v0 = Eigen::VectorXd::Zero(model.nv);
  Eigen::VectorXd tau_gravity = Eigen::VectorXd::Zero(model.nv);
  tau_gravity(2) = 9.81 * box_mass;

  const double dt = 1e-3;

  typedef JointLimitConstraintModel ConstraintModel;
  typedef ConstraintModel::ConstraintData ConstraintData;
  std::vector<ConstraintModel> constraint_models;
  std::vector<ConstraintData> constraint_datas;

  ConstraintModel joint_limit_constraint_model(model, ConstraintModel::JointIndexVector(1, 1));
  joint_limit_constraint_model.makeSelectionFilteredByLimitProximity(q0);
  constraint_models.push_back(joint_limit_constraint_model);

  for (const auto & cm : constraint_models)
    constraint_datas.push_back(cm.createData());

  const Eigen::VectorXd v_free_against_lower_bound =
    v0 + dt * aba(model, data, q0, v0, Eigen::VectorXd::Zero(model.nv), Convention::WORLD);
  const Eigen::VectorXd v_free_move_away =
    v0 + dt * aba(model, data, q0, v0, tau_gravity, Convention::WORLD);

  // Cholesky of the Delassus matrix
  crba(model, data, q0, Convention::WORLD);
  data.q_in = q0;
  auto & cmodel = constraint_models[0];
  auto & cdata = constraint_datas[0];
  cmodel.calc(model, data, cdata);
  ConstraintCholeskyDecomposition chol(model, data, constraint_models, constraint_datas);
  chol.rebuild(model, data, constraint_models, constraint_datas);
  chol.compute(model, data, constraint_models, constraint_datas, 1e-10);

  auto G_expression = chol.getDelassusOperatorCholeskyExpression();
  const auto G_plain = G_expression.matrix();
  const Eigen::MatrixXd delassus_matrix_plain = G_expression.matrix();

  Eigen::MatrixXd constraint_jacobian(cmodel.residualSize(), model.nv);
  constraint_jacobian.setZero();
  getConstraintsJacobian(model, data, constraint_models, constraint_datas, constraint_jacobian);

  // Gravity pushes the freeflyer against the lower bound
  {
    const Eigen::VectorXd g_against_lower_bound = constraint_jacobian * v_free_against_lower_bound;
    const Eigen::VectorXd g_tilde_against_lower_bound =
      g_against_lower_bound + cdata.constraint_residual / dt;

    Eigen::VectorXd constraint_velocity = Eigen::VectorXd::Zero(cmodel.residualSize());
    Eigen::VectorXd impulse_solution = Eigen::VectorXd::Zero(cmodel.residualSize());

    Eigen::VectorXd compliance = Eigen::VectorXd::Zero(g_tilde_against_lower_bound.size());
    G_expression.updateCompliance(compliance);

    ADMMConstraintSolver admm_solver(std::size_t(delassus_matrix_plain.rows()));
    ADMMSolverSettings admm_settings;
    admm_settings.absolute_feasibility_tol = 1e-13;
    admm_settings.relative_feasibility_tol = 1e-14;
    admm_settings.absolute_complementarity_tol = 1e-13;
    admm_settings.relative_complementarity_tol = 1e-14;
    ADMMSolverResult admm_result;
    admm_result.setConstraintImpulseGuess(impulse_solution);

    const bool has_converged = admm_solver.solve(
      G_expression, g_tilde_against_lower_bound, constraint_models, constraint_datas, admm_settings,
      admm_result);
    admm_result.retrieveConstraintImpulses(impulse_solution);
    BOOST_CHECK(has_converged);

    constraint_velocity = G_plain * impulse_solution + g_against_lower_bound;
    Eigen::VectorXd velocity_solution;
    admm_result.retrieveConstraintVelocities(velocity_solution);

    BOOST_CHECK(std::fabs(impulse_solution.dot(velocity_solution)) <= 1e-8);
    BOOST_CHECK(constraint_velocity.isZero(1e-6));
    BOOST_CHECK((velocity_solution - (G_plain * impulse_solution + g_tilde_against_lower_bound))
                  .isZero(1e-6));

    BOOST_CHECK(
      (-tau_gravity + constraint_jacobian.transpose() * impulse_solution / dt).isZero(1e-6));
  }

  // External torques compensate the gravity to push the freeflyer away from the lower bound
  {
    const Eigen::VectorXd g_move_away = constraint_jacobian * v_free_move_away;
    const Eigen::VectorXd g_tilde_move_away = g_move_away + cdata.constraint_residual / dt;

    Eigen::VectorXd velocity_solution = Eigen::VectorXd::Zero(cmodel.residualSize());
    Eigen::VectorXd impulse_solution = Eigen::VectorXd::Zero(cmodel.residualSize());

    Eigen::VectorXd compliance = Eigen::VectorXd::Zero(g_tilde_move_away.size());
    G_expression.updateCompliance(compliance);

    ADMMConstraintSolver admm_solver(std::size_t(delassus_matrix_plain.rows()));
    ADMMSolverSettings admm_settings;
    admm_settings.absolute_feasibility_tol = 1e-13;
    admm_settings.relative_feasibility_tol = 1e-14;
    admm_settings.absolute_complementarity_tol = 1e-13;
    admm_settings.relative_complementarity_tol = 1e-14;
    ADMMSolverResult admm_result;
    admm_result.setConstraintImpulseGuess(impulse_solution);

    const bool has_converged = admm_solver.solve(
      G_expression, g_tilde_move_away, constraint_models, constraint_datas, admm_settings,
      admm_result);
    admm_result.retrieveConstraintImpulses(impulse_solution);
    BOOST_CHECK(has_converged);

    velocity_solution = G_plain * impulse_solution + g_move_away;
    Eigen::VectorXd velocity_solution2;
    admm_result.retrieveConstraintVelocities(velocity_solution2);

    BOOST_CHECK(std::fabs(impulse_solution.dot(velocity_solution)) <= 1e-8);
    BOOST_CHECK(impulse_solution.isZero());
    BOOST_CHECK(velocity_solution.isApprox(g_move_away));
  }
}

BOOST_AUTO_TEST_CASE(joint_limit_composite)
{
  // We test limits for a joint with nq>1
  JointModelComposite joint;
  joint.addJoint(JointModelRX());
  joint.addJoint(JointModelRY());
  Model model;
  model.addJoint(0, joint, SE3::Identity(), "composite");
  model.lowerPositionLimit = Eigen::VectorXd::Ones(model.nq) * -10000;
  model.lowerPositionLimit[1] = 0;
  model.upperPositionLimit = Eigen::VectorXd::Ones(model.nq) * 10000;

  const SE3::Vector3 box_dims = SE3::Vector3::Ones();
  const double box_mass = 10;
  const Inertia box_inertia = Inertia::FromBox(box_mass, box_dims[0], box_dims[1], box_dims[2]);

  model.appendBodyToJoint(1, box_inertia);
  model.gravity.setZero();
  Data data(model);

  Eigen::VectorXd q0 = Eigen::VectorXd::Zero(model.nq);
  const Eigen::VectorXd v0 = Eigen::VectorXd::Zero(model.nv);
  const Eigen::VectorXd tau_push_against_lower_bound = -Eigen::VectorXd::Ones(model.nv);

  const double dt = 1e-3;

  typedef JointLimitConstraintModel ConstraintModel;
  typedef ConstraintModel::ConstraintData ConstraintData;
  std::vector<ConstraintModel> constraint_models;
  std::vector<ConstraintData> constraint_datas;

  ConstraintModel joint_limit_constraint_model(model, ConstraintModel::JointIndexVector(1, 1));
  joint_limit_constraint_model.makeSelectionFilteredByLimitProximity(q0);
  constraint_models.push_back(joint_limit_constraint_model);

  for (const auto & cm : constraint_models)
    constraint_datas.push_back(cm.createData());

  const Eigen::VectorXd v_free_against_lower_bound =
    v0 + dt * aba(model, data, q0, v0, tau_push_against_lower_bound, Convention::WORLD);
  const Eigen::VectorXd v_free_move_away =
    v0 + dt * aba(model, data, q0, v0, -tau_push_against_lower_bound, Convention::WORLD);

  // Cholesky of the Delassus matrix
  crba(model, data, q0, Convention::WORLD);
  data.q_in = q0;
  auto & cmodel = constraint_models[0];
  auto & cdata = constraint_datas[0];
  cmodel.calc(model, data, cdata);
  ConstraintCholeskyDecomposition chol(model, data, constraint_models, constraint_datas);
  chol.rebuild(model, data, constraint_models, constraint_datas);
  chol.compute(model, data, constraint_models, constraint_datas, 1e-10);

  auto G_expression = chol.getDelassusOperatorCholeskyExpression();
  const auto G_plain = G_expression.matrix();
  const Eigen::MatrixXd delassus_matrix_plain = G_expression.matrix();

  Eigen::MatrixXd constraint_jacobian(cmodel.residualSize(), model.nv);
  constraint_jacobian.setZero();
  getConstraintsJacobian(model, data, constraint_models, constraint_datas, constraint_jacobian);

  // External torques push the freeflyer against from the lower bound
  {
    const Eigen::VectorXd g_against_lower_bound = constraint_jacobian * v_free_against_lower_bound;
    const Eigen::VectorXd g_tilde_against_lower_bound =
      g_against_lower_bound + cdata.constraint_residual / dt;

    Eigen::VectorXd constraint_velocity = Eigen::VectorXd::Zero(cmodel.residualSize());
    Eigen::VectorXd impulse_solution = Eigen::VectorXd::Zero(cmodel.residualSize());

    Eigen::VectorXd compliance = Eigen::VectorXd::Zero(g_tilde_against_lower_bound.size());
    G_expression.updateCompliance(compliance);

    ADMMConstraintSolver admm_solver(std::size_t(delassus_matrix_plain.rows()));
    ADMMSolverSettings admm_settings;
    admm_settings.absolute_feasibility_tol = 1e-13;
    admm_settings.relative_feasibility_tol = 1e-14;
    admm_settings.absolute_complementarity_tol = 1e-13;
    admm_settings.relative_complementarity_tol = 1e-14;
    ADMMSolverResult admm_result;
    admm_result.setConstraintImpulseGuess(impulse_solution);

    const bool has_converged = admm_solver.solve(
      G_expression, g_tilde_against_lower_bound, constraint_models, constraint_datas, admm_settings,
      admm_result);
    admm_result.retrieveConstraintImpulses(impulse_solution);
    BOOST_CHECK(has_converged);

    constraint_velocity = G_plain * impulse_solution + g_against_lower_bound;

    Eigen::VectorXd velocity_solution;
    admm_result.retrieveConstraintVelocities(velocity_solution);

    BOOST_CHECK(std::fabs(impulse_solution.dot(velocity_solution)) <= 1e-8);
    BOOST_CHECK(std::abs(constraint_velocity[0]) < 1e-6);
    BOOST_CHECK((velocity_solution - (G_plain * impulse_solution + g_tilde_against_lower_bound))
                  .isZero(1e-6));

    BOOST_CHECK(
      std::abs(
        (tau_push_against_lower_bound + constraint_jacobian.transpose() * impulse_solution / dt)(1))
      < 1e-6);
  }

  // External torques push the freeflyer away from the lower bound
  {
    const Eigen::VectorXd g_move_away = constraint_jacobian * v_free_move_away;
    const Eigen::VectorXd g_tilde_move_away = g_move_away + cdata.constraint_residual / dt;

    Eigen::VectorXd velocity_solution = Eigen::VectorXd::Zero(cmodel.residualSize());
    Eigen::VectorXd impulse_solution = Eigen::VectorXd::Zero(cmodel.residualSize());

    Eigen::VectorXd compliance = Eigen::VectorXd::Zero(g_tilde_move_away.size());
    G_expression.updateCompliance(compliance);

    ADMMConstraintSolver admm_solver(std::size_t(delassus_matrix_plain.rows()));
    ADMMSolverSettings admm_settings;
    admm_settings.absolute_feasibility_tol = 1e-13;
    admm_settings.relative_feasibility_tol = 1e-14;
    admm_settings.absolute_complementarity_tol = 1e-13;
    admm_settings.relative_complementarity_tol = 1e-14;
    ADMMSolverResult admm_result;
    admm_result.setConstraintImpulseGuess(impulse_solution);

    const bool has_converged = admm_solver.solve(
      G_expression, g_tilde_move_away, constraint_models, constraint_datas, admm_settings,
      admm_result);
    admm_result.retrieveConstraintImpulses(impulse_solution);
    BOOST_CHECK(has_converged);

    velocity_solution = G_plain * impulse_solution + g_move_away;
    Eigen::VectorXd velocity_solution2;
    admm_result.retrieveConstraintVelocities(velocity_solution2);

    BOOST_CHECK(std::fabs(impulse_solution.dot(velocity_solution)) <= 1e-8);
    BOOST_CHECK(impulse_solution.isZero());
    BOOST_CHECK(velocity_solution.isApprox(g_move_away));
  }
}

BOOST_AUTO_TEST_CASE(test_copy_result)
{
  const Eigen::Index n = 6;
  ADMMSolverResult result;
  result.resize(n);
  result.x.setRandom();

  // copy constructor: maps must point to the copy's own storage
  ADMMSolverResult result_copy(result);
  BOOST_CHECK(result_copy.x.data() != result.x.data());
  BOOST_CHECK(result_copy.x.data() == result_copy.x_storage.map().data());
  BOOST_CHECK(result_copy.x.isApprox(result.x));

  // copy constructor: independence — modify original, copy unchanged
  const Eigen::VectorXd expected_x = result.x;
  result.x.setZero();
  BOOST_CHECK(result_copy.x.isApprox(expected_x));

  // copy assignment: maps must point to the assigned object's own storage
  ADMMSolverResult result_assigned;
  result_assigned = result_copy;
  BOOST_CHECK(result_assigned.x.data() != result_copy.x.data());
  BOOST_CHECK(result_assigned.x.data() == result_assigned.x_storage.map().data());
  BOOST_CHECK(result_assigned.x.isApprox(result_copy.x));

  // copy assignment: independence
  result_copy.x.setZero();
  BOOST_CHECK(result_assigned.x.isApprox(expected_x));
}
