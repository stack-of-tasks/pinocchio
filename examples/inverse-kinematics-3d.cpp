#include <iostream>

#include "pinocchio/multibody/sample-models.hpp"
#include "pinocchio/spatial/explog.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"

int main(int /* argc */, char ** /* argv */)
{
  pinocchio::Model model;
  pinocchio::buildModels::manipulator(model);
  pinocchio::Data data(model);

  const int JOINT_ID = 6;
  const pinocchio::SE3 oMdes(Eigen::Matrix3d::Identity(), Eigen::Vector3d(1., 0., 1.));

  Eigen::VectorXd q = pinocchio::neutral(model);
  const double eps = 1e-4;
  const int IT_MAX = 1000;
  const double DT = 1e-1;
  const double damp = 1e-12;

  pinocchio::Data::Matrix6x joint_jacobian(6, model.nv);
  joint_jacobian.setZero();

  bool success = false;
  Eigen::Vector3d err;
  Eigen::VectorXd v(model.nv);
  for (int i = 0;; i++)
  {
    pinocchio::forwardKinematics(model, data, q);
    const pinocchio::SE3 iMd = data.oMi[JOINT_ID].actInv(oMdes);
    err = iMd.translation(); // in joint frame
    if (err.norm() < eps)
    {
      success = true;
      break;
    }
    if (i >= IT_MAX)
    {
      success = false;
      break;
    }
    pinocchio::computeJointJacobian(
      model, data, q, JOINT_ID, joint_jacobian); // joint_jacobian expressed in the joint frame
    const auto J = -joint_jacobian.topRows<3>(); // Jacobian associated with the error
    const Eigen::Matrix3d JJt = J * J.transpose() + damp * Eigen::Matrix3d::Identity();
    v.noalias() = -J.transpose() * JJt.ldlt().solve(err);
    q = pinocchio::integrate(model, q, v * DT);
    if (!(i % 10))
      std::cout << i << ": error = " << err.transpose() << std::endl;
  }

  if (success)
  {
    std::cout << "Convergence achieved!" << std::endl;
  }
  else
  {
    std::cout
      << "\nWarning: the iterative algorithm has not reached convergence to the desired precision"
      << std::endl;
  }

  std::cout << "\nresult: " << q.transpose() << std::endl;
  std::cout << "\nfinal error: " << err.transpose() << std::endl;
}
