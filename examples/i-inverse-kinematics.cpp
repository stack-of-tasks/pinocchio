#include "pinocchio/parsers/sample-models.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"

int main(int argc, char ** argv)
{
  pinocchio::Model model;
  pinocchio::buildModels::manipulator(model);
  pinocchio::Data data(model);

  const int    JOINT_ID = 6;
  const double DT       = 1e-1;
  Eigen::VectorXd q     = pinocchio::neutral(model);
  Eigen::Vector3d xdes;        xdes << 0.5, -0.5, 0.5;

  pinocchio::Data::Matrix6x J(6,model.nv); J.setZero();
  unsigned int svdOptions = Eigen::ComputeThinU | Eigen::ComputeThinV;

  for (int i=0 ; i<100 ; ++i)
    {
      pinocchio::computeJointJacobians(model,data,q);
      pinocchio::getJointJacobian(model,data,JOINT_ID,pinocchio::LOCAL,J);
      const Eigen::Vector3d & x   = data.oMi[JOINT_ID].translation();
      const Eigen::Matrix3d & R   = data.oMi[JOINT_ID].rotation();
      const Eigen::Vector3d & err = R.transpose()*(x-xdes);
      const Eigen::VectorXd v     = -J.topRows<3>().bdcSvd(svdOptions).solve(err);
      q = pinocchio::integrate(model,q,v*DT);
      if(!(i%10)) std::cout << "error = " << err.transpose() << std::endl;
    }

  // Computing error for final q
  pinocchio::forwardKinematics(model,data,q);
  const Eigen::Vector3d & x   = data.oMi[JOINT_ID].translation();
  const Eigen::Vector3d & err = x-xdes;
  std::cout << "\nfinal error: " << err.transpose() << std::endl;
 
}
