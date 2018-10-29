#include "pinocchio/parsers/sample-models.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"

int main(int argc, char ** argv)
{
  se3::Model model;
  se3::buildModels::manipulator(model);
  se3::Data data(model);

  const int    JOINT_ID = 6;
  const double DT       = 1e-1;
  Eigen::VectorXd q     = se3::neutral(model);
  Eigen::Vector3d xdes;        xdes << 0.5, -0.5, 0.5;

  se3::Data::Matrix6x J(6,model.nv); J.setZero();
  unsigned int svdOptions = Eigen::ComputeThinU | Eigen::ComputeThinV;

  for (int i=0 ; i<100 ; ++i)
    {
      se3::computeJointJacobians(model,data,q);
      se3::getJointJacobian<se3::LOCAL>(model,data,JOINT_ID,J);
      const Eigen::Vector3d & x   = data.oMi[JOINT_ID].translation();
      const Eigen::Matrix3d & R   = data.oMi[JOINT_ID].rotation();
      const Eigen::Vector3d & err = R.transpose()*(x-xdes);
      const Eigen::VectorXd v     = -J.topRows<3>().bdcSvd(svdOptions).solve(err);
      q = se3::integrate(model,q,v*DT);
      if(!(i%10)) std::cout << "error = " << err.transpose() << std::endl;
    }

  // Computing error for final q
  se3::forwardKinematics(model,data,q);
  const Eigen::Vector3d & x   = data.oMi[JOINT_ID].translation();
  const Eigen::Vector3d & err = x-xdes;
  std::cout << "\nfinal error: " << err.transpose() << std::endl;
 
}
