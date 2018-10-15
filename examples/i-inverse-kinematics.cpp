#include "pinocchio/multibody/model.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"

int main(int argc, char ** argv)
{
  std::string filename = (argc<=1) ? "ur5.urdf" : argv[1];
  se3::Model model;
  se3::urdf::buildModel(filename,model); 
  se3::Data data(model);

  const int    JOINT_ID = 6;
  const double DT       = 1e-1;
  Eigen::VectorXd q(model.nq); q    << 2, -.5, 1.8, 1.8, 2.6, -2;
  Eigen::Vector3d xdes;        xdes << 0, -0.5, 0.5;

  se3::Data::Matrix6x J(6,model.nv); J.fill(0);
  unsigned int svdOptions = Eigen::ComputeThinU | Eigen::ComputeThinV;

  for (int i=0 ; i<100 ; ++i)
    {
      se3::computeJointJacobians(model,data,q);
      se3::getJointJacobian<se3::LOCAL>(model,data,JOINT_ID,J);
      const Eigen::Vector3d & x   = data.oMi[JOINT_ID].translation();
      const Eigen::Matrix3d & R   = data.oMi[JOINT_ID].rotation();
      const Eigen::Vector3d & err = R.transpose()*(x-xdes);
      const Eigen::VectorXd v     = -J.topRows<3>().bdcSvd(svdOptions).solve(err);
      q   = se3::integrate(model,q,v*DT);
      if(!(i%10)) std::cout << err.transpose() << std::endl;
    }
 
}
