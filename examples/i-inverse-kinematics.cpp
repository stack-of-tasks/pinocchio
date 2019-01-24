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
  Eigen::Vector3d xdes;        xdes << 0.5, -0.5, 0.5;  

  Eigen::VectorXd q     = pinocchio::neutral(model);
  const double eps      = 1e-4;
  const int IT_MAX      = 1000
  const double DT       = 1e-1;

  pinocchio::Data::Matrix6x J(6,model.nv); J.setZero();
  unsigned int svdOptions = Eigen::ComputeThinU | Eigen::ComputeThinV;

  for (int i=0 ; i<IT_MAX ; ++i)
    {
      pinocchio::forwardKinematics(model,data,q);
      const Eigen::Vector3d & x   = data.oMi[JOINT_ID].translation();
      const Eigen::Matrix3d & R   = data.oMi[JOINT_ID].rotation();
      const Eigen::Vector3d & err = R.transpose()*(x-xdes);
      if(err.norm() < eps)
        {
          std::cout << "Convergence achieved!" << std::endl;
          break;
        }
      pinocchio::jointJacobian(model,data,JOINT_ID,pinocchio::LOCAL,J);
      const Eigen::VectorXd v     = -J.topRows<3>().bdcSvd(svdOptions).solve(err);
      q = pinocchio::integrate(model,q,v*DT);
      if(!(i%10)) std::cout << "error = " << err.transpose() << std::endl;
    }

  if(i==IT_MAX)
    std::cout << "\nWarning: the iterative algorithm has not reached convergence to the desired precision" << std::endl;

  std::cout << "\nresult: " << q.transpose() << std::endl;
  std::cout << "\nfinal error: " << err.transpose() << std::endl;
 
}
