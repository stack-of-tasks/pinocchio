#include "pinocchio/parsers/sample-models.hpp"
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
  const double eps  = 1e-4;
  const int IT_MAX  = 1000;
  const double DT   = 1e-1;

  pinocchio::Data::Matrix6x J(6,model.nv); J.setZero();
  unsigned int svdOptions = Eigen::ComputeThinU | Eigen::ComputeThinV;
  Eigen::JacobiSVD<pinocchio::Data::Matrix6x> svd(6, model.nv, svdOptions);

  typedef Eigen::Matrix<double, 6, 1> Vector6d;
  Vector6d err;
  Eigen::VectorXd v(model.nv);
  for (int i=0;;i++)
  {
    pinocchio::forwardKinematics(model,data,q);
    const pinocchio::SE3 dMi = oMdes.actInv(data.oMi[JOINT_ID]);
    err = pinocchio::log6(dMi).toVector();
    if(err.norm() < eps)
    {
      std::cout << "Convergence achieved!" << std::endl;
      break;
    }
    if (i >= IT_MAX)
    {
      std::cout << "\nWarning: the iterative algorithm has not reached convergence to the desired precision" << std::endl;
      break;
    }
    pinocchio::computeJointJacobian(model,data,q,JOINT_ID,J);
    pinocchio::SE3::Matrix6 Jlog;
    pinocchio::Jlog6(dMi, Jlog);
    v = - svd.compute(Jlog * J).solve(err);
    q = pinocchio::integrate(model,q,v*DT);
    if(!(i%10))
      std::cout << "error = " << err.transpose() << std::endl;
  }

  std::cout << "\nresult: " << q.transpose() << std::endl;
  std::cout << "\nfinal error: " << err.transpose() << std::endl;
}
