#include <iostream>
#include <iomanip>

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/parser/urdf.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/kinematics.hpp"

int main(int argc, const char**argv)
{
  std::string filename = "/home/nmansard/src/rbdl/rbdl_evaluate_performances/models/simple_humanoid.urdf";
  if(argc>1) filename = argv[1];

  se3::Model model = se3::buildModel(filename);
  model.gravity.linear( Eigen::Vector3d(0,0,9.8));
  se3::Data data(model);

  Eigen::VectorXd q = Eigen::VectorXd::Zero(model.nq);
  Eigen::VectorXd v = Eigen::VectorXd::Zero(model.nv);
  Eigen::VectorXd a = Eigen::VectorXd::Zero(model.nv);
 
  //kinematics(model,data,q,v);
  rnea(model,data,q,v,a);

  using namespace Eigen;
  using namespace se3;

  std::cout << std::setprecision(10);

  std::cout << "Number of dof : " << model.nv << std::endl;
  std::cout << "rnea(0,0,0) = g(0) = " << data.tau.transpose() << std::endl;

  for( int i=0;i<model.nbody;++i )
    {
      if(model.parents[i]!=i-1)
	std::cout << "************** END EFFECTOR" << std::endl;

      std::cout << "\n\n === " << i << " ========================" << std::endl;
      std::cout << "Joint "<<i<<" = " << model.names[i] << std::endl;
      std::cout << "m"<<i<<" = \n" << (SE3::Matrix4)data.oMi[i] << std::endl;
      std::cout << "v"<<i<<" = \n" << SE3::Vector6(data.v[i]).transpose()<< std::endl;
      std::cout << "a"<<i<<" = \n" << SE3::Vector6(data.a[i]).transpose() << std::endl;
      std::cout << "f"<<i<<" = \n" << SE3::Vector6(data.f[i]).transpose() << std::endl;
   }

  return 0;
}
