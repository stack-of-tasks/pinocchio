#include "pinocchio/multibody/model.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/kinematics.hpp"

int main(int argc, char ** argv)
{
  std::string filename = (argc<=1) ? "ur5.urdf" : argv[1];
  se3::Model model;
  se3::urdf::buildModel(filename,model); 
  se3::Data data(model);
  Eigen::VectorXd q(model.nq); q    << 2, -.5, 1.8, 1.8, 2.6, -2;

  se3::forwardKinematics(model,data,q);

  for (int k=0 ; k<model.njoints ; ++k)
    std::cout << model.names[k] << "\t: "
              << data.oMi[k].translation().transpose() << std::endl;
}
