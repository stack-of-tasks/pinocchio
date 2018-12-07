#include "pinocchio/multibody/model.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"

int main(int argc, char ** argv)
{
  std::string filename = (argc<=1) ? "ur5.urdf" : argv[1];
  pinocchio::Model model;
  pinocchio::urdf::buildModel(filename,model);
  pinocchio::Data data(model);
  Eigen::VectorXd q = pinocchio::randomConfiguration(model);
  std::cout << "q = " << q.transpose() << std::endl;

  pinocchio::forwardKinematics(model,data,q);

  for (int k=0 ; k<model.njoints ; ++k)
    std::cout << model.names[k] << "\t: "
              << data.oMi[k].translation().transpose() << std::endl;
}
