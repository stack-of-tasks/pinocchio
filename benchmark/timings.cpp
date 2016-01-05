#include "pinocchio/spatial/fwd.hpp"
#include "pinocchio/spatial/se3.hpp"
#include "pinocchio/multibody/joint.hpp"
#include "pinocchio/multibody/visitor.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/non-linear-effects.hpp"
#include "pinocchio/algorithm/cholesky.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/center-of-mass.hpp"
#include "pinocchio/simulation/compute-all-terms.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/multibody/parser/urdf.hpp"
#include "pinocchio/multibody/parser/sample-models.hpp"

#ifdef WITH_HPP_FCL
#include "pinocchio/multibody/geometry.hpp"
#include "pinocchio/multibody/parser/urdf-with-geometry.hpp"
  #ifdef WITH_HPP_MODEL_URDF
  #include <hpp/util/debug.hh>
  #include <hpp/model/device.hh>
  #include <hpp/model/body.hh>
  #include <hpp/model/collision-object.hh>
  #include <hpp/model/joint.hh>
  #include <hpp/model/urdf/util.hh>
  #endif
#endif

#include <iostream>

#include "pinocchio/tools/timer.hpp"

#include <Eigen/StdVector>
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::VectorXd)

int main(int argc, const char ** argv)
{
  using namespace Eigen;
  using namespace se3;

  StackTicToc timer(StackTicToc::US);
  #ifdef NDEBUG
  const int NBT = 1000*100;
  #else
    const int NBT = 1;
    std::cout << "(the time score in debug mode is not relevant) " << std::endl;
  #endif
    
  se3::Model model;

  std::string filename = PINOCCHIO_SOURCE_DIR"/models/simple_humanoid.urdf";
  if(argc>1) filename = argv[1];
  if( filename == "HS") 
    se3::buildModels::humanoidSimple(model,true);
  else if( filename == "H2" )
    se3::buildModels::humanoid2d(model);
  else
    model = se3::urdf::buildModel(filename,JointModelFreeFlyer());
  std::cout << "nq = " << model.nq << std::endl;

  se3::Data data(model);
  VectorXd q = VectorXd::Random(model.nq);
  VectorXd qdot = VectorXd::Random(model.nv);
  VectorXd qddot = VectorXd::Random(model.nv);

  std::vector<VectorXd> qs     (NBT);
  std::vector<VectorXd> qdots  (NBT);
  std::vector<VectorXd> qddots (NBT);
  for(size_t i=0;i<NBT;++i)
    {
      qs[i]     = Eigen::VectorXd::Random(model.nq);
      qs[i].segment<4>(3) /= qs[i].segment<4>(3).norm();
      qdots[i]  = Eigen::VectorXd::Random(model.nv);
      qddots[i] = Eigen::VectorXd::Random(model.nv);
    }

 
  timer.tic();
  SMOOTH(NBT)
    {
      rnea(model,data,qs[_smooth],qdots[_smooth],qddots[_smooth]);
    }
  std::cout << "RNEA = \t\t"; timer.toc(std::cout,NBT);

  timer.tic();
  SMOOTH(NBT)
  {
    nonLinearEffects(model,data,qs[_smooth],qdots[_smooth]);
  }
  std::cout << "NLE = \t\t"; timer.toc(std::cout,NBT);

  timer.tic();
  SMOOTH(NBT)
  {
    rnea(model,data,qs[_smooth],qdots[_smooth],Eigen::VectorXd::Zero(model.nv));
  }
  std::cout << "NLE via RNEA = \t\t"; timer.toc(std::cout,NBT);
 
  timer.tic();
  SMOOTH(NBT)
    {
      crba(model,data,qs[_smooth]);
    }
  std::cout << "CRBA = \t\t"; timer.toc(std::cout,NBT);

  timer.tic();
  SMOOTH(NBT)
  {
    computeAllTerms(model,data,qs[_smooth],qdots[_smooth]);
  }
  std::cout << "computeAllTerms = \t\t"; timer.toc(std::cout,NBT);
  
  double total = 0;
  SMOOTH(NBT)
    {
      crba(model,data,qs[_smooth]);
      timer.tic();
      cholesky::decompose(model,data);
      total += timer.toc(timer.DEFAULT_UNIT);
    }
  std::cout << "Cholesky = \t" << (total/NBT) 
	    << " " << timer.unitName(timer.DEFAULT_UNIT) <<std::endl;
 
  timer.tic();
  SMOOTH(NBT)
    {
      computeJacobians(model,data,qs[_smooth]);
    }
  std::cout << "Jacobian = \t"; timer.toc(std::cout,NBT);

  timer.tic();
  SMOOTH(NBT)
    {
      jacobianCenterOfMass(model,data,qs[_smooth],false);
    }
  std::cout << "COM+Jcom = \t"; timer.toc(std::cout,NBT);
  
  timer.tic();
  SMOOTH(NBT)
  {
    centerOfMassAcceleration(model,data,qs[_smooth], qdots[_smooth], qddots[_smooth], false);
  }
  std::cout << "COM+vCOM+aCOM = \t"; timer.toc(std::cout,NBT);

  timer.tic();
  SMOOTH(NBT)
  {
    geometry(model,data,qs[_smooth]);
  }
  std::cout << "Geometry = \t"; timer.toc(std::cout,NBT);

#ifdef WITH_HPP_FCL
  std::string romeo_filename = PINOCCHIO_SOURCE_DIR"/models/romeo.urdf";
  std::string romeo_meshDir  = PINOCCHIO_SOURCE_DIR"/models/";
  std::pair < Model, GeometryModel > romeo = se3::urdf::buildModelAndGeom(romeo_filename, romeo_meshDir, se3::JointModelFreeFlyer());
  se3::Model romeo_model = romeo.first;
  se3::GeometryModel romeo_model_geom = romeo.second;
  Data romeo_data(romeo_model);
  GeometryData romeo_data_geom(romeo_data, romeo_model_geom);

  VectorXd q_romeo = VectorXd::Random(romeo_model.nq);
  VectorXd qdot_romeo = VectorXd::Random(romeo_model.nv);
  VectorXd qddot_romeo = VectorXd::Random(romeo_model.nv);

  std::vector<VectorXd> qs_romeo     (NBT);
  std::vector<VectorXd> qdots_romeo  (NBT);
  std::vector<VectorXd> qddots_romeo (NBT);
  for(size_t i=0;i<NBT;++i)
  {
    qs_romeo[i]     = Eigen::VectorXd::Random(romeo_model.nq);
    qs_romeo[i].segment<4>(3) /= qs[i].segment<4>(3).norm();
    qdots_romeo[i]  = Eigen::VectorXd::Random(romeo_model.nv);
    qddots_romeo[i] = Eigen::VectorXd::Random(romeo_model.nv);
  }

  timer.tic();
  SMOOTH(NBT)
  {
    geometry(romeo_model,romeo_data,qs_romeo[_smooth]);
  }
  double geom_time = timer.toc(StackTicToc::US)/NBT;

  timer.tic();
  SMOOTH(NBT)
  {
    se3::geometry(romeo_model, romeo_data, qs_romeo[_smooth]);
    updateCollisionGeometry(romeo_model,romeo_data,romeo_model_geom,romeo_data_geom,qs_romeo[_smooth]);
  }
  std::cout << "Update Collision Geometry = \t";
  std::cout << timer.toc(StackTicToc::US)/NBT - geom_time
                       << StackTicToc::unitName(StackTicToc::US) << std::endl;

  timer.tic();
  SMOOTH(NBT)
  {
    romeo_data_geom.collide(1,10);
  }
  std::cout << "Collision Test between two geometry objects = \t"; timer.toc(std::cout,NBT);

  timer.tic();
  SMOOTH(NBT)
  {
    romeo_data_geom.isColliding();
  }
  std::cout << "Collision Test : is the robot colliding ? = \t"; timer.toc(std::cout,NBT);


  timer.tic();
  SMOOTH(1000)
  {
    romeo_data_geom.computeDistance(1, 10);
  }
  std::cout << "Compute Distance between two geometry objects = \t"; timer.toc(std::cout,1000);

  timer.tic();
  SMOOTH(1000)
  {
    romeo_data_geom.computeDistances();
  }
  std::cout << "Compute Distances for all collision pairs = \t"; timer.toc(std::cout,1000);

  #ifdef WITH_HPP_MODEL_URDF


  std::string filenameHRP2 = PINOCCHIO_SOURCE_DIR"/models/hrp2_14_reduced.urdf";
  std::string meshDirHRP2  = "/local/fvalenza/devel/install/share/";
  std::pair < Model, GeometryModel > robotHRP2 = se3::urdf::buildModelAndGeom(filenameHRP2, meshDirHRP2, se3::JointModelFreeFlyer());

  Data dataHRP2(robotHRP2.first);
  GeometryData data_geomHRP2(dataHRP2, robotHRP2.second);

  // Configuration to be tested
  Eigen::VectorXd q_pino(robotHRP2.first.nq);
  Eigen::VectorXd q_hpp(robotHRP2.first.nq);
  q_pino <<   0.7832759914634781, -0.6287488328607549, 0.6556417293694411, 0.0, 0.0, 0.9708378748503661, 0.23973698245374014, -0.05964761341484225, -0.07981055048413252, 0.16590322948375874, -0.004144442702387867, 0.26483964517748976, 0.26192191007679017, -0.050295369867256, -0.37034363990486474, 0.02150198818252944, 0.1266227625588889, 0.38, -0.38, 0.38, -0.38, 0.38, -0.38, -0.3899134972725518, -0.5925498643189667, -0.19613681199200853, -1.2401320912584846, 0.2763998757815979, -0.027768398868772835, 0.75, -0.75, 0.75, -0.75, 0.75, -0.75, 0.031489444476891226, -0.012849646226002981;
  q_hpp  <<   0.7832759914634781, -0.6287488328607549, 0.6556417293694411, 0.23973698245374014, 0.0, 0.0, 0.9708378748503661, -0.05964761341484225, -0.07981055048413252, 0.16590322948375874, -0.004144442702387867, 0.26483964517748976, 0.26192191007679017, -0.050295369867256, -0.37034363990486474, 0.02150198818252944, 0.1266227625588889, 0.38, -0.38, 0.38, -0.38, 0.38, -0.38, -0.3899134972725518, -0.5925498643189667, -0.19613681199200853, -1.2401320912584846, 0.2763998757815979, -0.027768398868772835, 0.75, -0.75, 0.75, -0.75, 0.75, -0.75, 0.031489444476891226, -0.012849646226002981;


  se3::geometry(robotHRP2.first, dataHRP2, q_pino);
  se3::updateCollisionGeometry(robotHRP2.first, dataHRP2, robotHRP2.second, data_geomHRP2, q_pino);


  /// *************  HPP  ************* /// 
  /// ********************************* ///


  hpp::model::HumanoidRobotPtr_t humanoidRobot =
    hpp::model::HumanoidRobot::create ("hrp2_14");
  hpp::model::urdf::loadHumanoidModel(humanoidRobot, "freeflyer",
              "hrp2_14_description", "hrp2_14_reduced",
              "", "");



  humanoidRobot->currentConfiguration (q_hpp);
  humanoidRobot->computeForwardKinematics ();


  timer.tic();
  SMOOTH(5)
  {
    humanoidRobot->computeDistances ();
  }
  std::cout << "Compute Distances HPP - disabled pairs by srdf = \t"; timer.toc(std::cout,5);


  #endif
#endif

  timer.tic();
  SMOOTH(NBT)
  {
    kinematics(model,data,qs[_smooth],qdots[_smooth]);
  }
  std::cout << "Kinematics = \t"; timer.toc(std::cout,NBT);
  
  timer.tic();
  SMOOTH(NBT)
  {
    dynamics(model,data,qs[_smooth],qdots[_smooth], qddots[_smooth]);
  }
  std::cout << "Dynamics = \t"; timer.toc(std::cout,NBT);

  std::cout << "--" << std::endl;
  return 0;
}
