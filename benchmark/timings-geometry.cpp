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
#include "pinocchio/algorithm/collisions.hpp"
#include "pinocchio/multibody/parser/urdf.hpp"
#include "pinocchio/multibody/parser/sample-models.hpp"


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

#include <iostream>

#include "pinocchio/tools/timer.hpp"

#include <Eigen/StdVector>
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::VectorXd)

int main()
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
    


  std::string romeo_filename = PINOCCHIO_SOURCE_DIR"/models/romeo.urdf";
  std::string romeo_meshDir  = PINOCCHIO_SOURCE_DIR"/models/";
  std::pair < Model, GeometryModel > romeo = se3::urdf::buildModelAndGeom(romeo_filename, romeo_meshDir, se3::JointModelFreeFlyer());
  se3::Model romeo_model = romeo.first;
  se3::GeometryModel romeo_model_geom = romeo.second;
  Data romeo_data(romeo_model);
  GeometryData romeo_data_geom(romeo_data, romeo_model_geom);


  std::vector<VectorXd> qs_romeo     (NBT);
  std::vector<VectorXd> qdots_romeo  (NBT);
  std::vector<VectorXd> qddots_romeo (NBT);
  for(size_t i=0;i<NBT;++i)
  {
    qs_romeo[i]     = Eigen::VectorXd::Random(romeo_model.nq);
    qs_romeo[i].segment<4>(3) /= qs_romeo[i].segment<4>(3).norm();
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
    updateCollisionGeometry(romeo_model,romeo_data,romeo_model_geom,romeo_data_geom,qs_romeo[_smooth], true);
  }
  double update_col_time = timer.toc(StackTicToc::US)/NBT - geom_time;
  std::cout << "Update Collision Geometry < false > = \t" << update_col_time << " " << StackTicToc::unitName(StackTicToc::US) << std::endl;

  timer.tic();
  SMOOTH(NBT)
  {
    updateCollisionGeometry(romeo_model,romeo_data,romeo_model_geom,romeo_data_geom,qs_romeo[_smooth], true);
    for (std::vector<se3::GeometryData::CollisionPair_t>::iterator it = romeo_data_geom.collision_pairs.begin(); it != romeo_data_geom.collision_pairs.end(); ++it)
    {
      romeo_data_geom.collide(it->first, it->second);
    }
  }
  double collideTime = timer.toc(StackTicToc::US)/NBT - (update_col_time + geom_time);
  std::cout << "Collision test between two geometry objects (mean time) = \t" << collideTime / romeo_data_geom.nCollisionPairs
            << StackTicToc::unitName(StackTicToc::US) << std::endl;


  timer.tic();
  SMOOTH(NBT)
  {
    computeCollisions(romeo_model,romeo_data,romeo_model_geom,romeo_data_geom,qs_romeo[_smooth], true);
  }
  double is_colliding_time = timer.toc(StackTicToc::US)/NBT - (update_col_time + geom_time);
  std::cout << "Collision Test : robot in collision? = \t" << is_colliding_time
            << StackTicToc::unitName(StackTicToc::US) << std::endl;


  timer.tic();
  SMOOTH(1000)
  {
    computeDistances(romeo_model,romeo_data,romeo_model_geom,romeo_data_geom,qs_romeo[_smooth]);
  }
  double computeDistancesTime = timer.toc(StackTicToc::US)/1000 - (update_col_time + geom_time);
  std::cout << "Compute distance between two geometry objects (mean time) = \t" << computeDistancesTime / romeo_data_geom.nCollisionPairs
            << " " << StackTicToc::unitName(StackTicToc::US) << " " << romeo_data_geom.nCollisionPairs << " col pairs" << std::endl;



#ifdef WITH_HPP_MODEL_URDF

  std::string filenameHRP2 = PINOCCHIO_SOURCE_DIR"/models/hrp2_14_reduced.urdf";
  std::string meshDirHRP2  = "/local/fvalenza/devel/install/share/";
  std::pair < Model, GeometryModel > hrp2_pino = se3::urdf::buildModelAndGeom(filenameHRP2, meshDirHRP2, se3::JointModelFreeFlyer());

  Model model_hrp2_pino = hrp2_pino.first;
  GeometryModel geometry_model_hrp2_pino = hrp2_pino.second;
  Data data_hrp2_pino(model_hrp2_pino);
  GeometryData data_geom_hrp2_pino(data_hrp2_pino, geometry_model_hrp2_pino);

  const int NBD = 5; // nb of distances test

  std::vector<VectorXd> qs_hrp2_pino     (NBD); 
  std::vector<VectorXd> qdots_hrp2_pino  (NBD); 
  std::vector<VectorXd> qddots_hrp2_pino (NBD); 
  for(size_t i=0;i<NBD;++i)
  {
    qs_hrp2_pino[i]     = Eigen::VectorXd::Random(model_hrp2_pino.nq);
    qs_hrp2_pino[i].segment<4>(3) /= qs_hrp2_pino[i].segment<4>(3).norm();
    qdots_hrp2_pino[i]  = Eigen::VectorXd::Random(model_hrp2_pino.nv);
    qddots_hrp2_pino[i] = Eigen::VectorXd::Random(model_hrp2_pino.nv);
  }
  std::vector<VectorXd> qs_hrp2_hpp     (qs_hrp2_pino);
  std::vector<VectorXd> qdots_hrp2_hpp  (qdots_hrp2_pino);
  std::vector<VectorXd> qddots_hrp2_hpp (qddots_hrp2_pino);

  for (size_t i = 0; i < NBD; ++i)
  {
    Vector4d quaternion;
    quaternion <<  qs_hrp2_pino[i][6], qs_hrp2_pino[i][3], qs_hrp2_pino[i][4], qs_hrp2_pino[i][5];
    qs_hrp2_hpp[i].segment<4>(3) = quaternion ;
  }
  // q_pino <<   0.7832759914634781, -0.6287488328607549, 0.6556417293694411, 0.0, 0.0, 0.9708378748503661, 0.23973698245374014, -0.05964761341484225, -0.07981055048413252, 0.16590322948375874, -0.004144442702387867, 0.26483964517748976, 0.26192191007679017, -0.050295369867256, -0.37034363990486474, 0.02150198818252944, 0.1266227625588889, 0.38, -0.38, 0.38, -0.38, 0.38, -0.38, -0.3899134972725518, -0.5925498643189667, -0.19613681199200853, -1.2401320912584846, 0.2763998757815979, -0.027768398868772835, 0.75, -0.75, 0.75, -0.75, 0.75, -0.75, 0.031489444476891226, -0.012849646226002981;
  // q_hpp  <<   0.7832759914634781, -0.6287488328607549, 0.6556417293694411, 0.23973698245374014, 0.0, 0.0, 0.9708378748503661, -0.05964761341484225, -0.07981055048413252, 0.16590322948375874, -0.004144442702387867, 0.26483964517748976, 0.26192191007679017, -0.050295369867256, -0.37034363990486474, 0.02150198818252944, 0.1266227625588889, 0.38, -0.38, 0.38, -0.38, 0.38, -0.38, -0.3899134972725518, -0.5925498643189667, -0.19613681199200853, -1.2401320912584846, 0.2763998757815979, -0.027768398868772835, 0.75, -0.75, 0.75, -0.75, 0.75, -0.75, 0.031489444476891226, -0.012849646226002981;




  /// *************  HPP  ************* /// 
  /// ********************************* ///


  hpp::model::HumanoidRobotPtr_t humanoidRobot =
    hpp::model::HumanoidRobot::create ("hrp2_14");
  hpp::model::urdf::loadHumanoidModel(humanoidRobot, "freeflyer",
              "hrp2_14_description", "hrp2_14_reduced",
              "", "");



  timer.tic();
  SMOOTH(NBD)
  {
    computeCollisions(model_hrp2_pino,data_hrp2_pino,geometry_model_hrp2_pino,data_geom_hrp2_pino,qs_hrp2_pino[_smooth], true);
  }
  double is_hrp2_colliding_time_pino = timer.toc(StackTicToc::US)/NBD;
  std::cout << "Pinocchio - Collision Test : update + robot in collision? = \t" << is_hrp2_colliding_time_pino
            << StackTicToc::unitName(StackTicToc::US) << std::endl;

  timer.tic();
  SMOOTH(NBD)
  {
    humanoidRobot->currentConfiguration (qs_hrp2_hpp[_smooth]);
    humanoidRobot->computeForwardKinematics ();
    humanoidRobot->collisionTest();
  }
  double is_hrp2_colliding_time_hpp = timer.toc(StackTicToc::US)/NBD;
  std::cout << "HPP - Collision Test : update + robot in collision? = \t" << is_hrp2_colliding_time_hpp
            << StackTicToc::unitName(StackTicToc::US) << std::endl;


  timer.tic();
  SMOOTH(NBD)
  {
    computeDistances(model_hrp2_pino, data_hrp2_pino, geometry_model_hrp2_pino, data_geom_hrp2_pino, qs_hrp2_pino[_smooth]);
  }
  computeDistancesTime = timer.toc(StackTicToc::US)/NBD ;
  std::cout << "Pinocchio - Update + Compute distances" << data_geom_hrp2_pino.nCollisionPairs << " col pairs\t" << computeDistancesTime 
            << " " << StackTicToc::unitName(StackTicToc::US) << std::endl;

  timer.tic();
  SMOOTH(NBD)
  {
    humanoidRobot->currentConfiguration (qs_hrp2_hpp[_smooth]);
    humanoidRobot->computeForwardKinematics ();
    humanoidRobot->computeDistances ();
  }
  double hpp_compute_distances = timer.toc(StackTicToc::US)/NBD ;
  std::cout << "HPP - Update + Compute distances " << humanoidRobot->distanceResults().size() << " col pairs\t" << hpp_compute_distances 
            << " " << StackTicToc::unitName(StackTicToc::US) << std::endl;

#endif

  return 0;
}
