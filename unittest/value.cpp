/*
 * Compare the value obtained with the RNEA with the values obtained from
 * RBDL. The test is not complete. It only validates the RNEA for the revolute
 * joints. The free-flyer is not tested. It should be extended to account for
 * the free flyer and for the other algorithms.
 *
 * Additionnal notes: the RNEA is an algorithm that can be used to validate
 * many others (in particular, the mass matrix (CRBA) can be numerically
 * validated from the RNEA, then the center-of-mass jacobian can be validated
 * from the mass matrix, etc.
 *
 */

#include <iostream>
#include <iomanip>

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/parser/urdf.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/kinematics.hpp"

int main(int argc, const char**argv)
{
  std::string filename = PINOCCHIO_SOURCE_DIR"/models/simple_humanoid.urdf";
  if(argc>1) filename = argv[1];

  se3::Model model = se3::urdf::buildModel(filename,true);
  model.gravity.linear( Eigen::Vector3d(0,0,-9.8));
  se3::Data data(model);

  
  Eigen::VectorXd q = Eigen::VectorXd::Zero(model.nq);
  Eigen::VectorXd v = Eigen::VectorXd::Zero(model.nv);
  Eigen::VectorXd a = Eigen::VectorXd::Zero(model.nv);
 
  std::cout << std::setprecision(10);

  /* --- Test # 000 --- */
  {
    Eigen::VectorXd expected(model.nv);
    expected <<   0, 0, 1281.84, 0, -40.5132, 0, 4.4492, -1.5386, 0, -1.5386, -1.5386, 0, -4.4492, -1.5386, 0, -1.5386, -1.5386, 0, -37.436, 0, 0, -2.548, 0, 0, 0.392, 0, 0.392, 0, -2.548, 0, 0, 0.392, 0, 0.392, 0 ;
    q = Eigen::VectorXd::Zero(model.nq);
    v = Eigen::VectorXd::Zero(model.nv);
    a = Eigen::VectorXd::Zero(model.nv);
    rnea(model,data,q,v,a);
    assert( expected.isApprox(data.tau) && "Test # 000 failed" );
  }

  /* --- Test #  0V0--- */
  {
    Eigen::VectorXd expected(model.nv);
    expected <<  -48.10636, -73.218816, 1384.901025, -7.2292939, -186.342371, -20.6685852, -0.78887946, -1.869651075, 1.8889752, -2.036768175, -2.105948175, -1.023232, -18.3738505, -4.133954895, 9.0456456, -4.276615035, -4.143427035, -2.534896, -180.338765, 15.71570676, -29.8639164, -59.917862, -2.3307916, -2.7648728, -45.76782776, 3.4151272, -18.4320456, -4.5768072, -76.60945104, -0.5897908, -2.640844, -63.93417064, 5.359156, -25.8309196, -6.976116;
 q = Eigen::VectorXd::Zero(model.nq);
    for(int i=6;i<model.nv;++i) v[i] = i/10.;
    a = Eigen::VectorXd::Zero(model.nv);
    rnea(model,data,q,v,a);
    //std::cout << (expected-data.tau).norm() << std::endl;
    assert( expected.isApprox(data.tau,1e-6) && "Test # 0V0 failed" );
  }

  /* --- Test # 0VA--- */
  {
    Eigen::VectorXd expected(model.nv);
    expected << -15.3331636, -0.67891816, 1273.80521, 102.9435113, 110.3509945, 81.52296995, 13.31476408, 14.26606068, 3.682505252, 9.048274318, 4.663303518, 2.05308568, 12.54347834, 25.92680911, 6.327105656, 16.71123385, 8.96650473, 3.54200704, 70.15812475, 77.02410963, 73.81994844, 41.73185754, 28.75786872, 28.94251127, 31.65847724, 20.40431127, 18.18579154, 6.838471928, 50.44193173, 34.07362801, 34.53507156, 38.33983417, 24.61507156, 22.2842788, 8.23435884;
    q = Eigen::VectorXd::Zero(model.nq);
    for(int i=6;i<model.nv;++i) v[i] = i/100.;
    for(int i=6;i<model.nv;++i) a[i] = i/10.;
    rnea(model,data,q,v,a);
    assert( expected.isApprox(data.tau,1e-6) && "Test # 0VA failed");
  }

  /* --- Test # Q00 --- */
  {
    Eigen::VectorXd expected(model.nv);
    expected << 5.367234435e-15, -2.587860481e-14, 1281.84, -133.3062501, 198.975587, -7.120345979e-16, 15.06850407, 58.39287139, -22.14971864, 17.14327289, 1.291543104, 0.7402017048, -4.386231387, 22.73949408, -19.01681794, 0.8839600793, -0.3197599308, -0.466827706, 65.47086697, 32.71449398, -4.250622066, -0.7937685568, -0.15349648, -1.070480752, -3.066302263, -0.3557903212, -0.2183951073, 0.182684221, -0.6648425468, -2.902772493, 0.1250340934, 0.4402877138, -0.3158584741, -0.0865162794, 0.3918733239;
    for(int i=7;i<model.nq;++i) q[i] = (i-1)/10.;
    v = Eigen::VectorXd::Zero(model.nv);
    a = Eigen::VectorXd::Zero(model.nv);
    rnea(model,data,q,v,a);
    assert( expected.isApprox(data.tau,1e-6) && "Test # Q00 failed");
  }

  /* --- Test # QVA --- */
  {
    Eigen::VectorXd expected(model.nv);
    expected <<  -1.911650826, 1.250211406, 1284.82058, -139.0188156, 201.744449, 1.554847332, 15.1910084, 59.27339983, -21.70753738, 17.84339797, 1.754639468, 0.670280632, -2.968778954, 23.0776205, -17.56870284, 1.765761886, 0.2889992363, -0.392159764, 68.83598707, 34.59002827, -4.604435817, -0.3832225891, 1.085231916, -0.348635267, -2.831371037, -1.047616506, -0.228384161, 0.5656880079, 1.302869049, 0.8481280783, 0.7042182131, 1.554751317, -0.3908790552, -0.1294643218, 1.421077555;
    for(int i=7;i<model.nq;++i) q[i] = (i-1)/10.;
    for(int i=6;i<model.nv;++i) v[i] = i/100.;
    for(int i=6;i<model.nv;++i) a[i] = i/100.;
    rnea(model,data,q,v,a);
    assert( expected.isApprox(data.tau,1e-6) && "Test # QVA failed");
  }

  q = Eigen::VectorXd::Zero(model.nq);
  v = Eigen::VectorXd::Zero(model.nv);
  a = Eigen::VectorXd::Zero(model.nv);
  for(int i=7;i<model.nq;++i) q[i] = (i-1)/10.;
 
  Eigen::Vector3d rpy(1,2,3);
  Eigen::Matrix3d R = ( Eigen::AngleAxisd(rpy[0],Eigen::Vector3d::UnitX())
			* Eigen::AngleAxisd(rpy[1],Eigen::Vector3d::UnitY())
			* Eigen::AngleAxisd(rpy[2],Eigen::Vector3d::UnitZ())).matrix();
  q.segment<4>(3) = Eigen::Quaterniond(R).coeffs();


  // std::cout << "R1 = " << ( Eigen::AngleAxisd(rpy[0],Eigen::Vector3d::UnitX())).matrix() << std::endl;
  // std::cout << "R12 = " << ( Eigen::AngleAxisd(rpy[1],Eigen::Vector3d::UnitY())
  // 			     * Eigen::AngleAxisd(rpy[0],Eigen::Vector3d::UnitX())).matrix() << std::endl;


  // std::cout << "R123 = " << R << std::endl;


  //kinematics(model,data,q,v);
  rnea(model,data,q,v,a);

  using namespace Eigen;
  using namespace se3;

  // std::cout << std::setprecision(10);

  // std::cout << "Number of dof : " << model.nv << std::endl;
  // std::cout << "rnea(0,0,0) = g(0) = " << data.tau.transpose() << std::endl;

  // for( int i=0;i<3/*model.nbody*/;++i )
  //   {
  //     if(model.parents[i]!=i-1)
  // 	std::cout << "************** END EFFECTOR" << std::endl;

  //     std::cout << "\n\n === " << i << " ========================" << std::endl;
  //     std::cout << "Joint "<<i<<" = " << model.names[i] << std::endl;
  //     std::cout << "m"<<i<<" = \n" << (SE3::Matrix4)data.oMi[i] << std::endl;
  //     std::cout << "v"<<i<<" = \n" << SE3::Vector6(data.v[i]).transpose()<< std::endl;
  //     std::cout << "a"<<i<<" = \n" << SE3::Vector6(data.a[i]).transpose() << std::endl;
  //     std::cout << "f"<<i<<" = \n" << SE3::Vector6(data.f[i]).transpose() << std::endl;
  //   }

  return 0;
}
