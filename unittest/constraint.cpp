#include <iostream>

#include "pinocchio/spatial/force.hpp"
#include "pinocchio/spatial/motion.hpp"
#include "pinocchio/spatial/se3.hpp"
#include "pinocchio/spatial/inertia.hpp"
#include "pinocchio/multibody/force-set.hpp"
#include "pinocchio/multibody/joint/joint-revolute.hpp"

bool testForceSet()
{
  using namespace se3;
  typedef Eigen::Matrix<double,4,4> Matrix4;
  typedef SE3::Matrix6 Matrix6;
  typedef SE3::Vector3 Vector3;
  typedef Force::Vector6 Vector6;

  SE3 amb = SE3::Random();
  SE3 bmc = SE3::Random();
  SE3 amc = amb*bmc;

  ForceSet F(12);
  ForceSet F2(Eigen::Matrix<double,3,2>::Zero(),Eigen::Matrix<double,3,2>::Zero());
  F.block(10,2) = F2;
  assert( F.matrix().col(10).norm() == 0.0 );
  assert( isnan(F.matrix()(0,9)) );

  std::cout << "F10 = " << F2.matrix() << std::endl;
  std::cout << "F = " << F.matrix() << std::endl;

  ForceSet F3(Eigen::Matrix<double,3,12>::Random(),Eigen::Matrix<double,3,12>::Random());
  ForceSet F4 = amb.act(F3);
  SE3::Matrix6 aXb= amb;
  assert( (aXb.transpose().inverse()*F3.matrix()).isApprox(F4.matrix()) );

  ForceSet bF = bmc.act(F3);
  ForceSet aF = amb.act(bF); 
  ForceSet aF2 = amc.act(F3);
  assert( aF.matrix().isApprox( aF2.matrix() ) );

  ForceSet F36 = amb.act(F3.block(3,6));
  assert( (aXb.transpose().inverse()*F3.matrix().block(0,3,6,6)).isApprox(F36.matrix()) );
  
  ForceSet F36full(12); F36full.block(3,6) = amb.act(F3.block(3,6)); 
  assert( (aXb.transpose().inverse()*F3.matrix().block(0,3,6,6))
	  .isApprox(F36full.matrix().block(0,3,6,6)) );

  return true;
}

bool testConstraintRX()
{
  using namespace se3;

  Inertia Y = Inertia::Random();
  JointRX::ConstraintRevolute S;

  std::cout << "Y = \n" << Y.matrix() << std::endl;
  std::cout << "S = \n" << ((ConstraintXd)S).matrix() << std::endl;

  ForceSet F(1); F.block(0,1) = Y*S;
  std::cout << "Y*S = \n" << (Y*S).matrix() << std::endl;
  std::cout << "F=Y*S = \n" << F.matrix() << std::endl;
  assert( F.matrix().isApprox( Y.toMatrix().col(3) ) );

  ForceSet F2( Eigen::Matrix<double,3,9>::Random(),Eigen::Matrix<double,3,9>::Random() );
  Eigen::MatrixXd StF2 = S.transpose()*F2.block(5,3);
  assert( StF2.isApprox( ConstraintXd(S).matrix().transpose()*F2.matrix().block(0,5,6,3) ) );

  return true;
}

int main()
{
  testForceSet();
  testConstraintRX();
  return 1;
}

