#ifdef NDEBUG
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
#endif

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/center-of-mass.hpp"
#include "pinocchio/tools/timer.hpp"
#include "pinocchio/multibody/parser/sample-models.hpp"

#include <iostream>
#include <boost/utility/binary.hpp>

void timings(const se3::Model & model, se3::Data& data, long flag)
{
  using namespace se3;
  StackTicToc timer(StackTicToc::US); 
#ifdef NDEBUG
  const int NBT = 1000*1000;
#else 
  const int NBT = 1;
  std::cout << "(the time score in debug mode is not relevant)  " ;
#endif

  bool verbose = flag & (flag-1) ; // True is two or more binaries of the flag are 1.
  if(verbose) std::cout <<"--" << std::endl;
  Eigen::VectorXd q = Eigen::VectorXd::Zero(model.nq);

  if( flag >> 0 & 1 )
    {
      timer.tic();
      SMOOTH(NBT)
      {
	centerOfMass(model,data,q);
      }
      if(verbose) std::cout << "COM =\t";
      timer.toc(std::cout,NBT);
    }
  if( flag >> 1 & 1 )
    {
      timer.tic();
      SMOOTH(NBT)
      {
	centerOfMass(model,data,q,false);
      }
      if(verbose) std::cout << "Without sub-tree =\t";
      timer.toc(std::cout,NBT);
    }
  if( flag >> 2 & 1 )
    {
      timer.tic();
      SMOOTH(NBT)
      {
	jacobianCenterOfMass(model,data,q);
      }
      if(verbose) std::cout << "Jcom =\t";
      timer.toc(std::cout,NBT);
    }
}

void assertValues(const se3::Model & model, se3::Data& data)
{
  using namespace Eigen;
  using namespace se3;

  VectorXd q = VectorXd::Zero(model.nq);
  data.M.fill(0);  crba(model,data,q);

  { /* Test COM against CRBA*/
    Vector3d com = centerOfMass(model,data,q);
    assert( data.com[0].isApprox( getComFromCrba(model,data) ));
  }

  { /* Test COM against Jcom (both use different way of compute the COM. */
    Vector3d com = centerOfMass(model,data,q);
    jacobianCenterOfMass(model,data,q);
    assert(com.isApprox(data.com[0]));
  }

  { /* Test Jcom against CRBA  */
    Eigen::MatrixXd Jcom = jacobianCenterOfMass(model,data,q);
    assert( Jcom.isApprox( getJacobianComFromCrba(model,data) ));
  }

  std::cout << "com = [ " << data.com[0].transpose() << " ];" << std::endl;
  std::cout << "mass = [ " << data.mass[0] << " ];" << std::endl;
  std::cout << "Jcom = [ " << data.Jcom << " ];" << std::endl;
  std::cout << "M3 = [ " << data.M.topRows<3>() << " ];" << std::endl;
}
  
int main()
{
  using namespace Eigen;
  using namespace se3;

  se3::Model model;
  se3::buildModels::humanoidSimple(model);
  se3::Data data(model);

  assertValues(model,data);
  timings(model,data,BOOST_BINARY(111));

  return 0;
}
