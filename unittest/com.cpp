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
      if(verbose) std::cout << "Wo stree =\t";
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
  crba(model,data,q);

  { /* Test COM against CRBA*/
    centerOfMass(model,data,q);
    assert( data.com[0].isApprox( getComFromCrba(model,data) ));

  }

  { /* Test COM against Jcom (both use different way of compute the COM. */
    Vector3d com = data.com[0];
    data.M.fill(0);  crba(model,data,q);
    jacobianCenterOfMass(model,data,q);
    assert(com.isApprox(jacobianCenterOfMass(model,data,q)));
  }

  { /* Test Jcom against CRBA  */
    assert(data.Jcom.isApprox(data.M.block(0,0,3,model.nv)/data.mass[0] ));
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

#ifndef NDEBUG 
  assertValues(model,data);
#else
  timings(model,data,BOOST_BINARY(101));
#endif

  return 0;
}
