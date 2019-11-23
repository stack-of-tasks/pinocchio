//
// Copyright (c) 2015-2019 CNRS INRIA
//

/*
 * Test the CRBA algorithm. The code validates both the computation times and
 * the value by comparing the results of the CRBA with the reconstruction of
 * the mass matrix using the RNEA.
 * For a strong timing benchmark, see benchmark/timings.
 *
 */

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/centroidal.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/center-of-mass.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/parsers/sample-models.hpp"
#include "pinocchio/utils/timer.hpp"

#include <iostream>

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

template<typename JointModel>
static void addJointAndBody(pinocchio::Model & model,
                            const pinocchio::JointModelBase<JointModel> & joint,
                            const std::string & parent_name,
                            const std::string & name,
                            const pinocchio::SE3 placement = pinocchio::SE3::Random(),
                            bool setRandomLimits = true)
{
  using namespace pinocchio;
  typedef typename JointModel::ConfigVector_t CV;
  typedef typename JointModel::TangentVector_t TV;
  
  Model::JointIndex idx;
  
  if (setRandomLimits)
    idx = model.addJoint(model.getJointId(parent_name),joint,
                         SE3::Random(),
                         name + "_joint",
                         TV::Random() + TV::Constant(1),
                         TV::Random() + TV::Constant(1),
                         CV::Random() - CV::Constant(1),
                         CV::Random() + CV::Constant(1)
                         );
  else
    idx = model.addJoint(model.getJointId(parent_name),joint,
                         placement, name + "_joint");
  
  model.addJointFrame(idx);
  
  model.appendBodyToJoint(idx,Inertia::Random(),SE3::Identity());
  model.addBodyFrame(name + "_body", idx);
}

BOOST_AUTO_TEST_SUITE ( BOOST_TEST_MODULE )

BOOST_AUTO_TEST_CASE ( test_crba )
{
  pinocchio::Model model;
  pinocchio::buildModels::humanoidRandom(model);
  pinocchio::Data data(model);
  
  #ifdef NDEBUG
    #ifdef _INTENSE_TESTING_
      const size_t NBT = 1000*1000;
    #else
      const size_t NBT = 10;
    #endif

    Eigen::VectorXd q = Eigen::VectorXd::Zero(model.nq);
 
    PinocchioTicToc timer(PinocchioTicToc::US); timer.tic();
    SMOOTH(NBT)
      {
        crba(model,data,q);
      }
    timer.toc(std::cout,NBT);
  
  #else
    const size_t NBT = 1;
    using namespace Eigen;
    using namespace pinocchio;

    Eigen::MatrixXd M(model.nv,model.nv);
    Eigen::VectorXd q = Eigen::VectorXd::Ones(model.nq);
    q.segment <4> (3).normalize();
    Eigen::VectorXd v = Eigen::VectorXd::Zero(model.nv);
    Eigen::VectorXd a = Eigen::VectorXd::Zero(model.nv);
    data.M.fill(0);  crba(model,data,q);
    data.M.triangularView<Eigen::StrictlyLower>() = data.M.transpose().triangularView<Eigen::StrictlyLower>();

    /* Joint inertia from iterative crba. */
    const Eigen::VectorXd bias = rnea(model,data,q,v,a);
    for(int i=0;i<model.nv;++i)
    {
      M.col(i) = rnea(model,data,q,v,Eigen::VectorXd::Unit(model.nv,i)) - bias;
    }

    // std::cout << "Mcrb = [ " << data.M << "  ];" << std::endl;
    // std::cout << "Mrne = [  " << M << " ]; " << std::endl;
    BOOST_CHECK(M.isApprox(data.M,1e-12));
    
    std::cout << "(the time score in debug mode is not relevant)  " ;
    
    q = Eigen::VectorXd::Zero(model.nq);
   
    PinocchioTicToc timer(PinocchioTicToc::US); timer.tic();
    SMOOTH(NBT)
    {
      crba(model,data,q);
    }
    timer.toc(std::cout,NBT);
  
  #endif // ifndef NDEBUG

}
  
BOOST_AUTO_TEST_CASE(test_minimal_crba)
{
  pinocchio::Model model;
  pinocchio::buildModels::humanoidRandom(model);
  pinocchio::Data data(model), data_ref(model);
  
  model.lowerPositionLimit.head<7>().fill(-1.);
  model.upperPositionLimit.head<7>().fill(1.);
  
  Eigen::VectorXd q = randomConfiguration(model,model.lowerPositionLimit,model.upperPositionLimit);
  Eigen::VectorXd v(Eigen::VectorXd::Random(model.nv));
  
  crba(model,data_ref,q);
  data_ref.M.triangularView<Eigen::StrictlyLower>() = data_ref.M.transpose().triangularView<Eigen::StrictlyLower>();
  
  crbaMinimal(model,data,q);
  data.M.triangularView<Eigen::StrictlyLower>() = data.M.transpose().triangularView<Eigen::StrictlyLower>();
  
  BOOST_CHECK(data.M.isApprox(data_ref.M));
  
  ccrba(model,data_ref,q,v);
  computeJointJacobians(model,data_ref,q);
  BOOST_CHECK(data.Ag.isApprox(data_ref.Ag));
  BOOST_CHECK(data.J.isApprox(data_ref.J));
  
}

BOOST_AUTO_TEST_SUITE_END ()

