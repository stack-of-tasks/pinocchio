//
// Copyright (c) 2016 CNRS
//
// This file is part of Pinocchio
// Pinocchio is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// Pinocchio is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// General Lesser Public License for more details. You should have
// received a copy of the GNU Lesser General Public License along with
// Pinocchio If not, see
// <http://www.gnu.org/licenses/>.

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/compute-all-terms.hpp"
#include "pinocchio/spatial/act-on-set.hpp"
#include "pinocchio/parsers/sample-models.hpp"
#include "pinocchio/tools/timer.hpp"

#include "pinocchio/multibody/joint/joint.hpp"

#include <iostream>

#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE JointTest
#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

template <typename T>
void test_joint_methods (T & jmodel, typename T::JointDataDerived & jdata)
{
    Eigen::VectorXd q1(Eigen::VectorXd::Random (jmodel.nq()));
    Eigen::VectorXd q1_dot(Eigen::VectorXd::Random (jmodel.nv()));
    Eigen::VectorXd q2(Eigen::VectorXd::Random (jmodel.nq()));
    double u = 0.3;
    se3::Inertia::Matrix6 Ia(se3::Inertia::Random().matrix());
    bool update_I = false;

    if(T::shortname() == "JointModelRUBX" ||
       T::shortname() == "JointModelRUBY" ||
       T::shortname() == "JointModelRUBZ")
    {
      // normalize cos/sin
      q1.normalize();
      q2.normalize();
    }

    jmodel.calc(jdata, q1, q1_dot);
    jmodel.calc_aba(jdata, Ia, update_I);



    se3::JointModel jma(jmodel);
    se3::JointData jda(jdata);

                                                            
    jma.calc(jda, q1, q1_dot);
    jma.calc_aba(jda, Ia, update_I); 

    std::string error_prefix("JointModel on " + T::shortname());
    BOOST_CHECK_MESSAGE(jmodel.nq() == jma.nq() ,std::string(error_prefix + " - nq "));
    BOOST_CHECK_MESSAGE(jmodel.nv() == jma.nv() ,std::string(error_prefix + " - nv "));

    BOOST_CHECK_MESSAGE(jmodel.idx_q() == jma.idx_q() ,std::string(error_prefix + " - Idx_q "));
    BOOST_CHECK_MESSAGE(jmodel.idx_v() == jma.idx_v() ,std::string(error_prefix + " - Idx_v "));
    BOOST_CHECK_MESSAGE(jmodel.id() == jma.id() ,std::string(error_prefix + " - JointId "));

    BOOST_CHECK_MESSAGE(jmodel.integrate(q1,q1_dot).isApprox(jma.integrate(q1,q1_dot)) ,std::string(error_prefix + " - integrate "));
    BOOST_CHECK_MESSAGE(jmodel.interpolate(q1,q2,u).isApprox(jma.interpolate(q1,q2,u)) ,std::string(error_prefix + " - interpolate "));
    BOOST_CHECK_MESSAGE(jmodel.randomConfiguration( -1 * Eigen::VectorXd::Ones(jmodel.nq()),
                                                Eigen::VectorXd::Ones(jmodel.nq())).size()
                                    == jma.randomConfiguration(-1 * Eigen::VectorXd::Ones(jma.nq()),
                                                              Eigen::VectorXd::Ones(jma.nq())).size()
                        ,std::string(error_prefix + " - RandomConfiguration dimensions "));
    BOOST_CHECK_MESSAGE(jmodel.difference(q1,q2).isApprox(jma.difference(q1,q2)) ,std::string(error_prefix + " - difference "));
    BOOST_CHECK_MESSAGE(jmodel.distance(q1,q2) == jma.distance(q1,q2) ,std::string(error_prefix + " - distance "));


    BOOST_CHECK_MESSAGE((jda.S().matrix()).isApprox((((se3::ConstraintXd)jdata.S).matrix())),std::string(error_prefix + " - ConstraintXd "));
    BOOST_CHECK_MESSAGE((jda.M()) == (jdata.M),std::string(error_prefix + " - Joint transforms ")); // ==  or isApprox ?
    BOOST_CHECK_MESSAGE((jda.v()) == (jdata.v),std::string(error_prefix + " - Joint motions "));
    BOOST_CHECK_MESSAGE((jda.c()) == (jdata.c),std::string(error_prefix + " - Joint bias "));
    
    BOOST_CHECK_MESSAGE((jda.U()).isApprox(jdata.U),std::string(error_prefix + " - Joint U inertia matrix decomposition "));
    BOOST_CHECK_MESSAGE((jda.Dinv()).isApprox(jdata.Dinv),std::string(error_prefix + " - Joint DInv inertia matrix decomposition "));
    BOOST_CHECK_MESSAGE((jda.UDinv()).isApprox(jdata.UDinv),std::string(error_prefix + " - Joint UDInv inertia matrix decomposition "));
}

struct TestJoint{

  template <typename T>
  void operator()(const T ) const
  {
    T jmodel;
    jmodel.setIndexes(0,0,0);
    typename T::JointDataDerived jdata = jmodel.createData();

    test_joint_methods(jmodel, jdata);    
  }

  template <int NQ, int NV>
  void operator()(const se3::JointModelDense<NQ,NV> & ) const
  {
    // JointModelDense will be removed soon
  }

  void operator()(const se3::JointModelRevoluteUnaligned & ) const
  {
    se3::JointModelRevoluteUnaligned jmodel(1.5, 1., 0.);
    jmodel.setIndexes(0,0,0);
    se3::JointModelRevoluteUnaligned::JointDataDerived jdata = jmodel.createData();

    test_joint_methods(jmodel, jdata);
  }

  void operator()(const se3::JointModelPrismaticUnaligned & ) const
  {
    se3::JointModelPrismaticUnaligned jmodel(1.5, 1., 0.);
    jmodel.setIndexes(0,0,0);
    se3::JointModelPrismaticUnaligned::JointDataDerived jdata = jmodel.createData();

    test_joint_methods(jmodel, jdata);
  }

};


BOOST_AUTO_TEST_SUITE ( JointTest)

BOOST_AUTO_TEST_CASE ( test_all_joints )
{
  using namespace Eigen;
  using namespace se3;

  Model model;
  buildModels::humanoidSimple(model);
  se3::Data data(model);


  boost::mpl::for_each<JointModelVariant::types>(TestJoint());


}
BOOST_AUTO_TEST_SUITE_END ()
