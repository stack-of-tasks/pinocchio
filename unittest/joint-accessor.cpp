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
#include "pinocchio/algorithm/operational-frames.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/compute-all-terms.hpp"
#include "pinocchio/spatial/act-on-set.hpp"
#include "pinocchio/multibody/parser/sample-models.hpp"
#include "pinocchio/tools/timer.hpp"

#include "pinocchio/multibody/joint/joint-accessor.hpp"

#include <iostream>

#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE JointAccessorTest
#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

BOOST_AUTO_TEST_SUITE ( JointAccessorTest)

BOOST_AUTO_TEST_CASE ( test_read_model )
{
  using namespace Eigen;
  using namespace se3;

  se3::Model model;
  se3::buildModels::humanoidSimple(model);
  se3::Data data(model);

  Eigen::VectorXd q1(randomConfiguration(model, -1 * Eigen::VectorXd::Ones(model.nq), Eigen::VectorXd::Ones(model.nq) ));
  Eigen::VectorXd q1_dot(Eigen::VectorXd::Random(model.nv));
  Eigen::VectorXd q2(randomConfiguration(model, -1 * Eigen::VectorXd::Ones(model.nq), Eigen::VectorXd::Ones(model.nq) ));
  double u = 0.3;

  for (Model::JointIndex i=1; i < (Model::JointIndex) model.nbody; ++i)
  {  
    const JointModelVariant & jv = model.joints[i];
    JointModelAccessor jma(model.joints[i]);

    BOOST_CHECK_MESSAGE(integrate(jv,q1,q1_dot).isApprox(jma.integrate(q1,q1_dot)) ,"Joint Model Accessor - integrate error");
    BOOST_CHECK_MESSAGE(interpolate(jv,q1,q2,u).isApprox(jma.interpolate(q1,q2,u)) ,"Joint Model Accessor - interpolate error");
    BOOST_CHECK_MESSAGE(randomConfiguration(jv, -1 * Eigen::VectorXd::Ones(nq(jv)),
                                                Eigen::VectorXd::Ones(nq(jv))).size()
                                    == jma.randomConfiguration(-1 * Eigen::VectorXd::Ones(jma.nq()),
                                                              Eigen::VectorXd::Ones(jma.nq())).size()
                        ,"Dimensions of random configuration are not the same");
    BOOST_CHECK_MESSAGE(difference(jv,q1,q2).isApprox(jma.difference(q1,q2)) ,"Joint Model Accessor - difference error");
    BOOST_CHECK_MESSAGE(distance(jv,q1,q2) == jma.distance(q1,q2) ,"Joint Model Accessor - distance error");

    BOOST_CHECK_MESSAGE(nq(jv) == jma.nq() ,"Dimensions nq are not the same");
    BOOST_CHECK_MESSAGE(nv(jv) == jma.nv() ,"Dimensions nv are not the same");

    BOOST_CHECK_MESSAGE(idx_q(jv) == jma.idx_q() ,"Idx_q in model are not the same");
    BOOST_CHECK_MESSAGE(idx_v(jv) == jma.idx_v() ,"Idx_v in model are not the same");
    BOOST_CHECK_MESSAGE(id(jv) == jma.id() ,"Joint id in model are not the same");
  }


  
}

BOOST_AUTO_TEST_CASE ( test_read_data )
{
  using namespace Eigen;
  using namespace se3;

  Model model;
  buildModels::humanoidSimple(model);
  se3::Data data(model);

   
  for (Model::JointIndex i=1; i < (Model::JointIndex) model.nbody; ++i)
  {  
    JointModelAccessor jma(model.joints[i]);
    JointDataAccessor  jda(data.joints[jma.id()]);

    BOOST_CHECK_MESSAGE((jda.S().matrix()).isApprox((constraint_xd(data.joints[jma.id()]).matrix())),"ConstraintXd are not equals");
    BOOST_CHECK_MESSAGE((jda.M()) == (joint_transform(data.joints[jma.id()])),"Joint transforms are not equals");
    // BOOST_CHECK_MESSAGE((jda.v()) == (motion(data.joints[jma.id()])),"Joint motions are not equals"); // Joints not updated, can not be equals                                                                                                               
    BOOST_CHECK_MESSAGE((jda.c()) == (bias(data.joints[jma.id()])),"Joint bias are not equals");
    
    BOOST_CHECK_MESSAGE((jda.U()).isApprox((u_inertia(data.joints[jma.id()]))),"Joint U inertia matrix decomposition are not equals");
    BOOST_CHECK_MESSAGE((jda.Dinv()).isApprox((dinv_inertia(data.joints[jma.id()]))),"Joint DInv inertia matrix decomposition are not equals");
    BOOST_CHECK_MESSAGE((jda.UDinv()).isApprox((udinv_inertia(data.joints[jma.id()]))),"Joint UDInv inertia matrix decomposition are not equals");
  }

  Eigen::VectorXd q(Eigen::VectorXd::Ones (model.nq));
  Eigen::VectorXd v(Eigen::VectorXd::Ones (model.nv));
  computeAllTerms(model, data, q, v);

  for (Model::JointIndex i=1; i < (Model::JointIndex) model.nbody; ++i)
  {  
    JointModelAccessor jma(model.joints[i]);
    JointDataAccessor  jda(data.joints[jma.id()]);

    BOOST_CHECK_MESSAGE((jda.S().matrix()).isApprox((constraint_xd(data.joints[jma.id()]).matrix())),"ConstraintXd are not equals");
    BOOST_CHECK_MESSAGE((jda.M()) == (joint_transform(data.joints[jma.id()])),"Joint transforms are not equals"); // ==  or isApprox ?
    BOOST_CHECK_MESSAGE((jda.v()) == (motion(data.joints[jma.id()])),"Joint motions are not equals");                                                                                                                
    BOOST_CHECK_MESSAGE((jda.c()) == (bias(data.joints[jma.id()])),"Joint bias are not equals");
    
    BOOST_CHECK_MESSAGE((jda.U()).isApprox((u_inertia(data.joints[jma.id()]))),"Joint U inertia matrix decomposition are not equals");
    BOOST_CHECK_MESSAGE((jda.Dinv()).isApprox((dinv_inertia(data.joints[jma.id()]))),"Joint DInv inertia matrix decomposition are not equals");
    BOOST_CHECK_MESSAGE((jda.UDinv()).isApprox((udinv_inertia(data.joints[jma.id()]))),"Joint UDInv inertia matrix decomposition are not equals");
  }

}

BOOST_AUTO_TEST_SUITE_END ()

