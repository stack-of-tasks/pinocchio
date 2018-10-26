//
// Copyright (c) 2016-2018 CNRS
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
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/parsers/sample-models.hpp"
#include "pinocchio/algorithm/finite-differences.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/jacobian.hpp"

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

using namespace se3;
using namespace Eigen;

template<bool local>
Data::Matrix6x finiteDiffJacobian(const Model & model, Data & data, const Eigen::VectorXd & q, const Model::JointIndex joint_id)
{
  Data::Matrix6x res(6,model.nv); res.setZero();
  VectorXd q_integrate (model.nq);
  VectorXd v_integrate (model.nq); v_integrate.setZero();
  
  forwardKinematics(model,data,q);
  const SE3 oMi_ref = data.oMi[joint_id];
  
  const VectorXd fd_increment = finiteDifferenceIncrement(model);
 
  double eps = 1e-8;
  for(int k=0; k<model.nv; ++k)
  {
    // Integrate along kth direction
    eps = fd_increment[k];
    v_integrate[k] = eps;
    q_integrate = integrate(model,q,v_integrate);
    
    forwardKinematics(model,data,q_integrate);
    const SE3 & oMi = data.oMi[joint_id];
    
    if (local)
      res.col(k) = log6(oMi_ref.inverse()*oMi).toVector();
    else
      res.col(k) = oMi_ref.act(log6(oMi_ref.inverse()*oMi)).toVector();
    
    res.col(k) /= eps;
    
    v_integrate[k] = 0.;
  }
  
  return res;
}

template<typename Matrix>
void filterValue(MatrixBase<Matrix> & mat, typename Matrix::Scalar value)
{
  for(int k = 0; k < mat.size(); ++k)
    mat.derived().data()[k] =  std::fabs(mat.derived().data()[k]) <= value?0:mat.derived().data()[k];
}

struct FiniteDiffJoint
{
  template<typename JointModel>
  static void init (JointModelBase<JointModel> & /*jmodel*/) {}
  
  template<typename JointModel>
  void operator()(JointModelBase<JointModel> & jmodel) const
  {
    typedef typename JointModel::ConfigVector_t CV;
    typedef typename JointModel::TangentVector_t TV;
    typedef typename LieGroup<JointModel>::type LieGroupType;
    
    init(jmodel); jmodel.setIndexes(0,0,0);
    typename JointModel::JointDataDerived jdata = jmodel.createData();
    CV q; LieGroupType().random(q);
    jmodel.calc(jdata,q);
    SE3 M_ref(jdata.M);
    
    CV q_int(jmodel.nq());
    TV v(jmodel.nv()); v.setZero();
    double eps = 1e-4;
    
    Eigen::Matrix<double,6,JointModel::NV> S(6,jmodel.nv()), S_ref(jdata.S.matrix());
    
    eps = jmodel.finiteDifferenceIncrement();
    for(int k=0;k<jmodel.nv();++k)
    {
      v[k] = eps;
      q_int = LieGroupType().integrate(q,v);
      jmodel.calc(jdata,q_int);
      SE3 M_int = jdata.M;
      
      S.col(k) = log6(M_ref.inverse()*M_int).toVector();
      S.col(k) /= eps;
      
      v[k] = 0.;
    }
    
    BOOST_CHECK(S.isApprox(S_ref,eps*1e1));
    std::cout << "name: " << jmodel.classname() << std::endl;
    std::cout << "S_ref:\n" << S_ref << std::endl;
    std::cout << "S:\n" << S << std::endl;
  }
};

template<>
void FiniteDiffJoint::init<JointModelRevoluteUnaligned>(JointModelBase<JointModelRevoluteUnaligned> & jmodel)
{
  jmodel.derived().axis.setRandom(); jmodel.derived().axis.normalize();
}

template<>
void FiniteDiffJoint::init<JointModelPrismaticUnaligned>(JointModelBase<JointModelPrismaticUnaligned> & jmodel)
{
  jmodel.derived().axis.setRandom(); jmodel.derived().axis.normalize();
}

template<>
void FiniteDiffJoint::init<JointModelComposite>(JointModelBase<JointModelComposite> & jmodel)
{
  jmodel.derived().addJoint(JointModelRX());
  jmodel.derived().addJoint(JointModelRZ());
}

template<>
void FiniteDiffJoint::operator()< JointModelComposite > (JointModelBase<JointModelComposite> & ) const
{
  // DO NOT CHECK BECAUSE IT IS NOT WORKINK YET - TODO
//  typedef typename JointModel::ConfigVector_t CV;
//  typedef typename JointModel::TangentVector_t TV;
//
//  se3::JointModelComposite jmodel((se3::JointModelRX())/*, (se3::JointModelRY())*/);
//  jmodel.setIndexes(0,0,0);
//
//  se3::JointModelComposite::JointDataDerived jdata = jmodel.createData();
//
//  CV q = jmodel.random();
//  jmodel.calc(jdata,q);
//  SE3 M_ref(jdata.M);
//
//  CV q_int;
//  TV v(Eigen::VectorXd::Random(jmodel.nv())); v.setZero();
//  double eps = 1e-4;
//
//  assert(q.size() == jmodel.nq()&& "nq false");
//  assert(v.size() == jmodel.nv()&& "nv false");
//  Eigen::MatrixXd S(6,jmodel.nv()), S_ref(ConstraintXd(jdata.S).matrix());
//
//  eps = jmodel.finiteDifferenceIncrement();
//  for(int k=0;k<jmodel.nv();++k)
//  {
//    v[k] = eps;
//    q_int = jmodel.integrate(q,v);
//    jmodel.calc(jdata,q_int);
//    SE3 M_int = jdata.M;
//
//    S.col(k) = log6(M_ref.inverse()*M_int).toVector();
//    S.col(k) /= eps;
//
//    v[k] = 0.;
//  }
//
//  std::cout << "S\n" << S << std::endl;
//  std::cout << "S_ref\n" << S_ref << std::endl;
  // BOOST_CHECK(S.isApprox(S_ref,eps*1e1)); //@TODO Uncomment to test once JointComposite maths are ok
}

BOOST_AUTO_TEST_SUITE ( BOOST_TEST_MODULE )

BOOST_AUTO_TEST_CASE(increment)
{
  typedef double Scalar;
  
  Model model;
  buildModels::humanoidRandom(model);
  
  VectorXd fd_increment(model.nv);
  fd_increment = finiteDifferenceIncrement(model);
  
  for(int k=0; k<model.nv; ++k)
  {
    BOOST_CHECK(fd_increment[k] > Eigen::NumTraits<Scalar>::epsilon());
    BOOST_CHECK(fd_increment[k] < 1e-3);
  }
}

BOOST_AUTO_TEST_CASE (test_S_finit_diff)
{
  boost::mpl::for_each<JointModelVariant::types>(FiniteDiffJoint());
}

BOOST_AUTO_TEST_CASE (test_jacobian_vs_finit_diff)
{
  se3::Model model;
  se3::buildModels::humanoidRandom(model);
  se3::Data data(model);
  
  const VectorXd fd_increment = finiteDifferenceIncrement(model);

  VectorXd q = VectorXd::Ones(model.nq);
  q.segment<4>(3).normalize();
  computeJointJacobians(model,data,q);

  Model::Index idx = model.existJointName("rarm2")?model.getJointId("rarm2"):(Model::Index)(model.njoints-1);
  Data::Matrix6x Jrh(6,model.nv); Jrh.fill(0);
  
  getJointJacobian<WORLD>(model,data,idx,Jrh);
  Data::Matrix6x Jrh_finite_diff = finiteDiffJacobian<false>(model,data,q,idx);
  BOOST_CHECK(Jrh_finite_diff.isApprox(Jrh,fd_increment.maxCoeff()*1e1));
  
  getJointJacobian<LOCAL>(model,data,idx,Jrh);
  Jrh_finite_diff = finiteDiffJacobian<true>(model,data,q,idx);
  BOOST_CHECK(Jrh_finite_diff.isApprox(Jrh,fd_increment.maxCoeff()*1e1));
}

BOOST_AUTO_TEST_SUITE_END()
