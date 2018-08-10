//
// Copyright (c) 2018 CNRS
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

#include "pinocchio/fwd.hpp"
#include "pinocchio/multibody/joint/joint-generic.hpp"
#include "pinocchio/multibody/liegroup/liegroup.hpp"

#include <iostream>

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

BOOST_AUTO_TEST_CASE(test_jointRX_motion_space)
{
  using CppAD::AD;
  using CppAD::NearEqual;

  typedef AD<double> AD_double;
  typedef se3::JointCollectionDefaultTpl<AD_double> JointCollectionAD;
  typedef se3::JointCollectionDefaultTpl<double> JointCollection;
  
  typedef se3::SE3Tpl<AD_double> SE3AD;
  typedef se3::MotionTpl<AD_double> MotionAD;
  typedef se3::SE3Tpl<double> SE3;
  typedef se3::MotionTpl<double> Motion;
  typedef se3::ConstraintTpl<Eigen::Dynamic,double> ConstraintXd;
  
  typedef Eigen::Matrix<AD_double,Eigen::Dynamic,1> VectorXAD;
  typedef Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> MatrixX;
  
  typedef JointCollectionAD::JointModelRX JointModelRXAD;
  typedef JointModelRXAD::ConfigVector_t ConfigVectorAD;
//  typedef JointModelRXAD::TangentVector_t TangentVectorAD;
  typedef JointCollectionAD::JointDataRX JointDataRXAD;
  
  typedef JointCollection::JointModelRX JointModelRX;
  typedef JointModelRX::ConfigVector_t ConfigVector;
  typedef JointModelRX::TangentVector_t TangentVector;
  typedef JointCollection::JointDataRX JointDataRX;
  
  JointModelRX jmodel; jmodel.setIndexes(0,0,0);
  JointDataRX jdata(jmodel.createData());
  
  JointModelRXAD jmodel_ad = jmodel.cast<AD_double>();
  JointDataRXAD jdata_ad(jmodel_ad.createData());
  
  typedef se3::LieGroup<JointModelRX>::type JointOperation;
  ConfigVector q(jmodel.nq()); JointOperation().random(q);
  ConfigVectorAD q_ad(q.cast<AD_double>());
  
   // Zero order
  jmodel_ad.calc(jdata_ad,q_ad);
  jmodel.calc(jdata,q);
  
  SE3 M1(jdata.M);
  SE3AD M2(jdata_ad.M);
  BOOST_CHECK(M1.isApprox(M2.cast<double>()));
  
  // First order
  TangentVector v(TangentVector::Random(jmodel.nv()));
  VectorXAD X(jmodel_ad.nv());
  
  for(Eigen::DenseIndex k = 0; k < jmodel.nv(); ++k)
  {
    X[k] = v[k];
  }
  CppAD::Independent(X);
  jmodel_ad.calc(jdata_ad,q_ad,X);
  jmodel.calc(jdata,q,v);
  VectorXAD Y(6);
  MotionAD m_ad(jdata_ad.v);
  Motion m(jdata.v);
  ConstraintXd Sref(jdata.S.matrix());
  
  for(Eigen::DenseIndex k = 0; k < 3; ++k)
  {
    Y[k+Motion::LINEAR] = m_ad.linear()[k];
    Y[k+Motion::ANGULAR] = m_ad.angular()[k];
  }

  CppAD::ADFun<double> vjoint(X,Y);
  
  CPPAD_TESTVECTOR(double) x((size_t)jmodel_ad.nv());
  for(Eigen::DenseIndex k = 0; k < jmodel.nv(); ++k)
  {
    x[(size_t)k] = v[k];
  }
  
  CPPAD_TESTVECTOR(double) jac = vjoint.Jacobian(x);
  MatrixX S(6,jac.size()/6);
  S = Eigen::Map<MatrixX>(jac.data(),S.rows(),S.cols());
  
  BOOST_CHECK(m.isApprox(m_ad.cast<double>()));
  
  BOOST_CHECK(Sref.matrix().isApprox(S));
}

BOOST_AUTO_TEST_SUITE_END()
