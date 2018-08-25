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

#include <cppad/cg.hpp>

#include "pinocchio/fwd.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"

#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/aba.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"

#include "pinocchio/parsers/sample-models.hpp"

#include <iostream>

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

  BOOST_AUTO_TEST_CASE(test_crba_code_generation)
  {
    typedef double Scalar;
    typedef CppAD::cg::CG<Scalar> CGScalar;
    typedef CppAD::AD<CGScalar> ADScalar;
    
    typedef Eigen::Matrix<ADScalar,Eigen::Dynamic,1> ADVector;
    
    typedef se3::ModelTpl<Scalar> Model;
    typedef Model::Data Data;
    
    typedef se3::ModelTpl<ADScalar> ADModel;
    typedef ADModel::Data ADData;
    
    Model model;
    se3::buildModels::humanoidSimple(model);
    model.lowerPositionLimit.head<3>().fill(-1.);
    model.upperPositionLimit.head<3>().fill(1.);
    Data data(model);
    
    ADModel ad_model = model.cast<ADScalar>();
    ADData ad_data(ad_model);
    
    // Sample random configuration
    typedef Model::ConfigVectorType CongigVectorType;
    typedef Model::TangentVectorType TangentVectorType;
    CongigVectorType q(model.nq);
    q = se3::randomConfiguration(model);
    
    TangentVectorType v(TangentVectorType::Random(model.nv));
    TangentVectorType a(TangentVectorType::Random(model.nv));
    
    typedef ADModel::ConfigVectorType ADCongigVectorType;
    typedef ADModel::TangentVectorType ADTangentVectorType;
    
    ADCongigVectorType ad_q = q.cast<ADScalar>();
    ADTangentVectorType ad_v = v.cast<ADScalar>();
    ADTangentVectorType ad_a = a.cast<ADScalar>();

    ADTangentVectorType & X = ad_a;
    CppAD::Independent(X);
    
    se3::rnea(ad_model,ad_data,ad_q,ad_v,ad_a);
    ADVector Y(model.nv); Y = ad_data.tau;
    
    CppAD::ADFun<CGScalar> fun(X,Y);
    
    // generates source code
    CppAD::cg::ModelCSourceGen<Scalar> cgen(fun, "rnea");
    cgen.setCreateJacobian(true);
    cgen.setCreateForwardZero(true);
    cgen.setCreateForwardOne(true);
    cgen.setCreateReverseOne(true);
    cgen.setCreateReverseTwo(true);
    CppAD::cg::ModelLibraryCSourceGen<Scalar> libcgen(cgen);

    // compile source code
    CppAD::cg::DynamicModelLibraryProcessor<Scalar> p(libcgen);

    CppAD::cg::GccCompiler<Scalar> compiler;
    std::unique_ptr<CppAD::cg::DynamicLib<Scalar>> dynamicLib = p.createDynamicLibrary(compiler);

    // save to files (not really required)
    CppAD::cg::SaveFilesModelLibraryProcessor<Scalar> p2(libcgen);
    p2.saveSources();
   
    // use the generated code
    std::unique_ptr<CppAD::cg::GenericModel<Scalar> > rnea_generated = dynamicLib->model("rnea");
    
    CPPAD_TESTVECTOR(Scalar) x((size_t)model.nv);
    Eigen::Map<TangentVectorType>(x.data(),model.nv,1) = a;
    
    CPPAD_TESTVECTOR(Scalar) tau = rnea_generated->ForwardZero(x);
    
    Eigen::Map<TangentVectorType> tau_map(tau.data(),model.nv,1);
    Data::TangentVectorType tau_ref = se3::rnea(model,data,q,v,a);
    BOOST_CHECK(tau_map.isApprox(tau_ref));
    
    se3::crba(model,data,q);
    data.M.triangularView<Eigen::StrictlyLower>()
    = data.M.transpose().triangularView<Eigen::StrictlyLower>();
    
    CPPAD_TESTVECTOR(Scalar) dtau_da = rnea_generated->Jacobian(x);
    Eigen::Map<EIGEN_PLAIN_ROW_MAJOR_TYPE(Data::MatrixXs)> M_map(dtau_da.data(),model.nv,model.nv);
    BOOST_CHECK(M_map.isApprox(data.M));
  }

BOOST_AUTO_TEST_SUITE_END()
