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

#include <boost/variant.hpp> // to avoid C99 warnings

#include <cppad/cg/support/cppadcg_eigen.hpp>

#include <iostream>

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

  BOOST_AUTO_TEST_CASE(test_simple_cppadcg)
  {
    using namespace CppAD;
    using namespace CppAD::cg;
    
    typedef CG<double> CGD;
    typedef AD<CGD> ADCG;
    
    /***************************************************************************
     *                               the model
     **************************************************************************/
    
    // independent variable vector
    CppAD::vector<ADCG> x(2);
    x[0] = 2.;
    x[1] = 3.;
    Independent(x);
    
    // dependent variable vector
    CppAD::vector<ADCG> y(1);
    
    // the model
    ADCG a = x[0] / 1. + x[1] * x[1];
    y[0] = a / 2;
    
    ADFun<CGD> fun(x, y); // the model tape
    
    /***************************************************************************
     *                        Generate the C source code
     **************************************************************************/
    
    /**
     * start the special steps for source code generation for a Jacobian
     */
    CodeHandler<double> handler;
    
    CppAD::vector<CGD> indVars(2);
    handler.makeVariables(indVars);
    
    CppAD::vector<CGD> jac = fun.SparseJacobian(indVars);
    
    LanguageC<double> langC("double");
    LangCDefaultVariableNameGenerator<double> nameGen;
    
    std::ostringstream code;
    handler.generateCode(code, langC, jac, nameGen);
    std::cout << code.str();
  }
    
  BOOST_AUTO_TEST_CASE(test_eigen_support)
  {
    using namespace CppAD;
    using namespace CppAD::cg;
    
    // use a special object for source code generation
    typedef CG<double> CGD;
    typedef AD<CGD> ADCG;
    
    typedef Eigen::Matrix<ADCG,Eigen::Dynamic,1> ADCGVector;
    
    ADCGVector vec_zero(ADCGVector::Zero(100));
    BOOST_CHECK(vec_zero.isZero());
    
    ADCGVector vec_ones(10);
    vec_ones.fill((ADCG)1);
    BOOST_CHECK(vec_ones.isOnes());
    
    ADCG value_one(1.);
    
    ADCG value_nan;
    value_nan = NAN;
    
    ADCGVector vec_nan(10);
    vec_nan.fill((ADCG)NAN);
//    BOOST_CHECK(vec_ones.allFinite());
    std::cout << vec_nan.transpose() << std::endl;
    
  }
    
  BOOST_AUTO_TEST_CASE(test_dynamic_link)
  {
    using namespace CppAD;
    using namespace CppAD::cg;
    
    // use a special object for source code generation
    typedef CG<double> CGD;
    typedef AD<CGD> ADCG;
    
    typedef AD<double> ADScalar;
    
    /***************************************************************************
     *                               the model
     **************************************************************************/
    
    // independent variable vector
    std::vector<ADCG> x(2);
    Independent(x);
    
    // dependent variable vector
    std::vector<ADCG> y(1);
    
    // the model equation
    ADCG a = x[0] / 1. + x[1] * x[1];
    y[0] = a / 2;
    
    ADFun<CGD> fun(x, y);
    
    
    /***************************************************************************
     *                       Create the dynamic library
     *                  (generates and compiles source code)
     **************************************************************************/
    // generates source code
    ModelCSourceGen<double> cgen(fun, "model");
    cgen.setCreateJacobian(true);
    cgen.setCreateForwardOne(true);
    cgen.setCreateReverseOne(true);
    cgen.setCreateReverseTwo(true);
    ModelLibraryCSourceGen<double> libcgen(cgen);
    
    // compile source code
    DynamicModelLibraryProcessor<double> p(libcgen);
    
    GccCompiler<double> compiler;
    std::unique_ptr<DynamicLib<double>> dynamicLib = p.createDynamicLibrary(compiler);
    
    // save to files (not really required)
    SaveFilesModelLibraryProcessor<double> p2(libcgen);
    p2.saveSources();
    
    /***************************************************************************
     *                       Use the dynamic library
     **************************************************************************/
    
    std::unique_ptr<GenericModel<double>> model = dynamicLib->model("model");
    CPPAD_TESTVECTOR(double) xv(x.size()); xv[0] = 2.5; xv[1] = 3.5;
    CPPAD_TESTVECTOR(double) jac = model->Jacobian(xv);
    
    std::vector<ADScalar> x_ad(2);
    Independent(x_ad);
    
    // dependent variable vector
    std::vector<ADScalar> y_ad(1);
    
    // the model equation
    ADScalar a_ad = x_ad[0] / 1. + x_ad[1] * x_ad[1];
    y_ad[0] = a_ad / 2;
    
    ADFun<double> ad_fun(x_ad, y_ad);

    CPPAD_TESTVECTOR(double) jac_ref = ad_fun.Jacobian(xv);
    
    // print out the result
    std::cout << jac[0] << " " << jac[1] << std::endl;
    
    BOOST_CHECK(Eigen::Map<Eigen::Vector2d>(jac.data()).isApprox(Eigen::Map<Eigen::Vector2d>(jac_ref.data())));
  }

BOOST_AUTO_TEST_SUITE_END()
