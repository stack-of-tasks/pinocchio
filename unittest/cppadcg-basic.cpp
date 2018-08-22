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

#include <cppad/cg.hpp>

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

BOOST_AUTO_TEST_SUITE_END()
