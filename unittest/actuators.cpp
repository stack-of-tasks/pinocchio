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

#include "pinocchio/spatial/fwd.hpp"
#include "pinocchio/actuators/dc-linear-motor-model.hpp"
#include "pinocchio/actuators/dc-linear-first-order-motor-model.hpp"
#include "pinocchio/actuators/dc-two-snd-order-linear-motor-model.hpp"
#include "pinocchio/actuators/dc-temp-non-linear-motor-model.hpp"
#include <iostream>
#include <fstream>
#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>
#include "ext_actuators.hpp"

template <>
const std::string
se3::traits_motor_data<double, se3::ActuatorDCTwoSndOrderLinearMotorData>::
outputfilename
="output-two-snd-order-linear.dat" ;

template <>
const std::string
se3::traits_motor_data<double, se3::ActuatorDCTempNonLinearMotorData>::
outputfilename
="output-temp-non-linear.dat" ;

template <>
const std::string
se3::traits_motor_data<double, se3::ActuatorDCTempCurrentNLMotorData>::
outputfilename
="output-temp-current-nl.dat" ;

template <>
const std::string
se3::traits_motor_data<double, se3::ActuatorDCMotorData>::
outputfilename
="output-linear.dat" ;

template <>
const std::string
se3::traits_motor_data<double, se3::ActuatorDCFirstOrderMotorData>::
outputfilename
="output-first-order-linear.dat" ;

BOOST_AUTO_TEST_SUITE ( BOOST_TEST_MODULE )
typedef double Scalar;

BOOST_AUTO_TEST_CASE ( test_dc_non_linear_motor_actuator )
{
  typedef se3::ActuatorDCTwoSndOrderLinearMotorModel<Scalar> MotorModel;
  typedef se3::test_dc_motors<MotorModel,
			      se3::ActuatorDCTwoSndOrderLinearMotorData >
			      TwoSndOrderLinearDCMotorTest;

  TwoSndOrderLinearDCMotorTest aTwoSndOrderLinearDCMotorTest;
		 
  BOOST_CHECK(aTwoSndOrderLinearDCMotorTest.Test());
}

BOOST_AUTO_TEST_CASE ( test_dc_temp_non_linear_motor_actuator )
{
  typedef se3::ActuatorDCTempNonLinearMotorModel<Scalar> MotorModel;
  typedef se3::test_dc_motors<MotorModel,
			      se3::ActuatorDCTempNonLinearMotorData >
			      TempNonLinearDCMotorTest;

  TempNonLinearDCMotorTest aTempNonLinearDCMotorTest;
  
  BOOST_CHECK(aTempNonLinearDCMotorTest.Test());
}

BOOST_AUTO_TEST_CASE ( test_dc_temp_current_nl_motor_actuator )
{
  typedef se3::ActuatorDCTempCurrentNLMotorModel<Scalar> MotorModel;
  typedef se3::test_dc_motors<MotorModel,
			      se3::ActuatorDCTempCurrentNLMotorData >
			      TempCurrentNLDCMotorTest;

  TempCurrentNLDCMotorTest aTempCurrentNLDCMotorTest;
  
  BOOST_CHECK(aTempCurrentNLDCMotorTest.Test());
}

BOOST_AUTO_TEST_CASE ( test_dc_linear_motor_actuator )
{
  typedef se3::ActuatorDCMotorModel<Scalar> MotorModel;
  typedef se3::test_dc_motors<MotorModel,
			      se3::ActuatorDCMotorData >
			      DCMotorTest;

  DCMotorTest aDCMotorTest;
  
  BOOST_CHECK(aDCMotorTest.Test());
}

BOOST_AUTO_TEST_CASE ( test_dc_first_order_motor_actuator )
{
  typedef se3::ActuatorDCFirstOrderMotorModel<Scalar> MotorModel;
  typedef se3::test_dc_motors<MotorModel,
			      se3::ActuatorDCFirstOrderMotorData >
			      DCFirstOrderMotorTest;

  DCFirstOrderMotorTest aDCFirstOrderMotorTest;
  
  BOOST_CHECK(aDCFirstOrderMotorTest.Test());
}


} // End of BOOST_AUTO_TEST_SUITE
