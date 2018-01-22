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
#include "pinocchio/actuators/dc-non-linear-motor-model.hpp"
#include <iostream>
#include <fstream>
#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>
#include "ext_actuators.hpp"

template <>
const std::string
se3::traits_motor_data<double>::
outputfilename
="output-non-linear.dat" ;



BOOST_AUTO_TEST_SUITE ( BOOST_TEST_MODULE )
typedef double Scalar;

BOOST_AUTO_TEST_CASE ( test_dc_non_linear_motor_actuator )
{
  typedef se3::ActuatorDCNonLinearMotorModel<Scalar> MotorModel;
  typedef se3::ActuatorDCNonLinearMotorData<Scalar> MotorData;
  typedef se3::test_dc_motors<MotorModel,MotorData,
			      se3::traits_motor_data <typename MotorData::Scalar_t,
						      MotorData> >
			      NonLinearDCMotorTest;

  NonLinearDCMotorTest aNonLinearDCMotorTest;
		 
  BOOST_CHECK(aNonLinearDCMotorTest.Test());
}

BOOST_AUTO_TEST_CASE ( test_dc_linear_motor_actuator )
{
  using namespace Eigen;
  using namespace se3;

  ActuatorDCMotorModel<Scalar> aMotorModel;
  ActuatorDCMotorData<Scalar> aMotorData = aMotorModel.createData();

  // Maxon motor 118754
  aMotorData.rotorInertia(10.1e-7);
  aMotorData.rotorResistor(5.29);
  aMotorData.torqueConst(35.3e-3);
  aMotorData.speedTorqueGrad(2.352e-4);
  aMotorData.backEMF(0.035237);//0.070475);

  ActuatorDCMotorData<Scalar>::X_t astate;
  astate[0] = 0;
  astate[1] = 0;
  
  ActuatorDCMotorData<Scalar>::U_t acontrol;
  acontrol[0] = 0.0; // Volts

  typedef Eigen::Matrix<Scalar,3,1,0> Vector3Scalar;
  Force f_ext;
  double tz = 0.28;
  tz = 0.0;
  f_ext.linear(Vector3Scalar::Zero(3,1));
  f_ext.angular(Vector3Scalar(0.0,0.0,tz));

  // Euler integration
  double dt_control=0.001;
  double dt_sim=1e-6;
  // Frequency of the closed loop system
  // double f=20;
  // double w=f;

  // \f$ \dot{x} \f$
  ActuatorDCMotorData<Scalar>::dX_t dstate;
  
  // Selection matrix
  ActuatorDCMotorData<Scalar>::S_t aS;
  aS.Zero(6,1);
  aS[5] = 1.0;
  aMotorData.setS(aS);

  // Target position
  double theta_d = M_PI/2;
  double dtheta_d = 0.0;
  
  // Control precision output.
  std::ofstream aof;
  aof.open("/tmp/output.dat",std::ofstream::out);
  aof.precision(8);
  aof << std::scientific ;
  
  // PD gains.
  // p.212 from Robot modeling and control.
  // B = D_m + K_b*K_m
  // Note: we changed B_m to D_m 
  double K=10;
  double K_P,K_D;
  K_P = 1.0*K;
  K_D = 0.1;
  double K_I = 0.1;
  Force aForce;
  double sum_e = 0;
  
  for(unsigned long int i=0;i<(unsigned long int)(5.0/dt_sim);i++)
    {
      aMotorModel.ode_func(dstate,astate,acontrol,f_ext,aMotorData);
      astate[0] = astate[0] + dt_sim*dstate[0] + 0.5*dt_sim*dt_sim*dstate[1];
      astate[1] = astate[1] + dstate[1]*dt_sim;
      aMotorModel.get_force(aMotorData, aForce);

      if (i%(unsigned long int)(dt_control/dt_sim)==0)      
	{
	  double error = theta_d - astate[0];
	  double v_error = dtheta_d -astate[1];
	  sum_e += error;
	  
	  acontrol[0] = K_P * error + K_D * (v_error) + K_I * sum_e;

	  aof << dt_sim*(double)i << " "
	      << error << " "
	      << v_error << " "
	      << acontrol[0] << "  " 
	      << aMotorData.c()[0] << " "
	      << aMotorData.c()[1] << " "
	      << aMotorData.c()[2] << " "
	      << astate[0] << " "
	      << astate[1] << " "
	      << K_P << " " << K_D << " "
	      << std::endl;
	}
    }
  aof.close();
  BOOST_CHECK(aForce.angular()[2]!=0.0);
}



BOOST_AUTO_TEST_CASE ( test_dc_first_order_motor_actuator )
{
  using namespace Eigen;
  using namespace se3;

  ActuatorDCFirstOrderMotorModel<Scalar> aMotorModel;
  ActuatorDCFirstOrderMotorData<Scalar> aMotorData = aMotorModel.createData();

  // Maxon motor 118754
  aMotorData.rotorInertia(10.1e-7);
  aMotorData.rotorResistor(5.29);
  aMotorData.torqueConst(35.3e-3);
  aMotorData.speedTorqueGrad(2.352e-4);
  aMotorData.backEMF(0.035237);//0.070475);

  ActuatorDCFirstOrderMotorData<Scalar>::X_t astate;
  astate[0] = 0;
  
  ActuatorDCFirstOrderMotorData<Scalar>::U_t acontrol;
  acontrol[0] = 0.0; // Volts

  typedef Eigen::Matrix<Scalar,3,1,0> Vector3Scalar;
  Force f_ext;
  double tz = 0.28;
  tz = 0.0;
  f_ext.linear(Vector3Scalar::Zero(3,1));
  f_ext.angular(Vector3Scalar(0.0,0.0,tz));

  // Euler integration
  double dt_control=0.001;
  double dt_sim=1e-5;
  // Frequency of the closed loop system
  double f=20;
  double w=f;

  // \f$ \dot{x} \f$
  ActuatorDCFirstOrderMotorData<Scalar>::dX_t dstate;
  
  // Selection matrix
  ActuatorDCFirstOrderMotorData<Scalar>::S_t aS;
  aS.Zero(6,1);
  aS[5] = 1.0;
  aMotorData.setS(aS);

  // Target position
  double theta_d = M_PI/2;
  double dtheta_d = 0.0;
  
  // Control precision output.
  std::ofstream aof;
  aof.open("/tmp/output2.dat",std::ofstream::out);
  aof.precision(8);
  aof << std::scientific ;
  
  // PD gains.
  // p.212 from Robot modeling and control.
  // B = D_m + K_b*K_m
  // Note: we changed B_m to D_m 
  double K_P= w*w*aMotorData.c()[3];
  double K_D= 0.0;//2*w*aMotorData.c()[3] - B;
  Force aForce;
  for(unsigned long int i=0;i<(unsigned long int)(5.0/dt_sim);i++)
    {
      aMotorModel.ode_func(dstate,astate,acontrol,f_ext,aMotorData);
      astate[0] = astate[0] + dt_sim*dstate[0] ;
      
      aMotorModel.get_force(aMotorData, aForce);
      if (i%(unsigned long int)(dt_control/dt_sim)==0)
	{
	  double error = theta_d - astate[0];
	  double v_error = dtheta_d - dstate[0];
      
	  acontrol[0] = K_P * error + K_D * (v_error)/dt_control;
	  aof << dt_sim*(double)i << " "
	      << error << " "
	      << v_error << " "
	      << acontrol[0] << "  " 
	      << aMotorData.c()[0] << " "
	      << aMotorData.c()[1] << " "
	      << aMotorData.c()[2] << " "
	      << astate[0] << " "
	      << K_P << " " << K_D << " "
	      << std::endl;
	}
    }
  aof.close();
  BOOST_CHECK(aForce.angular()[2]!=0.0);
}

} // End of BOOST_AUTO_TEST_SUITE
