//
// Copyright (c) 2017 LAAS CNRS
//
// Author: Olivier Stasse
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

#ifndef _se3_example_test_actuator_hpp_
#define _se3_example_test_actuator_hpp_
#include <pinocchio/actuators/dc-non-linear-motor-model.hpp>
#include <pinocchio/actuators/dc-linear-motor-model.hpp>

namespace se3
{
  
  template <typename Scalar,
	    typename ActuatorMotorData = ActuatorDCNonLinearMotorData<Scalar> >
  class traits_motor_data
  {
  public:
    static const std::string outputfilename;
    
    static void init(ActuatorMotorData &aMotorData)
    {
      aMotorData.terminalInductance(4.65e-3);
    }
    static void computeGains(Scalar &K_P, Scalar &K_D, Scalar &K_I)
    {
      double K=100;
      K_P = 1.0 *K;
      K_D = 0.1;
      K_I = 0.1;
    }
			     
    static void integration(typename ActuatorMotorData::X_t &astate,
			    typename ActuatorMotorData::dX_t &dstate,
			    double dt_sim) 
    {
      astate[0] = astate[0] + dt_sim*dstate[0] + 0.5*dt_sim*dt_sim*dstate[1];
      astate[1] = astate[1] + dstate[1]*dt_sim;
      astate[2] = astate[2] + dstate[2]*dt_sim;
    }
    
    static void output_data(std::ofstream &aof,
			    ActuatorMotorData &aMotorData,
			    typename ActuatorMotorData::X_t &astate)
    {
      for(unsigned int i=0;i<6;i++)
	aof << aMotorData.c()[i] << " ";
      for(unsigned int i=0;i<3;i++)
	aof << astate[i] << " ";
    }
  };
  template <typename MotorModel,
	    typename MotorData,
	    typename traits_md_tempt =
	    traits_motor_data<typename MotorData::Scalar_t, MotorData> >
  class test_dc_motors
  {
  public:
    bool Test()
    {
      using namespace Eigen;
      using namespace se3;
      
      MotorModel aMotorModel;
      MotorData aMotorData = aMotorModel.createData();
      
      // Maxon motor 118754
      aMotorData.rotorInertia(10.1e-7);
      aMotorData.rotorResistor(5.29);
      aMotorData.torqueConst(35.3e-3);
      aMotorData.speedTorqueGrad(2.352e-4);
      aMotorData.backEMF(0.035237);//0.070475;
      traits_md_tempt::init(aMotorData);

      typename MotorData::X_t astate;
      astate[0] = 0;
      astate[1] = 0;
    
      typename MotorData::U_t acontrol;
      acontrol[0] = 0.0; // Volts
    
      typedef Eigen::Matrix<typename MotorData::Scalar_t,3,1,0> Vector3Scalar;
      Force f_ext;
      double tz = 0.28;
      tz = 0.0;
      f_ext.linear(Vector3Scalar::Zero(3,1));
      f_ext.angular(Vector3Scalar(0.0,0.0,tz));
    
      // Euler integration
      double dt_control=0.001;
      double dt_sim=1e-6;
      
      // \f$ \dot{x} \f$
      typename MotorData::dX_t dstate;
  
      // Selection matrix
      typename MotorData::S_t aS;
      aS.Zero(6,1);
      aS[5] = 1.0;
      aMotorData.setS(aS);
    
      // Target position
      double theta_d = M_PI/2;
      double dtheta_d = 0.0;
    
      // Control precision output.
      std::ofstream aof;
      aof.open(traits_md_tempt::outputfilename.c_str(),
	       std::ofstream::out);
      aof.precision(8);
      aof << std::scientific ;
    
      // PD gains.
      double K=10;
      double K_P,K_D;
      K_P = 1.0*K;
      K_D = 0.1;
      double K_I = 0.1;
      Force aForce;
      double sum_e = 0;

      traits_md_tempt::computeGains(K_P,K_D,K_I);
      
      for(unsigned long int i=0;i<(unsigned long int)(5.0/dt_sim);i++)
	{
	  // dstate = f(state,control,f_ext, parameters)
	  aMotorModel.ode_func(dstate,astate,acontrol,f_ext,aMotorData);
	  // state_k+1 = state_k + dt * dstate
	  traits_md_tempt::integration(astate,dstate,dt_sim);
	  // Extract forces
	  aMotorModel.get_force(aMotorData, aForce);

	  // When control time is reached
	  if (i%(unsigned long int)(dt_control/dt_sim)==0)      
	    {
	      // Compute error
	      double error = theta_d - astate[0];
	      double v_error = dtheta_d-astate[1];
	      sum_e += error;

	      // Apply control
	      acontrol[0] = K_P * error + K_D * (v_error) + K_I * sum_e;

	      // Record general informations 
	      aof << dt_sim*(double)i << " "
		  << error << " "
		  << v_error << " "
		  << acontrol[0] << "  ";

	      // Record specific informations
	      traits_md_tempt::output_data(aof,aMotorData,astate);

	      //
	      aof << K_P << " " << K_D << " " << K_I 
		  << std::endl;
	    }
	}
      aof.close();
      return(aForce.angular()[2]!=0.0);
    }
  };
  
} // end of namespace se3
#endif /* se_example_test_actuator_hpp_ */
