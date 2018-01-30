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
#include <pinocchio/actuators/dc-two-snd-order-linear-motor-model.hpp>
#include <pinocchio/actuators/dc-linear-motor-model.hpp>

namespace se3
{

  template<typename Scalar,
	   template<typename S> class T>
  class traits_motor_data;
  
  /// Specific parts of the tests for the non linear motor model (with inductance)
  template<typename Scalar>
  class traits_motor_data<Scalar, ActuatorDCTwoSndOrderLinearMotorData>
  {
  public:
    typedef ActuatorDCTwoSndOrderLinearMotorData<Scalar> ActuatorMotorData;
    static const std::string outputfilename;
    
    static void init(ActuatorMotorData &aMotorData)
    {
      aMotorData.terminalInductance(4.65e-3);
      aMotorData.rotorResistor(5.29);
    }
    
    static void computeGains(ActuatorMotorData &,
			     Scalar &K_P, Scalar &K_D, Scalar &K_I)
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

  /// Specific parts of the tests for the non linear motor model (with temperature)
  template <typename Scalar,
	    template<typename S> class ActuatorDCTempTwoSndOrderLinearMotorData >
  class traits_motor_data
  {
  public:
    typedef ActuatorDCTempTwoSndOrderLinearMotorData<Scalar> ActuatorMotorData;
    static const std::string outputfilename;
    
    static void init(ActuatorMotorData &aMotorData)
    {
      /// Gearhead thermal resistance (K/W)
      aMotorData.resistorOne(6.2);
      /// Winding thermal resistance when heating (K/W)
      aMotorData.resistorTwo(2.0);
      /// Winding thermal resistance at ambiant temperature (Ohms)
      aMotorData.resistorTA(17.5);
      /// Max.permissible winding temperature (C)
      aMotorData.maxPermissibleWindingTemp(155.0);
      /// Thermal time constant winding (s)
      aMotorData.thermTimeCstWinding(25.0);
      /// Ambient temperature for the motor characteristics.
      aMotorData.thermAmbient(25.0);
      
    }
    static void computeGains(ActuatorMotorData &,
			     Scalar &K_P, Scalar &K_D, Scalar &K_I)
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
    
    static void output_data(std::ofstream &,
			    ActuatorMotorData &,
			    typename ActuatorMotorData::X_t &)
    { }
  };


  /// Specific parts of the tests for the linear motor model
  template<typename Scalar>
  class traits_motor_data<Scalar, ActuatorDCMotorData>
  {
  public:
    typedef ActuatorDCMotorData<Scalar> ActuatorMotorData;
    static const std::string outputfilename;
    
    static void init(ActuatorMotorData &aMotorData)
    {
      aMotorData.rotorResistor(5.29);
    }
    
    static void computeGains(ActuatorMotorData &,
			     Scalar &K_P, Scalar &K_D, Scalar &K_I)
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
    }
    
    static void output_data(std::ofstream &,
			    ActuatorMotorData &,
			    typename ActuatorMotorData::X_t &)
    { }
  };

  

  /// Specific parts of the tests for the linear motor model
  template<typename Scalar>
  class traits_motor_data<Scalar, ActuatorDCFirstOrderMotorData>
  {
  public:
    typedef ActuatorDCFirstOrderMotorData<Scalar> ActuatorMotorData;
    static const std::string outputfilename;
    
    static void init(ActuatorMotorData &aMotorData)
    {
      aMotorData.rotorResistor(5.29);
    }
    
    static void computeGains(ActuatorMotorData &aMotorData,
			     Scalar &K_P, Scalar &K_D, Scalar &)
    {
      // Frequency of the closed loop system
      double f=20;
      double w=f;
      K_P= w*w*aMotorData.c()[3];
      K_D= 0.0;//2*w*aMotorData.c()[3] - B;

    }
			     
    static void integration(typename ActuatorMotorData::X_t &astate,
			    typename ActuatorMotorData::dX_t &dstate,
			    double dt_sim) 
    {
      astate[0] = astate[0] + dt_sim*dstate[0];
    }
    
    static void output_data(std::ofstream &,
			    ActuatorMotorData &,
			    typename ActuatorMotorData::X_t &)
    { }
  };

  // Common part of the tests.
  template <typename MotorModel,
	    template<typename S> class MotorData> 
  class test_dc_motors
  {
  public:
    bool Test()
    {
      typedef typename MotorModel::Scalar_t Scalar_t;
      typedef traits_motor_data<Scalar_t,MotorData > traits_md_tempt;
      typedef  MotorData<Scalar_t> MotorDataSpec_t;
      using namespace Eigen;
      using namespace se3;
      
      MotorModel aMotorModel;
      MotorData<Scalar_t> aMotorData = aMotorModel.createData();
      
      // Maxon motor 118754
      aMotorData.rotorInertia(10.1e-7);
      aMotorData.torqueConst(35.3e-3);
      aMotorData.speedTorqueGrad(2.352e-4);
      aMotorData.backEMF(0.035237);//0.070475;
      traits_md_tempt::init(aMotorData);

      typename MotorDataSpec_t::X_t astate;
      for(unsigned int i=0;i<astate.rows();i++)
	astate[i] = 0.0;
    
      typename MotorDataSpec_t::U_t acontrol;
      for(unsigned int i=0;i<acontrol.rows();i++)
	acontrol[i] = 0.0; 
    
      typedef Eigen::Matrix<Scalar_t,3,1,0> Vector3Scalar;
      Force f_ext;
      double tz = 0.28;
      tz = 0.0;
      f_ext.linear(Vector3Scalar::Zero(3,1));
      f_ext.angular(Vector3Scalar(0.0,0.0,tz));
    
      // Euler integration
      double dt_control=0.001;
      double dt_sim=1e-6;
      
      // \f$ \dot{x} \f$
      typename MotorDataSpec_t::dX_t dstate;
  
      // Selection matrix
      typename MotorDataSpec_t::S_t aS;
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

      traits_md_tempt::computeGains(aMotorData,
				    K_P,K_D,K_I);
      
      for(unsigned long int i=0;i<(unsigned long int)(5.0/dt_sim);i++)
	{
	  // dstate = f(state,control,f_ext, parameters)
	  aMotorModel.calc(dstate,astate,acontrol,f_ext,aMotorData);
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

	      for(unsigned int j=0;j<aMotorData.c().rows();j++)
		aof << aMotorData.c()[j] << " ";

	      for(unsigned int j=0;j<astate.rows();j++)
		aof << astate[j] << " ";
	      
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
