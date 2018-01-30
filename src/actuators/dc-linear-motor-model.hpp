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

#ifndef _se3_motor_model_hpp_
#define _se3_motor_model_hpp_

#include <iostream>
#include "pinocchio/macros.hpp"
#include "pinocchio/actuators/actuator-model.hpp"


namespace se3
{
  /** \addtogroup actuators_motor_group_sl_dc_motor Linear motor model (second order)
      \ingroup actuators_motor_group
      The model used for this linear version of the DC motor is defined by
      the following state vector
      \f$ \mathbf{x} = [ x_0, x_1 ]^{\top} = [ \theta_m, \dot{\theta}_m ]^{\top} \f$ with \f$\theta_m\f$ the motor angle.<br>
      The ODE function \f$ \dot{\bf{x}}= f(\bf{x},\bf{u},\bf{c},\bf{f}_{ext}) \f$ describing the dynamics of the motor for a set of parameters
      \f$ \bf{c} \f$, with control vector \f$ \bf{u} \f$ and submitted to external forces \f$ \bf{f}_{ext} \f$ by the multi-body rigid dynamics is:<br>
      
      \f[
      \left(
      \begin{matrix}
      \dot{\theta}_m \\
      \ddot{\theta}_m \\
      \end{matrix}
      \right)
      = 
      \left(
      \begin{matrix}
      \dot{\theta}_m \\
      \frac{K_m}{R B_m}V - (\frac{K_m K_b}{R B_m} +
      \frac{D_m}{B_m}) \dot{\theta}_m -\frac{\tau_l}{B_m}
      \end{matrix}
      \right)
      \f]
      with \f$ B_m\f$ the rotor inertia, \f$ K_m \f$ the motor torque constant,
      \f$ R \f$ the armature resistance, \f$ i_a \f$ the current armature,
      \f$ \tau_m \f$ the motor torque, \f$ V \f$ the armature voltage,
      \f$ V_b \f$ the back electromotive force (E.M.F.), 
      \f$ D_m \f$ the motor friction and includes friction in the brushes and gears,
      the robot position \f$ \theta_m \f$,
      \f$ \tau_l \f$ the motor load, the gear train inertia \f$ B_g \f$,
      \f$ K_b \f$ the back E.M.F. constant.

      As the dynamics can be also written:
      \f[
      \left(
      \begin{matrix}
      \dot{x}_0 \\
      \dot{x}_1 \\
      \end{matrix}
      \right)
      = 
      \left(
      \begin{matrix}
      x_1 \\
      c_{1} V + c_{2} x_1 + c_3 \tau_l
      \end{matrix}
      \right)
      = 
      \left(
      \begin{matrix}
      x_1 \\
      c_{1} u + c_{2} x_1 + c_3 \bf{S}\bf{f}_{ext}
      \end{matrix}
      \right)
      \f]
      
      Thus 
      \f[ 
      \begin{array}{rcl}
      c_1 &= &\frac{K_b}{R B_m}\\
      c_2 &= & -\frac{K_m K_b}{R B_m} - \frac{D_m}{B_m}\\
      c_3 &= & -\frac{1}{B_m}\\
      \end{array}
      \f]
      The control vector \f$ \bf{u} \f$ is the voltage \f$ V \f$. <br>
      \f$ \bf{S} \f$ is the selection matrix projecting the spatial force \f$\bf{f}_{ext}\f$ expressed in the motor coordinates. <br>
      The observation function \f$h(\bf{x},\bf{u},\bf{c},\bf{f}_{ext})\f$ is defined as:
      \f[ 
      \left(
      \begin{matrix}
      i_a \\
      \tau_m\\
      V_b \\
      \end{matrix}
      \right)
      = 
      \left(
      \begin{matrix}
      \frac{V}{R} - \frac{K_b}{R}\dot{\theta}_m \\
      \frac{V K_m}{R} - \frac{K_b K_m}{R}\dot{\theta}_m\\
      K_b \dot{\theta}_m\\
      \end{matrix}
      \right)
      \f]
   **/

  /**
   *  @brief Template for handling data related to a second order linear DC motor model.
   *  \ingroup actuators_motor_group_sl_dc_motor
   *
   *  @tparam Scalar_ {description} 
   */
  template<typename Scalar_> 
  class ActuatorDCMotorData : ActuatorDataBase<ActuatorDCMotorData<Scalar_> >
  {
  public:
    typedef Scalar_ Scalar_t;
    typedef typename Eigen::Matrix<Scalar_, 13,1 > Parameters_t;
    typedef typename Eigen::Matrix<Scalar_, 3,1> Observations_t;
    typedef typename Eigen::Matrix<Scalar_, 6,1> S_t;
    typedef typename Eigen::Matrix<Scalar_, 2,1> X_t;
    typedef typename Eigen::Matrix<Scalar_, 2,1> dX_t;
    typedef typename Eigen::Matrix<Scalar_, 1,1> U_t;

    enum InternalParameters
      {
	P_ROTOR_INERTIA=6,
	P_ROTOR_RESISTOR,
	P_TORQUE_CST,
	P_SPEED_TORQUE_GRD,
	P_BACK_EMF,
	P_THONE_RESISTOR,
	P_THTWO_RESISTOR,
	P_TA_RESISTOR,
	P_MAX_PERM_WINDING_T,
	P_THERM_TIME_CST_WINDING,
	P_NOMINAL_CURRENT,
	P_AMBIENT_TEMP
      };
    /// Return observation vector.
    const Observations_t & h() const { return h_;}
    
    /// Return parameters 
    const Parameters_t & c() const { return c_;}
    
    /// Returns selection matrix.
    const S_t & S() const { return S_;}

    ///@{ Internal parameters of the actuator.
    /// \brief Rotor inertia (kg/m2) - \f$ B_m \f$
    void rotorInertia(Scalar_ c)
    { c_[P_ROTOR_INERTIA] = c; updateParameters();}

    /// \brief Rotor resistor (Ohm) - \f$ R\f$
    void rotorResistor(Scalar_ c)
    { c_[P_ROTOR_RESISTOR] = c; updateParameters();}
    
    /// \brief Torque constant (Nm/A) - \f$ K_m \f$
    void torqueConst(Scalar_ c)
    { c_[P_TORQUE_CST] = c; updateParameters();}
    
    /// \brief Speed torque gradient (rads^-1/Nm) \f$ D_m \f$
    void speedTorqueGrad(Scalar_ c) 
    {c_[P_SPEED_TORQUE_GRD] = c; updateParameters();}
    
    /// \brief Terminal Inductance (Henry)
    //void terminalInductance(Scalar_ &c)
    //    {c_[7] = c;}
    
    /// \brief Back electro magnetic force constant \f$ K_b \f$
    /// It is the inverse of the speed torque gradient constant
    /// (\f$k_N\f$) in Maxon datasheet.
    /// More exactly the maxon datasheet gives rpm/V.
    /// So it is \f$\frac{60}{ 2 \pi k_N)}\f$
    void backEMF(Scalar_ c) 
    {c_[P_BACK_EMF] = c;updateParameters();}

    /// \brief Gearhead thermal resistance  \f$ R_{th1} \f$
    /// at ambient temperature units: K/W
    void resistorOne(Scalar_ c)
    {c_[P_THONE_RESISTOR] = c;updateParameters();}
    
    /// \brief Winding thermal resistance  \f$ R_{th2} \f$
    /// at ambient temperature units: K/W
    void resistorTwo(Scalar_ c)
    {c_[P_THTWO_RESISTOR] = c;updateParameters();}

    /// \brief Winding resistance  \f$ R_{ta} \f$
    /// at ambient temperature units: Ohms
    void resistorTA(Scalar_ c)
    {c_[P_TA_RESISTOR] = c;updateParameters();}
      
    /// \brief Max. permissible winding temperature  \f$ T_{max} \f$
    /// catalog value
    /// units: C
    void maxPermissibleWindingTemp(Scalar_ c)
    {c_[P_MAX_PERM_WINDING_T] = c;updateParameters();}

    /// \brief Therm. time constant winding \f$ \tau_W \f$
    /// units: seconds (s)
    void thermTimeCstWinding(Scalar_ c)
    {c_[P_THERM_TIME_CST_WINDING] = c; updateParameters();}

    /// \brief Nominal current
    /// units: Ampert (A)
    void nominalCurrent(Scalar_ c)
    {c_[P_NOMINAL_CURRENT] = c; updateParameters();}

    /// \brief Nominal Temperature
    /// units: Celsius (C)
    void nominalTemperature(Scalar_ c)
    {c_[P_AMBIENT_TEMP] = c; updateParameters();}

    void setS(S_t &S)
    {S_ = S;}
    ///@}

    /// \brief Returns the maximum overload possible for a given duration.
    /// Implements
    /// \f[ K = \sqrt{\frac{1}{1 - exp\left[-\frac{t_{on}}{\tau_W}\right] }} \f]
    Scalar_ getMaxPermissibleOverloadForONTime(Scalar_ aDuration)
    {
      return sqrt(1/(1-exp(-aDuration/c_[P_THERM_TIME_CST_WINDING])));
    }

    /// \brief Returns the overload for a given motor current and the current stator temperature (\f$ T_S \f$)
    /// Implements
    /// \f[ K = \frac{I_{mot}}{I_N} \sqrt{ \frac{T_{max} -25}{T_{max}-T_S} \frac{R_{th1}}{R_{th1}+R_{th2}} } \f] 
    Scalar_ getOverloadForCurrent(Scalar_ aCurrent, Scalar_ TS)
    {
      return aCurrent/c_[P_NOMINAL_CURRENT]*sqrt((c_[P_MAX_PERM_WINDING_T]- 25)/(c_[P_MAX_PERM_WINDING_T] - TS) *
						 (c_[P_THONE_RESISTOR]/(c_[P_THONE_RESISTOR] + c_[P_THTWO_RESISTOR])));
    }

    /// \brief Returns the maximum ON time at a given overload factor K
    /// Implements
    /// \f[ t_{on} = \tau_{W} ln \frac{K^2}{K^2-1} \f]
    Scalar_ getMaxONTimeForK(Scalar_ aK)
    {
      return c_[P_THERM_TIME_CST_WINDING] * log(aK*aK/(aK*aK-1.0));
    }

    /// \brief Returns the maximum current \f$ I_{mot}\f$ at a given overload factor \f$ K\f$.
    /// Implements
    /// \f[ I_{mot} = K I_n \sqrt{ \frac{T_{max}-T_S}{T_{max} -25} \frac{R_{th1}+R_{th2}}{R_{th1}} }\f]
    Scalar_ getMaxCurrentForK(Scalar_ aK)
    {
      return aK * c_[P_NOMINAL_CURRENT]*sqrt((c_[P_MAX_PERM_WINDING_T] - c_[P_AMBIENT_TEMP])/(c_[P_MAX_PERM_WINDING_T]- 25) *
					     (c_[P_THONE_RESISTOR] + c_[P_THTWO_RESISTOR])/(c_[P_THONE_RESISTOR]));
    }
    
    Observations_t & h() { return h_;}
    
  protected:
    /// Update the first three parameters of the actuators:
    void updateParameters()
    {
      // c_1 = torque_cst / (rotorInertia * rotorResistor)
      c_[0] = c_[P_TORQUE_CST] /(c_[P_ROTOR_INERTIA] * c_[P_ROTOR_RESISTOR]);
      // c_2 = - torque_cst * back_emf_cst  / (rotorInertia * rotorResistor)
      //        - speedTorqueGrad^{-1} / rotorInertia
      c_[1] = -c_[P_TORQUE_CST]*c_[P_BACK_EMF] /(c_[P_ROTOR_INERTIA] * c_[P_ROTOR_RESISTOR])
	- c_[P_SPEED_TORQUE_GRD]/c_[P_ROTOR_INERTIA];
      // c_3 = -1/rotorInertia
      c_[2] = -1/c_[P_ROTOR_INERTIA];
    }

    /// Observation variables
    Observations_t h_;
    
    /// Vector parameters.
    Parameters_t c_;
    
    /// Selection matrix
    S_t S_;
    
  };

  /**
   *  @brief Template which implements a second order linear DC motor model.
   *  \ingroup actuators_motor_group_sl_dc_motor
   *
   *  @tparam Scalar_ {description} 
   */
  template< typename Scalar_> 
  class ActuatorDCMotorModel : ActuatorModelBase<ActuatorDCMotorModel<Scalar_> >
  {

    typedef Eigen::Matrix<Scalar_,3,1,0> Vector3Scalar;
    
  public:
    typedef Scalar_ Scalar_t;
    ActuatorDCMotorModel() {}

    void calc(typename ActuatorDCMotorData<Scalar_>::dX_t & dstate,
	      typename ActuatorDCMotorData<Scalar_>::X_t & state,
	      typename ActuatorDCMotorData<Scalar_>::U_t & control,
	      Force & fext,
	      ActuatorDCMotorData<Scalar_> &data)
    {
      typedef ActuatorDCMotorData<Scalar_> adc;
      // Update dstate
      dstate[0] = state[1];
      dstate[1] = data.c()[0] * control[0] + data.c()[1] * state[0] +
	data.c()[2] *data.S().dot(fext.toVector());
      
      // Update observation
      // Current
      data.h()[0] = control[0] /data.c()[adc::P_ROTOR_RESISTOR]
	- data.c()[adc::P_BACK_EMF] *state[1]/data.c()[adc::P_ROTOR_RESISTOR];
      // Motor torque
      data.h()[1] = data.c()[adc::P_TORQUE_CST] *data.h()[0];
      // Back emf potential 
      data.h()[2] = data.c()[adc::P_BACK_EMF] * state[1];
    }
    

    void get_force(ActuatorDCMotorData<Scalar_> &data, Force &aForce) const
    {
      aForce.linear(Vector3Scalar::Zero(3,1));
      aForce.angular(Vector3Scalar(0.0,0.0,data.h()[1]));
    }

    ActuatorDCMotorData<Scalar_> createData() const
    { return ActuatorDCMotorData<Scalar_>(); };
    
  protected:
  
    // Store actuator name.
    std::string name_;

  };
  
  template <typename Scalar_>
  struct adb_traits<ActuatorDCMotorData<Scalar_> >
  {
    typedef ActuatorDCMotorData<Scalar_> ActuatorDataDerived;
  };

  template <typename Scalar_>
  struct amb_traits<ActuatorDCMotorModel<Scalar_> >
  {
    typedef  ActuatorDCMotorModel<Scalar_> ActuatorModelDerived;
    typedef  ActuatorDCMotorData<Scalar_> ActuatorDataDerived;
  };


} // end of namespace se3

#endif /* _se3_motor_model_ */
