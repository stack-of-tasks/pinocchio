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

#ifndef _se3_temp_current_non_linear_motor_model_hpp_
#define _se3_temp_current_non_linear_motor_model_hpp_

#include <iostream>
#include "pinocchio/macros.hpp"
#include "pinocchio/actuators/actuator-model.hpp"


namespace se3
{
  /** \addtogroup actuators_motor_group_t_current_nl_dc_motor Non linear motor model controlled using current (temperature)
      \ingroup actuators_motor_group
      The model used for this non linear version of the DC motor is defined by
      the following state vector
      \f$ \mathbf{x} = [ x_0, x_1, x_2 ]^{\top} = [ \theta_m, \dot{\theta}_m , T]^{\top} \f$ with \f$\theta_m\f$ the motor angle, and \f$ T \f$ the temperature.<br>
      The ODE function \f$ \dot{\bf{x}}= f(\bf{x},\bf{u},\bf{c},\bf{f}_{ext}) \f$ describing the dynamics of the motor for a set of parameters
      \f$ \bf{c} \f$, with control vector \f$ \bf{u} \f$ and submitted to external forces \f$ \bf{f}_{ext} \f$ by the multi-body rigid dynamics is:<br>
      
      \f[
      \left(
      \begin{matrix}
      \dot{\theta}_m \\
      \ddot{\theta}_m \\
      \dot{T} \\
      \end{matrix}
      \right)
      = 
      \left(
      \begin{matrix}
      \dot{\theta}_m \\
      \frac{K_m}{B_m }i_a - \frac{D_m}{B_m} \dot{\theta}_m - \frac{\bf{S} \bf{f}_{ext}}{B_m} \\
      \frac{R_{TA} i_a^2}{C(1-\alpha_{Cu}(R_{th1}+R_{th2})R_{TA}i_a^2) } - \frac{D_T}{C} (T - T_A)
      \end{matrix}
      \right)
      \f]
      with \f$ B_m\f$ the rotor inertia, \f$ K_m \f$ the motor torque constant,
      \f$ R \f$ the armature resistance, \f$ i_a \f$ the current armature,
      \f$ \tau_m \f$ the motor torque, \f$ V \f$ the armature voltage,
      \f$ V_b \f$ the back electromotive force (E.M.F.), 
      \f$ D_m \f$ the motor friction and includes friction in the brushes and gears,
      the robot position \f$ \theta_m \f$,
      \f$ \tau_l \f$ the motor load,
      \f$ K_b \f$ the back E.M.F. constant,
      \f$ C \f$ the heat capacity and \f$ D_T \f$ the thermal dissipation factor.

      Note that the winding resistor is increasing linearly with the temperature and is given by:
      \f[ R = R_{TA} (1 + \alpha_{Cu} (T -T_{A}))\f]
      the current is given by:
      \f[ i_a = \frac{V - V_b}{R} \f]
      the motor torque is given by:
      \f[ \tau_m = K_m i_a = K_m \frac{V-V_b}{R} \f]
      the back emf is given by:
      \f[ V_b = K_b \theta_m \f] 
      the dynamic equation of the motor is given by:
      \f[ B_m \ddot{\theta}_m + D_m \dot{\theta}_m  = \tau_m - \bf{S} \bf{f}_{ext} \f] 
      the temperature variation is given by:
      \f[ \dot{T} = \frac{R_{TA} i_a^2}{C(1-\alpha_{Cu}(R_{th1}+R_{th2})R_{TA}i_a^2) } - \frac{D_T}{C} (T - T_A)\f]
      
      As the dynamics can be also written:
      \f[
      \left(
      \begin{matrix}
      \dot{x}_0 \\
      \dot{x}_1 \\
      \dot{x}_2 \\
      \end{matrix}
      \right)
      = 
      \left(
      \begin{matrix}
      x_1 \\
      c_0 i_a + c_{1} \dot{\theta}_m + c_2 \tau_l \\
      \frac{c_3 i_a^2}{c_4 + c_5 i_a^2} + c_6 T +c_7\\
      \end{matrix}
      \right)
      = 
      \left(
      \begin{matrix}
      x_1 \\
      c_{0} u + c_1 x_1 + c_2 \bf{S}\bf{f}_{ext} \\
      \frac{c_3 u^2}{c_4 + c_5 u^2} + c_6 x_2 + c_7\\
      \end{matrix}
      \right)
      \f]
      
      Thus 
      \f[ 
      \begin{array}{rcl}
      c_0 &= & \frac{K_m}{B_m}\\
      c_1 &= & -\frac{D_m}{B_m}\\
      c_2 &= & -\frac{1}{B_m} \\
      c_3 &= & R_{TA} \\
      c_4 &= & C \\
      c_5 &= & -C\alpha_{Cu}(R_{th1}+R_{th2})R_{TA} \\
      c_6 &= & -\frac{D_T}{C} \\
      c_7 &= & \frac{D_T T_A}{C}
      \end{array}
      \f]
      The control vector \f$ \bf{u} \f$ is the voltage \f$ V \f$. <br>
      \f$ \bf{S} \f$ is the selection matrix projecting the spatial force \f$\bf{f}_{ext}\f$ expressed in the motor coordinates. <br>
      The observation function \f$h(\bf{x},\bf{u},\bf{c},\bf{f}_{ext})\f$ is defined as:
      \f[ 
      \left(
      \begin{matrix}
      \tau_m\\
      V_b \\
      \end{matrix}
      \right)
      = 
      \left(
      \begin{matrix}
      K_m i_a \\
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
  class ActuatorDCTempCurrentNLMotorData : ActuatorDataBase<ActuatorDCTempCurrentNLMotorData<Scalar_> >
  {
  public:
    typedef Scalar_ Scalar_t;
    typedef typename Eigen::Matrix<Scalar_, 21,1 > Parameters_t;
    typedef typename Eigen::Matrix<Scalar_, 3,1> Observations_t;
    typedef typename Eigen::Matrix<Scalar_, 6,1> S_t;
    typedef typename Eigen::Matrix<Scalar_, 3,1> X_t;
    typedef typename Eigen::Matrix<Scalar_, 3,1> dX_t;
    typedef typename Eigen::Matrix<Scalar_, 1,1> U_t;

    enum InternalParameters
      {
	P_ROTOR_INERTIA=8,
	P_TORQUE_CST,
	P_SPEED_TORQUE_GRD,
	P_BACK_EMF,
	P_THONE_RESISTOR,
	P_THTWO_RESISTOR,
	P_TA_RESISTOR,
	P_MAX_PERM_WINDING_T,
	P_THERM_TIME_CST_WINDING,
	P_AMBIENT_TEMP,
	P_NOMINAL_CURRENT,
	P_HEAT_DISSIPATION,
	P_HEAT_CAPACITY
      };
    /// Return observation vector.
    const Observations_t & h() const { return h_;}
    
    /// Return parameters 
    const Parameters_t & c() const { return c_;}
    
    /// Returns selection matrix.
    const S_t & S() const { return S_;}

    ///@{ Internal parameters of the actuator.
    /// \brief Rotor inertia (kg/m2) - \f$ B_m \f$
    /// Mandatory
    void rotorInertia(Scalar_ c)
    { c_[P_ROTOR_INERTIA] = c; updateParameters();}

    
    /// \brief Torque constant (Nm/A) - \f$ K_m \f$
    /// Mandatory
    void torqueConst(Scalar_ c)
    { c_[P_TORQUE_CST] = c; updateParameters();}
    
    /// \brief Speed torque gradient (rads^-1/Nm) \f$ D_m \f$
    /// Mandatory
    void speedTorqueGrad(Scalar_ c) 
    {c_[P_SPEED_TORQUE_GRD] = c; updateParameters();}
    
    /// \brief Back electro magnetic force constant \f$ K_b \f$
    /// It is the inverse of the speed torque gradient constant
    /// (\f$k_N\f$) in Maxon datasheet.
    /// More exactly the maxon datasheet gives rpm/V.
    /// So it is \f$\frac{60}{ 2 \pi k_N)}\f$
    /// Mandatory
    void backEMF(Scalar_ c) 
    {c_[P_BACK_EMF] = c;updateParameters();}

    /// \brief Gearhead thermal resistance  \f$ R_{th1} \f$
    /// at ambient temperature units: K/W
    /// Mandatory
    void resistorOne(Scalar_ c)
    {c_[P_THONE_RESISTOR] = c;updateParameters();}
    
    /// \brief Winding thermal resistance  \f$ R_{th2} \f$
    /// at ambient temperature units: K/W
    /// Mandatory
    void resistorTwo(Scalar_ c)
    {c_[P_THTWO_RESISTOR] = c;updateParameters();}

    /// \brief Winding resistance  \f$ R_{ta} \f$
    /// at ambient temperature units: Ohms
    /// Mandatory
    void resistorTA(Scalar_ c)
    {c_[P_TA_RESISTOR] = c;updateParameters();}
      
    /// \brief Max. permissible winding temperature  \f$ T_{max} \f$
    /// catalog value
    /// units: C
    /// Optional: it is allowing to compute max overload factor and
    /// and current.
    void maxPermissibleWindingTemp(Scalar_ c)
    {c_[P_MAX_PERM_WINDING_T] = c;}

    /// \brief Therm. time constant winding \f$ \tau_W \f$
    /// units: seconds (s)
    /// Optional: it is allowing to compute max overload factor and
    /// and current.
    void thermTimeCstWinding(Scalar_ c)
    {c_[P_THERM_TIME_CST_WINDING] = c; updateParameters();}

    /// \brief Ambient temperature for the motor characteristics. \f$ T_A \f$
    /// Typically \f$ 25 C \f$
    /// Mandatory
    void thermAmbient(Scalar_ c)
    {c_[P_AMBIENT_TEMP] = c; updateParameters();}

    /// \brief Nominal current
    void nominalCurrent(Scalar_ c)
    { c_[P_NOMINAL_CURRENT] = c; updateParameters();}

    /// \brief Thermal dissipation \f$ D_T \f$
    /// units: W/K
    void thermDissipation(Scalar_ c)
    { c_[P_HEAT_DISSIPATION] = c; updateParameters();}

    /// \brief Heat Capacity \f$ C \f$
    /// units: J/K
    void heatCapacity(Scalar_ c)
    { c_[P_HEAT_CAPACITY] = c; updateParameters();}

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
    /// \f[ K = \frac{I_{mot}}{I_N} \sqrt{ \frac{T_{max} -T_{A}}{T_{max}-T_S} \frac{R_{th1}}{R_{th1}+R_{th2}} } \f] 
    Scalar_ getOverloadForCurrent(Scalar_ aCurrent, Scalar_ TS)
    {
      return aCurrent/c_[P_NOMINAL_CURRENT]*
	sqrt((c_[P_MAX_PERM_WINDING_T]- c_[P_AMBIENT_TEMP])/
	     (c_[P_MAX_PERM_WINDING_T] - TS) *
	     (c_[P_THONE_RESISTOR]/(c_[P_THONE_RESISTOR] + c_[P_THTWO_RESISTOR])));
    }

    /// \brief Returns the maximum ON time at a given overload factor K
    /// Implements
    /// \f[ t_{on} = \tau_{W} ln \frac{K^2}{K^2-1} \f]
    Scalar_ getMaxONTimeForK(Scalar_ aK)
    {
      return c_[P_THERM_TIME_CST_WINDING] * log(aK*aK/(aK*aK-1.0));
    }

    /// \brief Returns the maximum current \f$ I_{mot}\f$ at a given overload factor \f$ K\f$ and the current stator
    /// temperature (\f$ T_S \f$).
    /// Implements
    /// \f[ I_{mot} = K I_n \sqrt{ \frac{T_{max}-T_S}{T_{max} -T_{A}} \frac{R_{th1}+R_{th2}}{R_{th1}} }\f]
    Scalar_ getMaxCurrentForK(Scalar_ aK, Scalar_ TS)
    {
      return aK * c_[P_NOMINAL_CURRENT]*sqrt((c_[P_MAX_PERM_WINDING_T] - TS)/
					     (c_[P_MAX_PERM_WINDING_T]- c_[P_AMBIENT_TEMP]) *
					     (c_[P_THONE_RESISTOR] + c_[P_THTWO_RESISTOR])/(c_[P_THONE_RESISTOR]));
    }
    
    Observations_t & h() { return h_;}
    
  protected:
    /// Update the first three parameters of the actuators:
    void updateParameters()
    {
      // c_0 = torque_cst / rotorInertia
      c_[0] = c_[P_TORQUE_CST]/c_[P_ROTOR_INERTIA];

      // c_1 = -speedTorqueGrad/rotorInertia
      c_[1] = -  c_[P_SPEED_TORQUE_GRD]/c_[P_ROTOR_INERTIA];
    
      // c_2 = -1/rotorInertia
      c_[2] = -1/c_[P_ROTOR_INERTIA];
      
      // c_3 = R_Ta
      c_[3] = c_[P_TA_RESISTOR];

      // c_4 = C
      c_[4] = c_[P_HEAT_CAPACITY];

      // c_5 = -C\alpha_{Cu}(R_{th1}+R_{th2})R_{TA} 
      c_[5] = - alpha_cu_ *(c_[P_THONE_RESISTOR] + c_[P_THTWO_RESISTOR])*c_[3]*c_[4];

      // c_5 = -D_T/C
      c_[6] = -c_[P_HEAT_DISSIPATION]/c_[4];

      // c_6 = D_T T_A/C
      c_[7] = c_[P_HEAT_DISSIPATION] * c_[P_AMBIENT_TEMP]/c_[4];

    }

    /// Observation variables
    Observations_t h_;
    
    /// Vector parameters.
    Parameters_t c_;
    
    /// Selection matrix
    S_t S_;

    /// Copper resistance
    double alpha_cu_;
    
  };

  /**
   *  @brief Template which implements a second order linear DC motor model.
   *  \ingroup actuators_motor_group_sl_dc_motor
   *
   *  @tparam Scalar_ {description} 
   */
  template< typename Scalar_> 
  class ActuatorDCTempCurrentNLMotorModel : ActuatorModelBase<ActuatorDCTempCurrentNLMotorModel<Scalar_> >
  {

    typedef Eigen::Matrix<Scalar_,3,1,0> Vector3Scalar;
    
  public:
    typedef Scalar_ Scalar_t;
    ActuatorDCTempCurrentNLMotorModel() {}

    void calc(typename ActuatorDCTempCurrentNLMotorData<Scalar_>::dX_t & dstate,
	      typename ActuatorDCTempCurrentNLMotorData<Scalar_>::X_t & state,
	      typename ActuatorDCTempCurrentNLMotorData<Scalar_>::U_t & control,
	      Force & fext,
	      ActuatorDCTempCurrentNLMotorData<Scalar_> &data)
    {
      // Update dstate
      dstate[0] = state[1];
      dstate[1] = data.c()[0] * control[0] + data.c()[1] * state[1] + data.c()[2]*data.S().dot(fext.toVector());
      dstate[2] = data.c()[3]*control[0]*control[0]/(1+data.c()[4]*control[0]*control[0])
	- data.c()[5] * ( state[2] - data.c()[ActuatorDCTempCurrentNLMotorData<Scalar_>::P_AMBIENT_TEMP]);
      // Update observation
      // Current
      data.h()[0] = data.c()[ActuatorDCTempCurrentNLMotorData<Scalar_>::P_TORQUE_CST]*control[0];
      // Motor torque
      data.h()[1] = data.c()[ActuatorDCTempCurrentNLMotorData<Scalar_>::P_BACK_EMF]*state[1];
    }
    

    void get_force(ActuatorDCTempCurrentNLMotorData<Scalar_> &data, Force &aForce) const
    {
      aForce.linear(Vector3Scalar::Zero(3,1));
      aForce.angular(Vector3Scalar(0.0,0.0,data.h()[1]));
    }

    ActuatorDCTempCurrentNLMotorData<Scalar_> createData() const
    { return ActuatorDCTempCurrentNLMotorData<Scalar_>(); };
    
  protected:
  
    // Store actuator name.
    std::string name_;

  };
  
  template <typename Scalar_>
  struct adb_traits<ActuatorDCTempCurrentNLMotorData<Scalar_> >
  {
    typedef ActuatorDCTempCurrentNLMotorData<Scalar_> ActuatorDataDerived;
  };

  template <typename Scalar_>
  struct amb_traits<ActuatorDCTempCurrentNLMotorModel<Scalar_> >
  {
    typedef  ActuatorDCTempCurrentNLMotorModel<Scalar_> ActuatorModelDerived;
    typedef  ActuatorDCTempCurrentNLMotorData<Scalar_> ActuatorDataDerived;
  };


} // end of namespace se3

#endif /* _se3_temp_non_linear_motor_model_ */
