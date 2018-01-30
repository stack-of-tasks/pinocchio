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

#ifndef _se3_temp_non_linear_motor_model_hpp_
#define _se3_temp_non_linear_motor_model_hpp_

#include <iostream>
#include "pinocchio/macros.hpp"
#include "pinocchio/actuators/actuator-model.hpp"


namespace se3
{
  /** \addtogroup actuators_motor_group_tnl_dc_motor Non linear motor model (temperature)
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
      \frac{K_m}{B_m R_{TA}(1-\alpha_{Cu} ( T-T_{A}))}V - \left(\frac{K_m K_b}{B_mR_{TA}(1-\alpha_{Cu} ( T-T_{A}))} +
      \frac{D_m}{B_m}\right) \dot{\theta}_m -\frac{\tau_l}{B_m} \\
      \frac{(R_{th1} + R_{th2})(V-K_b \dot{\theta}_m)^2}{(1+\alpha_{Cu}(T-T_{A})) R_{TA}-\alpha_{cu}(R_{th1}+R_{th2})(V-K_b \dot{\theta}_m)^2}
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
      \f$ K_b \f$ the back E.M.F. constant.

      Note that the winding resistor is increasing linearly with the temperature and is given by:
      \f[ R = R_{TA} (1 + \alpha_{Cu} (T -T_{A}))\f]
      and the current is given by:
      \f[ i_a = K_m \frac{V}{R} \f]
      
      As the dynamics can be also written:
      \f[
      \left(
      \begin{matrix}
      \dot{x}_0 \\
      \dot{x}_1 \\
      \dot{T} \\
      \end{matrix}
      \right)
      = 
      \left(
      \begin{matrix}
      x_1 \\
      \frac{c_{0}}{c_1 +c_2 T} V + \frac{c_{3} + c_4 T }{c_1 + c_2 T} \dot{\theta}_m + c_5 \tau_l \\
      \frac{c_6 (V- c_7 \dot{\theta}_m)^2}{c_8 + c_{9} T + c_{10} (V- c_7 \dot{\theta}_m)^2} \\
      \end{matrix}
      \right)
      = 
      \left(
      \begin{matrix}
      x_1 \\
      \frac{c_{0}}{c_1 +c_2 x_2} u + \frac{c_{3} + c_4 x_2 }{c_1 + c_2 x_2} x_1 + c_5 \bf{S}\bf{f}_{ext} \\
      \frac{c_6 (u- c_7 x_1)^2}{c_8 + c_{9} x_2 + c_{10} (u- c_7 x_1)^2} \\
      \end{matrix}
      \right)
      \f]
      
      Thus 
      \f[ 
      \begin{array}{rcl}
      c_0 &= & K_m\\
      c_1 &= & B_m R_{TA} (1+\alpha_{Cu} T_{A})\\
      c_2 &= & -B_m R_{TA} \alpha_{Cu} \\
      c_3 &= & K_m K_B + D_m R_{TA} (1 + \alpha_{Cu} T_{A}) \\
      c_4 &= & -D_m R_{TA} \alpha_{Cu} \\
      c_5 &= & -\frac{1}{B_m} \\
      c_6 &= & (R_{th1}+R_{th2})\\
      c_7 &= & -K_b\\
      c_8 &= & R_{TA} - \alpha_{Cu} T_A R_{TA} \\
      c_{9} &=&  \alpha_{Cu} R_{TA} \\
      c_{10} &=& -\alpha_{Cu}(R_{th1}+R_{th2}) \\
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
  class ActuatorDCTempNonLinearMotorData : ActuatorDataBase<ActuatorDCTempNonLinearMotorData<Scalar_> >
  {
  public:
    typedef Scalar_ Scalar_t;
    typedef typename Eigen::Matrix<Scalar_, 22,1 > Parameters_t;
    typedef typename Eigen::Matrix<Scalar_, 3,1> Observations_t;
    typedef typename Eigen::Matrix<Scalar_, 6,1> S_t;
    typedef typename Eigen::Matrix<Scalar_, 3,1> X_t;
    typedef typename Eigen::Matrix<Scalar_, 3,1> dX_t;
    typedef typename Eigen::Matrix<Scalar_, 1,1> U_t;

    enum InternalParameters
      {
	P_ROTOR_INERTIA=11,
	P_TORQUE_CST,
	P_SPEED_TORQUE_GRD,
	P_BACK_EMF,
	P_THONE_RESISTOR,
	P_THTWO_RESISTOR,
	P_TA_RESISTOR,
	P_MAX_PERM_WINDING_T,
	P_THERM_TIME_CST_WINDING,
	P_AMBIENT_TEMP,
	P_NOMINAL_CURRENT
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

    /// \brief Ambient temperature for the motor characteristics.
    /// Typically \f$ 25 C \f$
    /// Mandatory
    void thermAmbient(Scalar_ c)
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
      // c_1 = torque_cst 
      c_[0] = c_[P_TORQUE_CST];

      // c_2 = B_m R_{TA} (1+T_A \alpha_{Cu} )
      c_[1] = c_[P_ROTOR_INERTIA] * c_[P_TA_RESISTOR]*
	(1+ c_[P_AMBIENT_TEMP] * alpha_cu_);
    
      // c_3 = -B_m R_{TA} \alpha_{Cu}
      c_[2] = -c_[P_ROTOR_INERTIA] * c_[P_TA_RESISTOR] * alpha_cu_;
      
      // c_4 = K_B + D_m R_{TA} + D_m R_{TA} \alpha_{Cu} T_{A}
      c_[3] = c_[P_BACK_EMF] * c_[P_TORQUE_CST] + c_[P_SPEED_TORQUE_GRD]*c_[P_TA_RESISTOR] *
	(1+alpha_cu_ * c_[P_AMBIENT_TEMP]);

      // c_5 = -D_m R_{TA} \alpha_{Cu}
      c_[4] = -c_[P_SPEED_TORQUE_GRD] * c_[P_TA_RESISTOR] * alpha_cu_;

      // c_6 = -frac{1}{B_m}
      c_[5] = -1/c_[P_ROTOR_INERTIA];

      // c_7 = (R_{th1} + R_{th2})
      c_[6] = c_[P_THONE_RESISTOR] + c_[P_THTWO_RESISTOR];

      // c_8 = -K_b
      c_[7] = -c_[P_BACK_EMF];

      /// \f$ c_9 = R_{TA} - \alpha_{Cu} T_A R_{TA} \f$
      c_[8] = c_[P_TA_RESISTOR] - alpha_cu_ * c_[P_AMBIENT_TEMP] * c_[P_TA_RESISTOR];

      // \f$ c_{10} = \alpha_{Cu} R_{TA} \f$
      c_[9] = alpha_cu_ *c_[P_TA_RESISTOR];

      // \f$ c_{11} = -\alpha_{Cu} (R_{th1} + R_{th2}) \f$
      c_[10] = alpha_cu_ * (c_[P_THONE_RESISTOR] + c_[P_THTWO_RESISTOR]);
      
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
  class ActuatorDCTempNonLinearMotorModel : ActuatorModelBase<ActuatorDCTempNonLinearMotorModel<Scalar_> >
  {

    typedef Eigen::Matrix<Scalar_,3,1,0> Vector3Scalar;
    
  public:
    typedef Scalar_ Scalar_t;
    ActuatorDCTempNonLinearMotorModel() {}

    void calc(typename ActuatorDCTempNonLinearMotorData<Scalar_>::dX_t & dstate,
	      typename ActuatorDCTempNonLinearMotorData<Scalar_>::X_t & state,
	      typename ActuatorDCTempNonLinearMotorData<Scalar_>::U_t & control,
	      Force & fext,
	      ActuatorDCTempNonLinearMotorData<Scalar_> &data)
    {
      // Update dstate
      dstate[0] = state[1];
      dstate[1] = data.c()[0]/(data.c()[1] + data.c()[2]*state[2]) * control[0] +
	state[1]*(data.c()[3]+data.c()[4]*state[2])/(data.c()[1] + data.c()[2]*state[2])
	+ data.c()[5] *data.S().dot(fext.toVector());
      double umc7x1 = control[0]-data.c()[7]*state[1];
      
      dstate[2] = data.c()[6]*umc7x1*umc7x1/
	(data.c()[8] + data.c()[9]*state[2]+ data.c()[10]*umc7x1*umc7x1);
      
      // Update observation
      // Current
      data.h()[0] = control[0] /data.c()[ActuatorDCTempNonLinearMotorData<Scalar_>::P_THONE_RESISTOR ]
	- data.c()[ActuatorDCTempNonLinearMotorData<Scalar_>::P_BACK_EMF] *state[1]/
	data.c()[ActuatorDCTempNonLinearMotorData<Scalar_>::P_THONE_RESISTOR ];
      // Motor torque
      data.h()[1] = data.c()[ActuatorDCTempNonLinearMotorData<Scalar_>::P_TORQUE_CST] *data.h()[0];
      // Back emf potential 
      data.h()[2] = data.c()[ActuatorDCTempNonLinearMotorData<Scalar_>::P_BACK_EMF] * state[1];
    }
    

    void get_force(ActuatorDCTempNonLinearMotorData<Scalar_> &data, Force &aForce) const
    {
      aForce.linear(Vector3Scalar::Zero(3,1));
      aForce.angular(Vector3Scalar(0.0,0.0,data.h()[1]));
    }

    ActuatorDCTempNonLinearMotorData<Scalar_> createData() const
    { return ActuatorDCTempNonLinearMotorData<Scalar_>(); };
    
  protected:
  
    // Store actuator name.
    std::string name_;

  };
  
  template <typename Scalar_>
  struct adb_traits<ActuatorDCTempNonLinearMotorData<Scalar_> >
  {
    typedef ActuatorDCTempNonLinearMotorData<Scalar_> ActuatorDataDerived;
  };

  template <typename Scalar_>
  struct amb_traits<ActuatorDCTempNonLinearMotorModel<Scalar_> >
  {
    typedef  ActuatorDCTempNonLinearMotorModel<Scalar_> ActuatorModelDerived;
    typedef  ActuatorDCTempNonLinearMotorData<Scalar_> ActuatorDataDerived;
  };


} // end of namespace se3

#endif /* _se3_temp_non_linear_motor_model_ */
