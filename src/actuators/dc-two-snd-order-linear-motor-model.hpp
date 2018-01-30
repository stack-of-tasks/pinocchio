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

#ifndef _se3_two_snd_order_linear_motor_model_hpp_
#define _se3_two_snd_order_linear_motor_model_hpp_

#include <iostream>
#include "pinocchio/macros.hpp"
#include "pinocchio/actuators/actuator-model.hpp"


namespace se3
{
  /** \addtogroup actuators_motor_group_nl_dc_motor Non linear motor model.
      \ingroup actuators_motor_group 
      The model used for this non linear version of the DC motor is defined by
      the following state vector
      \f$ \mathbf{x} = [ x_0, x_1, x_2 ]^{\top} = [ \theta_m, \dot{\theta}_m, i_a ]^{\top} \f$ with \f$\theta_m\f$ the motor angle.<br>
      The ODE function \f$ \dot{\bf{x}}= f(\bf{x},\bf{u},\bf{c},\bf{f}_{ext}) \f$ describing the dynamics of the motor for a set of parameters
      \f$ \bf{c} \f$, with control vector \f$ \bf{u} \f$ and submitted to external forces \f$ \bf{f}_{ext} \f$ by the multi-body rigid dynamics is:<br>
      
      \f[
      \left(
      \begin{matrix}
      \dot{\theta}_m \\
      \ddot{\theta}_m \\
      \dot{i}_a \\
      \end{matrix}
      \right)
      = 
      \left(
      \begin{matrix}
      \dot{\theta}_m \\
      \frac{K_m}{B_m} i_a  - \frac{D_m}{B_m} \dot{\theta}_m -\frac{\tau_l}{B_m} \\
      -\frac{R}{L} i_a - \frac{K_b}{L}\dot{\theta}_m + \frac{V}{L} \\
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
      \f$ K_b \f$ the back E.M.F. constant,
      \f$L\f$ the motor inductance.

      As the dynamics can be also written:
      \f[
      \left(
      \begin{matrix}
      \dot{x}_0 \\
      \dot{x}_1 \\
      \dot{x}_2 \				\
      \end{matrix}
      \right)
      = 
      \left(
      \begin{matrix}
      x_1 \\
      c_{1} i_a + c_{2} x_1 + c_3 \tau_l \\
      c_4 i_a + c_5 x_1 + c_6 V
      \end{matrix}
      \right)
      = 
      \left(
      \begin{matrix}
      x_1 \\
      c_{1} x_2 + c_{2} x_1 + c_3 \bf{S}\bf{f}_{ext} \\
      c_{4} x_2 + c_5 x_1 + c_6 u\\
      \end{matrix}
      \right)
      \f]
      
      Thus 
      \f[ 
      \begin{array}{rcl}
      c_1 &= &\frac{K_m}{B_m}\\
      c_2 &= & - \frac{D_m}{B_m}\\
      c_3 &= & -\frac{1}{B_m}\\
      c_4 &= & -\frac{R}{L} \\
      c_5 &= & -\frac{K_b}{L} \\
      c_6 &= & \frac{1}{L} \\
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
      K_m i_a\\
      K_b \dot{\theta}_m\\
      \end{matrix}
      \right)
      \f]
   */

  /**
   *  @brief Template for handling data related to a non linear DC motor model.
   *  \ingroup actuators_motor_group_nl_dc_motor
   *
   *  @tparam Scalar_ {description} 
   */
  template<typename Scalar_> 
  class ActuatorDCTwoSndOrderLinearMotorData :
    ActuatorDataBase<ActuatorDCTwoSndOrderLinearMotorData<Scalar_> >
  {
  public:
    typedef Scalar_ Scalar_t;
    typedef typename Eigen::Matrix<Scalar_, 12,1 > Parameters_t;
    typedef typename Eigen::Matrix<Scalar_, 2,1> Observations_t;
    typedef typename Eigen::Matrix<Scalar_, 6,1> S_t;
    typedef typename Eigen::Matrix<Scalar_, 3,1> X_t;
    typedef typename Eigen::Matrix<Scalar_, 3,1> dX_t;
    typedef typename Eigen::Matrix<Scalar_, 1,1> U_t;

    /// Return observation vector.
    const Observations_t & h() const { return h_;}
    
    /// Return parameters 
    const Parameters_t & c() const { return c_;}
    
    /// Returns selection matrix.
    const S_t & S() const { return S_;}

    enum InternalParameters
      {
	P_ROTOR_INERTIA=6,
	P_ROTOR_RESISTOR,
	P_TORQUE_CST,
	P_SPEED_TORQUE_GRD,
	P_BACK_EMF,
	P_TERMINAL_INDUCTANCE
      };
    ///@{ Internal parameters of the actuator.
    /// \brief Rotor inertia (kg/m2) - \f$ B_m \f$
    void rotorInertia(Scalar_ c)
    { c_[P_ROTOR_INERTIA] = c; updateFirstParameters();}

    /// \brief Rotor resistor (Ohm) - \f$ R\f$
    void rotorResistor(Scalar_ c)
    { c_[P_ROTOR_RESISTOR] = c; updateFirstParameters();}
    
    /// \brief Torque constant (Nm/A) - \f$ K_m \f$
    void torqueConst(Scalar_ c)
    { c_[P_TORQUE_CST] = c; updateFirstParameters();}
    
    /// \brief Speed torque gradient (rads^-1/Nm) \f$ D_m \f$
    void speedTorqueGrad(Scalar_ c) 
    {c_[P_SPEED_TORQUE_GRD] = c; updateFirstParameters();}
    
    /// \brief Back electro magnetic force constant \f$ K_b \f$
    /// It is the inverse of the speed torque gradient constant
    /// (\f$k_N\f$) in Maxon datasheet.
    /// More exactly the maxon datasheet gives rpm/V.
    /// So it is \f$\frac{60}{ 2 \pi k_N)}\f$
    void backEMF(Scalar_ c) 
    {c_[P_BACK_EMF] = c;updateFirstParameters();}

    /// \brief Terminal Inductance (Henry)
    void terminalInductance(Scalar_ c)
    {c_[P_TERMINAL_INDUCTANCE] = c; updateFirstParameters();}
    

    void setS(S_t &S)
    {S_ = S;}
    ///@}

    Observations_t & h() { return h_;}
    
  protected:
    /// Update the first three parameters of the actuators:
    void updateFirstParameters()
    {
      // c_1 = torque_cst 
      c_[0] = c_[P_TORQUE_CST]/c_[P_ROTOR_INERTIA];
      // c_2 = - speedTorqueGrad/rotorInertia
      c_[1] = -c_[P_SPEED_TORQUE_GRD]/c_[P_ROTOR_INERTIA];
      // c_3 = -1/rotorInertia
      c_[2] = -1/c_[P_ROTOR_INERTIA];
      // c_4 = rotorResistor/terminalInductance
      c_[3] = -c_[P_ROTOR_RESISTOR]/c_[P_TERMINAL_INDUCTANCE];
      // c_5 = - backEMF/terminalInductance
      c_[4] = -c_[P_BACK_EMF]/c_[P_TERMINAL_INDUCTANCE];
      // c_6 = 1/terminalInductance
      c_[5] = 1/c_[P_TERMINAL_INDUCTANCE];
    }

    /// Observation variables
    Observations_t h_;
    
    /// Vector parameters.
    Parameters_t c_;
    
    /// Selection matrix
    S_t S_;
    
  };

  /**
   *  @brief Template implementing a non linear DC motor model.
   *  \ingroup actuators_motor_group_nl_dc_motor
   *
   *  @tparam Scalar_ {description} 
   */
  template< typename Scalar_> 
  class ActuatorDCTwoSndOrderLinearMotorModel :
    ActuatorModelBase<ActuatorDCTwoSndOrderLinearMotorModel<Scalar_> >
  {

    typedef Eigen::Matrix<Scalar_,3,1,0> Vector3Scalar;
    
  public:
    typedef Scalar_ Scalar_t;
    ActuatorDCTwoSndOrderLinearMotorModel() {}

    void calc(typename ActuatorDCTwoSndOrderLinearMotorData<Scalar_>::dX_t & dstate,
	      typename ActuatorDCTwoSndOrderLinearMotorData<Scalar_>::X_t & state,
	      typename ActuatorDCTwoSndOrderLinearMotorData<Scalar_>::U_t & control,
	      Force & fext,
	      ActuatorDCTwoSndOrderLinearMotorData<Scalar_> &data)
    {
      // Update dstate
      // dtheta/dt = dtheta/dt
      dstate[0] = state[1];
      // ddtheta/ddt = 
      dstate[1] = data.c()[0] * state[2] + data.c()[1] * state[1] +
	data.c()[2] *data.S().dot(fext.toVector());
      /// di/dt = (-Ri + V - K_b dtheta/dt)/L
      dstate[2] = data.c()[3] * state[2] + data.c()[4]* state[1] + data.c()[5] * control[0];
	
      // Update observation
      // Motor torque
      data.h()[0] = data.c()[ActuatorDCTwoSndOrderLinearMotorData<Scalar_>::P_TORQUE_CST] * state[2];
      // Back emf potential 
      data.h()[1] = data.c()[ActuatorDCTwoSndOrderLinearMotorData<Scalar_>::P_BACK_EMF] * state[1];
    }
    

    void get_force(ActuatorDCTwoSndOrderLinearMotorData<Scalar_> &data,
		   Force &aForce) const
    {
      aForce.linear(Vector3Scalar::Zero(3,1));
      aForce.angular(Vector3Scalar(0.0,0.0,data.h()[1]));
    }

    ActuatorDCTwoSndOrderLinearMotorData<Scalar_> createData() const
    { return ActuatorDCTwoSndOrderLinearMotorData<Scalar_>(); };
    
  protected:
  
    // Store actuator name.
    std::string name_;

  };
  
  template <typename Scalar_>
  struct adb_traits<ActuatorDCTwoSndOrderLinearMotorData<Scalar_> >
  {
    typedef ActuatorDCTwoSndOrderLinearMotorData<Scalar_> ActuatorDataDerived;
  };

  template <typename Scalar_>
  struct amb_traits<ActuatorDCTwoSndOrderLinearMotorModel<Scalar_> >
  {
    typedef  ActuatorDCTwoSndOrderLinearMotorModel<Scalar_> ActuatorModelDerived;
    typedef  ActuatorDCTwoSndOrderLinearMotorData<Scalar_> ActuatorDataDerived;
  };


} // end of namespace se3

#endif /* _se3_two_snd_order_linear_motor_model_ */
