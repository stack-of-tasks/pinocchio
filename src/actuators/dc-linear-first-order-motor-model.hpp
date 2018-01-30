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

#ifndef _se3_dc_linear_first_order_motor_model_hpp_
#define _se3_dc_linear_first_order_motor_model_hpp_


#include "pinocchio/macros.hpp"
#include "pinocchio/actuators/actuator-model.hpp"


namespace se3
{
  /** \addtogroup actuators_motor_group_fl_dc_motor DC linear model (first order)
      \ingroup actuators_motor_group
      The model used for this linear version of the DC motor is defined by
      the following state vector
      \f$ \mathbf{x} = [ x_0 ]^{\top} = [ \theta_m ]^{\top} \f$ with \f$\theta_m\f$ the motor angle.<br>
      The ODE function \f$ \dot{\bf{x}}= f(\bf{x},\bf{u},\bf{c},\bf{f}_{ext}) \f$ describing the dynamics of the motor for a set of parameters
      \f$ \bf{c} \f$, with control vector \f$ \bf{u} \f$ and submitted to external forces \f$ \bf{f}_{ext} \f$ by the multi-body rigid dynamics is:<br>
      
      \f[
      \left(
      \begin{matrix}
      \dot{\theta}_m \\
      \end{matrix}
      \right)
      = 
      \left(
      \begin{matrix}
      \frac{K_m K_b}{K_b + R D_m}V  -\frac{\tau_l}{K_b + R D_m}
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
      \end{matrix}
      \right)
      = 
      \left(
      \begin{matrix}
      c_{1} V + c_2 \tau_l
      \end{matrix}
      \right)
      = 
      \left(
      \begin{matrix}
      x_1 \\
      c_{1} u + c_2 \bf{S}\bf{f}_{ext}
      \end{matrix}
      \right)
      \f]
      
      Thus 
      \f[ 
      \begin{array}{rcl}
      c_1 &= &\frac{K_b K_m}{K_b + R D_m}\\
      c_2 &= & -\frac{1}{K_b + R D_m} \\
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
   *  @brief Template for handling data related to a first order linear DC motor model.
   *  \ingroup actuators_motor_group_fl_dc_motor
   *
   *  @tparam Scalar_ {description} 
   */  
  template<typename Scalar_> 
  class ActuatorDCFirstOrderMotorData : ActuatorDataBase<ActuatorDCFirstOrderMotorData<Scalar_> >
  {
  public:
    typedef Scalar_ Scalar_t;
    typedef typename Eigen::Matrix<Scalar_, 10,1 > Parameters_t;
    typedef typename Eigen::Matrix<Scalar_, 3,1> Observations_t;
    typedef typename Eigen::Matrix<Scalar_, 6,1> S_t;
    typedef typename Eigen::Matrix<Scalar_, 2,1> X_t;
    typedef typename Eigen::Matrix<Scalar_, 2,1> dX_t;
    typedef typename Eigen::Matrix<Scalar_, 1,1> U_t;

    /// Return observation vector.
    const Observations_t & h() const { return h_;}
    
    /// Return parameters 
    const Parameters_t & c() const { return c_;}
    
    /// Returns selection matrix.
    const S_t & S() const { return S_;}

    ///@{ Internal parameters of the actuator.
    /// \brief Rotor inertia (kg/m2) - \f$ B_m \f$
    void rotorInertia(Scalar_ c)
    { c_[3] = c; updateFirstThreeParameters();}

    /// \brief Rotor resistor (Ohm) - \f$ R\f$
    void rotorResistor(Scalar_ c)
    { c_[4] = c; updateFirstThreeParameters();}
    
    /// \brief Torque constant (Nm/A) - \f$ K_m \f$
    void torqueConst(Scalar_ c)
    { c_[5] = c; updateFirstThreeParameters();}
    
    /// \brief Speed torque gradient (rads^-1/Nm) \f$ D_m \f$
    void speedTorqueGrad(Scalar_ c) 
    {c_[6] = 1/c; updateFirstThreeParameters();}
    
    /// \brief Terminal Inductance (Henry)
    //void terminalInductance(Scalar_ &c)
    //    {c_[7] = c;}
    
    /// \brief Back electro magnetic force constant \f$ K_b \f$
    /// It is the inverse of the speed torque gradient constant
    /// (\f$k_N\f$) in Maxon datasheet.
    /// More exactly the maxon datasheet gives rpm/V.
    /// So it is \f$\frac{60}{ 2 \pi k_N)}\f$
    void backEMF(Scalar_ c) 
    {c_[7] = c;}

    void setS(S_t &S)
    {S_ = S;}
    ///@}

    Observations_t & h() { return h_;}
    
  protected:
    /// Update the first three parameters of the actuators:
    void updateFirstThreeParameters()
    {
      // c_1 = torque_cst / (rotorInertia * rotorResistor)
      c_[0] = c_[5] /(c_[3] * c_[4]);
      // c_2 = - back_emf_cst  / (rotorInertia * rotorResistor)
      //        - speedTorqueGrad^{-1} / rotorInertia
      c_[1] = -c_[5]*c_[7] /(c_[5] + c_[3]*c_[6] ) ;
    }

    /// Observation variables
    Observations_t h_;
    
    /// Vector parameters.
    Parameters_t c_;
    
    /// Selection matrix
    S_t S_;
    
    /// State vector
    X_t x_;
  };

  /**
   *  @brief Template implementing a first order linear DC motor model.
   *  \ingroup actuators_motor_group_fl_dc_motor
   *
   *  @tparam Scalar_ {description} 
   */
  template< typename Scalar_> 
  class ActuatorDCFirstOrderMotorModel :
    ActuatorModelBase<ActuatorDCFirstOrderMotorModel<Scalar_> >
  {

    typedef Eigen::Matrix<Scalar_,3,1,0> Vector3Scalar;
    
  public:
    typedef Scalar_ Scalar_t;
    ActuatorDCFirstOrderMotorModel() {}

    void calc(typename ActuatorDCFirstOrderMotorData<Scalar_>::dX_t & dstate,
	      typename ActuatorDCFirstOrderMotorData<Scalar_>::X_t & state,
	      typename ActuatorDCFirstOrderMotorData<Scalar_>::U_t & control,
	      Force & fext,
	      ActuatorDCFirstOrderMotorData<Scalar_> &data)
    {
      // Update dstate
      dstate[0] = data.c()[0] * control[0] + data.c()[1] * state[0] +
	data.c()[2] *data.S().dot(fext.toVector());
      // Update observation
      // Current
      data.h()[0] = control[0] /data.c()[4] - data.c()[7] *state[1]/data.c()[4];
      // Motor torque
      data.h()[1] = data.c()[5] *data.h()[0];
      // Back emf potential 
      data.h()[2] = data.c()[7] * state[1];
    }
    

    void get_force(ActuatorDCFirstOrderMotorData<Scalar_> &data, Force &aForce) const
    {
      aForce.linear(Vector3Scalar::Zero(3,1));
      aForce.angular(Vector3Scalar(0.0,0.0,data.h()[1]));
    }

    ActuatorDCFirstOrderMotorData<Scalar_> createData() const
    { return ActuatorDCFirstOrderMotorData<Scalar_>(); };
    
  protected:
  
    // Store actuator name.
    std::string name_;

  };
  
  template <typename Scalar_>
  struct adb_traits<ActuatorDCFirstOrderMotorData<Scalar_> >
  {
    typedef ActuatorDCFirstOrderMotorData<Scalar_> ActuatorDataDerived;
  };

  template <typename Scalar_>
  struct amb_traits<ActuatorDCFirstOrderMotorModel<Scalar_> >
  {
    typedef  ActuatorDCFirstOrderMotorModel<Scalar_> ActuatorModelDerived;
    typedef  ActuatorDCFirstOrderMotorData<Scalar_> ActuatorDataDerived;
  };


} // end of namespace se3

#endif /* _se3_dc_linear_first_order_motor_model_ */
