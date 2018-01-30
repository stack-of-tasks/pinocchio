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

#ifndef _ACTUATOR_MODEL_HH_
#define _ACTUATOR_MODEL_HH_

#include<string>

#include "pinocchio/spatial/force.hpp"

namespace se3
{
  /*! \addtogroup actuators_group Actuators
      An actuator model is defined by a state vector \f$ \mathbf{x} \f$ and an ordinary differential equation 
      \f$ \dot{\bf{x}}= f(\bf{x}, \bf{u}, \bf{c},\bf{f}_{ext}) \f$ describing its dynamics for a set of
      parameters \f$ \bf{c} \f$, with control vector \f$ \bf{u} \f$ and submitted to external forces \f$ \bf{f}_{ext} \f$ by the multi-body rigid dynamics.

      The dynamics is given by the method #ActuatorModelBase::ode_func().

      The force produced by the actuator is given by #ActuatorModelBase::get_force().
      
      The observations are given by the method #ActuatorModelBase::h().

      The parameters are provided by the method #ActuatorModelBase::c().
      
      The state type is given by ActuatorModelBase::X_t,
      the type of the state derivative is ActuatorModelBase::dX_t,
      the control type is given by ActuatorModelBase::U_t,
      the selection matrix type is given by ActuatorModelBase::S_t,
      the observation type is ActuatorModelBase::h_t,
      the parameters type is ActuatorModelBase::Parameters_t.

      The semantic of each type is specific to each implementation.
   */
  template<typename C> struct amb_traits;
  template<typename C> struct adb_traits;
  
  template<typename Derived>
  struct ActuatorDataBase
  {
    typedef typename adb_traits<Derived>::ActuatorDataDerived ActuatorDataDerived;
    ActuatorDataDerived& derived() { return *static_cast<Derived*>(this);}
    const ActuatorDataDerived & derived()
      const { return *static_cast<const Derived *>(this);}
  }; // struct ActuatorDataBase
  
  template<typename Derived>
  struct ActuatorModelBase
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    typedef typename amb_traits<Derived>::ActuatorModelDerived ActuatorModelDerived;
    typedef typename amb_traits<Derived>::ActuatorDataDerived ActuatorDataDerived;
    typedef typename ActuatorDataDerived::Scalar_t Scalar_t;
    typedef typename ActuatorDataDerived::X_t X_t;
    typedef typename ActuatorDataDerived::dX_t dX_t;
    typedef typename ActuatorDataDerived::U_t U_t;
    typedef typename ActuatorDataDerived::S_t S_t;
    typedef typename ActuatorDataDerived::Observations_t Observations_t;
    typedef typename ActuatorDataDerived::Parameters_t Parameters_t;
    
    ActuatorModelDerived & derived() { return *static_cast<Derived *>(this);}
    const ActuatorModelDerived& derived() const
    { return *static_cast<const Derived*>(this); }

    ActuatorDataDerived createData() const { return derived().createData(); }

    void calc(dX_t & dstate,
	       X_t & state,
	       U_t & control,
	       Force fext,
	       ActuatorDataDerived &data) const
    { derived().calc(dstate,data,state);}

    void get_force(ActuatorDataDerived &data, Force &aForce) const
    {
      derived().get_force(data,aForce);
    };

    const Observations_t & h(ActuatorDataDerived &data, X_t & state)
      const { return derived(data,state).h; }

    const Parameters_t & c(ActuatorDataDerived &data) const
    { return derived(data).c;}

    const S_t & S() const { return derived().S;}

    const U_t & u() const { return derived().u;}

  }; // struct ActuatorModelBase
}
#endif /* _ACTUATOR_MODEL_HH_ */
