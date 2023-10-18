//
// Copyright (c) 2019 INRIA
//

#ifndef __pinocchio_multibody_data_serialization_hpp__
#define __pinocchio_multibody_data_serialization_hpp__

#include <boost/serialization/variant.hpp>
#include <boost/serialization/vector.hpp>

#include "pinocchio/serialization/fwd.hpp"
#include "pinocchio/serialization/aligned-vector.hpp"
#include "pinocchio/serialization/spatial.hpp"
#include "pinocchio/serialization/joints.hpp"
#include "pinocchio/serialization/frame.hpp"

#define PINOCCHIO_MAKE_DATA_NVP(ar,data,field_name) \
  ar & make_nvp(#field_name,data.field_name)

namespace boost
{
  namespace serialization
  {
    template<class Archive, typename Scalar, int Options, template<typename,int> class JointCollectionTpl>
    void serialize(Archive & ar,
                   pinocchio::DataTpl<Scalar,Options,JointCollectionTpl> & data,
                   const unsigned int /*version*/)
    {
      PINOCCHIO_MAKE_DATA_NVP(ar,data,joints);
      PINOCCHIO_MAKE_DATA_NVP(ar,data,a);
      PINOCCHIO_MAKE_DATA_NVP(ar,data,oa);
      PINOCCHIO_MAKE_DATA_NVP(ar,data,a_gf);
      PINOCCHIO_MAKE_DATA_NVP(ar,data,oa_gf);
      PINOCCHIO_MAKE_DATA_NVP(ar,data,v);
      PINOCCHIO_MAKE_DATA_NVP(ar,data,ov);
      PINOCCHIO_MAKE_DATA_NVP(ar,data,f);
      PINOCCHIO_MAKE_DATA_NVP(ar,data,of);
      PINOCCHIO_MAKE_DATA_NVP(ar,data,h);
      PINOCCHIO_MAKE_DATA_NVP(ar,data,oh);
      PINOCCHIO_MAKE_DATA_NVP(ar,data,oMi);
      PINOCCHIO_MAKE_DATA_NVP(ar,data,liMi);
      PINOCCHIO_MAKE_DATA_NVP(ar,data,tau);
      PINOCCHIO_MAKE_DATA_NVP(ar,data,nle);
      PINOCCHIO_MAKE_DATA_NVP(ar,data,g);
      PINOCCHIO_MAKE_DATA_NVP(ar,data,oMf);
      PINOCCHIO_MAKE_DATA_NVP(ar,data,Ycrb);
      PINOCCHIO_MAKE_DATA_NVP(ar,data,dYcrb);
      PINOCCHIO_MAKE_DATA_NVP(ar,data,M);
      PINOCCHIO_MAKE_DATA_NVP(ar,data,Minv);
      PINOCCHIO_MAKE_DATA_NVP(ar,data,C);
      PINOCCHIO_MAKE_DATA_NVP(ar,data,dHdq);
      PINOCCHIO_MAKE_DATA_NVP(ar,data,dFdq);
      PINOCCHIO_MAKE_DATA_NVP(ar,data,dFdv);
      PINOCCHIO_MAKE_DATA_NVP(ar,data,dFda);
      PINOCCHIO_MAKE_DATA_NVP(ar,data,SDinv);
      PINOCCHIO_MAKE_DATA_NVP(ar,data,UDinv);
      PINOCCHIO_MAKE_DATA_NVP(ar,data,IS);
      PINOCCHIO_MAKE_DATA_NVP(ar,data,vxI);
      PINOCCHIO_MAKE_DATA_NVP(ar,data,Ivx);
      PINOCCHIO_MAKE_DATA_NVP(ar,data,B);
      PINOCCHIO_MAKE_DATA_NVP(ar,data,oinertias);
      PINOCCHIO_MAKE_DATA_NVP(ar,data,oYcrb);
      PINOCCHIO_MAKE_DATA_NVP(ar,data,doYcrb);
      PINOCCHIO_MAKE_DATA_NVP(ar,data,ddq);
      PINOCCHIO_MAKE_DATA_NVP(ar,data,Yaba);
      PINOCCHIO_MAKE_DATA_NVP(ar,data,u);
      PINOCCHIO_MAKE_DATA_NVP(ar,data,Ag);
      PINOCCHIO_MAKE_DATA_NVP(ar,data,dAg);
      PINOCCHIO_MAKE_DATA_NVP(ar,data,hg);
      PINOCCHIO_MAKE_DATA_NVP(ar,data,dhg);
      PINOCCHIO_MAKE_DATA_NVP(ar,data,Ig);
      PINOCCHIO_MAKE_DATA_NVP(ar,data,Fcrb);
      PINOCCHIO_MAKE_DATA_NVP(ar,data,lastChild);
      PINOCCHIO_MAKE_DATA_NVP(ar,data,nvSubtree);
      PINOCCHIO_MAKE_DATA_NVP(ar,data,start_idx_v_fromRow);
      PINOCCHIO_MAKE_DATA_NVP(ar,data,end_idx_v_fromRow);
      PINOCCHIO_MAKE_DATA_NVP(ar,data,U);
      PINOCCHIO_MAKE_DATA_NVP(ar,data,D);
      PINOCCHIO_MAKE_DATA_NVP(ar,data,Dinv);
      PINOCCHIO_MAKE_DATA_NVP(ar,data,parents_fromRow);
      PINOCCHIO_MAKE_DATA_NVP(ar,data,supports_fromRow);
      PINOCCHIO_MAKE_DATA_NVP(ar,data,nvSubtree_fromRow);
      PINOCCHIO_MAKE_DATA_NVP(ar,data,J);
      PINOCCHIO_MAKE_DATA_NVP(ar,data,dJ);
      PINOCCHIO_MAKE_DATA_NVP(ar,data,ddJ);
      PINOCCHIO_MAKE_DATA_NVP(ar,data,psid);
      PINOCCHIO_MAKE_DATA_NVP(ar,data,psidd);
      PINOCCHIO_MAKE_DATA_NVP(ar,data,dVdq);
      PINOCCHIO_MAKE_DATA_NVP(ar,data,dAdq);
      PINOCCHIO_MAKE_DATA_NVP(ar,data,dAdv);
      PINOCCHIO_MAKE_DATA_NVP(ar,data,dtau_dq);
      PINOCCHIO_MAKE_DATA_NVP(ar,data,dtau_dv);
      PINOCCHIO_MAKE_DATA_NVP(ar,data,ddq_dq);
      PINOCCHIO_MAKE_DATA_NVP(ar,data,ddq_dv);
      PINOCCHIO_MAKE_DATA_NVP(ar,data,iMf);
      PINOCCHIO_MAKE_DATA_NVP(ar,data,com);
      PINOCCHIO_MAKE_DATA_NVP(ar,data,vcom);
      PINOCCHIO_MAKE_DATA_NVP(ar,data,acom);
      PINOCCHIO_MAKE_DATA_NVP(ar,data,mass);
      PINOCCHIO_MAKE_DATA_NVP(ar,data,Jcom);
      PINOCCHIO_MAKE_DATA_NVP(ar,data,kinetic_energy);
      PINOCCHIO_MAKE_DATA_NVP(ar,data,potential_energy);
      PINOCCHIO_MAKE_DATA_NVP(ar,data,JMinvJt);
      PINOCCHIO_MAKE_DATA_NVP(ar,data,lambda_c);
      PINOCCHIO_MAKE_DATA_NVP(ar,data,torque_residual);
      PINOCCHIO_MAKE_DATA_NVP(ar,data,dq_after);
      PINOCCHIO_MAKE_DATA_NVP(ar,data,impulse_c);
      PINOCCHIO_MAKE_DATA_NVP(ar,data,staticRegressor);
      PINOCCHIO_MAKE_DATA_NVP(ar,data,bodyRegressor);
      PINOCCHIO_MAKE_DATA_NVP(ar,data,jointTorqueRegressor);
      PINOCCHIO_MAKE_DATA_NVP(ar,data,d2tau_dqdq);
      PINOCCHIO_MAKE_DATA_NVP(ar,data,d2tau_dvdv);
      PINOCCHIO_MAKE_DATA_NVP(ar,data,d2tau_dqdv);
      PINOCCHIO_MAKE_DATA_NVP(ar,data,d2tau_dadq);
      PINOCCHIO_MAKE_DATA_NVP(ar,data,kinematic_hessians);
    }
    
  } // namespace serialization
} // namespace boost

#undef PINOCCHIO_MAKE_DATA_NVP

#endif // ifndef __pinocchio_multibody_data_serialization_hpp__

