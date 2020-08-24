//
// Copyright (c) 2019 CNRS INRIA
//

#ifndef __pinocchio_center_of_mass_derivatives_hxx__
#define __pinocchio_center_of_mass_derivatives_hxx__

#include "pinocchio/multibody/visitor.hpp"
#include "pinocchio/algorithm/check.hpp"

namespace pinocchio
{
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl,
           typename Matrix3xOut>
  struct CoMVelocityDerivativesForwardStep
    : public fusion::JointUnaryVisitorBase< CoMVelocityDerivativesForwardStep<Scalar,Options,JointCollectionTpl,Matrix3xOut> >
  {
    /* Assumes that both computeForwardKinematicsDerivatives and centerOfMass (or
     * equivalent methods like computeAllTerms) have been computed beforehand. */

    typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
    typedef DataTpl<Scalar,Options,JointCollectionTpl> Data;
    
    typedef boost::fusion::vector<const Model &,
                                  Data &,
                                  Matrix3xOut &
                                  > ArgsType;
    
    template<typename JointModel>
    static void algo(const JointModelBase<JointModel> & jmodel,
                     JointDataBase<typename JointModel::JointDataDerived> & jdata,
                     const Model & model,
                     Data & data,
                     const Eigen::MatrixBase<Matrix3xOut> & vcom_partial_dq)
    {
      typedef typename Model::JointIndex JointIndex;
      typedef typename Data::Motion Motion;
      
      const JointIndex & i = jmodel.id();
      const JointIndex & parent = model.parents[i];

      Matrix3xOut & dvcom_dq = PINOCCHIO_EIGEN_CONST_CAST(Matrix3xOut,vcom_partial_dq);
      typename SizeDepType<JointModel::NV>::template ColsReturn<typename Data::Matrix3x>::Type
        dvcom_dqi = jmodel.jointCols(dvcom_dq);

      Motion & vpc = data.v[0] = (parent>0) ? (data.v[i]-(Motion)jdata.v()) : Motion::Zero();
      vpc.linear() -= data.vcom[i]; // vpc = v_{parent+c} = [ v_parent+vc; w_parent ]
      
        typename SizeDepType<JointModel::NV>::template ColsReturn<typename Data::Matrix6>::Type vxS =
      SizeDepType<JointModel::NV>::middleCols(data.M6tmp,0,jmodel.nv());
      vxS = vpc.cross(jdata.S());
      
      dvcom_dqi.noalias() = (data.mass[i]/data.mass[0])*data.oMi[i].rotation()
        *( vxS.template middleRows<3>(Motion::LINEAR) - cross( data.com[i], vxS.template middleRows<3>(Motion::ANGULAR)) );
    }
  };
  
  template<typename Scalar, int Options, template<typename,int> class JointCollectionTpl,
           typename Matrix3xOut>
  inline void getCenterOfMassVelocityDerivatives(const ModelTpl<Scalar,Options,JointCollectionTpl> & model,
                                                 DataTpl<Scalar,Options,JointCollectionTpl> & data,
                                                 const Eigen::MatrixBase<Matrix3xOut> & vcom_partial_dq)
  {
    EIGEN_STATIC_ASSERT_SAME_MATRIX_SIZE(Matrix3xOut,Data::Matrix3x);
    
    PINOCCHIO_CHECK_ARGUMENT_SIZE(vcom_partial_dq.cols(),  model.nv);
    assert(model.check(data) && "data is not consistent with model.");
    
    typedef ModelTpl<Scalar,Options,JointCollectionTpl> Model;
    typedef typename Model::JointIndex JointIndex;

    Matrix3xOut & dvcom_dq = PINOCCHIO_EIGEN_CONST_CAST(Matrix3xOut,vcom_partial_dq);
      
    typedef CoMVelocityDerivativesForwardStep<Scalar,Options,JointCollectionTpl,Matrix3xOut> Pass1;
    for(JointIndex i = 1; i < (JointIndex)model.njoints; i ++)
    {
      Pass1::run(model.joints[i],data.joints[i],
                 typename Pass1::ArgsType(model,data,dvcom_dq));
    }
    
    data.v[0].setZero();
  }

} // namespace pinocchio

#endif // ifndef __pinocchio_center_of_mass_derivatives_hxx__

