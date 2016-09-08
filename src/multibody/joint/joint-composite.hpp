//
// Copyright (c) 2016 CNRS
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

#ifndef __se3_joint_composite_hpp__
#define __se3_joint_composite_hpp__

#include "pinocchio/assert.hpp"
#include "pinocchio/multibody/joint/joint-basic-visitors.hpp"

namespace se3
{

  struct JointComposite;
  struct JointModelComposite;
  struct JointDataComposite;

  template<>
  struct traits<JointComposite>
  {

    enum {
      NQ = Eigen::Dynamic,
      NV = Eigen::Dynamic
    };
    typedef JointDataComposite JointData;
    typedef JointModelComposite JointModel;
    typedef ConstraintXd Constraint_t;
    typedef SE3 Transformation_t;
    typedef Motion Motion_t;
    typedef Motion Bias_t;

    typedef Eigen::Matrix<double,6,Eigen::Dynamic> F_t;
    // [ABA]
    typedef Eigen::Matrix<double,6,Eigen::Dynamic> U_t;
    typedef Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> D_t;
    typedef Eigen::Matrix<double,6,Eigen::Dynamic> UD_t;

    typedef Eigen::Matrix<double,Eigen::Dynamic,1> ConfigVector_t;
    typedef Eigen::Matrix<double,Eigen::Dynamic,1> TangentVector_t;
  };
  template<> struct traits<JointDataComposite> { typedef JointComposite Joint; };
  template<> struct traits<JointModelComposite> { typedef JointComposite Joint; };

  struct JointDataComposite : public JointDataBase<JointDataComposite> 
  {
    typedef JointComposite Joint;
    SE3_JOINT_TYPEDEF;

    JointDataVector joints;
    int nq_composite,nv_composite;

    Constraint_t S;
    Transformation_t M;
    Motion_t v;
    Bias_t c;

    
    // // [ABA] specific data
    U_t U;
    D_t Dinv;
    UD_t UDinv;


    // JointDataComposite()  {} // can become necessary if we want a vector of JointDataComposite ?
    JointDataComposite( JointDataVector & joints, int nq, int nv )
    : joints(joints)
    , nq_composite(nq)
    , nv_composite(nv)
    , S(Eigen::MatrixXd::Zero(6, nv_composite))
    , M(Transformation_t::Identity())
    , v(Motion_t::Zero())
    , c(Bias_t::Zero())
    , U(Eigen::MatrixXd::Zero(6, nv_composite))
    , Dinv(Eigen::MatrixXd::Zero(nv_composite, nv_composite))
    , UDinv(Eigen::MatrixXd::Zero(6, nv_composite))
    {}

  };

  struct JointModelComposite : public JointModelBase<JointModelComposite> 
  {
    typedef JointComposite Joint;
    SE3_JOINT_TYPEDEF;
    SE3_JOINT_USE_INDEXES;
    using JointModelBase<JointModelComposite>::id;
    using JointModelBase<JointModelComposite>::setIndexes;

    std::size_t max_joints;
    JointModelVector joints;
    int nq_composite,nv_composite;

    // Same as JointModelComposite(1)
    JointModelComposite() : max_joints(1)
                          , joints(0)
                          , nq_composite(0)
                          , nv_composite(0)
                          {} 
    JointModelComposite(std::size_t max_number_of_joints) : max_joints(max_number_of_joints)
                                                          , joints(0)
                                                          , nq_composite(0)
                                                          , nv_composite(0)
                                                          {} 
    
    template <typename D1>
    JointModelComposite(const JointModelBase<D1> & jmodel1) : max_joints(1)
                                                            , joints(0)
                                                            , nq_composite(jmodel1.nq())
                                                            , nv_composite(jmodel1.nv())
    {
        joints.push_back(jmodel1.derived());
    }

    template <typename D1, typename D2 >
    JointModelComposite(const JointModelBase<D1> & jmodel1, const JointModelBase<D2> & jmodel2)
    : max_joints(2)
    , joints(0)
    , nq_composite(jmodel1.nq() + jmodel2.nq())
    , nv_composite(jmodel1.nv() + jmodel2.nv())
    {
      joints.push_back(jmodel1.derived());
      joints.push_back(jmodel2.derived());
    }
    
    template <typename D1, typename D2, typename D3 >
    JointModelComposite(const JointModelBase<D1> & jmodel1,
                        const JointModelBase<D2> & jmodel2,
                        const JointModelBase<D3> & jmodel3)
    : max_joints(3)
    , joints(0)
    , nq_composite(jmodel1.nq() + jmodel2.nq() + jmodel3.nq())
    , nv_composite(jmodel1.nv() + jmodel2.nv() + jmodel3.nv())
    {
      joints.push_back(jmodel1.derived());
      joints.push_back(jmodel2.derived());
      joints.push_back(jmodel3.derived());
    }

    // JointModelComposite( const JointModelVector & models ) : max_joints(models.size()) , joints(models) {}
    
    template < typename D>
    std::size_t addJointModel(const JointModelBase<D> & jmodel)
    {
      std::size_t nb_joints = joints.size();
      if (!isFull())
      {
        joints.push_back(jmodel.derived());
        nq_composite += jmodel.nq();
        nv_composite += jmodel.nv();
        nb_joints = joints.size();
      }
      return nb_joints;
    }

    bool isFull() const
    {
      return joints.size() == max_joints ; 
    }

    bool isEmpty() const
    {
      return joints.size() <= 0 ; 
    }
    JointData createData() const
    {
      JointDataVector res;
      for (JointModelVector::const_iterator i = joints.begin(); i != joints.end(); ++i)
      {
        res.push_back(::se3::createData(*i));
      }
      return JointData(res, nq_composite, nv_composite);
    }


    void calc (JointData & data,
               const Eigen::VectorXd & qs) const
    {
      data.M.setIdentity();
      for (JointDataVector::iterator i = data.joints.begin(); i != data.joints.end(); ++i)
      {
        JointDataVector::iterator::difference_type index = i - data.joints.begin();
        calc_zero_order(joints[(std::size_t)index], *i, qs);
        data.M =  data.M * joint_transform(*i);
      }
    }

    void calc (JointData & data,
               const Eigen::VectorXd & qs,
               const Eigen::VectorXd & vs ) const
    {
      data.M.setIdentity();
      data.v.setZero();
      for (JointDataVector::iterator i = data.joints.begin(); i != data.joints.end(); ++i)
      {
        JointDataVector::iterator::difference_type index = i - data.joints.begin();
        calc_first_order(joints[(std::size_t)index], *i, qs, vs);
        data.M =  data.M * joint_transform(*i);
      }

      SE3 iMcomposite(SE3::Identity());
      data.v = motion(data.joints[joints.size()-1]);
      int start_col = nv_composite;
      int sub_constraint_dimension = (int)constraint_xd(data.joints[joints.size()-1]).matrix().cols();

      for (JointDataVector::reverse_iterator i = data.joints.rbegin()+1; i != data.joints.rend(); ++i)
      {
        sub_constraint_dimension = (int)constraint_xd(*i).matrix().cols();
        start_col -= sub_constraint_dimension;

        iMcomposite = joint_transform(*(i-1)) * iMcomposite;
        data.v += iMcomposite.actInv(motion(*i));
        data.S.matrix().middleCols(start_col,sub_constraint_dimension) = iMcomposite.actInv(constraint_xd(*i));

        // Motion acceleration_d_entrainement_coriolis = Motion::Zero(); // TODO: compute
        // data.c += iMcomposite.actInv(bias(*i)) + acceleration_d_entrainement_coriolis;
      }
    }
   
    
    void calc_aba(JointData & data, Inertia::Matrix6 & I, const bool update_I) const
    {
    // calc has been called previously in abaforwardstep1 so data.M, data.v are up to date
      data.U.setZero();
      data.Dinv.setZero();
      data.UDinv.setZero();
      for (JointDataVector::iterator i = data.joints.begin(); i != data.joints.end(); ++i)
      {
        JointDataVector::iterator::difference_type index = i - data.joints.begin();
        ::se3::calc_aba(joints[(std::size_t)index], *i, I, false);
      }
      data.U = I * data.S;
      Eigen::MatrixXd tmp = data.S.matrix().transpose() * I * data.S.matrix();
      data.Dinv = tmp.inverse();
      data.UDinv = data.U * data.Dinv;

      if (update_I)
        I -= data.UDinv * data.U.transpose();
    }

    ConfigVector_t integrate_impl(const Eigen::VectorXd & qs,const Eigen::VectorXd & vs) const
    {
      ConfigVector_t result(Eigen::VectorXd::Zero(nq_composite));
      for (JointModelVector::const_iterator i = joints.begin(); i != joints.end(); ++i)
      {
        result.segment(::se3::idx_q(*i),::se3::nq(*i)) += ::se3::integrate(*i,qs,vs);
      }
      return result;
    }

    ConfigVector_t interpolate_impl(const Eigen::VectorXd & q0,const Eigen::VectorXd & q1, const double u) const
    {
      ConfigVector_t result(Eigen::VectorXd::Zero(nq_composite));
      for (JointModelVector::const_iterator i = joints.begin(); i != joints.end(); ++i)
      {
        result.segment(::se3::idx_q(*i),::se3::nq(*i)) += ::se3::interpolate(*i,q0,q1,u);
      }
      return result;
    }

    ConfigVector_t randomConfiguration_impl(const ConfigVector_t & lower_pos_limit, const ConfigVector_t & upper_pos_limit ) const throw (std::runtime_error)
    { 
      ConfigVector_t result(Eigen::VectorXd::Zero(nq_composite));
      for (JointModelVector::const_iterator i = joints.begin(); i != joints.end(); ++i)
      {
        result.segment(::se3::idx_q(*i),::se3::nq(*i)) += 
                                ::se3::randomConfiguration(*i,
                                                          lower_pos_limit.segment(::se3::idx_q(*i), ::se3::nq(*i)),
                                                          upper_pos_limit.segment(::se3::idx_q(*i), ::se3::nq(*i))
                                                          );
      }
      return result;
    } 

    TangentVector_t difference_impl(const Eigen::VectorXd & q0,const Eigen::VectorXd & q1) const
    { 
      TangentVector_t result(Eigen::VectorXd::Zero(nv_composite));
      for (JointModelVector::const_iterator i = joints.begin(); i != joints.end(); ++i)
      {
        result.segment(::se3::idx_v(*i),::se3::nv(*i)) += ::se3::difference(*i,q0,q1);
      }
      return result;
    } 

    double distance_impl(const Eigen::VectorXd & q0,const Eigen::VectorXd & q1) const
    { 
      return difference_impl(q0,q1).norm();
    }

    int     nv_impl() const { return nv_composite; }
    int     nq_impl() const { return nq_composite; }


    // Be sure to call it AFTER setIndexes
    void updateComponentsIndexes()
    {
      int current_idx_q, current_idx_v;
      int next_idx_q = idx_q();
      int next_idx_v = idx_v();

      for (JointModelVector::iterator i = joints.begin(); i != joints.end(); ++i)
      {
        current_idx_q = next_idx_q;
        current_idx_v = next_idx_v;
        ::se3::setIndexes(*i,id(),current_idx_q, current_idx_v);
        next_idx_q = current_idx_q + ::se3::nq(*i);
        next_idx_v = current_idx_v + ::se3::nv(*i);
      }
    }

    static const std::string shortname()
    {
      return std::string("JointModelComposite");
    }

    template <class D>
    bool operator == (const JointModelBase<D> &) const
    {
      return false;
    }
    
    bool operator == (const JointModelBase<JointModelComposite> & jmodel) const
    {
      return jmodel.id() == id()
              && jmodel.idx_q() == idx_q()
              && jmodel.idx_v() == idx_v();
    }


template<typename D>
    typename SizeDepType<NQ>::template SegmentReturn<D>::ConstType
    jointConfigSelector(const Eigen::MatrixBase<D>& a) const { return a.segment(i_q,nq_composite); }
    template<typename D>
    typename SizeDepType<NQ>::template SegmentReturn<D>::Type
    jointConfigSelector( Eigen::MatrixBase<D>& a) const { return a.segment(i_q,nq_composite); }

    template<typename D>
    typename SizeDepType<NV>::template SegmentReturn<D>::ConstType
    jointVelocitySelector(const Eigen::MatrixBase<D>& a) const { return a.segment(i_v,nv_composite);  }
    template<typename D>
    typename SizeDepType<NV>::template SegmentReturn<D>::Type
    jointVelocitySelector( Eigen::MatrixBase<D>& a) const { return a.segment(i_v,nv_composite);  }

    template<typename D>
    typename SizeDepType<NV>::template ColsReturn<D>::ConstType 
    jointCols(const Eigen::MatrixBase<D>& A) const { return A.segment(i_v,nv_composite);  }
    template<typename D>
    typename SizeDepType<NV>::template ColsReturn<D>::Type 
    jointCols(Eigen::MatrixBase<D>& A) const { return A.segment(i_v,nv_composite);  }

template<typename D>
  typename SizeDepType<Eigen::Dynamic>::template SegmentReturn<D>::ConstType
  jointConfigSelector_impl(const Eigen::MatrixBase<D>& a) const { return a.segment(i_q,nq_composite); }
  template<typename D>
  typename SizeDepType<Eigen::Dynamic>::template SegmentReturn<D>::Type
  jointConfigSelector_impl(Eigen::MatrixBase<D>& a) const { return a.segment(i_q,nq_composite); }
  template<typename D>
  typename SizeDepType<Eigen::Dynamic>::template SegmentReturn<D>::ConstType
  jointVelocitySelector_impl(const Eigen::MatrixBase<D>& a) const { return a.segment(i_v,nv_composite); }
  template<typename D>
  typename SizeDepType<Eigen::Dynamic>::template SegmentReturn<D>::Type
  jointVelocitySelector_impl(Eigen::MatrixBase<D>& a) const { return a.segment(i_v,nv_composite); }

  template<typename D>
  typename SizeDepType<Eigen::Dynamic>::template ColsReturn<D>::ConstType 
  jointCols_impl(const Eigen::MatrixBase<D>& A) const { return A.middleCols(i_v,nv_composite); }
  template<typename D>
  typename SizeDepType<Eigen::Dynamic>::template ColsReturn<D>::Type 
  jointCols_impl(Eigen::MatrixBase<D>& A) const { return A.middleCols(i_v,nv_composite); }

  };
  

    inline std::ostream & operator << (std::ostream & os, const JointModelComposite & jmodel)
    {
      os << "JointModelComposite containing following models:\n" ;
      for (JointModelVector::const_iterator i = jmodel.joints.begin(); i != jmodel.joints.end(); ++i)
      {
        os << shortname(*i) << std::endl;
      }
      return os;
    }

} // namespace se3


#endif // ifndef __se3_joint_composite_hpp__
