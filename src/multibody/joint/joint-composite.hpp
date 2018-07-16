//
// Copyright (c) 2016,2018 CNRS
//
// This file is part of Pinocchio
// Pinocchio is free software: you can redistribute it
// and/or modify it under the terljMj of the GNU Lesser General Public
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

#include "pinocchio/multibody/joint/fwd.hpp"
#include "pinocchio/multibody/joint/joint-collection.hpp"
#include "pinocchio/multibody/joint/joint-basic-visitors.hpp"
#include "pinocchio/container/aligned-vector.hpp"
#include "pinocchio/spatial/act-on-set.hpp"

namespace se3
{

  template<typename JointCollection> struct JointCompositeTpl;

  template<typename _JointCollection>
  struct traits< JointCompositeTpl<_JointCollection> >
  {
    typedef _JointCollection JointCollection;
    
    enum {
      Options = JointCollection::Options,
      NQ = Eigen::Dynamic,
      NV = Eigen::Dynamic
    };
    
    typedef typename JointCollection::Scalar Scalar;
    typedef JointDataCompositeTpl<JointCollection> JointDataDerived;
    typedef JointModelCompositeTpl<JointCollection> JointModelDerived;
    typedef ConstraintTpl<Eigen::Dynamic,Scalar,Options> Constraint_t;
    typedef SE3Tpl<Scalar,Options> Transformation_t;
    typedef MotionTpl<Scalar,Options> Motion_t;
    typedef MotionTpl<Scalar,Options> Bias_t;

    typedef Eigen::Matrix<Scalar,6,Eigen::Dynamic,Options> F_t;
    // [ABA]
    typedef Eigen::Matrix<Scalar,6,Eigen::Dynamic,Options> U_t;
    typedef Eigen::Matrix<Scalar,Eigen::Dynamic,Eigen::Dynamic,Options> D_t;
    typedef Eigen::Matrix<Scalar,6,Eigen::Dynamic,Options> UD_t;

    typedef Eigen::Matrix<Scalar,Eigen::Dynamic,1,Options> ConfigVector_t;
    typedef Eigen::Matrix<Scalar,Eigen::Dynamic,1,Options> TangentVector_t;
  };
  
  template<typename JointCollection>
  struct traits< JointModelCompositeTpl<JointCollection> >
  { typedef JointCompositeTpl<JointCollection> JointDerived; };
  
  template<typename JointCollection>
  struct traits< JointDataCompositeTpl<JointCollection> >
  { typedef JointCompositeTpl<JointCollection> JointDerived; };
  
  template<typename JointCollection>
  struct JointDataCompositeTpl : public JointDataBase< JointDataCompositeTpl<JointCollection> >
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    typedef JointDataTpl<JointCollection> JointDataVariant;
//    typedef typename JointCollection::JointDataVariant JointDataVariant;
    
    typedef JointDataBase< JointDataCompositeTpl<JointCollection> > Base;
    typedef container::aligned_vector<JointDataVariant> JointDataVector;
//    typedef boost::array<JointDataVariant,njoints> JointDataVector;
    
    typedef typename Base::Transformation_t Transformation_t;
    typedef typename Base::Motion_t Motion_t;
    typedef typename Base::Bias_t Bias_t;
    typedef typename Base::Constraint_t Constraint_t;
    typedef typename Base::U_t U_t;
    typedef typename Base::D_t D_t;
    typedef typename Base::UD_t UD_t;

    // JointDataComposite()  {} // can become necessary if we want a vector of JointDataComposite ?
    
    JointDataCompositeTpl(const JointDataVector & joint_data, const int /*nq*/, const int nv)
    : joints(joint_data), iMlast(joint_data.size()), pjMi(joint_data.size())
    , S(nv)
    , M(), v(), c()
    , U(6,nv), Dinv(nv,nv), UDinv(6,nv)
    {}
    
    /// \brief Vector of joints
    JointDataVector joints;
   
    /// \brief Transforms from previous joint to last joint
    container::aligned_vector<Transformation_t> iMlast;

    /// \brief Transforms from previous joint to joint i
    container::aligned_vector<Transformation_t> pjMi;

    Constraint_t S;
    Transformation_t M;
    Motion_t v;
    Bias_t c;
    
    // // [ABA] specific data
    U_t U;
    D_t Dinv;
    UD_t UDinv;

  };

  template<typename JointCollection>
  struct JointModelCompositeTpl : public JointModelBase< JointModelCompositeTpl<JointCollection> >
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    typedef JointModelTpl<JointCollection> JointModelVariant;
//    typedef typename JointCollection::JointModelVariant JointModelVariant;
    
    typedef typename traits<JointModelCompositeTpl>::JointDerived JointType;
    typedef typename traits<JointType>::JointDataDerived JointDataDerived;
    
    enum
    {
      Options = traits<JointType>::Options,
      NV = traits<JointType>::NV,
      NQ = traits<JointType>::NQ
    };
    
    typedef typename traits<JointType>::Scalar Scalar;
    typedef SE3Tpl<Scalar,Options> SE3;
    typedef JointModelBase< JointModelCompositeTpl<JointCollection> > Base;
    typedef JointDataCompositeTpl<JointCollection> JointData;
    typedef container::aligned_vector<JointModelVariant> JointModelVector;
//    typedef boost::array<JointModelVariant,njoints> JointModelVector;
    typedef typename traits<JointType>::Transformation_t Transformation_t;
    typedef typename traits<JointType>::Constraint_t Constraint_t;
    typedef typename traits<JointType>::ConfigVector_t ConfigVector_t;
    typedef typename traits<JointType>::TangentVector_t TangentVector_t;
    
    using Base::id;
    using Base::idx_q;
    using Base::idx_v;
    using Base::setIndexes;
    using Base::nq;
    using Base::nv;

    /// \brief Default contructor
    JointModelCompositeTpl()
    : joints()
    , jointPlacements()
    , m_nq(0)
    , m_nv(0)
    {}
    
    ///
    /// \brief Constructor with one joint.
    ///
    /// \param jmodel Model of the first joint.
    /// \param placement Placement of the first joint w.r.t. the joint origin.
    ///
    template<typename JointModel>
    JointModelCompositeTpl(const JointModelBase<JointModel> & jmodel,
                           const SE3 & placement = SE3::Identity())
    : joints(1,(JointModelVariant)jmodel.derived())
    , jointPlacements(1,placement)
    , m_nq(jmodel.nq())
    , m_nv(jmodel.nv())
    , m_idx_q(1,0), m_nqs(1,jmodel.nq())
    , m_idx_v(1,0), m_nvs(1,jmodel.nv())
    {}
    
    ///
    /// \brief Copy constructor.
    ///
    /// \param other JointModel to copy.
    ///
    JointModelCompositeTpl(const JointModelCompositeTpl & other)
    : Base(other)
    , joints(other.joints)
    , jointPlacements(other.jointPlacements)
    , m_nq(other.m_nq)
    , m_nv(other.m_nv)
    , m_idx_q(other.m_idx_q), m_nqs(other.m_nqs)
    , m_idx_v(other.m_idx_v), m_nvs(other.m_nvs)
    {}
    
    
    ///
    /// \brief Add a joint to the vector of joints.
    ///
    /// \param jmodel Model of the joint to add.
    /// \param placement Placement of the joint relatively to its predecessor.
    ///
    template<typename JointModel>
    void addJoint(const JointModelBase<JointModel> & jmodel, const SE3 & placement = SE3::Identity())
    {
      joints.push_back((JointModelVariant)jmodel.derived());
      jointPlacements.push_back(placement);
      
      m_nq += jmodel.nq(); m_nv += jmodel.nv();
      
      updateJointIndexes();
    }
    
    JointData createData() const
    {
      typename JointData::JointDataVector jdata(joints.size());
      for (int i = 0; i < (int)joints.size(); ++i)
        jdata[(size_t)i] = ::se3::createData<JointCollection>(joints[(size_t)i]);
      return JointDataDerived(jdata,nq(),nv());
    }

    template<typename _JointCollection>
    friend struct JointCompositeCalcZeroOrderStep;
    void calc(JointData & data, const Eigen::VectorXd & qs) const;

    template<typename _JointCollection>
    friend struct JointCompositeCalcFirstOrderStep;
    void calc(JointData & data, const Eigen::VectorXd & qs, const Eigen::VectorXd & vs) const;
    
    void calc_aba(JointData & data, Inertia::Matrix6 & I, const bool update_I) const
    {
      data.U.noalias() = I * data.S;
      Eigen::Matrix<Scalar,Eigen::Dynamic,Eigen::Dynamic,Options> tmp (data.S.matrix().transpose() * data.U);
      data.Dinv = tmp.inverse();
      data.UDinv.noalias() = data.U * data.Dinv;

      if (update_I)
        I -= data.UDinv * data.U.transpose();
    }

    Scalar finiteDifferenceIncrement() const
    {
      using std::max;
      Scalar eps = 0;
      for(typename JointModelVector::const_iterator it = joints.begin();
          it != joints.end(); ++it)
        eps = max((Scalar)::se3::finiteDifferenceIncrement(*it),eps);
      
      return eps;
    }

    int nv_impl() const { return m_nv; }
    int nq_impl() const { return m_nq; }

    /**
     * @brief      Update the indexes of subjoints in the stack 
     */
    void setIndexes_impl(JointIndex id, int q, int v)
    {
      Base::setIndexes_impl(id, q, v);
      updateJointIndexes();
    }

    static std::string classname() { return std::string("JointModelCompositeTpl"); }
    std::string shortname() const { return classname(); }

    JointModelComposite & operator=(const JointModelComposite & other)
    {
      Base::operator=(other);
      m_nq = other.m_nq;
      m_nv = other.m_nv;
      joints = other.joints;
      jointPlacements = other.jointPlacements;
      
        
      return *this;
    }
    
    /// \brief Vector of joints contained in the joint composite.
    JointModelVector joints;
    /// \brief Vector of joint placements. Those placements correspond to the origin of the joint relatively to their parent.
    container::aligned_vector<SE3> jointPlacements;
    /// \brief Dimensions of the config and tangent space of the composite joint.
    int m_nq,m_nv;

    template<typename D>
    typename SizeDepType<NQ>::template SegmentReturn<D>::ConstType
    jointConfigSelector(const Eigen::MatrixBase<D>& a) const { return a.segment(Base::i_q,nq()); }
    template<typename D>
    typename SizeDepType<NQ>::template SegmentReturn<D>::Type
    jointConfigSelector( Eigen::MatrixBase<D>& a) const { return a.segment(Base::i_q,nq()); }

    template<typename D>
    typename SizeDepType<NV>::template SegmentReturn<D>::ConstType
    jointVelocitySelector(const Eigen::MatrixBase<D>& a) const { return a.segment(Base::i_v,nv());  }
    template<typename D>
    typename SizeDepType<NV>::template SegmentReturn<D>::Type
    jointVelocitySelector( Eigen::MatrixBase<D>& a) const { return a.segment(Base::i_v,nv());  }

    template<typename D>
    typename SizeDepType<NV>::template ColsReturn<D>::ConstType 
    jointCols(const Eigen::MatrixBase<D>& A) const { return A.middleCols(Base::i_v,nv());  }
    template<typename D>
    typename SizeDepType<NV>::template ColsReturn<D>::Type 
    jointCols(Eigen::MatrixBase<D>& A) const { return A.middleCols(Base::i_v,nv());  }

    template<typename D>
    typename SizeDepType<Eigen::Dynamic>::template SegmentReturn<D>::ConstType
    jointConfigSelector_impl(const Eigen::MatrixBase<D>& a) const { return a.segment(Base::i_q,nq()); }
    template<typename D>
    typename SizeDepType<Eigen::Dynamic>::template SegmentReturn<D>::Type
    jointConfigSelector_impl(Eigen::MatrixBase<D>& a) const { return a.segment(Base::i_q,nq()); }
    template<typename D>
    typename SizeDepType<Eigen::Dynamic>::template SegmentReturn<D>::ConstType
    jointVelocitySelector_impl(const Eigen::MatrixBase<D>& a) const { return a.segment(Base::i_v,nv()); }
    template<typename D>
    typename SizeDepType<Eigen::Dynamic>::template SegmentReturn<D>::Type
    jointVelocitySelector_impl(Eigen::MatrixBase<D>& a) const { return a.segment(Base::i_v,nv()); }

    template<typename D>
    typename SizeDepType<Eigen::Dynamic>::template ColsReturn<D>::ConstType 
    jointCols_impl(const Eigen::MatrixBase<D>& A) const { return A.middleCols(Base::i_v,nv()); }
    template<typename D>
    typename SizeDepType<Eigen::Dynamic>::template ColsReturn<D>::Type 
    jointCols_impl(Eigen::MatrixBase<D>& A) const { return A.middleCols(Base::i_v,nv()); }
    
  protected:
    
    /// \brief Update the indexes of the joints contained in the composition according
    /// to the position of the joint composite.
    void updateJointIndexes()
    {
      int idx_q = this->idx_q();
      int idx_v = this->idx_v();
      
      m_idx_q.resize(joints.size());
      m_idx_v.resize(joints.size());
      m_nqs.resize(joints.size());
      m_nvs.resize(joints.size());
      
      for(size_t i = 0; i < joints.size(); ++i)
      {
        JointModelVariant & joint = joints[i];
        
        m_idx_q[i] = idx_q; m_idx_v[i] = idx_v;
        ::se3::setIndexes(joint,i,idx_q,idx_v);
        m_nqs[i] = ::se3::nq(joint);
        m_nvs[i] = ::se3::nv(joint);
        idx_q += m_nqs[i]; idx_v += m_nvs[i];
      }
    }
    
    /// Keep information of both the dimension and the position of the joints in the composition.
    
    /// \brief Index in the config vector
    std::vector<int> m_idx_q;
    /// \brief Dimension of the segment in the config vector
    std::vector<int> m_nqs;
    /// \brief Index in the tangent vector
    std::vector<int> m_idx_v;
    /// \brief Dimension of the segment in the tangent vector
    std::vector<int> m_nvs;
  };
  

  template<typename JointCollection>
  inline std::ostream & operator <<(std::ostream & os, const JointModelCompositeTpl<JointCollection> & jmodel)
  {
    typedef typename JointModelCompositeTpl<JointCollection>::JointModelVector JointModelVector;
    
    os << "JointModelComposite containing following models:\n" ;
    for (typename JointModelVector::const_iterator it = jmodel.joints.begin();
         it != jmodel.joints.end(); ++it)
      os << "  " << shortname(*it) << std::endl;
    
    return os;
  }

} // namespace se3

/* --- Details -------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------- */
/* --- Details -------------------------------------------------------------- */
#include "pinocchio/multibody/joint/joint-composite.hxx"

#endif // ifndef __se3_joint_composite_hpp__
