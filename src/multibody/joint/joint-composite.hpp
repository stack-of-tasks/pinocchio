//
// Copyright (c) 2016 CNRS
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
#include "pinocchio/multibody/joint/joint-variant.hpp"
#include "pinocchio/multibody/joint/joint-basic-visitors.hpp"
#include "pinocchio/container/aligned-vector.hpp"
#include "pinocchio/spatial/act-on-set.hpp"


namespace se3
{

  struct JointComposite;

  template<>
  struct traits<JointComposite>
  {

    enum {
      NQ = Eigen::Dynamic,
      NV = Eigen::Dynamic
    };
    
    typedef double Scalar;
    typedef JointDataComposite JointDataDerived;
    typedef JointModelComposite JointModelDerived;
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
  
  template<> struct traits< JointDataComposite > { typedef JointComposite JointDerived; };
  template<> struct traits< JointModelComposite > { typedef JointComposite JointDerived; };

  struct JointDataComposite : public JointDataBase< JointDataComposite >
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    typedef JointDataBase<JointDataComposite> Base;
    typedef JointComposite Joint;
    typedef container::aligned_vector<JointDataVariant> JointDataVector;
//    typedef boost::array<JointDataVariant,njoints> JointDataVector;
    
    typedef Base::Transformation_t Transformation_t;
    typedef Base::Motion_t Motion_t;
    typedef Base::Bias_t Bias_t;
    typedef Base::Constraint_t Constraint_t;
    typedef Base::U_t U_t;
    typedef Base::D_t D_t;
    typedef Base::UD_t UD_t;

    // JointDataComposite()  {} // can become necessary if we want a vector of JointDataComposite ?
    
    JointDataComposite(const JointDataVector & joint_data, const int /*nq*/, const int nv)
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

  struct JointModelComposite : public JointModelBase< JointModelComposite >
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    enum
    {
      NV = traits<JointComposite>::NV,
      NQ = traits<JointComposite>::NQ
    };
    
    typedef traits<JointComposite>::Scalar Scalar;
    typedef JointModelBase<JointModelComposite> Base;
    typedef JointDataComposite JointData;
    typedef container::aligned_vector<JointModelVariant> JointModelVector;
//    typedef boost::array<JointModelVariant,njoints> JointModelVector;
    typedef traits<JointComposite>::Transformation_t Transformation_t;
    typedef traits<JointComposite>::Constraint_t Constraint_t;
    typedef traits<JointComposite>::ConfigVector_t ConfigVector_t;
    typedef traits<JointComposite>::TangentVector_t TangentVector_t;
    
    using Base::id;
    using Base::idx_q;
    using Base::idx_v;
    using Base::setIndexes;
    using Base::nq;
    using Base::nv;

    /// \brief Empty contructor
    JointModelComposite()
    : joints()
    , jointPlacements()
    , m_nq(0)
    , m_nv(0)
    {}
    
    ///
    /// \brief Default constructor with at least one joint
    ///
    /// \param jmodel Model of the first joint.
    /// \param placement Placement of the first joint wrt the joint origin
    ///
    template<typename JointModel>
    JointModelComposite(const JointModelBase<JointModel> & jmodel, const SE3 & placement = SE3::Identity())
    : joints(1,jmodel.derived())
    , jointPlacements(1,placement)
    , m_nq(jmodel.nq())
    , m_nv(jmodel.nv())
    , m_idx_q(1), m_nqs(1,jmodel.nq())
    , m_idx_v(1), m_nvs(1,jmodel.nv())
    {}
    
    ///
    /// \brief Copy constructor
    ///
    /// \param other Model to copy.
    ///
    JointModelComposite(const JointModelComposite & other)
    : Base(other)
    , joints(other.joints)
    , jointPlacements(other.jointPlacements)
    , m_nq(other.m_nq)
    , m_nv(other.m_nv)
    , m_idx_q(other.m_idx_q), m_nqs(other.m_nqs)
    , m_idx_v(other.m_idx_v), m_nvs(other.m_nvs)
    {}
    
    
    ///
    /// \brief Add a joint to the composition of joints
    ///
    /// \param jmodel Model of the joint to add.
    /// \param placement Placement of the joint relatively to its predecessor
    ///
    template<typename JointModel>
    void addJoint(const JointModelBase<JointModel> & jmodel, const SE3 & placement = SE3::Identity())
    {
      joints.push_back(jmodel.derived());
      jointPlacements.push_back(placement);
      
      m_nq += jmodel.nq(); m_nv += jmodel.nv();
      
      updateJointIndexes();
    }
    
    JointData createData() const
    {
      JointData::JointDataVector jdata(joints.size());
      for (int i = 0; i < (int)joints.size(); ++i)
        jdata[(size_t)i] = ::se3::createData(joints[(size_t)i]);
      return JointDataDerived(jdata,nq(),nv());
    }

    friend struct JointCompositeCalcZeroOrderStep;
    void calc(JointData & data, const Eigen::VectorXd & qs) const;

    friend struct JointCompositeCalcFirstOrderStep;
    void calc(JointData & data, const Eigen::VectorXd & qs, const Eigen::VectorXd & vs) const;
    
    void calc_aba(JointData & data, Inertia::Matrix6 & I, const bool update_I) const
    {
      data.U.noalias() = I * data.S;
      Eigen::MatrixXd tmp (data.S.matrix().transpose() * data.U);
      data.Dinv = tmp.inverse();
      data.UDinv.noalias() = data.U * data.Dinv;

      if (update_I)
        I -= data.UDinv * data.U.transpose();
    }

    Scalar finiteDifferenceIncrement() const
    {
      using std::max;
      Scalar eps = 0;
      for(JointModelVector::const_iterator it = joints.begin();
          it != joints.end(); ++it)
        eps = max((Scalar)::se3::finiteDifferenceIncrement(*it),eps);
      
      return eps;
    }

    int     nv_impl() const { return m_nv; }
    int     nq_impl() const { return m_nq; }


    /**
     * @brief      Update the indexes of subjoints in the stack 
     */
    void setIndexes_impl(JointIndex id, int q, int v)
    {
      Base::setIndexes_impl(id, q, v);
      updateJointIndexes();
    }

    static std::string classname() { return std::string("JointModelComposite"); }
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
    jointConfigSelector(const Eigen::MatrixBase<D>& a) const { return a.segment(i_q,nq()); }
    template<typename D>
    typename SizeDepType<NQ>::template SegmentReturn<D>::Type
    jointConfigSelector( Eigen::MatrixBase<D>& a) const { return a.segment(i_q,nq()); }

    template<typename D>
    typename SizeDepType<NV>::template SegmentReturn<D>::ConstType
    jointVelocitySelector(const Eigen::MatrixBase<D>& a) const { return a.segment(i_v,nv());  }
    template<typename D>
    typename SizeDepType<NV>::template SegmentReturn<D>::Type
    jointVelocitySelector( Eigen::MatrixBase<D>& a) const { return a.segment(i_v,nv());  }

    template<typename D>
    typename SizeDepType<NV>::template ColsReturn<D>::ConstType 
    jointCols(const Eigen::MatrixBase<D>& A) const { return A.middleCols(i_v,nv());  }
    template<typename D>
    typename SizeDepType<NV>::template ColsReturn<D>::Type 
    jointCols(Eigen::MatrixBase<D>& A) const { return A.middleCols(i_v,nv());  }

    template<typename D>
    typename SizeDepType<Eigen::Dynamic>::template SegmentReturn<D>::ConstType
    jointConfigSelector_impl(const Eigen::MatrixBase<D>& a) const { return a.segment(i_q,nq()); }
    template<typename D>
    typename SizeDepType<Eigen::Dynamic>::template SegmentReturn<D>::Type
    jointConfigSelector_impl(Eigen::MatrixBase<D>& a) const { return a.segment(i_q,nq()); }
    template<typename D>
    typename SizeDepType<Eigen::Dynamic>::template SegmentReturn<D>::ConstType
    jointVelocitySelector_impl(const Eigen::MatrixBase<D>& a) const { return a.segment(i_v,nv()); }
    template<typename D>
    typename SizeDepType<Eigen::Dynamic>::template SegmentReturn<D>::Type
    jointVelocitySelector_impl(Eigen::MatrixBase<D>& a) const { return a.segment(i_v,nv()); }

    template<typename D>
    typename SizeDepType<Eigen::Dynamic>::template ColsReturn<D>::ConstType 
    jointCols_impl(const Eigen::MatrixBase<D>& A) const { return A.middleCols(i_v,nv()); }
    template<typename D>
    typename SizeDepType<Eigen::Dynamic>::template ColsReturn<D>::Type 
    jointCols_impl(Eigen::MatrixBase<D>& A) const { return A.middleCols(i_v,nv()); }
    
    
    
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
  

  inline std::ostream & operator << (std::ostream & os, const JointModelComposite & jmodel)
  {
    typedef JointModelComposite::JointModelVector JointModelVector;
    os << "JointModelComposite containing following models:\n" ;
    for (JointModelVector::const_iterator it = jmodel.joints.begin();
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
