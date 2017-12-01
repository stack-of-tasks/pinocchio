//
// Copyright (c) 2016-2017 CNRS
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

#ifndef __se3_vector_space_operation_hpp__
#define __se3_vector_space_operation_hpp__

#include <stdexcept>

#include <pinocchio/multibody/liegroup/operation-base.hpp>

namespace se3
{
  template<int Size> struct VectorSpaceOperation;
  template<int Size> struct traits<VectorSpaceOperation<Size> > {
    typedef double Scalar;
    enum {
      NQ = Size,
      NV = Size
    };
    typedef Eigen::Matrix<Scalar,NQ,1> ConfigVector_t;
    typedef Eigen::Matrix<Scalar,NV,1> TangentVector_t;
  };

  template<int Size = Eigen::Dynamic>
  struct VectorSpaceOperation : public LieGroupOperationBase <VectorSpaceOperation<Size> >
  {
    typedef VectorSpaceOperation<Size>  LieGroupDerived;

    SE3_LIE_GROUP_TYPEDEF_TEMPLATE;

    /// Constructor
    /// \param size size of the vector space: should be the equal to template
    ///        argument for static sized vector-spaces.
    VectorSpaceOperation (int size = Size) : size_ (size)
    {
      assert (size_ >= 0);
      assert (Size == Eigen::Dynamic || size_ == Size);
    }

    /// Constructor
    /// \param size size of the vector space: should be the equal to template
    ///        argument for static sized vector-spaces.
    VectorSpaceOperation (const LieGroupDerived& other) : size_ (other.size_)
    {
      assert (size_ >= 0);
      assert (Size == Eigen::Dynamic || size_ == Size);
    }

    Index nq () const
    {
      return size_;
    }
    Index nv () const
    {
      return size_;
    }

    ConfigVector_t neutral () const
    {
      ConfigVector_t n;
      if (Size == Eigen::Dynamic)
        n.resize (size_);
      n.setZero ();
      return n;
    }

    std::string name () const
    {
      std::ostringstream oss; oss << "R^" << size_;
      return oss.str ();
    }

    template <class ConfigL_t, class ConfigR_t, class Tangent_t>
    static void difference_impl(const Eigen::MatrixBase<ConfigL_t> & q0,
                                const Eigen::MatrixBase<ConfigR_t> & q1,
                                const Eigen::MatrixBase<Tangent_t> & d)
    {
      const_cast< Eigen::MatrixBase<Tangent_t>& > (d) = q1 - q0;
    }

    template <class ConfigIn_t, class Velocity_t, class ConfigOut_t>
    static void integrate_impl(const Eigen::MatrixBase<ConfigIn_t> & q,
                               const Eigen::MatrixBase<Velocity_t> & v,
                               const Eigen::MatrixBase<ConfigOut_t> & qout)
    {
      const_cast< Eigen::MatrixBase<ConfigOut_t>& > (qout) = q + v;
    }

    // template <class ConfigL_t, class ConfigR_t>
    // static double squaredDistance_impl(const Eigen::MatrixBase<ConfigL_t> & q0,
                                       // const Eigen::MatrixBase<ConfigR_t> & q1)
    
    template <class Config_t>
    static void normalize_impl (const Eigen::MatrixBase<Config_t>& /*qout*/)
    {}

    template <class Config_t>
    void random_impl (const Eigen::MatrixBase<Config_t>& qout) const
    {
      qout.setRandom();
    }

    template <class ConfigL_t, class ConfigR_t, class ConfigOut_t>
    void randomConfiguration_impl
    (const Eigen::MatrixBase<ConfigL_t> & lower_pos_limit,
     const Eigen::MatrixBase<ConfigR_t> & upper_pos_limit,
     const Eigen::MatrixBase<ConfigOut_t> & qout) const
    {
      ConfigOut_t& res = const_cast< Eigen::MatrixBase<ConfigOut_t>& > (qout).derived();
      for (int i = 0; i < nq (); ++i)
      {
        if(lower_pos_limit[i] == -std::numeric_limits<Scalar>::infinity() ||
           upper_pos_limit[i] ==  std::numeric_limits<Scalar>::infinity() )
        {
          std::ostringstream error;
          error << "non bounded limit. Cannot uniformly sample joint at rank " << i;
          // assert(false && "non bounded limit. Cannot uniformly sample joint revolute");
          throw std::runtime_error(error.str());
        }
        res[i] = lower_pos_limit[i] + (( upper_pos_limit[i] - lower_pos_limit[i]) * rand())/RAND_MAX;
      }
    }
  private:
    Index size_;
  }; // struct VectorSpaceOperation

} // namespace se3

#endif // ifndef __se3_vector_space_operation_hpp__
