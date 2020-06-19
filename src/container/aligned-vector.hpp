//
// Copyright (c) 2016-2020 CNRS INRIA
//

#ifndef __pinocchio_container_aligned_vector_hpp__
#define __pinocchio_container_aligned_vector_hpp__

#include <vector>
#include <Eigen/StdVector>

#define PINOCCHIO_ALIGNED_STD_VECTOR(Type) \
  ::pinocchio::container::aligned_vector<Type>
//  std::vector<Type,Eigen::aligned_allocator<Type> >

namespace pinocchio
{
  namespace container
  {
  
    ///
    /// \brief Specialization of an std::vector with an aligned allocator. This specialization might be used when the type T is or contains some Eigen members.
    ///
    /// \tparam T Type of the elements.
    ///
    template<typename T>
    struct aligned_vector : public std::vector<T, Eigen::aligned_allocator<T> >
    {
      typedef ::std::vector<T, Eigen::aligned_allocator<T> > vector_base;
      typedef const vector_base & const_vector_base_ref;
      typedef vector_base & vector_base_ref;
      
      typedef T value_type;
      typedef typename vector_base::allocator_type allocator_type;
      typedef typename vector_base::size_type size_type;
      typedef typename vector_base::iterator iterator;
      
      explicit aligned_vector(const allocator_type & a = allocator_type()) : vector_base(a) {}
      template<typename InputIterator>
      aligned_vector(InputIterator first, InputIterator last, const allocator_type& a = allocator_type())
      : vector_base(first, last, a) {}
      aligned_vector(const aligned_vector & c) : vector_base(c) {}
      explicit aligned_vector(size_type num, const value_type & val = value_type())
      : vector_base(num, val) {}
      aligned_vector(iterator start, iterator end) : vector_base(start, end) {}
      aligned_vector & operator=(const aligned_vector& x)
      { vector_base::operator=(x); return *this; }
      
      vector_base & base() { return *static_cast<vector_base*>(this); }
      const vector_base & base() const { return *static_cast<const vector_base*>(this); }
      
    }; // struct aligned_vector
    
    template<class T>
    bool operator==(const aligned_vector<T>& lhs, const aligned_vector<T>& rhs)
    {
      return lhs.base() == rhs.base();
    }
    
  } // namespace container
  
} // namespace pinocchio

#endif // ifndef __pinocchio_container_aligned_vector_hpp__
