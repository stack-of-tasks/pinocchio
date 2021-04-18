//
// Copyright (c) 2021 INRIA
//

#ifndef __pinocchio_serialization_static_buffer_hpp__
#define __pinocchio_serialization_static_buffer_hpp__

#include <vector>

namespace pinocchio
{
  namespace serialization
  {
    
    /// \brief Static buffer with pre-allocated memory.
    struct StaticBuffer
    {
      
      /// \brief Defautl constructor from a given size
      explicit StaticBuffer(const size_t n)
      : m_size(n)
      {
        m_data.reserve(n);
      }
      
      /// \brief Returns the current size of the buffer
      size_t size() const
      {
        return m_size;
      }
      
      /// \brief Returns the pointer on the data
      char * data()
      {
        return m_data.data();
      }
      
      /// \brief Returns the pointer on the data (const version)
      const char * data() const
      {
        return m_data.data();
      }
      
      /// \brief Increase the capacity of the vector to a value that's greater or equal to new_size.
      ///
      /// \param[in] new_size New capacity of the buffer.
      ///
      void resize(const size_t new_size)
      {
        m_size = new_size;
        m_data.reserve(new_size);
      }
      
    protected:
      
      size_t m_size;
      std::vector<char> m_data;
    };
    
  }
}

#endif // ifndef __pinocchio_serialization_static_buffer_hpp__
