//
// Copyright (c) 2019 INRIA
//

#ifndef __pinocchio_serialization_vector_hpp__
#define __pinocchio_serialization_vector_hpp__

#include <vector>

#include <boost/version.hpp>
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/vector.hpp>

namespace boost
{
  namespace serialization
  {
    

#if BOOST_VERSION / 100 % 1000 == 58
    namespace fixme
    {
      
      template<class T>
      struct nvp :
      public std::pair<const char *, T *>,
      public wrapper_traits<const nvp< T > >
      {
        //private:
        nvp(const nvp & rhs) :
        std::pair<const char *, T *>(rhs.first, rhs.second)
        {}
      public:
        explicit nvp(const char * name_, T & t) :
        // note: added _ to suppress useless gcc warning
        std::pair<const char *, T *>(name_, boost::addressof(t))
        {}
        
        const char * name() const {
          return this->first;
        }
        T & value() const {
          return *(this->second);
        }
        
        const T & const_value() const {
          return *(this->second);
        }
        
        template<class Archive>
        void save(
                  Archive & ar,
                  const unsigned int /* file_version */
        ) const {
          ar.operator<<(const_value());
        }
        template<class Archive>
        void load(
                  Archive & ar,
                  const unsigned int /* file_version */
        ){
          ar.operator>>(value());
        }
        BOOST_SERIALIZATION_SPLIT_MEMBER()
      };
      

      template<class T, class Allocator>
      struct nvp< std::vector<T,Allocator> > :
      public std::pair<const char *, std::vector<T,Allocator> *>,
      public wrapper_traits<const nvp< std::vector<T,Allocator> > >
      {
        //private:
        nvp(const nvp & rhs) :
        std::pair<const char *, std::vector<T,Allocator> *>(rhs.first, rhs.second)
        {}
      public:
        explicit nvp(const char * name_, std::vector<T,Allocator> & t) :
        // note: added _ to suppress useless gcc warning
        std::pair<const char *, std::vector<T,Allocator> *>(name_, boost::addressof(t))
        {}
        
        const char * name() const {
          return this->first;
        }
        
        std::vector<T,Allocator> & value() const {
          return *(this->second);
        }
        
        const std::vector<T,Allocator> & const_value() const {
          return *(this->second);
        }
        
        template<class Archive>
        void save(Archive & ar,
                  const unsigned int /* file_version */
        ) const
        {
          const size_t count(const_value().size());
          ar << BOOST_SERIALIZATION_NVP(count);
          if (!const_value().empty())
          {
            for(typename std::vector<T,Allocator>::const_iterator hint = const_value().begin();
                hint != const_value().end(); ++hint)
            {
              ar & boost::serialization::make_nvp("item", *hint);
            }
          }
        }
        
        template<class Archive>
        void load(Archive & ar,
                  const unsigned int /* file_version */
        )
        {
          std::size_t count;
          ar >> BOOST_SERIALIZATION_NVP(count);
          value().resize(count);
          for(typename std::vector<T,Allocator>::iterator hint = value().begin();
              hint != value().end(); ++hint)
          {
            ar >> boost::serialization::make_nvp("item", *hint);
          }
        }
        
        BOOST_SERIALIZATION_SPLIT_MEMBER()
      };
      
      
    }
    
    template<class T, class Allocator>
    inline const fixme::nvp< std::vector<T,Allocator> >
    make_nvp(const char * name, std::vector<T,Allocator> & t)
    {
      return fixme::nvp< std::vector<T,Allocator> >(name, t);
    }
#else
    template<class T, class Allocator>
    inline const nvp< std::vector<T,Allocator> >
    make_nvp(const char * name, std::vector<T,Allocator> & t)
    {
      return nvp< std::vector<T,Allocator> >(name, t);
    }
#endif

  }
}

#endif // ifndef __pinocchio_serialization_vector_hpp__
