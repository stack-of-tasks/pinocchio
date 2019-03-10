//
// Copyright (c) 2017-2019 CNRS INRIA
//

#ifndef __pinocchio_serialization_serializable_hpp__
#define __pinocchio_serialization_serializable_hpp__

#include "pinocchio/serialization/archive.hpp"

namespace pinocchio
{
  namespace serialization
  {
    
    template<class Derived>
    struct Serializable
    {
    private:
      
      Derived & derived() { return *static_cast<Derived*>(this); }
      const Derived & derived() const { return *static_cast<const Derived*>(this); }
      
    public:
      
      /// \brief Loads a Derived object from a text file.
      void loadFromText(const std::string & filename) throw (std::invalid_argument)
      {
        loadFromText(derived(),filename);
      }
      
      /// \brief Saves a Derived object as a text file.
      void saveToText(const std::string & filename) const throw (std::invalid_argument)
      {
        saveToText(derived(),filename);
      }
      
      /// \brief Loads a Derived object from an XML file.
      void loadFromXML(const std::string & filename,
                       const std::string & tag_name) throw (std::invalid_argument)
      {
        loadFromXML(derived(),filename,tag_name);
      }
      
      /// \brief Saves a Derived object as an XML file.
      void saveToXML(const std::string & filename,
                     const std::string & tag_name) const throw (std::invalid_argument)
      {
        saveToXML(derived(),filename,tag_name);
      }
      
      /// \brief Loads a Derived object from an binary file.
      void loadFromBinary(const std::string & filename) throw (std::invalid_argument)
      {
        loadFromBinary(derived(),filename);
      }
      
      /// \brief Saves a Derived object as an binary file.
      void saveToBinary(const std::string & filename) const throw (std::invalid_argument)
      {
        saveToBinary(derived(),filename);
      }
    };
    
  }
} // namespace pinocchio

#endif // ifndef __pinocchio_serialization_serializable_hpp__

