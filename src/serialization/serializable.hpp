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
      void loadFromText(const std::string & filename)
      {
        pinocchio::serialization::loadFromText(derived(),filename);
      }
      
      /// \brief Saves a Derived object as a text file.
      void saveToText(const std::string & filename) const
      {
        pinocchio::serialization::saveToText(derived(),filename);
      }
      
      /// \brief Loads a Derived object from a stream string.
      void loadFromStringStream(std::istringstream & is)
      {
        pinocchio::serialization::loadFromStringStream(derived(),is);
      }
      
      /// \brief Saves a Derived object to a string stream.
      void saveToStringStream(std::stringstream & ss) const
      {
        pinocchio::serialization::saveToStringStream(derived(),ss);
      }
      
      /// \brief Loads a Derived object from a  string.
      void loadFromString(const std::string & str)
      {
        pinocchio::serialization::loadFromString(derived(),str);
      }
      
      /// \brief Saves a Derived object to a string.
      std::string saveToString() const
      {
        return pinocchio::serialization::saveToString(derived());
      }
      
      /// \brief Loads a Derived object from an XML file.
      void loadFromXML(const std::string & filename,
                       const std::string & tag_name)
      {
        pinocchio::serialization::loadFromXML(derived(),filename,tag_name);
      }
      
      /// \brief Saves a Derived object as an XML file.
      void saveToXML(const std::string & filename,
                     const std::string & tag_name) const
      {
        pinocchio::serialization::saveToXML(derived(),filename,tag_name);
      }
      
      /// \brief Loads a Derived object from an binary file.
      void loadFromBinary(const std::string & filename)
      {
        pinocchio::serialization::loadFromBinary(derived(),filename);
      }
      
      /// \brief Saves a Derived object as an binary file.
      void saveToBinary(const std::string & filename) const
      {
        pinocchio::serialization::saveToBinary(derived(),filename);
      }
    };
    
  }
} // namespace pinocchio

#endif // ifndef __pinocchio_serialization_serializable_hpp__

