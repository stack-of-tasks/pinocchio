//
// Copyright (c) 2024 INRIA
//

#ifndef __pinocchio_parsers_meshloader_fwd_hpp__
#define __pinocchio_parsers_meshloader_fwd_hpp__

#include <memory>

#ifdef PINOCCHIO_WITH_COAL
  #include <coal/config.hh>
#endif // PINOCCHIO_WITH_COAL

#ifdef COAL_VERSION
namespace coal
{
  class MeshLoader;
  typedef std::shared_ptr<MeshLoader> MeshLoaderPtr;
} // namespace coal
#else
  namespace coal
  {
    class MeshLoader;
    typedef std::shared_ptr<MeshLoader> MeshLoaderPtr;
  } // namespace coal
#endif // COAL_VERSION

#endif // __pinocchio_parsers_meshloader_fwd_hpp__
