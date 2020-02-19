//
// Copyright (c) 2015-2020 CNRS INRIA
//

#include "pinocchio/bindings/python/parsers/urdf.hpp"
#include "pinocchio/bindings/python/parsers/srdf.hpp"
#include "pinocchio/bindings/python/parsers/sample-models.hpp"

namespace pinocchio
{
  namespace python
  {
    
    void exposeParsers()
    {
      exposeURDFParser();
      exposeSRDFParser();
      exposeSampleModels();
    }
    
  } // namespace python
} // namespace pinocchio

