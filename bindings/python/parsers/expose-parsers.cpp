//
// Copyright (c) 2015-2020 CNRS INRIA
//

#include "pinocchio/bindings/python/parsers/urdf.hpp"
#include "pinocchio/bindings/python/parsers/sdf.hpp"
#include "pinocchio/bindings/python/parsers/srdf.hpp"
#include "pinocchio/bindings/python/parsers/mjcf.hpp"

namespace pinocchio
{
  namespace python
  {

    void exposeParsers()
    {
      exposeSDFParser();
      exposeURDFParser();
      exposeSRDFParser();
      exposeMJCFParser();
    }

  } // namespace python
} // namespace pinocchio
