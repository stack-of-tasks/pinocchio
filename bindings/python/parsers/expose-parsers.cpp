//
// Copyright (c) 2015-2020 CNRS INRIA
//

#include "pinocchio/bindings/python/parsers/urdf.hpp"
#include "pinocchio/bindings/python/parsers/sdf.hpp"
#include "pinocchio/bindings/python/parsers/srdf.hpp"
#include "pinocchio/bindings/python/parsers/mjcf.hpp"
#include "pinocchio/bindings/python/parsers/model-graph.hpp"

namespace pinocchio
{
  namespace python
  {

    void exposeParsers()
    {
#if defined(PINOCCHIO_PYTHON_INTERFACE_MAIN_MODULE)
      exposeSDFParser();
      exposeURDFParser();
      exposeSRDFParser();
      exposeMJCFParser();
#endif // if defined(PINOCCHIO_PYTHON_INTERFACE_MAIN_MODULE)
      exposeGraph();
    }

  } // namespace python
} // namespace pinocchio
