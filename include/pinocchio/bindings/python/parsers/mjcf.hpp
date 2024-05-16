//
// Copyright (c) 2024 INRIA
//

#ifndef __pinocchio_python_parsers_mjcf_hpp__
#define __pinocchio_python_parsers_mjcf_hpp__

namespace pinocchio
{
  namespace python
  {
    void exposeMJCFModel();
    void exposeMJCFGeom();

    inline void exposeMJCFParser()
    {
      exposeMJCFModel();
      exposeMJCFGeom();
    }
  } // namespace python
} // namespace pinocchio

#endif // ifndef __pinocchio_python_parsers_mjcf_hpp__
