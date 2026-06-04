#include "pinocchio/bindings/python-nb/fwd.hpp"

#include "pinocchio/spatial.hpp"

PINOCCHIO_PYTHON_NAMESPACE_BEGIN
// collision/broadphase.cpp
void exposeBroadphase(nb::module_ m);
// collision/broadphase-callbacks.cpp
void exposeBroadphaseCallbacks(nb::module_ m);
// collision/collision.cpp
void exposeCollision(nb::module_ m);
#ifdef PINOCCHIO_PYTHON_INTERFACE_WITH_OPENMP
// collision/pool.cpp
void exposePoolCollision(nb::module_ m);
#endif

void exposeCoal(nb::module_ m)
{
  exposeBroadphase(m);
  exposeBroadphaseCallbacks(m);
  exposeCollision(m);
#ifdef PINOCCHIO_PYTHON_INTERFACE_WITH_OPENMP
  exposePoolCollision(m);
#endif
}
PINOCCHIO_PYTHON_NAMESPACE_END
