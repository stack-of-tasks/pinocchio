#include "pinocchio/bindings/python-nb/fwd.hpp"

#include "pinocchio/spatial.hpp"

PINOCCHIO_PYTHON_NAMESPACE_BEGIN
void exposeBroadphase(nb::module_ m);
void exposeBroadphaseCallbacks(nb::module_ m);
void exposeCollision(nb::module_ m);

void exposeCoal(nb::module_ m)
{
  // exposeBroadphase(m);
  // exposeBroadphaseCallbacks(m);
  exposeCollision(m);
}
PINOCCHIO_PYTHON_NAMESPACE_END
