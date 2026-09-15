#include "pinocchio/bindings/python-nb/fwd.hpp"

#include "pinocchio/spatial.hpp"

#include <coal/math/transform.h>

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
  // Implicit conversion invokes the destination's Python constructor.
  // Register these after both libraries have exposed their transform types.
  nb::borrow<nb::class_<coal::Transform3s>>(nb::type<coal::Transform3s>())
    .def("__init__", [](coal::Transform3s * self, const pinocchio::SE3 & placement) {
      new (self) coal::Transform3s(placement.rotation(), placement.translation());
    });
  nb::borrow<nb::class_<pinocchio::SE3>>(nb::type<pinocchio::SE3>())
    .def("__init__", [](pinocchio::SE3 * self, const coal::Transform3s & placement) {
      new (self) pinocchio::SE3(placement.getRotation(), placement.getTranslation());
    });
  nb::implicitly_convertible<pinocchio::SE3, coal::Transform3s>();
  nb::implicitly_convertible<coal::Transform3s, pinocchio::SE3>();

  exposeBroadphase(m);
  exposeBroadphaseCallbacks(m);
  exposeCollision(m);
#ifdef PINOCCHIO_PYTHON_INTERFACE_WITH_OPENMP
  exposePoolCollision(m);
#endif
}
PINOCCHIO_PYTHON_NAMESPACE_END
