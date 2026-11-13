import inspect
import unittest

import pinocchio as pin


def bound_signatures(scope, seen):
    """Yield (qualified name, signature) for every nanobind function reachable from scope."""
    if id(scope) in seen:
        return
    seen.add(id(scope))
    for name, attr in vars(scope).items():
        if hasattr(attr, "__nb_signature__"):
            for signature, *_ in attr.__nb_signature__:
                yield f"{scope.__name__}.{name}", signature
        elif inspect.isclass(attr) and attr.__module__.startswith("pinocchio"):
            yield from bound_signatures(attr, seen)
        elif inspect.ismodule(attr) and attr.__name__.startswith("pinocchio"):
            yield from bound_signatures(attr, seen)


class TestSignatures(unittest.TestCase):
    def test_eigen_arguments_are_converted(self):
        # A raw Eigen type in a signature means the translation unit that bound it did not
        # include nanobind's Eigen type casters: the function cannot be called from Python.
        unconverted = [
            f"{name}: {signature}"
            for name, signature in bound_signatures(pin.pinocchio_pywrap_default, set())
            if "Eigen::" in signature
        ]
        self.assertEqual(unconverted, [], "\n".join(unconverted))


if __name__ == "__main__":
    unittest.main()
