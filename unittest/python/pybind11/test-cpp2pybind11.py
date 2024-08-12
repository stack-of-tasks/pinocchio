import cpp2pybind11
import sys
import gc
import pinocchio

a = pinocchio.SE3.Random()
b = pinocchio.SE3.Random()
assert cpp2pybind11.multiply_se3_1(a, b) == a * b
assert cpp2pybind11.multiply_se3(a, b) == a * b
assert cpp2pybind11.no_wrapper.multiply_se3(a, b) == a * b
assert cpp2pybind11.multiply_se3(a) == a
# assert cpp2pybind11.no_wrapper.multiply_se3(a) == a


def print_ref_count(v, what=""):
    # - 2 because one for variable v and one for variable inside getrefcount
    idv = id(v)
    gc.collect()
    # n = len(gc.get_referrers(v))
    n = sys.getrefcount(v)
    print("ref count of", what, idv, n)


m = cpp2pybind11.make_model()
print_ref_count(m, "m")
print(cpp2pybind11.get_ptr(m))
print_ref_count(m, "m")

m.name = ""
cpp2pybind11.testModel1(m)
print_ref_count(m, "m")
assert m.name.startswith("testModel1")

addr2 = cpp2pybind11.testModel2(m, 1)
print_ref_count(m, "m")
assert m.name.startswith("testModel2")

addr3 = cpp2pybind11.testModel3(m, 2)
print_ref_count(m, "m")
assert addr2 == addr3

mm = cpp2pybind11.return_same_model_broken(m)
assert cpp2pybind11.get_ptr(m) != cpp2pybind11.get_ptr(mm)

mm = cpp2pybind11.return_same_model(m)
# Not sure why but the ref count of m and mm sticks to one
print_ref_count(m, "m")
mmm = m
print_ref_count(m, "m")
print_ref_count(mm, "mm")
assert cpp2pybind11.get_ptr(m) == cpp2pybind11.get_ptr(mm)

if False:
    print("deleting m")
    del m
    print("deleted m")
    print("mm is", mm)
else:
    print("deleting mm")
    del mm
    print("deleted mm")
    print("m is", m)
