#include <pinocchio/bindings/python/pybind11.hpp>

// This lines forces clang-format to keep the include split here
#include <pybind11/pybind11.h>

#include <boost/python.hpp>

#define SCALAR double
#define OPTIONS 0
#define JOINT_MODEL_COLLECTION ::pinocchio::JointCollectionDefaultTpl
#include <pinocchio/bindings/python/pybind11-all.hpp>

pinocchio::Model * make_model()
{
  pinocchio::Model * model = new pinocchio::Model;
  std::cout << "make_model: " << reinterpret_cast<intptr_t>(model) << std::endl;
  return model;
}

pinocchio::Model & return_same_model_copy(pinocchio::Model & m)
{
  return m;
}
pinocchio::Model * return_same_model_nocopy(pinocchio::Model & m)
{
  return &m;
}

pinocchio::SE3 multiply_se3(pinocchio::SE3 const & a, pinocchio::SE3 const & b)
{
  return a * b;
}

template<typename T>
intptr_t get_ptr(T & m)
{
  std::cout << &m << '\n' << m << std::endl;
  return reinterpret_cast<intptr_t>(&m);
}

void test1(int i)
{
  std::cout << "no conversion: " << ' ' << i << std::endl;
}
void testModel1(pinocchio::Model & model)
{
  std::cout << "testModel1: " << &model << std::endl;
  model.name = "testModel1: I modified the model name";
}
intptr_t testModel2(pinocchio::Model & model, int i)
{
  std::cout << "testModel2: " << &model << ' ' << i << std::endl;
  model.name = "testModel2: I modified the model name";
  return reinterpret_cast<intptr_t>(&model);
}
intptr_t testModel3(pinocchio::Model const & model, int i)
{
  std::cout << "testModel3: " << &model << ' ' << i << std::endl;
  return reinterpret_cast<intptr_t>(&model);
}

void testModel_manual(pybind11::object model)
{
  testModel1(pinocchio::python::from<pinocchio::Model &>(model));
}

using pinocchio::python::make_pybind11_function;

PYBIND11_MODULE(cpp2pybind11, m)
{
  using namespace pybind11::literals; // For _a

  pybind11::module::import("pinocchio");
  m.def("testModel_manual", testModel_manual);

  m.def("test1", make_pybind11_function(&test1));

  m.def("make_model", make_pybind11_function(&make_model));
  m.def(
    "return_same_model_broken", make_pybind11_function(&return_same_model_copy),
    pybind11::return_value_policy::reference);
  m.def(
    "return_same_model", make_pybind11_function(&return_same_model_nocopy),
    pybind11::return_value_policy::reference);

  m.def("get_ptr", make_pybind11_function(&get_ptr<pinocchio::Model>));
  m.def("get_se3_ptr", make_pybind11_function(&get_ptr<pinocchio::SE3>));

  m.def("multiply_se3_1", make_pybind11_function(&multiply_se3), "a"_a, "b"_a);
  m.def(
    "multiply_se3", make_pybind11_function(&multiply_se3), "a"_a,
    "b"_a = pinocchio::python::default_arg(pinocchio::SE3::Identity()));
  m.def("testModel1", make_pybind11_function(&testModel1));
  m.def("testModel2", make_pybind11_function(&testModel2));
  m.def("testModel3", make_pybind11_function(&testModel3));

  pybind11::module no_wrapper = m.def_submodule("no_wrapper");
  no_wrapper.def(
    "multiply_se3", &multiply_se3, "a"_a, "b"_a
    // does not work = pinocchio::SE3::Identity()
  );
  no_wrapper.def("testModel1", &testModel1);
  no_wrapper.def("testModel2", &testModel2);
  no_wrapper.def("testModel3", &testModel3);
}
