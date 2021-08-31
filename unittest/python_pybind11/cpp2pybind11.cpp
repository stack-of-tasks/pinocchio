#include <pinocchio/multibody/model.hpp>

#include <pybind11/pybind11.h>
#include <boost/python.hpp>

#include <pinocchio/bindings/python/pybind11.hpp>

pinocchio::Model* make_model()
{
  pinocchio::Model* model = new pinocchio::Model;
  std::cout << "make_model: " << reinterpret_cast<intptr_t>(model) << std::endl;
  return model;
}

pinocchio::Model& return_same_model(pinocchio::Model& m)
{
  return m;
}

intptr_t get_ptr(pinocchio::Model& model)
{
  return reinterpret_cast<intptr_t>(&model);
}

void test1(int i)
{
  std::cout << "no conversion: " << ' ' << i << std::endl;
}
void testModel1(pinocchio::Model& model)
{
  std::cout << "testModel1: " << &model << std::endl;
  model.name = "testModel1: I modified the model name";
}
intptr_t testModel2(pinocchio::Model& model, int i)
{
  std::cout << "testModel2: " << &model << ' ' << i << std::endl;
  model.name = "testModel2: I modified the model name";
  return reinterpret_cast<intptr_t>(&model);
}
intptr_t testModel3(pinocchio::Model const& model, int i)
{
  std::cout << "testModel3: " << &model << ' ' << i << std::endl;
  return reinterpret_cast<intptr_t>(&model);
}

void testModel_manual(pybind11::object model) {
  testModel1(pinocchio::cpp2pybind11::from<pinocchio::Model&>(model));
}

PYBIND11_MODULE(cpp2pybind11, m) {
  pybind11::module::import("pinocchio");
  m.def("testModel_manual", testModel_manual);

  m.def("test1", pinocchio::cpp2pybind11::make_function(&test1));

  m.def("make_model", pinocchio::cpp2pybind11::make_function(&make_model));
  m.def("return_same_model", pinocchio::cpp2pybind11::make_function(&return_same_model),
      pybind11::return_value_policy::reference);

  m.def("get_ptr", pinocchio::cpp2pybind11::make_function(&get_ptr));

  m.def("testModel1", pinocchio::cpp2pybind11::make_function(&testModel1));
  m.def("testModel2", pinocchio::cpp2pybind11::make_function(&testModel2));
  m.def("testModel3", pinocchio::cpp2pybind11::make_function(&testModel3));
}
