import pinocchio as pin
import numpy as np
import hppfcl

from pinocchio.visualize import MeshcatVisualizer
from time import sleep

def create_pin_cube_model(j0='freeflyer'):
    model = pin.Model()

    if j0 == 'freeflyer':
        j0 = pin.JointModelFreeFlyer()
    elif j0 == 'spherical':
        j0 = pin.JointModelSpherical()
    elif j0 == 'sphericalzyx':
        j0 = pin.JointModelSphericalZYX()
    else:
        raise ValueError("Unknown joint type")

    jointCube = model.addJoint(0, j0, pin.SE3.Identity(), "joint0")
    M = pin.SE3.Identity()
    model.appendBodyToJoint(jointCube, pin.Inertia.FromBox(1, 0.8, 0.4, 0.2), M)
    return model

def create_pin_geometry_cube_model(model):
    jointCube = model.getFrameId("joint0")
    geom_model = pin.GeometryModel()
    cube_shape = hppfcl.Box(0.8, 0.4, 0.2)  # x, y, z
    cube = pin.GeometryObject(
        "cube_shape", 0, jointCube, cube_shape, pin.SE3.Identity()
    )
    cube.meshColor = np.array([1.0, 0.1, 0.1, 0.5])
    geom_model.addGeometryObject(cube)
    return geom_model

def create_model(joint0_type='freeflyer'):
    model = create_pin_cube_model(joint0_type)
    geom_model = create_pin_geometry_cube_model(model)
    return model, geom_model

def pin_step(model, vizer, v_index_increment, dt = 0.1, enable_gravity=True):

    if not enable_gravity:
        model.gravity.setZero()

    data = model.createData()

    tau = np.zeros(model.nv)
    q = pin.neutral(model)
    v = np.zeros(model.nv)
    v[v_index_increment] += 1.

    vizer.display(q)
    print(f"{q=}, {v=}")

    sleep(1)

    a = pin.aba(model, data, q, v, tau)
    v_next = v + a * dt
    q_next = pin.integrate(model, q, v_next * dt)

    print(f"{a=}, {q_next=}, {v_next=}")
    return q_next, v_next


if __name__ == "__main__":

    list_joint0 = ['freeflyer', 'sphericalzyx', 'spherical']

    for joint0_name in list_joint0:

        print(f"\njoint0_name = {joint0_name}")

        model, geom_model = create_model(joint0_name)
        print(f"{model.nq=}")
        print(f"{model.nv=}")

        q = pin.neutral(model)
        print(f"neutral q configuration: {q=}")

        vizer = MeshcatVisualizer(model, geom_model, geom_model)
        vizer.initViewer(open=True, loadModel=True)
        vizer.display(q)

        sleep(2)

        for i in range(model.nv):
            print(f"v[{i}] += 1")
            q_next, v_next = pin_step(model, vizer, i, dt=0.2, enable_gravity=False)
            vizer.display(q_next)
            sleep(2)

