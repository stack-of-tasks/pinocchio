"""
Copyright (c) 2020 INRIA
Inspired from Antonio El Khoury PhD: https://tel.archives-ouvertes.fr/file/index/docid/833019/filename/thesis.pdf
Section 3.8.1 Computing minimum bounding capsules
"""

import numpy as np
import scipy.optimize as optimize

import hppfcl

"""
Capsule definition
a, b: the two extremities of the capsule segment
r: radius of the capsule
"""

EPSILON = 1e-8
CONSTRAINT_INFLATION_RATIO = 5e-3


def capsule_volume(a, b, r):
    return np.linalg.norm(b - a) * np.pi * r ** 2 + 4 / 3 * np.pi * r ** 3


def distance_points_segment(p, a, b):
    ap = p - a
    ab = b - a
    t = ap.dot(ab) / ab.dot(ab)
    t = np.clip(t, 0, 1)
    p_witness = a[None, :] + (b - a)[None, :] * t[:, None]
    dist = np.linalg.norm(p - p_witness, axis=1).max()
    return dist


def pca_approximation(vertices):
    mean = vertices.mean(axis=0)
    vertices -= mean
    u, s, vh = np.linalg.svd(vertices, full_matrices=True)
    components = vh
    pca_proj = vertices.dot(components.T)
    vertices += mean

    a0 = mean + components[0] * (pca_proj[:, 0].min() - EPSILON)
    b0 = mean + components[0] * (pca_proj[:, 0].max() + EPSILON)
    radius = np.linalg.norm(pca_proj[:, 1:], axis=1).max()
    return a0, b0, radius


def capsule_approximation(vertices):
    a0, b0, r0 = pca_approximation(vertices)
    constraint_inflation = CONSTRAINT_INFLATION_RATIO * r0
    x0 = np.array(list(a0) + list(b0) + [r0])
    constraint_cap = lambda x: distance_points_segment(vertices, x[:3], x[3:6]) - x[6]
    capsule_vol = lambda x: capsule_volume(x[:3], x[3:6], x[6])
    constraint = optimize.NonlinearConstraint(
        constraint_cap, lb=-np.inf, ub=-constraint_inflation
    )
    res = optimize.minimize(capsule_vol, x0, constraints=constraint)
    res_constraint = constraint_cap(res.x)
    assert (
        res_constraint <= 1e-4
    ), "The computed solution is invalid, a vertex is at a distance {:.5f} of the capsule.".format(
        res_constraint
    )
    a, b, r = res.x[:3], res.x[3:6], res.x[6]
    return a, b, r

def approximate_mesh(filename, lMg):
    mesh_loader = hppfcl.MeshLoader()
    mesh = mesh_loader.load(filename, np.ones(3))
    vertices = np.array([ lMg * mesh.vertices(i) for i in range(mesh.num_vertices) ])
    assert vertices.shape == (mesh.num_vertices, 3)
    a, b, r = capsule_approximation(vertices)
    return a, b, r

def parse_urdf(infile, outfile):
    from lxml import etree

    tree = etree.parse(infile)

    def get_path(fn):
        if fn.startswith('package://'):
            relpath = fn[len('package://'):]
            import os
            for rospath in os.environ['ROS_PACKAGE_PATH'].split(':'):
                abspath = os.path.join(rospath, relpath)
                if os.path.isfile(abspath):
                    return abspath
            raise ValueError("Could not find " + fn)
        return fn

    def get_transform(origin):
        from pinocchio import SE3, rpy
        _xyz = [ float(v) for v in origin.attrib.get('xyz', '0 0 0').split(' ') ]
        _rpy = [ float(v) for v in origin.attrib.get('rpy', '0 0 0').split(' ') ]
        return SE3 (rpy.rpyToMatrix(*_rpy), np.array(_xyz))

    def set_transform(origin, a, b):
        from pinocchio import rpy, Quaternion
        length = np.linalg.norm(b-a)
        z = (b - a) / length
        R = Quaternion.FromTwoVectors(np.array([0, 0, 1]), z).matrix()
        origin.attrib['xyz'] = " ".join([str(v) for v in ((a+b)/2) ])
        origin.attrib['rpy'] = " ".join([str(v) for v in rpy.matrixToRpy(R) ])

    from tqdm import tqdm
    for mesh in tqdm(tree.xpath('/robot/link/collision/geometry/mesh'), desc="Generating capsules"):
        geom = mesh.getparent()
        coll = geom.getparent()
        link = coll.getparent()
        if coll.find('origin') is None:
            o = etree.Element("origin")
            o.tail = geom.tail
            coll.insert(0, o)
        origin = coll.find('origin')
        lMg = get_transform(origin)

        meshfile = get_path(mesh.attrib['filename'])
        import os
        name = os.path.basename(meshfile)
        # Generate capsule
        a, b, radius = approximate_mesh (meshfile, lMg)
        length = np.linalg.norm(b-a)

        set_transform(origin, a, b)

        mesh.tag = "cylinder"
        mesh.attrib.pop('filename')
        mesh.attrib['radius'] = str(radius)
        mesh.attrib['length'] = str(length)
        coll.attrib['name'] = name

        if link.find('collision_checking') is None:
            link.append(etree.Element('collision_checking'))
        collision_checking = link.find('collision_checking')
        collision_checking.append(etree.Element('capsule'))
        collision_checking[-1].attrib['name'] = name

    tree.write(outfile)

if __name__ == "__main__":
    # Example for a single capsule
    #filename = "mesh.obj"
    #mesh_loader = hppfcl.MeshLoader()
    #mesh = mesh_loader.load(filename, np.ones(3))
    #vertices = mesh.vertices()
    #a, b, r = capsule_approximation(vertices)
    
    # Example for a whole URDF model
    # This path refers to Pinocchio source code but you can define your own directory here.
    pinocchio_model_dir = join(dirname(dirname(str(abspath(__file__)))), "models")
    urdf_filename = pinocchio_model_dir + "models/others/robots/ur_description/urdf/ur5_gripper.urdf"
    parse_urdf(urdf_filename, "ur5_gripper_with_capsules.urdf")
