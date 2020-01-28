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


filename = "mesh.obj"
mesh_loader = hppfcl.MeshLoader()
mesh = mesh_loader.load(filename, np.ones(3))
vertices = mesh.vertices()
a, b, r = capsule_approximation(vertices)
