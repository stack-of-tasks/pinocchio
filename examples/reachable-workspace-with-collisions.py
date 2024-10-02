import itertools
import sys
import time
from pathlib import Path

import meshcat.geometry as g
import numpy as np
import pinocchio as pin
from pinocchio.visualize import MeshcatVisualizer


def XYZRPYtoSE3(xyzrpy):
    rotate = pin.utils.rotate
    R = rotate("x", xyzrpy[3]) @ rotate("y", xyzrpy[4]) @ rotate("z", xyzrpy[5])
    p = np.array(xyzrpy[:3])
    return pin.SE3(R, p)


# Load the URDF model.
pinocchio_model_dir = Path(__file__).parent.parent / "models"

model_path = pinocchio_model_dir / "example-robot-data/robots"
mesh_dir = pinocchio_model_dir

urdf_path = model_path / "panda_description/urdf/panda.urdf"
srdf_path = model_path / "panda_description/srdf/panda.srdf"

robot, collision_model, visual_model = pin.buildModelsFromUrdf(urdf_path, mesh_dir)
data = robot.createData()

# Obstacle map
# Capsule obstacles will be placed at these XYZ-RPY parameters
oMobs = [
    [0.40, 0.0, 0.30, np.pi / 2, 0, 0],
    [-0.08, -0.0, 0.75, np.pi / 2, 0, 0],
    [0.23, -0.0, 0.04, np.pi / 2, 0, 0],
]

# Load visual objects and add them in collision/visual models
color = [1.0, 0.2, 0.2, 1.0]  # color of the capsules
rad, length = 0.1, 0.4  # radius and length of capsules
for i, xyzrpy in enumerate(oMobs):
    obs = pin.GeometryObject.CreateCapsule(rad, length)  # Pinocchio obstacle object
    obs.meshColor = np.array(
        [1.0, 0.2, 0.2, 1.0]
    )  # Don't forget me, otherwise I am transparent ...
    obs.name = "obs%d" % i  # Set object name
    obs.parentJoint = 0  # Set object parent = 0 = universe
    obs.placement = XYZRPYtoSE3(xyzrpy)  # Set object placement wrt parent
    collision_model.addGeometryObject(obs)  # Add object to collision model
    visual_model.addGeometryObject(obs)  # Add object to visual model

# Auto-collision pairs
collision_model.addAllCollisionPairs()
pin.removeCollisionPairs(robot, collision_model, srdf_path)

# Collision pairs
nobs = len(oMobs)
nbodies = collision_model.ngeoms - nobs
robotBodies = range(nbodies)
envBodies = range(nbodies, nbodies + nobs)
for a, b in itertools.product(robotBodies, envBodies):
    collision_model.addCollisionPair(pin.CollisionPair(a, b))

# Geom data
# Collision/visual models have been modified => re-generate corresponding data.
collision_data = pin.GeometryData(collision_model)
visual_data = pin.GeometryData(visual_model)

# Start a new MeshCat server and client.
viz = MeshcatVisualizer(robot, collision_model, visual_model)
try:
    viz.initViewer(open=True)
except ImportError as err:
    print(
        "Error while initializing the viewer. "
        "It seems you should install Python meshcat"
    )
    print(err)
    sys.exit(0)

# # Load the robot in the viewer.
viz.loadViewerModel()
# Display a robot configuration.
q0 = (robot.upperPositionLimit.T + robot.lowerPositionLimit.T) / 2
horizon = 0.2  # seconds
frame = robot.getFrameId(
    robot.frames[-1].name
)  # for example the last frame of the robot
n_samples = 5
facet_dims = 2

# To have convex hull computation or just the points of the reachable workspace and then
# compute it with cgal.
convex = False

if convex:
    verts, faces = pin.reachableWorkspaceWithCollisionsHull(
        robot, collision_model, q0, horizon, frame, n_samples, facet_dims
    )
    verts = verts.T

else:
    try:
        from CGAL.CGAL_Alpha_wrap_3 import *  # noqa: F403
        from CGAL.CGAL_Kernel import *  # noqa: F403
        from CGAL.CGAL_Mesh_3 import *  # noqa: F403
        from CGAL.CGAL_Polyhedron_3 import Polyhedron_3
    except ModuleNotFoundError:
        print("To compute non convex Polytope CGAL library needs to be installed.")
        sys.exit(0)

    def vertex_to_tuple(v):
        return v.x(), v.y(), v.z()

    def halfedge_to_triangle(he):
        p1 = he
        p2 = p1.next()
        p3 = p2.next()
        return [
            vertex_to_tuple(p1.vertex().point()),
            vertex_to_tuple(p2.vertex().point()),
            vertex_to_tuple(p3.vertex().point()),
        ]

    def alpha_shape_with_cgal(coords, alpha=None):
        """
        Compute the alpha shape of a set of points. (Code thanks to A. Skuric)
        Retrieved from http://blog.thehumangeo.com/2014/05/12/drawing-boundaries-in-python/

        :param coords : Coordinates of points
        :param alpha: List of alpha values to influence the gooeyness of the border.
        Smaller numbers don't fall inward as much as larger numbers.
        Too large, and you lose everything!
        :return: Shapely.MultiPolygons which is the hull of the input set of points
        """
        if alpha is None:
            bbox_diag = np.linalg.norm(np.max(coords, 0) - np.min(coords, 0))
            alpha_value = bbox_diag / 5
        else:
            alpha_value = np.mean(alpha)
        # Convert to CGAL point
        points = [Point_3(pt[0], pt[1], pt[2]) for pt in coords]  # noqa: F405
        # Compute alpha shape
        Q = Polyhedron_3()
        _a = alpha_wrap_3(points, alpha_value, 0.01, Q)  # noqa: F405
        alpha_shape_vertices = np.array(
            [vertex_to_tuple(vertex.point()) for vertex in Q.vertices()]
        )

        alpha_shape_faces = np.array(
            [np.array(halfedge_to_triangle(face.halfedge())) for face in Q.facets()]
        )

        return alpha_shape_vertices, alpha_shape_faces

    verts = pin.reachableWorkspaceWithCollisions(
        robot, collision_model, q0, horizon, frame, n_samples, facet_dims
    )
    verts = verts.T

    alpha = 0.1
    verts, faces = alpha_shape_with_cgal(verts, alpha)
    verts = faces.reshape(-1, 3)
    faces = np.arange(len(verts)).reshape(-1, 3)

print("------------------- Display Vertex")


# meshcat triangulated mesh
poly = g.TriangularMeshGeometry(vertices=verts, faces=faces)
viz.viewer["poly"].set_object(
    poly, g.MeshBasicMaterial(color=0x000000, wireframe=True, linewidth=12, opacity=0.2)
)

while True:
    viz.display(q0)
    viz.viewer["poly"].set_object(
        poly,
        g.MeshBasicMaterial(color=0x000000, wireframe=True, linewidth=2, opacity=0.2),
    )

    time.sleep(1e-2)
