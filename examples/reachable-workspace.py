import sys
import time
from os.path import abspath, dirname, join

import meshcat.geometry as g
import numpy as np
import pinocchio as pin
from pinocchio.visualize import MeshcatVisualizer

# Load the URDF model.
pinocchio_model_dir = join(dirname(dirname(str(abspath(__file__)))), "models")

model_path = join(pinocchio_model_dir, "example-robot-data/robots")
mesh_dir = pinocchio_model_dir

urdf_filename = "panda.urdf"
urdf_model_path = join(join(model_path, "panda_description/urdf"), urdf_filename)

robot, collision_model, visual_model = pin.buildModelsFromUrdf(
    urdf_model_path, mesh_dir
)
data = robot.createData()

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
convex = True

if convex:
    verts, faces = pin.reachableWorkspaceHull(
        robot, q0, horizon, frame, n_samples, facet_dims
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

    verts = pin.reachableWorkspace(robot, q0, horizon, frame, n_samples, facet_dims)
    verts = verts.T

    alpha = 0.2
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
