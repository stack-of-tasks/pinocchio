import os
import warnings
from pathlib import Path
from typing import ClassVar, List

import numpy as np

from .. import pinocchio_pywrap_default as pin
from ..deprecation import DeprecatedWarning
from ..utils import npToTuple
from . import BaseVisualizer

try:
    import meshcat
    import meshcat.geometry as mg
except ImportError:
    import_meshcat_succeed = False
else:
    import_meshcat_succeed = True

import base64

# DaeMeshGeometry
import xml.etree.ElementTree as Et
from typing import Any, Dict, Optional, Set, Union

MsgType = Dict[str, Union[str, bytes, bool, float, "MsgType"]]

try:
    import hppfcl

    WITH_HPP_FCL_BINDINGS = True
except ImportError:
    WITH_HPP_FCL_BINDINGS = False

DEFAULT_COLOR_PROFILES = {
    "gray": ([0.98, 0.98, 0.98], [0.8, 0.8, 0.8]),
    "white": ([1.0, 1.0, 1.0], [1.0, 1.0, 1.0]),
}
COLOR_PRESETS = DEFAULT_COLOR_PROFILES.copy()

FRAME_AXIS_POSITIONS = (
    np.array([[0, 0, 0], [1, 0, 0], [0, 0, 0], [0, 1, 0], [0, 0, 0], [0, 0, 1]])
    .astype(np.float32)
    .T
)
FRAME_AXIS_COLORS = (
    np.array([[1, 0, 0], [1, 0.6, 0], [0, 1, 0], [0.6, 1, 0], [0, 0, 1], [0, 0.6, 1]])
    .astype(np.float32)
    .T
)


def getColor(color):
    assert color is not None
    color = np.asarray(color)
    assert color.shape == (3,)
    return color.clip(0.0, 1.0)


def hasMeshFileInfo(geometry_object):
    """Check whether the geometry object contains a Mesh supported by MeshCat"""
    if geometry_object.meshPath == "":
        return False

    file_extension = Path(geometry_object.meshPath).suffix
    if file_extension.lower() in [".dae", ".obj", ".stl"]:
        return True

    return False


if import_meshcat_succeed:
    # Code adapted from Jiminy
    class Cone(mg.Geometry):
        """A cone of the given height and radius. By Three.js convention, the axis
        of rotational symmetry is aligned with the y-axis.
        """

        def __init__(
            self,
            height: float,
            radius: float,
            radialSegments: float = 32,
            openEnded: bool = False,
        ):
            super().__init__()
            self.radius = radius
            self.height = height
            self.radialSegments = radialSegments
            self.openEnded = openEnded

        def lower(self, object_data: Any) -> MsgType:
            return {
                "uuid": self.uuid,
                "type": "ConeGeometry",
                "radius": self.radius,
                "height": self.height,
                "radialSegments": self.radialSegments,
                "openEnded": self.openEnded,
            }

    class DaeMeshGeometry(mg.ReferenceSceneElement):
        def __init__(self, dae_path: str, cache: Optional[Set[str]] = None) -> None:
            """Load Collada files with texture images.
            Inspired from
            https://gist.github.com/danzimmerman/a392f8eadcf1166eb5bd80e3922dbdc5
            """
            # Init base class
            super().__init__()

            dae_path = Path(dae_path)

            # Attributes to be specified by the user
            self.path = None
            self.material = None
            self.intrinsic_transform = mg.tf.identity_matrix()

            # Raw file content
            dae_dir = dae_path.parent
            with dae_path.open() as text_file:
                self.dae_raw = text_file.read()

            # Parse the image resource in Collada file
            img_resource_paths = []
            img_lib_element = Et.parse(dae_path).find(
                "{http://www.collada.org/2005/11/COLLADASchema}library_images"
            )
            if img_lib_element:
                img_resource_paths = [
                    e.text for e in img_lib_element.iter() if e.tag.count("init_from")
                ]

            # Convert textures to data URL for Three.js ColladaLoader to load them
            self.img_resources = {}
            for img_path in img_resource_paths:
                # Return empty string if already in cache
                if cache is not None:
                    if img_path in cache:
                        self.img_resources[img_path] = ""
                        continue
                    cache.add(img_path)

                # Encode texture in base64
                img_path_abs = img_path
                if not img_path.is_absolute():
                    img_path_abs = os.path.normpath(dae_dir / img_path_abs)
                if not img_path_abs.is_file():
                    raise UserWarning(f"Texture '{img_path}' not found.")
                with Path(img_path_abs).open("rb") as img_file:
                    img_data = base64.b64encode(img_file.read())
                img_uri = f"data:image/png;base64,{img_data.decode('utf-8')}"
                self.img_resources[img_path] = img_uri

        def lower(self) -> Dict[str, Any]:
            """Pack data into a dictionary of the format that must be passed to
            `Visualizer.window.send`.
            """
            data = {
                "type": "set_object",
                "path": self.path.lower() if self.path is not None else "",
                "object": {
                    "metadata": {"version": 4.5, "type": "Object"},
                    "geometries": [],
                    "materials": [],
                    "object": {
                        "uuid": self.uuid,
                        "type": "_meshfile_object",
                        "format": "dae",
                        "data": self.dae_raw,
                        "resources": self.img_resources,
                        "matrix": list(self.intrinsic_transform.flatten()),
                    },
                },
            }
            if self.material is not None:
                self.material.lower_in_object(data)
            return data

        def set_scale(self, scale) -> None:
            self.intrinsic_transform[:3, :3] = np.diag(scale)

    # end code adapted from Jiminy

    class Plane(mg.Geometry):
        """A plane of the given width and height."""

        def __init__(
            self,
            width: float,
            height: float,
            widthSegments: float = 1,
            heightSegments: float = 1,
        ):
            super().__init__()
            self.width = width
            self.height = height
            self.widthSegments = widthSegments
            self.heightSegments = heightSegments

        def lower(self, object_data: Any) -> MsgType:
            return {
                "uuid": self.uuid,
                "type": "PlaneGeometry",
                "width": self.width,
                "height": self.height,
                "widthSegments": self.widthSegments,
                "heightSegments": self.heightSegments,
            }


if (
    WITH_HPP_FCL_BINDINGS
    and tuple(map(int, hppfcl.__version__.split("."))) >= (3, 0, 0)
    and hppfcl.WITH_OCTOMAP
):

    def loadOctree(octree: hppfcl.OcTree):
        boxes = octree.toBoxes()

        if len(boxes) == 0:
            return
        bs = boxes[0][3] / 2.0
        num_boxes = len(boxes)

        box_corners = np.array(
            [
                [bs, bs, bs],
                [bs, bs, -bs],
                [bs, -bs, bs],
                [bs, -bs, -bs],
                [-bs, bs, bs],
                [-bs, bs, -bs],
                [-bs, -bs, bs],
                [-bs, -bs, -bs],
            ]
        )

        all_points = np.empty((8 * num_boxes, 3))
        all_faces = np.empty((12 * num_boxes, 3), dtype=int)
        face_id = 0
        for box_id, box_properties in enumerate(boxes):
            box_center = box_properties[:3]

            corners = box_corners + box_center
            point_range = range(box_id * 8, (box_id + 1) * 8)
            all_points[point_range, :] = corners

            A = box_id * 8
            B = A + 1
            C = B + 1
            D = C + 1
            E = D + 1
            F = E + 1
            G = F + 1
            H = G + 1

            all_faces[face_id] = np.array([C, D, B])
            all_faces[face_id + 1] = np.array([B, A, C])
            all_faces[face_id + 2] = np.array([A, B, F])
            all_faces[face_id + 3] = np.array([F, E, A])
            all_faces[face_id + 4] = np.array([E, F, H])
            all_faces[face_id + 5] = np.array([H, G, E])
            all_faces[face_id + 6] = np.array([G, H, D])
            all_faces[face_id + 7] = np.array([D, C, G])
            # # top
            all_faces[face_id + 8] = np.array([A, E, G])
            all_faces[face_id + 9] = np.array([G, C, A])
            # # bottom
            all_faces[face_id + 10] = np.array([B, H, F])
            all_faces[face_id + 11] = np.array([H, B, D])

            face_id += 12

        colors = np.empty((all_points.shape[0], 3))
        colors[:] = np.ones(3)
        mesh = mg.TriangularMeshGeometry(all_points, all_faces, colors)
        return mesh

else:

    def loadOctree(octree):
        raise NotImplementedError("loadOctree need hppfcl with octomap support")


if WITH_HPP_FCL_BINDINGS:

    def loadMesh(mesh):
        if isinstance(mesh, (hppfcl.HeightFieldOBBRSS, hppfcl.HeightFieldAABB)):
            heights = mesh.getHeights()
            x_grid = mesh.getXGrid()
            y_grid = mesh.getYGrid()
            min_height = mesh.getMinHeight()

            X, Y = np.meshgrid(x_grid, y_grid)

            nx = len(x_grid) - 1
            ny = len(y_grid) - 1

            num_cells = (nx) * (ny) * 2 + (nx + ny) * 4 + 2

            num_vertices = X.size
            num_tris = num_cells

            faces = np.empty((num_tris, 3), dtype=int)
            vertices = np.vstack(
                (
                    np.stack(
                        (
                            X.reshape(num_vertices),
                            Y.reshape(num_vertices),
                            heights.reshape(num_vertices),
                        ),
                        axis=1,
                    ),
                    np.stack(
                        (
                            X.reshape(num_vertices),
                            Y.reshape(num_vertices),
                            np.full(num_vertices, min_height),
                        ),
                        axis=1,
                    ),
                )
            )

            face_id = 0
            for y_id in range(ny):
                for x_id in range(nx):
                    p0 = x_id + y_id * (nx + 1)
                    p1 = p0 + 1
                    p2 = p1 + nx + 1
                    p3 = p2 - 1

                    faces[face_id] = np.array([p0, p3, p1])
                    face_id += 1
                    faces[face_id] = np.array([p3, p2, p1])
                    face_id += 1

                    if y_id == 0:
                        p0_low = p0 + num_vertices
                        p1_low = p1 + num_vertices

                        faces[face_id] = np.array([p0, p1_low, p0_low])
                        face_id += 1
                        faces[face_id] = np.array([p0, p1, p1_low])
                        face_id += 1

                    if y_id == ny - 1:
                        p2_low = p2 + num_vertices
                        p3_low = p3 + num_vertices

                        faces[face_id] = np.array([p3, p3_low, p2_low])
                        face_id += 1
                        faces[face_id] = np.array([p3, p2_low, p2])
                        face_id += 1

                    if x_id == 0:
                        p0_low = p0 + num_vertices
                        p3_low = p3 + num_vertices

                        faces[face_id] = np.array([p0, p3_low, p3])
                        face_id += 1
                        faces[face_id] = np.array([p0, p0_low, p3_low])
                        face_id += 1

                    if x_id == nx - 1:
                        p1_low = p1 + num_vertices
                        p2_low = p2 + num_vertices

                        faces[face_id] = np.array([p1, p2_low, p2])
                        face_id += 1
                        faces[face_id] = np.array([p1, p1_low, p2_low])
                        face_id += 1

            # Last face
            p0 = num_vertices
            p1 = p0 + nx
            p2 = 2 * num_vertices - 1
            p3 = p2 - nx

            faces[face_id] = np.array([p0, p1, p2])
            face_id += 1
            faces[face_id] = np.array([p0, p2, p3])
            face_id += 1

        elif isinstance(mesh, (hppfcl.Convex, hppfcl.BVHModelBase)):
            if isinstance(mesh, hppfcl.BVHModelBase):
                num_vertices = mesh.num_vertices
                num_tris = mesh.num_tris

                call_triangles = mesh.tri_indices
                call_vertices = mesh.vertices

            elif isinstance(mesh, hppfcl.Convex):
                num_vertices = mesh.num_points
                num_tris = mesh.num_polygons

                call_triangles = mesh.polygons
                call_vertices = mesh.points

            faces = np.empty((num_tris, 3), dtype=int)
            for k in range(num_tris):
                tri = call_triangles(k)
                faces[k] = [tri[i] for i in range(3)]

            vertices = call_vertices()
            vertices = vertices.astype(np.float32)

        if num_tris > 0:
            mesh = mg.TriangularMeshGeometry(vertices, faces)
        else:
            mesh = mg.Points(
                mg.PointsGeometry(
                    vertices.T, color=np.repeat(np.ones((3, 1)), num_vertices, axis=1)
                ),
                mg.PointsMaterial(size=0.002),
            )

        return mesh

else:

    def loadMesh(mesh):
        raise NotImplementedError("loadMesh need hppfcl")


def loadPrimitive(geometry_object):
    import meshcat.geometry as mg

    # Cylinders need to be rotated
    R = np.array(
        [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, -1.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )
    RotatedCylinder = type(
        "RotatedCylinder", (mg.Cylinder,), {"intrinsic_transform": lambda self: R}
    )

    geom = geometry_object.geometry
    obj = None
    if WITH_HPP_FCL_BINDINGS and isinstance(geom, hppfcl.ShapeBase):
        if isinstance(geom, hppfcl.Capsule):
            if hasattr(mg, "TriangularMeshGeometry"):
                obj = createCapsule(2.0 * geom.halfLength, geom.radius)
            else:
                obj = RotatedCylinder(2.0 * geom.halfLength, geom.radius)
        elif isinstance(geom, hppfcl.Cylinder):
            obj = RotatedCylinder(2.0 * geom.halfLength, geom.radius)
        elif isinstance(geom, hppfcl.Cone):
            obj = RotatedCylinder(2.0 * geom.halfLength, 0, geom.radius, 0)
        elif isinstance(geom, hppfcl.Box):
            obj = mg.Box(npToTuple(2.0 * geom.halfSide))
        elif isinstance(geom, hppfcl.Sphere):
            obj = mg.Sphere(geom.radius)
        elif isinstance(geom, hppfcl.ConvexBase):
            obj = loadMesh(geom)

    if obj is None:
        msg = f"Unsupported geometry type for {geometry_object.name} ({type(geom)})"
        warnings.warn(msg, category=UserWarning, stacklevel=2)

    return obj


def createCapsule(length, radius, radial_resolution=30, cap_resolution=10):
    nbv = np.array([max(radial_resolution, 4), max(cap_resolution, 4)])
    h = length
    r = radius
    position = 0
    vertices = np.zeros((nbv[0] * (2 * nbv[1]) + 2, 3))
    for j in range(nbv[0]):
        phi = (2 * np.pi * j) / nbv[0]
        for i in range(nbv[1]):
            theta = (np.pi / 2 * i) / nbv[1]
            vertices[position + i, :] = np.array(
                [
                    np.cos(theta) * np.cos(phi) * r,
                    np.cos(theta) * np.sin(phi) * r,
                    -h / 2 - np.sin(theta) * r,
                ]
            )
            vertices[position + i + nbv[1], :] = np.array(
                [
                    np.cos(theta) * np.cos(phi) * r,
                    np.cos(theta) * np.sin(phi) * r,
                    h / 2 + np.sin(theta) * r,
                ]
            )
        position += nbv[1] * 2
    vertices[-2, :] = np.array([0, 0, -h / 2 - r])
    vertices[-1, :] = np.array([0, 0, h / 2 + r])
    indexes = np.zeros((nbv[0] * (4 * (nbv[1] - 1) + 4), 3))
    index = 0
    stride = nbv[1] * 2
    last = nbv[0] * (2 * nbv[1]) + 1
    for j in range(nbv[0]):
        j_next = (j + 1) % nbv[0]
        indexes[index + 0] = np.array(
            [j_next * stride + nbv[1], j_next * stride, j * stride]
        )
        indexes[index + 1] = np.array(
            [j * stride + nbv[1], j_next * stride + nbv[1], j * stride]
        )
        indexes[index + 2] = np.array(
            [j * stride + nbv[1] - 1, j_next * stride + nbv[1] - 1, last - 1]
        )
        indexes[index + 3] = np.array(
            [j_next * stride + 2 * nbv[1] - 1, j * stride + 2 * nbv[1] - 1, last]
        )
        for i in range(nbv[1] - 1):
            indexes[index + 4 + i * 4 + 0] = np.array(
                [j_next * stride + i, j_next * stride + i + 1, j * stride + i]
            )
            indexes[index + 4 + i * 4 + 1] = np.array(
                [j_next * stride + i + 1, j * stride + i + 1, j * stride + i]
            )
            indexes[index + 4 + i * 4 + 2] = np.array(
                [
                    j_next * stride + nbv[1] + i + 1,
                    j_next * stride + nbv[1] + i,
                    j * stride + nbv[1] + i,
                ]
            )
            indexes[index + 4 + i * 4 + 3] = np.array(
                [
                    j_next * stride + nbv[1] + i + 1,
                    j * stride + nbv[1] + i,
                    j * stride + nbv[1] + i + 1,
                ]
            )
        index += 4 * (nbv[1] - 1) + 4
    return mg.TriangularMeshGeometry(vertices, indexes)


class MeshcatVisualizer(BaseVisualizer):
    """A Pinocchio display using Meshcat"""

    FORCE_SCALE = 0.06
    FRAME_VEL_COLOR = 0x00FF00
    CAMERA_PRESETS: ClassVar = {
        "preset0": [
            np.zeros(3),  # target
            [3.0, 0.0, 1.0],  # anchor point (x, z, -y) lhs coords
        ],
        "preset1": [np.zeros(3), [1.0, 1.0, 1.0]],
        "preset2": [[0.0, 0.0, 0.6], [0.8, 1.0, 1.2]],
        "acrobot": [[0.0, 0.1, 0.0], [0.5, 0.0, 0.2]],
        "cam_ur": [[0.4, 0.6, -0.2], [1.0, 0.4, 1.2]],
        "cam_ur2": [[0.4, 0.3, 0.0], [0.5, 0.1, 1.4]],
        "cam_ur3": [[0.4, 0.3, 0.0], [0.6, 1.3, 0.3]],
        "cam_ur4": [[-1.0, 0.3, 0.0], [1.3, 0.1, 1.2]],  # x>0 to x<0
        "cam_ur5": [[-1.0, 0.3, 0.0], [-0.05, 1.5, 1.2]],
        "talos": [[0.0, 1.2, 0.0], [1.5, 0.3, 1.5]],
        "talos2": [[0.0, 1.1, 0.0], [1.2, 0.6, 1.5]],
    }

    def __init__(
        self,
        model=pin.Model(),
        collision_model=None,
        visual_model=None,
        copy_models=False,
        data=None,
        collision_data=None,
        visual_data=None,
    ):
        if not import_meshcat_succeed:
            msg = (
                "Error while importing the viewer client.\n"
                "Check whether meshcat is properly installed "
                "(pip install --user meshcat)."
            )
            raise ImportError(msg)

        super().__init__(
            model,
            collision_model,
            visual_model,
            copy_models,
            data,
            collision_data,
            visual_data,
        )
        self.static_objects = []

    def getViewerNodeName(self, geometry_object, geometry_type):
        """Return the name of the geometry object inside the viewer."""
        if geometry_type is pin.GeometryType.VISUAL:
            return self.viewerVisualGroupName + "/" + geometry_object.name
        elif geometry_type is pin.GeometryType.COLLISION:
            return self.viewerCollisionGroupName + "/" + geometry_object.name

    def initViewer(self, viewer=None, open=False, loadModel=False, zmq_url=None):
        """Start a new MeshCat server and client.
        Note: the server can also be started separately using the "meshcat-server"
        command in a terminal:
        this enables the server to remain active after the current script ends.
        """

        self.viewer = meshcat.Visualizer(zmq_url) if viewer is None else viewer

        self._node_default_cam = self.viewer["/Cameras/default"]
        self._node_background = self.viewer["/Background"]
        self._rot_cam_key = "rotated/<object>"
        self.static_objects = []

        self._check_meshcat_has_get_image()

        self._node_default_cam = self.viewer["/Cameras/default"]
        self._node_background = self.viewer["/Background"]
        self._rot_cam_key = "rotated/object"
        self.static_objects = []

        self._check_meshcat_has_get_image()

        if open:
            self.viewer.open()

        if loadModel:
            self.loadViewerModel()

    def reset(self):
        self.viewer.delete()
        self.static_objects = []

    def setBackgroundColor(self, preset_name: str = "gray", col_top=None, col_bot=None):
        """Set the background."""
        if col_top is not None:
            if col_bot is None:
                col_bot = col_top
        else:
            assert preset_name in COLOR_PRESETS.keys()
            col_top, col_bot = COLOR_PRESETS[preset_name]
        self._node_background.set_property("top_color", col_top)
        self._node_background.set_property("bottom_color", col_bot)

    def setCameraTarget(self, target: np.ndarray):
        self.viewer.set_cam_target(target)

    def setCameraPosition(self, position: np.ndarray):
        self.viewer.set_cam_pos(position)

    def setCameraPreset(self, preset_key: str):
        """Set the camera angle and position using a given preset."""
        assert preset_key in self.CAMERA_PRESETS
        cam_val = self.CAMERA_PRESETS[preset_key]
        self.setCameraTarget(cam_val[0])
        self.setCameraPosition(cam_val[1])

    def setCameraZoom(self, zoom: float):
        elt = self._node_default_cam[self._rot_cam_key]
        elt.set_property("zoom", zoom)

    def setCameraPose(self, pose):
        self._node_default_cam.set_transform(pose)

    def disableCameraControl(self):
        self.setCameraPosition([0, 0, 0])

    def enableCameraControl(self):
        self.setCameraPosition([3, 0, 1])

    def loadPrimitive(self, geometry_object: pin.GeometryObject):
        import meshcat.geometry as mg

        # Cylinders need to be rotated
        basic_three_js_transform = np.array(
            [
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, -1.0, 0.0],
                [0.0, 1.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )
        RotatedCylinder = type(
            "RotatedCylinder",
            (mg.Cylinder,),
            {"intrinsic_transform": lambda self: basic_three_js_transform},
        )

        # Cones need to be rotated

        geom = geometry_object.geometry
        obj = None
        if WITH_HPP_FCL_BINDINGS and isinstance(geom, hppfcl.ShapeBase):
            if isinstance(geom, hppfcl.Capsule):
                if hasattr(mg, "TriangularMeshGeometry"):
                    obj = createCapsule(2.0 * geom.halfLength, geom.radius)
                else:
                    obj = RotatedCylinder(2.0 * geom.halfLength, geom.radius)
            elif isinstance(geom, hppfcl.Cylinder):
                obj = RotatedCylinder(2.0 * geom.halfLength, geom.radius)
            elif isinstance(geom, hppfcl.Cone):
                obj = RotatedCylinder(2.0 * geom.halfLength, 0, geom.radius, 0)
            elif isinstance(geom, hppfcl.Box):
                obj = mg.Box(npToTuple(2.0 * geom.halfSide))
            elif isinstance(geom, hppfcl.Sphere):
                obj = mg.Sphere(geom.radius)
            elif isinstance(geom, hppfcl.Plane):
                To = np.eye(4)
                To[:3, 3] = geom.d * geom.n
                TranslatedPlane = type(
                    "TranslatedPlane",
                    (mg.Plane,),
                    {"intrinsic_transform": lambda self: To},
                )
                sx = geometry_object.meshScale[0] * 10
                sy = geometry_object.meshScale[1] * 10
                obj = TranslatedPlane(sx, sy)
            elif isinstance(geom, hppfcl.Ellipsoid):
                obj = mg.Ellipsoid(geom.radii)
            elif isinstance(geom, (hppfcl.Plane, hppfcl.Halfspace)):
                plane_transform: pin.SE3 = pin.SE3.Identity()
                # plane_transform.translation[:] = geom.d # Does not work
                plane_transform.rotation = pin.Quaternion.FromTwoVectors(
                    pin.ZAxis, geom.n
                ).toRotationMatrix()
                TransformedPlane = type(
                    "TransformedPlane",
                    (Plane,),
                    {"intrinsic_transform": lambda self: plane_transform.homogeneous},
                )
                obj = TransformedPlane(1000, 1000)
            elif isinstance(geom, hppfcl.ConvexBase):
                obj = loadMesh(geom)

        if obj is None:
            msg = f"Unsupported geometry type for {geometry_object.name} ({type(geom)})"
            warnings.warn(msg, category=UserWarning, stacklevel=2)
            obj = None

        return obj

    def loadMeshFromFile(self, geometry_object):
        # Mesh path is empty if Pinocchio is built without HPP-FCL bindings
        if geometry_object.meshPath == "":
            msg = (
                "Display of geometric primitives is supported only if "
                "pinocchio is build with HPP-FCL bindings."
            )
            warnings.warn(msg, category=UserWarning, stacklevel=2)
            return None

        # Get file type from filename extension.
        file_extension = Path(geometry_object.meshPath).suffix
        if file_extension.lower() == ".dae":
            obj = DaeMeshGeometry(geometry_object.meshPath)
        elif file_extension.lower() == ".obj":
            obj = mg.ObjMeshGeometry.from_file(geometry_object.meshPath)
        elif file_extension.lower() == ".stl":
            obj = mg.StlMeshGeometry.from_file(geometry_object.meshPath)
        else:
            msg = f"Unknown mesh file format: {geometry_object.meshPath}."
            warnings.warn(msg, category=UserWarning, stacklevel=2)
            obj = None

        return obj

    def loadViewerGeometryObject(self, geometry_object, geometry_type, color=None):
        """Load a single geometry object"""
        node_name = self.getViewerNodeName(geometry_object, geometry_type)
        meshcat_node = self.viewer[node_name]

        is_mesh = False
        try:
            obj = None
            if WITH_HPP_FCL_BINDINGS:
                if isinstance(geometry_object.geometry, hppfcl.ShapeBase):
                    obj = self.loadPrimitive(geometry_object)
                elif (
                    tuple(map(int, hppfcl.__version__.split("."))) >= (3, 0, 0)
                    and hppfcl.WITH_OCTOMAP
                    and isinstance(geometry_object.geometry, hppfcl.OcTree)
                ):
                    obj = loadOctree(geometry_object.geometry)
                elif hasMeshFileInfo(geometry_object):
                    obj = self.loadMeshFromFile(geometry_object)
                    is_mesh = True
                elif isinstance(
                    geometry_object.geometry,
                    (
                        hppfcl.BVHModelBase,
                        hppfcl.HeightFieldOBBRSS,
                        hppfcl.HeightFieldAABB,
                    ),
                ):
                    obj = loadMesh(geometry_object.geometry)
            if obj is None and hasMeshFileInfo(geometry_object):
                obj = self.loadMeshFromFile(geometry_object)
                is_mesh = True
            if obj is None:
                msg = (
                    "The geometry object named "
                    + geometry_object.name
                    + " is not supported by Pinocchio/MeshCat for vizualization."
                )
                warnings.warn(msg, category=UserWarning, stacklevel=2)
                return
        except Exception as e:
            msg = (
                "Error while loading geometry object: "
                f"{geometry_object.name}\nError message:\n{e}"
            )
            warnings.warn(msg, category=UserWarning, stacklevel=2)
            return

        if isinstance(obj, mg.Object):
            meshcat_node.set_object(obj)
        elif isinstance(obj, (mg.Geometry, mg.ReferenceSceneElement)):
            material = mg.MeshPhongMaterial()
            # Set material color from URDF, converting for triplet of doubles to a
            # single int.

            def to_material_color(rgba) -> int:
                """Convert rgba color as list into rgba color as int"""
                return (
                    int(rgba[0] * 255) * 256**2
                    + int(rgba[1] * 255) * 256
                    + int(rgba[2] * 255)
                )

            if color is None:
                meshColor = geometry_object.meshColor
            else:
                meshColor = color
            # Add transparency, if needed.
            material.color = to_material_color(meshColor)

            if float(meshColor[3]) != 1.0:
                material.transparent = True
                material.opacity = float(meshColor[3])

            geom_material = geometry_object.meshMaterial
            if geometry_object.overrideMaterial and isinstance(
                geom_material, pin.GeometryPhongMaterial
            ):
                material.emissive = to_material_color(geom_material.meshEmissionColor)
                material.specular = to_material_color(geom_material.meshSpecularColor)
                material.shininess = geom_material.meshShininess * 100.0

            if isinstance(obj, DaeMeshGeometry):
                obj.path = meshcat_node.path
                scale = list(np.asarray(geometry_object.meshScale).flatten())
                obj.set_scale(scale)
                if geometry_object.overrideMaterial:
                    obj.material = material
                meshcat_node.window.send(obj)
            else:
                meshcat_node.set_object(obj, material)

        # Apply the scaling
        if is_mesh and not isinstance(obj, DaeMeshGeometry):
            scale = list(np.asarray(geometry_object.meshScale).flatten())
            meshcat_node.set_property("scale", scale)

    def loadViewerModel(
        self,
        rootNodeName="pinocchio",
        color=None,
        collision_color=None,
        visual_color=None,
    ):
        """Load the robot in a MeshCat viewer.
        Parameters:
            rootNodeName: name to give to the robot in the viewer
            color: deprecated and optional, color to give to both the collision
                and visual models of the robot. This setting overwrites any color
                specified in the robot description. Format is a list of four
                RGBA floating-point numbers (between 0 and 1)
            collision_color: optional, color to give to the collision model of
                the robot. Format is a list of four RGBA floating-point numbers
                (between 0 and 1)
            visual_color: optional, color to give to the visual model of
                the robot. Format is a list of four RGBA floating-point numbers
                (between 0 and 1)
        """
        if color is not None:
            warnings.warn(
                "The 'color' argument is deprecated and will be removed in a "
                "future version of Pinocchio. Consider using "
                "'collision_color' and 'visual_color' instead.",
                category=DeprecatedWarning,
            )
            collision_color = color
            visual_color = color

        # Set viewer to use to gepetto-gui.
        self.viewerRootNodeName = rootNodeName

        # Collisions
        self.viewerCollisionGroupName = self.viewerRootNodeName + "/" + "collisions"

        if self.collision_model is not None:
            for collision in self.collision_model.geometryObjects:
                self.loadViewerGeometryObject(
                    collision, pin.GeometryType.COLLISION, collision_color
                )
        self.displayCollisions(False)

        # Visuals
        self.viewerVisualGroupName = self.viewerRootNodeName + "/" + "visuals"
        if self.visual_model is not None:
            for visual in self.visual_model.geometryObjects:
                self.loadViewerGeometryObject(
                    visual, pin.GeometryType.VISUAL, visual_color
                )
        self.displayVisuals(True)

        # Frames
        self.viewerFramesGroupName = self.viewerRootNodeName + "/" + "frames"
        self.displayFrames(False)

    def reload(self, new_geometry_object, geometry_type=None):
        """Reload a geometry_object given by its name and its type"""
        if geometry_type == pin.GeometryType.VISUAL:
            geom_model = self.visual_model
        else:
            geom_model = self.collision_model
            geometry_type = pin.GeometryType.COLLISION

        geom_id = geom_model.getGeometryId(new_geometry_object.name)
        geom_model.geometryObjects[geom_id] = new_geometry_object

        self.delete(new_geometry_object, geometry_type)
        visual = geom_model.geometryObjects[geom_id]
        self.loadViewerGeometryObject(visual, geometry_type)

    def clean(self):
        self.viewer.delete()

    def delete(self, geometry_object, geometry_type):
        viewer_name = self.getViewerNodeName(geometry_object, geometry_type)
        self.viewer[viewer_name].delete()

    def display(self, q=None):
        """
        Display the robot at configuration q in the viewer by placing all the bodies
        """
        if q is not None:
            pin.forwardKinematics(self.model, self.data, q)

        if self.display_collisions:
            self.updatePlacements(pin.GeometryType.COLLISION)

        if self.display_visuals:
            self.updatePlacements(pin.GeometryType.VISUAL)

        if self.display_frames:
            self.updateFrames()

    def updatePlacements(self, geometry_type):
        if geometry_type == pin.GeometryType.VISUAL:
            geom_model = self.visual_model
            geom_data = self.visual_data
        else:
            geom_model = self.collision_model
            geom_data = self.collision_data

        pin.updateGeometryPlacements(self.model, self.data, geom_model, geom_data)
        for visual in geom_model.geometryObjects:
            visual_name = self.getViewerNodeName(visual, geometry_type)
            # Get mesh pose.
            M = geom_data.oMg[geom_model.getGeometryId(visual.name)]
            # Manage scaling: force scaling even if this should be normally handled by
            # MeshCat (but there is a bug here)
            geom = visual.geometry
            if WITH_HPP_FCL_BINDINGS and isinstance(
                geom, (hppfcl.Plane, hppfcl.Halfspace)
            ):
                T = M.copy()
                T.translation += M.rotation @ (geom.d * geom.n)
                T = T.homogeneous
            else:
                T = M.homogeneous

            # Update viewer configuration.
            self.viewer[visual_name].set_transform(T)

        for visual in self.static_objects:
            visual_name = self.getViewerNodeName(visual, pin.GeometryType.VISUAL)
            M: pin.SE3 = visual.placement
            T = M.homogeneous
            self.viewer[visual_name].set_transform(T)

    def addGeometryObject(self, obj: pin.GeometryObject, color=None):
        """Add a visual GeometryObject to the viewer, with an optional color."""
        self.loadViewerGeometryObject(obj, pin.GeometryType.VISUAL, color)
        self.static_objects.append(obj)

    def _check_meshcat_has_get_image(self):
        if not hasattr(self.viewer, "get_image"):
            warnings.warn(
                "meshcat.Visualizer does not have the get_image() method."
                " You need meshcat >= 0.2.0 to get this feature."
            )

    def captureImage(self, w=None, h=None):
        """Capture an image from the Meshcat viewer and return an RGB array."""
        if w is not None or h is not None:
            # pass arguments when either is not None
            img = self.viewer.get_image(w, h)
        else:
            img = self.viewer.get_image()
        img_arr = np.asarray(img)
        return img_arr

    def displayCollisions(self, visibility):
        """Set whether to display collision objects or not."""
        if self.collision_model is None:
            self.display_collisions = False
        else:
            self.display_collisions = visibility
        self.viewer[self.viewerCollisionGroupName].set_property("visible", visibility)

        if visibility:
            self.updatePlacements(pin.GeometryType.COLLISION)

    def displayVisuals(self, visibility):
        """Set whether to display visual objects or not."""
        if self.visual_model is None:
            self.display_visuals = False
        else:
            self.display_visuals = visibility
        self.viewer[self.viewerVisualGroupName].set_property("visible", visibility)

        if visibility:
            self.updatePlacements(pin.GeometryType.VISUAL)

    def displayFrames(self, visibility, frame_ids=None, axis_length=0.2, axis_width=2):
        """Set whether to display frames or not."""
        self.display_frames = visibility
        if visibility:
            self.initializeFrames(frame_ids, axis_length, axis_width)
        self.viewer[self.viewerFramesGroupName].set_property("visible", visibility)

    def initializeFrames(self, frame_ids=None, axis_length=0.2, axis_width=2):
        """Initializes the frame objects for display."""
        import meshcat.geometry as mg

        self.viewer[self.viewerFramesGroupName].delete()
        self.frame_ids = []

        for fid, frame in enumerate(self.model.frames):
            if frame_ids is None or fid in frame_ids:
                frame_viz_name = f"{self.viewerFramesGroupName}/{frame.name}"
                self.viewer[frame_viz_name].set_object(
                    mg.LineSegments(
                        mg.PointsGeometry(
                            position=axis_length * FRAME_AXIS_POSITIONS,
                            color=FRAME_AXIS_COLORS,
                        ),
                        mg.LineBasicMaterial(
                            linewidth=axis_width,
                            vertexColors=True,
                        ),
                    )
                )
                self.frame_ids.append(fid)

    def updateFrames(self):
        """
        Updates the frame visualizations with the latest transforms from model data.
        """
        pin.updateFramePlacements(self.model, self.data)
        for fid in self.frame_ids:
            frame_name = self.model.frames[fid].name
            frame_viz_name = f"{self.viewerFramesGroupName}/{frame_name}"
            self.viewer[frame_viz_name].set_transform(self.data.oMf[fid].homogeneous)

    def drawFrameVelocities(self, frame_id: int, v_scale=0.2, color=FRAME_VEL_COLOR):
        pin.updateFramePlacement(self.model, self.data, frame_id)
        vFr = pin.getFrameVelocity(
            self.model, self.data, frame_id, pin.LOCAL_WORLD_ALIGNED
        )
        line_group_name = f"ee_vel/{frame_id}"
        self._draw_vectors_from_frame(
            [v_scale * vFr.linear], [frame_id], [line_group_name], [color]
        )

    def _draw_vectors_from_frame(
        self,
        vecs: List[np.ndarray],
        frame_ids: List[int],
        vec_names: List[str],
        colors: List[int],
    ):
        """Draw vectors extending from given frames."""
        import meshcat.geometry as mg

        if len(vecs) != len(frame_ids) or len(vecs) != len(vec_names):
            return ValueError(
                "Number of vectors and frames IDs or names is inconsistent."
            )
        for i, (fid, v) in enumerate(zip(frame_ids, vecs)):
            frame_pos = self.data.oMf[fid].translation
            vertices = np.array([frame_pos, frame_pos + v]).astype(np.float32).T
            name = vec_names[i]
            geometry = mg.PointsGeometry(position=vertices)
            geom_object = mg.LineSegments(
                geometry, mg.LineBasicMaterial(color=colors[i])
            )
            prefix = self.viewerVisualGroupName + "/lines/" + name
            self.viewer[prefix].set_object(geom_object)


__all__ = ["MeshcatVisualizer"]
