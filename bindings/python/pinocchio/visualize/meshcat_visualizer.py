from .. import pinocchio_pywrap_default as pin
from ..shortcuts import buildModelsFromUrdf, createDatas
from ..utils import npToTuple

from . import BaseVisualizer

import os
import warnings
import numpy as np
from distutils.version import LooseVersion
from typing import List

import meshcat
import meshcat.geometry as mg

# DaeMeshGeometry
import xml.etree.ElementTree as Et
import base64

from typing import Optional, Any, Dict, Union, Type, Set

MsgType = Dict[str, Union[str, bytes, bool, float, 'MsgType']]

try:
    import hppfcl
    WITH_HPP_FCL_BINDINGS = True
except ImportError:
    WITH_HPP_FCL_BINDINGS = False

DEFAULT_COLOR_PROFILES = {
    "gray": ([0.98, 0.98, 0.98], [0.8, 0.8, 0.8]),
    "white": (np.ones(3), )
}
COLOR_PRESETS = DEFAULT_COLOR_PROFILES.copy()


def getColor(color):
    assert color is not None
    color = np.asarray(color)
    assert color.shape == (3,)
    return color.clip(0., 1.)


def hasMeshFileInfo(geometry_object):
    """ Check whether the geometry object contains a Mesh supported by MeshCat """
    if geometry_object.meshPath == "":
        return False

    _, file_extension = os.path.splitext(geometry_object.meshPath)
    if file_extension.lower() in [".dae", ".obj", ".stl"]:
        return True

    return False

# Code adapted from Jiminy
class Cone(mg.Geometry):
    """A cone of the given height and radius. By Three.js convention, the axis
    of rotational symmetry is aligned with the y-axis.
    """
    def __init__(self, height: float, radius: float, radialSegments: float = 32, openEnded: bool = False):
        super().__init__()
        self.radius = radius
        self.height = height
        self.radialSegments = radialSegments
        self.openEnded = openEnded

    def lower(self, object_data: Any) -> MsgType:
        return {
            u"uuid": self.uuid,
            u"type": u"ConeGeometry",
            u"radius": self.radius,
            u"height": self.height,
            u"radialSegments": self.radialSegments,
            u"openEnded": self.openEnded,
        }

class DaeMeshGeometry(mg.ReferenceSceneElement):
    def __init__(self,
                 dae_path: str,
                 cache: Optional[Set[str]] = None) -> None:
        """Load Collada files with texture images.
        Inspired from
        https://gist.github.com/danzimmerman/a392f8eadcf1166eb5bd80e3922dbdc5
        """
        # Init base class
        super().__init__()

        # Attributes to be specified by the user
        self.path = None
        self.material = None

        # Raw file content
        dae_dir = os.path.dirname(dae_path)
        with open(dae_path, 'r') as text_file:
            self.dae_raw = text_file.read()

        # Parse the image resource in Collada file
        img_resource_paths = []
        img_lib_element = Et.parse(dae_path).find(
            "{http://www.collada.org/2005/11/COLLADASchema}library_images")
        if img_lib_element:
            img_resource_paths = [
                e.text for e in img_lib_element.iter()
                if e.tag.count('init_from')]

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
            if not os.path.isabs(img_path):
                img_path_abs = os.path.normpath(
                    os.path.join(dae_dir, img_path_abs))
            if not os.path.isfile(img_path_abs):
                raise UserWarning(f"Texture '{img_path}' not found.")
            with open(img_path_abs, 'rb') as img_file:
                img_data = base64.b64encode(img_file.read())
            img_uri = f"data:image/png;base64,{img_data.decode('utf-8')}"
            self.img_resources[img_path] = img_uri

    def lower(self) -> Dict[str, Any]:
        """Pack data into a dictionary of the format that must be passed to
        `Visualizer.window.send`.
        """
        data = {
            'type': 'set_object',
            'path': self.path.lower() if self.path is not None else "",
            'object': {
                'metadata': {'version': 4.5, 'type': 'Object'},
                'geometries': [],
                'materials': [],
                'object': {
                    'uuid': self.uuid,
                    'type': '_meshfile_object',
                    'format': 'dae',
                    'data': self.dae_raw,
                    'resources': self.img_resources
                }
            }
        }
        if self.material is not None:
            self.material.lower_in_object(data)
        return data

# end code adapted from Jiminy

class Plane(mg.Geometry):
    """A plane of the given width and height. 
    """
    def __init__(self, width: float, height: float, widthSegments: float = 1, heightSegments: float = 1):
        super().__init__()
        self.width = width
        self.height = height
        self.widthSegments = widthSegments
        self.heightSegments = heightSegments

    def lower(self, object_data: Any) -> MsgType:
        return {
            u"uuid": self.uuid,
            u"type": u"PlaneGeometry",
            u"width": self.width,
            u"height": self.height,
            u"widthSegments": self.widthSegments,
            u"heightSegments": self.heightSegments,
        }

def loadMesh(mesh):

    if isinstance(mesh,(hppfcl.HeightFieldOBBRSS, hppfcl.HeightFieldAABB)):
        heights = mesh.getHeights()
        x_grid = mesh.getXGrid()
        y_grid = mesh.getYGrid()
        min_height = mesh.getMinHeight()

        X, Y = np.meshgrid(x_grid,y_grid)

        nx = len(x_grid)-1
        ny = len(y_grid)-1

        num_cells = (nx) * (ny) * 2 + (nx+ny)*4 + 2

        num_vertices = X.size
        num_tris = num_cells

        faces = np.empty((num_tris,3),dtype=int)
        vertices = np.vstack((np.stack((X.reshape(num_vertices),Y.reshape(num_vertices),heights.reshape(num_vertices)),axis=1),
                              np.stack((X.reshape(num_vertices),Y.reshape(num_vertices),np.full(num_vertices,min_height)),axis=1)))

        face_id = 0
        for y_id in range(ny):
            for x_id in range(nx):
                p0 = x_id + y_id * (nx+1)
                p1 = p0 + 1
                p2 = p1 + nx + 1
                p3 = p2 - 1

                faces[face_id] = np.array([p0,p3,p1])
                face_id += 1
                faces[face_id] = np.array([p3,p2,p1])
                face_id += 1

                if y_id == 0:
                    p0_low = p0 + num_vertices
                    p1_low = p1 + num_vertices

                    faces[face_id] = np.array([p0,p1_low,p0_low])
                    face_id += 1
                    faces[face_id] = np.array([p0,p1,p1_low])
                    face_id += 1

                if y_id == ny-1:
                    p2_low = p2 + num_vertices
                    p3_low = p3 + num_vertices

                    faces[face_id] = np.array([p3,p3_low,p2_low])
                    face_id += 1
                    faces[face_id] = np.array([p3,p2_low,p2])
                    face_id += 1

                if x_id == 0:
                    p0_low = p0 + num_vertices
                    p3_low = p3 + num_vertices

                    faces[face_id] = np.array([p0,p3_low,p3])
                    face_id += 1
                    faces[face_id] = np.array([p0,p0_low,p3_low])
                    face_id += 1

                if x_id == nx-1:
                    p1_low = p1 + num_vertices
                    p2_low = p2 + num_vertices

                    faces[face_id] = np.array([p1,p2_low,p2])
                    face_id += 1
                    faces[face_id] = np.array([p1,p1_low,p2_low])
                    face_id += 1

        # Last face
        p0 = num_vertices
        p1 = p0 + nx
        p2 = 2*num_vertices-1
        p3 = p2 - nx

        faces[face_id] = np.array([p0,p1,p2])
        face_id += 1
        faces[face_id] = np.array([p0,p2,p3])
        face_id += 1

    elif isinstance(mesh,(hppfcl.Convex,hppfcl.BVHModelBase)):
        if isinstance(mesh,hppfcl.BVHModelBase):
            num_vertices = mesh.num_vertices
            num_tris = mesh.num_tris

            call_triangles = mesh.tri_indices
            call_vertices = mesh.vertices

        elif isinstance(mesh,hppfcl.Convex):
            num_vertices = mesh.num_points
            num_tris = mesh.num_polygons

            call_triangles = mesh.polygons
            call_vertices = mesh.points

        faces = np.empty((num_tris,3),dtype=int)
        for k in range(num_tris):
            tri = call_triangles(k)
            faces[k] = [tri[i] for i in range(3)]

        if LooseVersion(hppfcl.__version__) >= LooseVersion("1.7.7"):
            vertices = call_vertices()
        else:
            vertices = np.empty((num_vertices,3))
            for k in range(num_vertices):
                vertices[k] = call_vertices(k)

        vertices = vertices.astype(np.float32)

    if num_tris > 0:
        mesh = mg.TriangularMeshGeometry(vertices, faces)
    else:
        mesh = mg.Points(
                    mg.PointsGeometry(vertices.T, color=np.repeat(np.ones((3,1)),num_vertices,axis=1)),
                    mg.PointsMaterial(size=0.002))

    return mesh

def createCapsule(length, radius, radial_resolution = 30, cap_resolution = 10):
    nbv = np.array([max(radial_resolution, 4), max(cap_resolution, 4)])
    h = length
    r = radius
    position = 0
    vertices = np.zeros((nbv[0] * (2 * nbv[1]) + 2, 3))
    for j in range(nbv[0]):
        phi = (( 2 * np.pi * j) / nbv[0])
        for i in range(nbv[1]):
            theta = ((np.pi / 2 * i) / nbv[1])
            vertices[position + i, :] = np.array([np.cos(theta) * np.cos(phi) * r,
                                               np.cos(theta) * np.sin(phi) * r,
                                               -h / 2 - np.sin(theta) * r])
            vertices[position + i + nbv[1], :] = np.array([np.cos(theta) * np.cos(phi) * r,
                                                        np.cos(theta) * np.sin(phi) * r,
                                                        h / 2 + np.sin(theta) * r])
        position += nbv[1] * 2
    vertices[-2, :] = np.array([0, 0, -h / 2 - r])
    vertices[-1, :] = np.array([0, 0, h / 2 + r])
    indexes = np.zeros((nbv[0] * (4 * (nbv[1] - 1) + 4), 3))
    index = 0
    stride = nbv[1] * 2
    last = nbv[0] * (2 * nbv[1]) + 1
    for j in range(nbv[0]):
        j_next = (j + 1) % nbv[0]
        indexes[index + 0] = np.array([j_next * stride + nbv[1], j_next * stride, j * stride])
        indexes[index + 1] = np.array([j * stride + nbv[1], j_next * stride + nbv[1], j * stride])
        indexes[index + 2] = np.array([j * stride + nbv[1] - 1, j_next * stride + nbv[1] - 1, last - 1])
        indexes[index + 3] = np.array([j_next * stride + 2 * nbv[1] - 1, j * stride + 2 * nbv[1] - 1, last])
        for i in range(nbv[1]-1):
            indexes[index + 4 + i * 4 + 0] = np.array([j_next * stride + i, j_next * stride + i + 1, j * stride + i])
            indexes[index + 4 + i * 4 + 1] = np.array([j_next * stride + i + 1, j * stride + i + 1, j * stride + i])
            indexes[index + 4 + i * 4 + 2] = np.array([j_next * stride + nbv[1] + i + 1, j_next * stride + nbv[1] + i, j * stride + nbv[1] + i])
            indexes[index + 4 + i * 4 + 3] = np.array([j_next * stride + nbv[1] + i + 1, j * stride + nbv[1] + i, j * stride + nbv[1] + i + 1])
        index += 4 * (nbv[1] - 1) + 4
    return mg.TriangularMeshGeometry(vertices, indexes)

class MeshcatVisualizer(BaseVisualizer):
    """A Pinocchio display using Meshcat"""

    FORCE_SCALE = 0.06
    FRAME_VEL_COLOR = 0x00FF00
    CAMERA_PRESETS = {
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


    def getViewerNodeName(self, geometry_object, geometry_type):
        """Return the name of the geometry object inside the viewer."""
        if geometry_type is pin.GeometryType.VISUAL:
            return self.viewerVisualGroupName + '/' + geometry_object.name
        elif geometry_type is pin.GeometryType.COLLISION:
            return self.viewerCollisionGroupName + '/' + geometry_object.name

    def initViewer(self, viewer=None, open=False, loadModel=False):
        """Start a new MeshCat server and client.
        Note: the server can also be started separately using the "meshcat-server" command in a terminal:
        this enables the server to remain active after the current script ends.
        """

        self.viewer = meshcat.Visualizer() if viewer is None else viewer

        self._node_default_cam = self.viewer["/Cameras/default"]
        self._node_background = self.viewer["/Background"]
        self._rot_cam_key = "rotated/object"
        self.static_objects = []

        self._check_meshcat_has_get_image()

        if open:
            self.viewer.open()

        if loadModel:
            self.loadViewerModel()

    def setBackgroundColor(self, preset_name: str = "gray"):
        """Set the background."""
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
        basic_three_js_transform = np.array([[1.,  0.,  0.,  0.],
                      [0.,  0., -1.,  0.],
                      [0.,  1.,  0.,  0.],
                      [0.,  0.,  0.,  1.]])
        RotatedCylinder = type("RotatedCylinder", (mg.Cylinder,), {"intrinsic_transform": lambda self: basic_three_js_transform })

        # Cones need to be rotated

        geom: hppfcl.ShapeBase = geometry_object.geometry
        if isinstance(geom, hppfcl.Capsule):
            if hasattr(mg, 'TriangularMeshGeometry'):
                obj = createCapsule(2. * geom.halfLength, geom.radius)
            else:
                obj = RotatedCylinder(2. * geom.halfLength, geom.radius)
        elif isinstance(geom, hppfcl.Cylinder):
            obj = RotatedCylinder(2. * geom.halfLength, geom.radius)
        elif isinstance(geom, hppfcl.Cone):
            RotatedCone = type("RotatedCone", (Cone,), {"intrinsic_transform": lambda self: basic_three_js_transform })
            obj = RotatedCone(2. * geom.halfLength, geom.radius)
        elif isinstance(geom, hppfcl.Box):
            obj = mg.Box(npToTuple(2. * geom.halfSide))
        elif isinstance(geom, hppfcl.Sphere):
            obj = mg.Sphere(geom.radius)
        elif isinstance(geom, hppfcl.Plane):
            To = np.eye(4)
            To[:3, 3] = geom.d * geom.n
            TranslatedPlane = type("TranslatedPlane", (mg.Plane,), {"intrinsic_transform": lambda self: To})
            obj = TranslatedPlane(-10, 10)
        elif isinstance(geom, hppfcl.Ellipsoid):
            obj = mg.Ellipsoid(geom.radii)
        elif isinstance(geom, (hppfcl.Plane,hppfcl.Halfspace)):
            plane_transform : pin.SE3 = pin.SE3.Identity()
            # plane_transform.translation[:] = geom.d # Does not work
            plane_transform.rotation = pin.Quaternion.FromTwoVectors(pin.ZAxis,geom.n).toRotationMatrix()
            TransformedPlane = type("TransformedPlane", (Plane,), {"intrinsic_transform": lambda self: plane_transform.homogeneous })
            obj = TransformedPlane(1000,1000)
        elif isinstance(geom, hppfcl.ConvexBase):
            obj = loadMesh(geom)
        else:
            msg = "Unsupported geometry type for %s (%s)" % (geometry_object.name, type(geom) )
            warnings.warn(msg, category=UserWarning, stacklevel=2)
            obj = None

        return obj

    def loadMeshFromFile(self, geometry_object):

        # Mesh path is empty if Pinocchio is built without HPP-FCL bindings
        if geometry_object.meshPath == "":
            msg = "Display of geometric primitives is supported only if pinocchio is build with HPP-FCL bindings."
            warnings.warn(msg, category=UserWarning, stacklevel=2)
            return None

        # Get file type from filename extension.
        _, file_extension = os.path.splitext(geometry_object.meshPath)
        if file_extension.lower() == ".dae":
            obj = DaeMeshGeometry(geometry_object.meshPath)
        elif file_extension.lower() == ".obj":
            obj = mg.ObjMeshGeometry.from_file(geometry_object.meshPath)
        elif file_extension.lower() == ".stl":
            obj = mg.StlMeshGeometry.from_file(geometry_object.meshPath)
        else:
            msg = "Unknown mesh file format: {}.".format(geometry_object.meshPath)
            warnings.warn(msg, category=UserWarning, stacklevel=2)
            obj = None

        return obj

    def loadViewerGeometryObject(self, geometry_object, geometry_type, color=None):
        """Load a single geometry object"""
        viewer_name = self.getViewerNodeName(geometry_object, geometry_type)

        is_mesh = False
        try:
            if WITH_HPP_FCL_BINDINGS and isinstance(geometry_object.geometry, hppfcl.ShapeBase):
                obj = self.loadPrimitive(geometry_object)
            elif hasMeshFileInfo(geometry_object):
                obj = self.loadMeshFromFile(geometry_object)
                is_mesh = True
            elif WITH_HPP_FCL_BINDINGS and isinstance(geometry_object.geometry, (hppfcl.BVHModelBase,hppfcl.HeightFieldOBBRSS,hppfcl.HeightFieldAABB)):
                obj = loadMesh(geometry_object.geometry)
            else:
                msg = "The geometry object named " + geometry_object.name + " is not supported by Pinocchio/MeshCat for vizualization."
                warnings.warn(msg, category=UserWarning, stacklevel=2)
                return
            if obj is None:
                return
        except Exception as e:
            msg = "Error while loading geometry object: %s\nError message:\n%s" % (geometry_object.name, e)
            warnings.warn(msg, category=UserWarning, stacklevel=2)
            return

        meshcat_node = self.viewer[viewer_name]
        if isinstance(obj, mg.Object):
            meshcat_node.set_object(obj)
        elif isinstance(obj, (mg.Geometry, mg.ReferenceSceneElement)):
            material = mg.MeshPhongMaterial()
            # Set material color from URDF, converting for triplet of doubles to a single int.
            if color is None:
                meshColor = geometry_object.meshColor
            else:
                meshColor = color
            material.color = int(meshColor[0] * 255) * 256**2 + int(meshColor[1] * 255) * 256 + int(meshColor[2] * 255)
            # Add transparency, if needed.
            if float(meshColor[3]) != 1.0:
                material.transparent = True
                material.opacity = float(meshColor[3])

            if isinstance(obj, DaeMeshGeometry):
                obj.path = meshcat_node.path
                if geometry_object.overrideMaterial:
                    obj.material = material
                meshcat_node.window.send(obj)
            else:
                meshcat_node.set_object(obj, material)

        if is_mesh: # Apply the scaling
            scale = list(np.asarray(geometry_object.meshScale).flatten())
            meshcat_node.set_property("scale",scale)

    def loadViewerModel(self, rootNodeName="pinocchio", color = None):
        """Load the robot in a MeshCat viewer.
        Parameters:
            rootNodeName: name to give to the robot in the viewer
            color: optional, color to give to the robot. This overwrites the color present in the urdf.
                   Format is a list of four RGBA floats (between 0 and 1)
        """

        # Set viewer to use to gepetto-gui.
        self.viewerRootNodeName = rootNodeName

        # Collisions
        self.viewerCollisionGroupName = self.viewerRootNodeName + "/" + "collisions"

        for collision in self.collision_model.geometryObjects:
            self.loadViewerGeometryObject(collision,pin.GeometryType.COLLISION,color)
        self.displayCollisions(False)

        # Visuals
        self.viewerVisualGroupName = self.viewerRootNodeName + "/" + "visuals"

        for visual in self.visual_model.geometryObjects:
            self.loadViewerGeometryObject(visual,pin.GeometryType.VISUAL,color)
        self.displayVisuals(True)

    def reload(self, new_geometry_object, geometry_type = None):
        """ Reload a geometry_object given by its name and its type"""
        if geometry_type == pin.GeometryType.VISUAL:
            geom_model = self.visual_model
        else:
            geom_model = self.collision_model
            geometry_type = pin.GeometryType.COLLISION

        geom_id = geom_model.getGeometryId(new_geometry_object.name)
        geom_model.geometryObjects[geom_id] = new_geometry_object

        self.delete(new_geometry_object, geometry_type)
        visual = geom_model.geometryObjects[geom_id]
        self.loadViewerGeometryObject(visual,geometry_type)

    def clean(self):
        self.viewer.delete()

    def delete(self, geometry_object, geometry_type):
        viewer_name = self.getViewerNodeName(geometry_object, geometry_type)
        self.viewer[viewer_name].delete()

    def display(self, q = None):
        """Display the robot at configuration q in the viewer by placing all the bodies."""
        if q is not None:
            pin.forwardKinematics(self.model,self.data,q)

        if self.display_collisions:
            self.updatePlacements(pin.GeometryType.COLLISION)

        if self.display_visuals:
            self.updatePlacements(pin.GeometryType.VISUAL)

    def updatePlacements(self, geometry_type):
        if geometry_type == pin.GeometryType.VISUAL:
            geom_model = self.visual_model
            geom_data = self.visual_data
        else:
            geom_model = self.collision_model
            geom_data = self.collision_data

        pin.updateGeometryPlacements(self.model, self.data, geom_model, geom_data)
        for visual in geom_model.geometryObjects:
            visual_name = self.getViewerNodeName(visual,geometry_type)
            # Get mesh pose.
            M = geom_data.oMg[geom_model.getGeometryId(visual.name)]
            # Manage scaling: force scaling even if this should be normally handled by MeshCat (but there is a bug here)
            if hasMeshFileInfo(visual):
                scale = np.asarray(visual.meshScale).flatten()
                S = np.diag(np.concatenate((scale,[1.0])))
                T = np.array(M.homogeneous).dot(S)
            else:
                geom = visual.geometry
                if isinstance(geom,(hppfcl.Plane, hppfcl.Halfspace)):
                    T = M
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
            warnings.warn("meshcat.Visualizer does not have the get_image() method."
                          " You need meshcat >= 0.2.0 to get this feature.")

    def captureImage(self, w=None, h=None):
        """Capture an image from the Meshcat viewer and return an RGB array."""
        img = self.viewer.get_image(w, h)
        img_arr = np.asarray(img)
        return img_arr

    def displayCollisions(self,visibility):
        """Set whether to display collision objects or not."""
        if self.collision_model is None:
            self.display_collisions = False
        else:
            self.display_collisions = visibility
        self.viewer[self.viewerCollisionGroupName].set_property("visible", visibility)

        if visibility:
            self.updatePlacements(pin.GeometryType.COLLISION)

    def displayVisuals(self,visibility):
        """Set whether to display visual objects or not."""
        if self.visual_model is None:
            self.display_visuals = False
        else:
            self.display_visuals = visibility
        self.viewer[self.viewerVisualGroupName].set_property("visible", visibility)

        if visibility:
            self.updatePlacements(pin.GeometryType.VISUAL)

    def drawFrameVelocities(self, frame_id: int, v_scale=0.2, color=FRAME_VEL_COLOR):
        pin.updateFramePlacement(self.model, self.data, frame_id)
        vFr = pin.getFrameVelocity(
            self.model, self.data, frame_id, pin.LOCAL_WORLD_ALIGNED
        )
        line_group_name = f"ee_vel/{frame_id}"
        self._draw_vectors_from_frame([v_scale * vFr.linear], [frame_id], [line_group_name], [color])
 
    def _draw_vectors_from_frame(self, vecs: List[np.ndarray], frame_ids: List[int], vec_names: List[str], colors: List[int]):
        """Draw vectors extending from given frames."""
        import meshcat.geometry as mg
        assert len(vecs) == len(frame_ids), "Different number of vectors and frame_ids"
        assert len(vecs) == len(vec_names), "Different number of vectors and names"
        for i, (fid, v) in enumerate(zip(frame_ids, vecs)):
            frame_pos = self.data.oMf[fid].translation
            vertices = np.array([frame_pos, frame_pos + v]).astype(np.float32).T
            name = vec_names[i]
            geometry = mg.PointsGeometry(position=vertices)
            geom_object = mg.LineSegments(geometry, mg.LineBasicMaterial(color=colors[i]))
            prefix = self.viewerVisualGroupName + '/lines/' + name
            self.viewer[prefix].set_object(geom_object)
            

__all__ = ['MeshcatVisualizer']
