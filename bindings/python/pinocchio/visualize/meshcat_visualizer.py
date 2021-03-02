from .. import pinocchio_pywrap as pin
from ..utils import npToTuple

from . import BaseVisualizer

import os
import warnings
import numpy as np

try:
    import hppfcl
    WITH_HPP_FCL_BINDINGS = True
except:
    WITH_HPP_FCL_BINDINGS = False


def loadBVH(bvh):
    import meshcat.geometry as mg

    num_vertices = bvh.num_vertices
    num_tris = bvh.num_tris
    vertices = np.empty((num_vertices,3))
    faces = np.empty((num_tris,3),dtype=int)

    for k in range(num_tris):
        tri = bvh.tri_indices(k)
        faces[k] = [tri[i] for i in range(3)]

    for k in range(num_vertices):
        vert = bvh.vertices(k)
        vertices[k] = vert

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
    import meshcat.geometry
    return meshcat.geometry.TriangularMeshGeometry(vertices, indexes)

class MeshcatVisualizer(BaseVisualizer):
    """A Pinocchio display using Meshcat"""

    def getViewerNodeName(self, geometry_object, geometry_type):
        """Return the name of the geometry object inside the viewer."""
        if geometry_type is pin.GeometryType.VISUAL:
            return self.viewerVisualGroupName + '/' + geometry_object.name
        elif geometry_type is pin.GeometryType.COLLISION:
            return None # TODO: collision meshes

    def initViewer(self, viewer=None, open=False, loadModel=False):
        """Start a new MeshCat server and client.
        Note: the server can also be started separately using the "meshcat-server" command in a terminal:
        this enables the server to remain active after the current script ends.
        """

        import meshcat

        self.viewer = meshcat.Visualizer() if viewer is None else viewer

        if open:
            self.viewer.open()

        if loadModel:
            self.loadViewerModel()

    def loadPrimitive(self, geometry_object):

        import meshcat.geometry

        # Cylinders need to be rotated
        R = np.array([[1.,  0.,  0.,  0.],
                      [0.,  0., -1.,  0.],
                      [0.,  1.,  0.,  0.],
                      [0.,  0.,  0.,  1.]])
        RotatedCylinder = type("RotatedCylinder", (meshcat.geometry.Cylinder,), {"intrinsic_transform": lambda self: R })

        geom = geometry_object.geometry
        if isinstance(geom, hppfcl.Capsule):
            if hasattr(meshcat.geometry, 'TriangularMeshGeometry'):
                obj = createCapsule(2. * geom.halfLength, geom.radius)
            else:
                obj = RotatedCylinder(2. * geom.halfLength, geom.radius)
        elif isinstance(geom, hppfcl.Cylinder):
            obj = RotatedCylinder(2. * geom.halfLength, geom.radius)
        elif isinstance(geom, hppfcl.Box):
            obj = meshcat.geometry.Box(npToTuple(2. * geom.halfSide))
        elif isinstance(geom, hppfcl.Sphere):
            obj = meshcat.geometry.Sphere(geom.radius)
        else:
            msg = "Unsupported geometry type for %s (%s)" % (geometry_object.name, type(geom) )
            warnings.warn(msg, category=UserWarning, stacklevel=2)
            obj = None

        return obj

    def loadMesh(self, geometry_object):

        import meshcat.geometry

        # Mesh path is empty if Pinocchio is built without HPP-FCL bindings
        if geometry_object.meshPath == "":
            msg = "Display of geometric primitives is supported only if pinocchio is build with HPP-FCL bindings."
            warnings.warn(msg, category=UserWarning, stacklevel=2)
            return None

        # Get file type from filename extension.
        _, file_extension = os.path.splitext(geometry_object.meshPath)
        if file_extension.lower() == ".dae":
            obj = meshcat.geometry.DaeMeshGeometry.from_file(geometry_object.meshPath)
        elif file_extension.lower() == ".obj":
            obj = meshcat.geometry.ObjMeshGeometry.from_file(geometry_object.meshPath)
        elif file_extension.lower() == ".stl":
            obj = meshcat.geometry.StlMeshGeometry.from_file(geometry_object.meshPath)
        else:
            msg = "Unknown mesh file format: {}.".format(geometry_object.meshPath)
            warnings.warn(msg, category=UserWarning, stacklevel=2)
            obj = None

        return obj

    def loadViewerGeometryObject(self, geometry_object, geometry_type, color=None):
        """Load a single geometry object"""
        import meshcat.geometry

        viewer_name = self.getViewerNodeName(geometry_object, geometry_type)

        try:
            if WITH_HPP_FCL_BINDINGS and isinstance(geometry_object.geometry, hppfcl.ShapeBase):
                obj = self.loadPrimitive(geometry_object)
            elif WITH_HPP_FCL_BINDINGS and isinstance(geometry_object.geometry, hppfcl.BVHModelBase):
                obj = loadBVH(geometry_object.geometry)
            else:
                obj = self.loadMesh(geometry_object)
            if obj is None:
                return
        except Exception as e:
            msg = "Error while loading geometry object: %s\nError message:\n%s" % (geometry_object.name, e)
            warnings.warn(msg, category=UserWarning, stacklevel=2)
            return

        if isinstance(obj, meshcat.geometry.Object):
            self.viewer[viewer_name].set_object(obj)
        elif isinstance(obj, meshcat.geometry.Geometry):
            material = meshcat.geometry.MeshPhongMaterial()
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
            self.viewer[viewer_name].set_object(obj, material)

    def loadViewerModel(self, rootNodeName="pinocchio", color = None):
        """Load the robot in a MeshCat viewer.
        Parameters:
            rootNodeName: name to give to the robot in the viewer
            color: optional, color to give to the robot. This overwrites the color present in the urdf.
                   Format is a list of four RGBA floats (between 0 and 1)
        """

        # Set viewer to use to gepetto-gui.
        self.viewerRootNodeName = rootNodeName

        # Load robot meshes in MeshCat

        # Collisions
        # self.viewerCollisionGroupName = self.viewerRootNodeName + "/" + "collisions"
        self.viewerCollisionGroupName = None # TODO: collision meshes

        # Visuals
        self.viewerVisualGroupName = self.viewerRootNodeName + "/" + "visuals"

        for visual in self.visual_model.geometryObjects:
            self.loadViewerGeometryObject(visual,pin.GeometryType.VISUAL,color)

    def reload(self, new_geometry_object, geometry_type = None):
        """ Reload a geometry_object given by its name and its type"""
        geom_id = self.visual_model.getGeometryId(new_geometry_object.name)
        self.visual_model.geometryObjects[geom_id] = new_geometry_object

        visual = self.visual_model.geometryObjects[geom_id]
        self.delete(new_geometry_object, pin.GeometryType.VISUAL)
        self.loadViewerGeometryObject(visual,pin.GeometryType.VISUAL,color = None)

    def clean(self):
        self.viewer.delete()

    def delete(self, geometry_object, geometry_type):
        viewer_name = self.getViewerNodeName(geometry_object, geometry_type)
        self.viewer[viewer_name].delete()

    def display(self, q = None):
        """Display the robot at configuration q in the viewer by placing all the bodies."""
        if q is not None:
            pin.forwardKinematics(self.model,self.data,q)

        pin.updateGeometryPlacements(self.model, self.data, self.visual_model, self.visual_data)
        for visual in self.visual_model.geometryObjects:
            # Get mesh pose.
            M = self.visual_data.oMg[self.visual_model.getGeometryId(visual.name)]
            # Manage scaling
            scale = np.asarray(visual.meshScale).flatten()
            S = np.diag(np.concatenate((scale,[1.0])))
            T = np.array(M.homogeneous).dot(S)
            # Update viewer configuration.
            self.viewer[self.getViewerNodeName(visual,pin.GeometryType.VISUAL)].set_transform(T)

    def displayCollisions(self,visibility):
        """Set whether to display collision objects or not.
        WARNING: Plotting collision meshes is not yet available for MeshcatVisualizer."""
        # TODO
        warnings.warn("Plotting collision meshes is not available for MeshcatVisualizer", category=UserWarning, stacklevel=2)

    def displayVisuals(self,visibility):
        """Set whether to display visual objects or not
        WARNING: Visual meshes are always plotted for MeshcatVisualizer"""
        # TODO
        warnings.warn("Visual meshes are always plotted for MeshcatVisualizer", category=UserWarning, stacklevel=2)

__all__ = ['MeshcatVisualizer']
