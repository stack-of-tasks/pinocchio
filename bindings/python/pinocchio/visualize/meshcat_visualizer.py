from .. import pinocchio_pywrap as pin
from ..shortcuts import buildModelsFromUrdf, createDatas
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
            else:
                obj = self.loadMesh(geometry_object)
            if obj is None:
                return
        except Exception as e:
            msg = "Error while loading geometry object: %s\nError message:\n%s" % (geometry_object.name, e)
            warnings.warn(msg, category=UserWarning, stacklevel=2)
            return
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

    def display(self, q):
        """Display the robot at configuration q in the viewer by placing all the bodies."""
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
        import warnings
        warnings.warn("Plotting collision meshes is not available for MeshcatVisualizer", category=UserWarning, stacklevel=2)
        pass

    def displayVisuals(self,visibility):
        """Set whether to display visual objects or not
        WARNING: Visual meshes are always plotted for MeshcatVisualizer"""
        # TODO
        import warnings
        warnings.warn("Visual meshes are always plotted for MeshcatVisualizer", category=UserWarning, stacklevel=2)
        pass

__all__ = ['MeshcatVisualizer']
