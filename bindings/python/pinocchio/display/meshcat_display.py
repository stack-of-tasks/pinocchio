from .. import libpinocchio_pywrap as pin
from ..shortcuts import buildModelsFromUrdf, createDatas

from . import AbstractDisplay

import os
import numpy as np

class MeshcatDisplay(AbstractDisplay):
    """A Pinocchio display using Meshcat"""

    def getViewerNodeName(self, geometry_object, geometry_type):
        """Return the name of the geometry object inside the viewer"""
        pass

    def initDisplay(self, meshcat_visualizer=None, loadModel=False):
        """Start a new MeshCat server and client.
        Note: the server can also be started separately using the "meshcat-server" command in a terminal:
        this enables the server to remain active after the current script ends.
        """

        import meshcat

        self.viewer = meshcat.Visualizer() if meshcat_visualizer is None else meshcat_visualizer

        if loadModel:
            self.loadDisplayModel()

    def loadDisplayModel(self, rootNodeName="pinocchio", color = None):
        """Load the robot in a MeshCat viewer.
        Parameters:
            rootNodeName: name to give to the robot in the viewer
            color: optional, color to give to the robot. This overwrites the color present in the urdf.
                   Format is a list of four RGBA floats (between 0 and 1)
        """
        import meshcat.geometry
        # Set viewer to use to gepetto-gui.
        self.viewerRootNodeName = rootNodeName

        # Load robot meshes in MeshCat
        for visual in self.visual_model.geometryObjects:
            viewer_name = self.viewerRootNodeName + visual.name
            if visual.meshPath == "":
                raise IOError("Visual mesh file not found for link {}.".format(visual.name))
            # Get file type from filename extension.
            _, file_extension = os.path.splitext(visual.meshPath)
            if file_extension.lower() == ".dae":
                obj = meshcat.geometry.DaeMeshGeometry.from_file(visual.meshPath)
            elif file_extension.lower() == ".obj":
                obj = meshcat.geometry.ObjMeshGeometry.from_file(visual.meshPath)
            elif file_extension.lower() == ".stl":
                obj = meshcat.geometry.StlMeshGeometry.from_file(visual.meshPath)
            else:
                raise ImportError("Unknown mesh file format: {}.".format(visual.meshPath))
            material = meshcat.geometry.MeshPhongMaterial()
            # Set material color from URDF, converting for triplet of doubles to a single int.
            if color is None:
                meshColor = visual.meshColor
            else:
                meshColor = color
            material.color = int(meshColor[0] * 255) * 256**2 + int(meshColor[1] * 255) * 256 + int(meshColor[2] * 255)
            # Add transparency, if needed.
            if float(meshColor[3]) != 1.0:
                material.transparent = True
                material.opacity = float(meshColor[3])
            self.viewer[viewer_name].set_object(obj, material)

    def display(self, q):
        """Display the robot at configuration q in the viewer by placing all the bodies."""
        pin.forwardKinematics(self.model,self.data,q)
        pin.updateGeometryPlacements(self.model, self.data, self.visual_model, self.visual_data)
        for visual in self.visual_model.geometryObjects:
            # Get mesh pose.
            M = self.visual_data.oMg[self.visual_model.getGeometryId(visual.name)]
            # Manage scaling
            S = np.diag(np.concatenate((visual.meshScale,np.array([[1.0]]))).flat)
            T = np.array(M.homogeneous).dot(S)
            # Update viewer configuration.
            self.viewer[self.viewerRootNodeName + visual.name].set_transform(T)

    def displayCollisions(self,visibility):
        """Set whether to diplay collision objects or not"""
        pass

    def displayVisuals(self,visibility):
        """Set whether to diplay visual objects or not"""
        pass

__all__ = ['MeshcatDisplay']
