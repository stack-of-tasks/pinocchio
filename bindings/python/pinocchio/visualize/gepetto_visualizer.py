from .. import libpinocchio_pywrap as pin
from ..shortcuts import buildModelsFromUrdf, createDatas

from . import BaseVisualizer

class GepettoVisualizer(BaseVisualizer):
    """A Pinocchio display using Gepetto Viewer"""

    def getViewerNodeName(self, geometry_object, geometry_type):
        """Return the name of the geometry object inside the viewer"""
        if geometry_type is pin.GeometryType.VISUAL:
            return self.viewerVisualGroupName + '/' + geometry_object.name
        elif geometry_type is pin.GeometryType.COLLISION:
            return self.viewerCollisionGroupName + '/' + geometry_object.name

    def initViewer(self, viewer=None, windowName="python-pinocchio", sceneName="world", loadModel=False):
        """Init GepettoViewer by loading the gui and creating a window."""

        import gepetto.corbaserver
        try:
            self.viewer = gepetto.corbaserver.Client() if viewer is None else viewer
            gui = self.viewer.gui

            # Create window
            window_l = gui.getWindowList()
            if not windowName in window_l:
                self.windowID = self.viewer.gui.createWindow(windowName)
            else:
                self.windowID = self.viewer.gui.getWindowID(windowName)

            # Create scene if needed
            scene_l = gui.getSceneList()
            if sceneName not in scene_l:
                gui.createScene(sceneName)
            self.sceneName = sceneName
            gui.addSceneToWindow(sceneName, self.windowID)

            if loadModel:
                self.loadViewerModel()
        except:
            import warnings
            msg = ("Error while starting the viewer client.\n"
                   "Check whether gepetto-viewer is properly started"
                  )
            warnings.warn(msg, category=UserWarning, stacklevel=2)

    def loadViewerGeometryObject(self, geometry_object, geometry_type):
        """Load a single geometry object"""

        from ..rpy import npToTuple
        gui = self.viewer.gui

        meshName = self.getViewerNodeName(geometry_object,geometry_type)
        meshPath = geometry_object.meshPath
        meshTexturePath = geometry_object.meshTexturePath
        meshScale = geometry_object.meshScale
        meshColor = geometry_object.meshColor
        if gui.addMesh(meshName, meshPath):
            gui.setScale(meshName, npToTuple(meshScale))
            if geometry_object.overrideMaterial:
                gui.setColor(meshName, npToTuple(meshColor))
                if meshTexturePath is not '':
                    gui.setTexture(meshName, meshTexturePath)

    def loadViewerModel(self, rootNodeName="pinocchio"):
        """Create the scene displaying the robot meshes in gepetto-viewer"""

        # Start a new "scene" in this window, named "world", with just a floor.
        gui = self.viewer.gui
        self.viewerRootNodeName = self.sceneName + "/" + rootNodeName

        if not gui.nodeExists(self.viewerRootNodeName):
            gui.createGroup(self.viewerRootNodeName)

        self.viewerCollisionGroupName = self.viewerRootNodeName + "/" + "collisions"
        if not gui.nodeExists(self.viewerCollisionGroupName):
            gui.createGroup(self.viewerCollisionGroupName)

        self.viewerVisualGroupName = self.viewerRootNodeName + "/" + "visuals"
        if not gui.nodeExists(self.viewerVisualGroupName):
            gui.createGroup(self.viewerVisualGroupName)

        # iterate over visuals and create the meshes in the viewer
        for collision in self.collision_model.geometryObjects:
            self.loadViewerGeometryObject(collision,pin.GeometryType.COLLISION)
        self.displayCollisions(False)

        for visual in self.visual_model.geometryObjects:
            self.loadViewerGeometryObject(visual,pin.GeometryType.VISUAL)
        self.displayVisuals(True)

        # Finally, refresh the layout to obtain your first rendering.
        gui.refresh()

    def display(self, q):
        """Display the robot at configuration q in the viewer by placing all the bodies."""
        if 'viewer' not in self.__dict__:
            return

        gui = self.viewer.gui
        # Update the robot kinematics and geometry.
        pin.forwardKinematics(self.model,self.data,q)

        if self.display_collisions:
            pin.updateGeometryPlacements(self.model, self.data, self.collision_model, self.collision_data)
            gui.applyConfigurations (
                    [ self.getViewerNodeName(collision,pin.GeometryType.COLLISION) for collision in self.collision_model.geometryObjects ],
                    [ pin.se3ToXYZQUATtuple(self.collision_data.oMg[self.collision_model.getGeometryId(collision.name)]) for collision in self.collision_model.geometryObjects ]
                    )

        if self.display_visuals:
            pin.updateGeometryPlacements(self.model, self.data, self.visual_model, self.visual_data)
            gui.applyConfigurations (
                    [ self.getViewerNodeName(visual,pin.GeometryType.VISUAL) for visual in self.visual_model.geometryObjects ],
                    [ pin.se3ToXYZQUATtuple(self.visual_data.oMg[self.visual_model.getGeometryId(visual.name)]) for visual in self.visual_model.geometryObjects ]
                    )

        gui.refresh()

    def displayCollisions(self,visibility):
        """Set whether to display collision objects or not"""
        gui = self.viewer.gui
        self.display_collisions = visibility

        if visibility:
            visibility_mode = "ON"
        else:
            visibility_mode = "OFF"

        for collision in self.collision_model.geometryObjects:
            nodeName = self.getViewerNodeName(collision,pin.GeometryType.COLLISION)
            gui.setVisibility(nodeName,visibility_mode)

    def displayVisuals(self,visibility):
        """Set whether to display visual objects or not"""
        gui = self.viewer.gui
        self.display_visuals = visibility

        if visibility:
            visibility_mode = "ON"
        else:
            visibility_mode = "OFF"

        for visual in self.visual_model.geometryObjects:
            nodeName = self.getViewerNodeName(visual,pin.GeometryType.VISUAL)
            gui.setVisibility(nodeName,visibility_mode)

__all__ = ['GepettoVisualizer']
