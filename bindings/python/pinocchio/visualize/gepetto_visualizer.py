from .. import pinocchio_pywrap as pin
from ..shortcuts import buildModelsFromUrdf, createDatas
from ..utils import npToTuple

from . import BaseVisualizer

import warnings

try:
    import hppfcl
    WITH_HPP_FCL_BINDINGS = True
except:
    WITH_HPP_FCL_BINDINGS = False

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

        try:
            import gepetto.corbaserver
        except ImportError:
            import warnings
            msg = ("Error while importing the viewer client.\n"
                   "Check whether gepetto-viewer is properly installed"
                  )
            warnings.warn(msg, category=UserWarning, stacklevel=2)

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

    def loadPrimitive(self, meshName, geometry_object):

        gui = self.viewer.gui

        meshColor = geometry_object.meshColor

        geom = geometry_object.geometry
        if isinstance(geom, hppfcl.Capsule):
            return gui.addCapsule(meshName, geom.radius, 2. * geom.halfLength, npToTuple(meshColor))
        elif isinstance(geom, hppfcl.Cylinder):
            return gui.addCylinder(meshName, geom.radius, 2. * geom.halfLength, npToTuple(meshColor))
        elif isinstance(geom, hppfcl.Box):
            w, h, d = npToTuple(2. * geom.halfSide)
            return gui.addBox(meshName, w, h, d, npToTuple(meshColor))
        elif isinstance(geom, hppfcl.Sphere):
            return gui.addSphere(meshName, geom.radius, npToTuple(meshColor))
        elif isinstance(geom, hppfcl.Cone):
            return gui.addCone(meshName, geom.radius, 2. * geom.halfLength, npToTuple(meshColor))
        elif isinstance(geom, hppfcl.Convex):
            pts = [ npToTuple(geom.points(geom.polygons(f)[i])) for f in range(geom.num_polygons) for i in range(3) ]
            gui.addCurve(meshName, pts, npToTuple(meshColor))
            gui.setCurveMode(meshName, "TRIANGLES")
            gui.setLightingMode(meshName, "ON")
            gui.setBoolProperty(meshName, "BackfaceDrawing", True)
            return True
        elif isinstance(geom, hppfcl.ConvexBase):
            pts = [ npToTuple(geom.points(i)) for i in range(geom.num_points) ]
            gui.addCurve(meshName, pts, npToTuple(meshColor))
            gui.setCurveMode(meshName, "POINTS")
            gui.setLightingMode(meshName, "OFF")
            return True
        else:
            msg = "Unsupported geometry type for %s (%s)" % (geometry_object.name, type(geom) )
            warnings.warn(msg, category=UserWarning, stacklevel=2)
            return False

    def loadViewerGeometryObject(self, geometry_object, geometry_type):
        """Load a single geometry object"""

        gui = self.viewer.gui

        meshName = self.getViewerNodeName(geometry_object,geometry_type)
        meshPath = geometry_object.meshPath
        meshTexturePath = geometry_object.meshTexturePath
        meshScale = geometry_object.meshScale
        meshColor = geometry_object.meshColor

        try:
            if WITH_HPP_FCL_BINDINGS and isinstance(geometry_object.geometry, hppfcl.ShapeBase):
                success = self.loadPrimitive(meshName, geometry_object)
            else:
                if meshName == "":
                    msg = "Display of geometric primitives is supported only if pinocchio is build with HPP-FCL bindings."
                    warnings.warn(msg, category=UserWarning, stacklevel=2)
                    return
                success = gui.addMesh(meshName, meshPath)
            if not success:
                return
        except Exception as e:
            msg = "Error while loading geometry object: %s\nError message:\n%s" % (geometry_object.name, e)
            warnings.warn(msg, category=UserWarning, stacklevel=2)
            return

        gui.setScale(meshName, npToTuple(meshScale))
        if geometry_object.overrideMaterial:
            gui.setColor(meshName, npToTuple(meshColor))
            if meshTexturePath != '':
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
        if self.collision_model is not None:
            for collision in self.collision_model.geometryObjects:
                self.loadViewerGeometryObject(collision,pin.GeometryType.COLLISION)
        # Display collision if we have them and there is no visual
        self.displayCollisions(self.collision_model is not None and self.visual_model is None)

        if self.visual_model is not None:
            for visual in self.visual_model.geometryObjects:
                self.loadViewerGeometryObject(visual,pin.GeometryType.VISUAL)
        self.displayVisuals(self.visual_model is not None)

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
                    [ pin.SE3ToXYZQUATtuple(self.collision_data.oMg[self.collision_model.getGeometryId(collision.name)]) for collision in self.collision_model.geometryObjects ]
                    )

        if self.display_visuals:
            pin.updateGeometryPlacements(self.model, self.data, self.visual_model, self.visual_data)
            gui.applyConfigurations (
                    [ self.getViewerNodeName(visual,pin.GeometryType.VISUAL) for visual in self.visual_model.geometryObjects ],
                    [ pin.SE3ToXYZQUATtuple(self.visual_data.oMg[self.visual_model.getGeometryId(visual.name)]) for visual in self.visual_model.geometryObjects ]
                    )

        gui.refresh()

    def displayCollisions(self,visibility):
        """Set whether to display collision objects or not"""
        gui = self.viewer.gui
        self.display_collisions = visibility
        if self.collision_model is None: return

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
        if self.visual_model is None: return

        if visibility:
            visibility_mode = "ON"
        else:
            visibility_mode = "OFF"

        for visual in self.visual_model.geometryObjects:
            nodeName = self.getViewerNodeName(visual,pin.GeometryType.VISUAL)
            gui.setVisibility(nodeName,visibility_mode)

__all__ = ['GepettoVisualizer']
