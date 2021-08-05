from .. import pinocchio_pywrap as pin
from ..utils import npToTuple

from . import BaseVisualizer

import warnings

try:
    import hppfcl
    WITH_HPP_FCL_BINDINGS = True
except:
    WITH_HPP_FCL_BINDINGS = False

class RVizVisualizer(BaseVisualizer):
    """A Pinocchio display using RViz"""
    class Viewer:
        app = None
        viz = None
        viz_manager = None

    def initViewer(self, viewer=None, windowName="python-pinocchio", loadModel=False):
        """Init RVizViewer by starting a ros node and creating an RViz window."""
        from rospy import init_node, WARN
        from rosgraph import is_master_online
        from rviz import bindings as rviz
        from python_qt_binding.QtWidgets import QApplication

        if not is_master_online(): # Checks the master uri
            # ROS Master is offline
            warnings.warn("Error while importing the viz client.\n"
                          "Check whether ROS master (roscore) is properly started",
                          category=UserWarning, stacklevel=2)
            return None

        init_node('pinocchio_viewer', anonymous=True, log_level=WARN)

        if viewer == None:
            self.viewer = RVizVisualizer.Viewer()
            self.viewer.app = QApplication([])
            self.viewer.viz= rviz.VisualizationFrame()
            self.viewer.viz.setSplashPath( "" )
            self.viewer.viz.initialize()
            self.viewer.viz.setWindowTitle(windowName+"[*]")
            self.viewer.viz_manager = self.viewer.viz.getManager()
            self.viewer.viz.show()
        else:
            self.viewer = viewer

        if loadModel:
            self.loadViewerModel()

        return self.viewer

    def loadViewerModel(self, rootNodeName="pinocchio"):
        """Create the displays in RViz and create publishers for the MarkerArray"""
        from rospy import Publisher
        from visualization_msgs.msg import MarkerArray

        # Visuals
        self.visuals_publisher = Publisher(rootNodeName+ "_visuals", MarkerArray, queue_size=1,  latch=True)
        self.visual_Display = self.viewer.viz_manager.createDisplay("rviz/MarkerArray", rootNodeName + "_visuals", True)
        self.visual_Display.subProp("Marker Topic").setValue(rootNodeName + "_visuals")

        # Collisions
        self.collisions_publisher = Publisher(rootNodeName + "_collisions", MarkerArray, queue_size=1,  latch=True)
        self.collision_Display = self.viewer.viz_manager.createDisplay("rviz/MarkerArray", rootNodeName + "/" + rootNodeName + "_collisions", True)
        self.collision_Display.subProp("Marker Topic").setValue(rootNodeName + "_collisions")

        # Group
        root_group = self.viewer.viz_manager.getRootDisplayGroup()
        self.group_Display = self.viewer.viz_manager.createDisplay("rviz/Group", rootNodeName, True)
        self.group_Display.addChild(root_group.takeChild(self.visual_Display))    # Remove display from root group and add it to robot group
        self.group_Display.addChild(root_group.takeChild(self.collision_Display)) # Remove display from root group and add it to robot group

        self.seq = 0
        self.display()

    def clean(self):
        self.viewer.app.quit()
        self.viewer.app = None
        self.viewer.viz = None
        self.viewer.viz_manager = None

    def display(self, q = None):
        """Display the robot at configuration q in the viz by placing all the bodies."""
        # Update the robot kinematics and geometry.
        if q is not None:
            pin.forwardKinematics(self.model,self.data,q)

        pin.updateGeometryPlacements(self.model, self.data, self.collision_model, self.collision_data)
        self.plot(self.collisions_publisher, self.collision_model, self.collision_data)

        pin.updateGeometryPlacements(self.model, self.data, self.visual_model, self.visual_data)
        self.plot(self.visuals_publisher, self.visual_model, self.visual_data)

    def plot(self, publisher, model, data):
        from rospy import get_rostime
        from std_msgs.msg import Header, ColorRGBA
        from geometry_msgs.msg import Pose, Point, Quaternion
        from visualization_msgs.msg import MarkerArray, Marker

        self.seq +=1
        header = Header(frame_id='map', seq=self.seq, stamp=get_rostime()) # Common header for every marker

        marker_array = MarkerArray()
        for obj in model.geometryObjects:
            obj_id = model.getGeometryId(obj.name)
            obj_pose = pin.SE3ToXYZQUATtuple(data.oMg[obj_id])
            marker = Marker()
            marker.id = obj_id
            marker.header = header
            marker.action = Marker.ADD # Add/modify
            marker.pose = Pose(position=Point(*obj_pose[:3]), orientation=Quaternion(*obj_pose[3:]))
            marker.color = ColorRGBA(*obj.meshColor)

            if obj.meshTexturePath != "":
                warnings.warn("Textures are not supported in RVizVisualizer (for " + obj.name + ")", category=UserWarning, stacklevel=2)

            geom = obj.geometry
            if WITH_HPP_FCL_BINDINGS and isinstance(geom, hppfcl.ShapeBase):
                # append a primitive geometry
                if isinstance(geom, hppfcl.Cylinder):
                    d, l = 2*geom.radius, 2*geom.halfLength
                    marker.type = Marker.CYLINDER
                    marker.scale = Point(d,d,l)
                elif isinstance(geom, hppfcl.Box):
                    size = npToTuple(2.*geom.halfSide)
                    marker.type = Marker.CUBE
                    marker.scale = Point(*size)
                elif isinstance(geom, hppfcl.Sphere):
                    d = 2*geom.radius
                    marker.type = Marker.SPHERE
                    marker.scale = Point(d, d, d)
                else:
                    msg = "Unsupported geometry type for %s (%s)" % (obj.name, type(geom))
                    warnings.warn(msg, category=UserWarning, stacklevel=2)
                    continue
            else:
                # append a mesh
                marker.type = Marker.MESH_RESOURCE # Custom mesh
                marker.scale = Point(*npToTuple(obj.meshScale))
                marker.mesh_resource = 'file://' + obj.meshPath

            marker_array.markers.append(marker)

        publisher.publish(marker_array)

    def displayCollisions(self,visibility):
        """Set whether to display collision objects or not"""
        self.collision_Display.setEnabled(visibility)

    def displayVisuals(self,visibility):
        """Set whether to display visual objects or not"""
        self.visual_Display.setEnabled(visibility)

__all__ = ['RVizVisualizer']
