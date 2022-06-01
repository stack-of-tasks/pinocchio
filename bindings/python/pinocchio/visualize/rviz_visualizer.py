from .. import pinocchio_pywrap as pin
from ..utils import npToTuple

from . import BaseVisualizer

import warnings

try:
    import hppfcl
    WITH_HPP_FCL_BINDINGS = True
except ImportError:
    WITH_HPP_FCL_BINDINGS = False

def create_capsule_markers(marker_ref, oMg, d, l):
    """ Make capsule using two sphere and one cylinder"""
    from copy import deepcopy
    from visualization_msgs.msg import Marker
    from geometry_msgs.msg import Point
    displacment = pin.SE3.Identity()

    displacment.translation[2] = l/2.
    oMsphere_1 = oMg * displacment
    displacment.translation[2] = -l/2.
    oMsphere_2 = oMg * displacment

    marker_cylinder = marker_ref
    marker_cylinder.type = Marker.CYLINDER
    marker_cylinder.scale = Point(d,d,l)
    marker_cylinder.pose = SE3ToROSPose(oMg)

    marker_sphere_1 = deepcopy(marker_ref)
    marker_sphere_1.id += 10000 # How to ensure this id is not taken ?
    marker_sphere_1.type = Marker.SPHERE
    marker_sphere_1.scale = Point(d,d,d)
    marker_sphere_1.pose = SE3ToROSPose(oMsphere_1)

    marker_sphere_2 = deepcopy(marker_ref)
    marker_sphere_2.id += 20000 # How to ensure this id is not taken ?
    marker_sphere_2.type = Marker.SPHERE
    marker_sphere_2.scale = Point(d,d,d)
    marker_sphere_2.pose = SE3ToROSPose(oMsphere_2)

    return [marker_cylinder, marker_sphere_1, marker_sphere_2]

def SE3ToROSPose(oMg):
    """Converts SE3 matrix to ROS geometry_msgs/Pose format"""
    from geometry_msgs.msg import Pose, Point, Quaternion

    xyz_quat = pin.SE3ToXYZQUATtuple(oMg)
    return Pose(position=Point(*xyz_quat[:3]), orientation=Quaternion(*xyz_quat[3:]))

class RVizVisualizer(BaseVisualizer):
    """A Pinocchio display using RViz"""
    class Viewer:
        app = None
        viz = None
        viz_manager = None

    def initViewer(self, viewer=None, windowName="python-pinocchio", loadModel=False, initRosNode=True):
        """Init RVizViewer by starting a ros node (or not) and creating an RViz window."""
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

        if initRosNode:
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
        self.visual_ids = []

        # Collisions
        self.collisions_publisher = Publisher(rootNodeName + "_collisions", MarkerArray, queue_size=1,  latch=True)
        self.collision_Display = self.viewer.viz_manager.createDisplay("rviz/MarkerArray", rootNodeName + "_collisions", True)
        self.collision_Display.subProp("Marker Topic").setValue(rootNodeName + "_collisions")
        self.collision_ids = []

        # Group
        root_group = self.viewer.viz_manager.getRootDisplayGroup()
        self.group_Display = self.viewer.viz_manager.createDisplay("rviz/Group", rootNodeName, True)
        self.group_Display.addChild(root_group.takeChild(self.visual_Display))    # Remove display from root group and add it to robot group
        self.group_Display.addChild(root_group.takeChild(self.collision_Display)) # Remove display from root group and add it to robot group

        self.seq = 0
        self.display()

    def clean(self):
        """Delete all the objects from the whole scene """
        if hasattr(self, 'collisions_publisher'):
            self._clean(self.collisions_publisher)
            self.collision_ids = []

        if hasattr(self, 'visuals_publisher'):
            self._clean(self.visuals_publisher)
            self.visual_ids = []

    def display(self, q = None):
        """Display the robot at configuration q in the viz by placing all the bodies."""
        # Update the robot kinematics and geometry.
        if q is not None:
            pin.forwardKinematics(self.model,self.data,q)

        if self.collision_model is not None and hasattr(self, 'collisions_publisher'):
            pin.updateGeometryPlacements(self.model, self.data, self.collision_model, self.collision_data)
            self.collision_ids = self._plot(self.collisions_publisher, self.collision_model, self.collision_data, self.collision_ids)

        if self.visual_model is not None and hasattr(self, 'visuals_publisher'):
            pin.updateGeometryPlacements(self.model, self.data, self.visual_model, self.visual_data)
            self.visual_ids = self._plot(self.visuals_publisher, self.visual_model, self.visual_data, self.visual_ids)

    def _plot(self, publisher, model, data, previous_ids=()):
        """Create markers for each object of the model and publish it as MarkerArray (also delete unused previously created markers)"""
        from rospy import get_rostime
        from std_msgs.msg import Header, ColorRGBA
        from geometry_msgs.msg import Point
        from visualization_msgs.msg import MarkerArray, Marker

        self.seq +=1
        header = Header(frame_id='map', seq=self.seq, stamp=get_rostime()) # Common header for every marker

        marker_array = MarkerArray()
        for obj in model.geometryObjects:
            obj_id = model.getGeometryId(obj.name)

            # Prepare marker
            marker = Marker()
            marker.id = obj_id
            marker.header = header
            marker.action = Marker.ADD # same as Marker.MODIFY
            marker.pose = SE3ToROSPose(data.oMg[obj_id])
            marker.color = ColorRGBA(*obj.meshColor)

            if obj.meshTexturePath != "":
                warnings.warn("Textures are not supported in RVizVisualizer (for " + obj.name + ")", category=UserWarning, stacklevel=2)

            # Create geometry
            geom = obj.geometry
            if WITH_HPP_FCL_BINDINGS and isinstance(geom, hppfcl.ShapeBase):
                # append a primitive geometry
                if isinstance(geom, hppfcl.Cylinder):
                    d, l = 2*geom.radius, 2*geom.halfLength
                    marker.type = Marker.CYLINDER
                    marker.scale = Point(d,d,l)
                    marker_array.markers.append(marker)
                elif isinstance(geom, hppfcl.Box):
                    size = npToTuple(2.*geom.halfSide)
                    marker.type = Marker.CUBE
                    marker.scale = Point(*size)
                    marker_array.markers.append(marker)
                elif isinstance(geom, hppfcl.Sphere):
                    d = 2*geom.radius
                    marker.type = Marker.SPHERE
                    marker.scale = Point(d, d, d)
                    marker_array.markers.append(marker)
                elif isinstance(geom, hppfcl.Capsule):
                    d, l = 2*geom.radius, 2 * geom.halfLength
                    marker_array.markers.extend(create_capsule_markers(marker, data.oMg[obj_id], d, l))
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

        # Remove unused markers
        new_ids = [marker.id for marker in marker_array.markers]
        for old_id in previous_ids:
            if not old_id in new_ids:
                marker_remove = Marker()
                marker_remove.header = header
                marker_remove.id = old_id
                marker_remove.action = Marker.DELETE
                marker_array.markers.append(marker_remove)

        # Publish markers
        publisher.publish(marker_array)

        # Return list of markers id
        return new_ids

    def _clean(self, publisher):
        """Delete all the markers from a topic (use one marker with action DELETEALL)"""
        from rospy import get_rostime
        from std_msgs.msg import Header
        from visualization_msgs.msg import MarkerArray, Marker

        # Increment seq number
        self.seq +=1

        # Prepare a clean_all marker
        marker = Marker()
        marker.header = Header(frame_id='map', seq=self.seq, stamp=get_rostime())
        marker.action = Marker.DELETEALL

        # Add the marker to a MarkerArray
        marker_array = MarkerArray(markers = [marker])

        # Publish marker
        publisher.publish(marker_array)

    def displayCollisions(self,visibility):
        """Set whether to display collision objects or not"""
        self.collision_Display.setEnabled(visibility)

    def displayVisuals(self,visibility):
        """Set whether to display visual objects or not"""
        self.visual_Display.setEnabled(visibility)

    def sleep(self, dt):
        from python_qt_binding.QtTest import QTest
        QTest.qWait(1e3*dt)


__all__ = ['RVizVisualizer']
