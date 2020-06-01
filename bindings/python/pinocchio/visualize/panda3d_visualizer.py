import warnings

from .. import pinocchio_pywrap as pin
from ..utils import npToTuple
from .base_visualizer import BaseVisualizer

try:
    import hppfcl
    WITH_HPP_FCL_BINDINGS = True
except ImportError:
    WITH_HPP_FCL_BINDINGS = False

class Panda3dVisualizer(BaseVisualizer):
    """
    A Pinocchio display using panda3d engine.
    """

    def initViewer(self, viewer=None, load_model=False): # pylint: disable=arguments-differ
        """Init the viewer by attaching to / creating a GUI viewer."""
        self.visual_group = None
        self.collision_group = None
        self.display_visuals = False
        self.display_collisions = False
        self.viewer = viewer

        from panda3d_viewer import Viewer as Panda3dViewer
        if viewer is None:
            self.viewer = Panda3dViewer(window_title="python-pinocchio")

        if load_model:
            self.loadViewerModel(group_name=self.model.name)

    def loadViewerModel(self, group_name, color=None): # pylint: disable=arguments-differ
        """Create a group of nodes displaying the robot meshes in the viewer."""
        self.visual_group = group_name + "/visuals"
        self.collision_group = group_name + "/collisions"

        self.viewer.append_group(self.visual_group)
        self.viewer.append_group(self.collision_group)

        def append(root, obj):
            geom = obj.geometry
            if WITH_HPP_FCL_BINDINGS and isinstance(geom, hppfcl.ShapeBase):
                # append a primitive geometry
                if isinstance(geom, hppfcl.Capsule):
                    r, l = geom.radius, 2 * geom.halfLength
                    self.viewer.append_capsule(root, obj.name, r, l)
                elif isinstance(geom, hppfcl.Cylinder):
                    r, l = geom.radius, 2 * geom.halfLength
                    self.viewer.append_cylinder(root, obj.name, r, l)
                elif isinstance(geom, hppfcl.Box):
                    size = npToTuple(2. * geom.halfSide)
                    self.viewer.append_box(root, obj.name, size)
                elif isinstance(geom, hppfcl.Sphere):
                    self.viewer.append_sphere(root, obj.name, geom.radius)
                else:
                    msg = "Unsupported geometry type for %s (%s)" % (
                        obj.name, type(geom))
                    warnings.warn(msg, category=UserWarning, stacklevel=2)
                    return
            else:
                # append a mesh
                scale = npToTuple(obj.meshScale)
                self.viewer.append_mesh(root, obj.name, obj.meshPath, scale)

            if obj.overrideMaterial:
                rgba = npToTuple(obj.meshColor)
                path = obj.meshTexturePath
                self.viewer.set_material(root, obj.name, rgba, path)
            elif color is not None:
                self.viewer.set_material(root, obj.name, color)

        self.displayVisuals(False)
        self.displayCollisions(False)

        for obj in self.visual_model.geometryObjects:
            append(self.visual_group, obj)

        for obj in self.collision_model.geometryObjects:
            append(self.collision_group, obj)

        self.displayVisuals(True)

    def getViewerNodeName(self, geometry_object, geometry_type):
        """Return the name of the geometry object inside the viewer."""
        if geometry_type is pin.GeometryType.VISUAL:
            return self.visual_group + '/' + geometry_object.name
        elif geometry_type is pin.GeometryType.COLLISION:
            return self.collision_group + '/' + geometry_object.name

    def display(self, q):
        """Display the robot at configuration q in the viewer by placing all the bodies."""
        pin.forwardKinematics(self.model, self.data, q)

        def move(group, model, data):
            pin.updateGeometryPlacements(self.model, self.data, model, data)
            name_pose_dict = {}
            for obj in model.geometryObjects:
                oMg = data.oMg[model.getGeometryId(obj.name)]
                x, y, z, qx, qy, qz, qw = pin.SE3ToXYZQUATtuple(oMg)
                name_pose_dict[obj.name] = (x, y, z), (qw, qx, qy, qz)
            self.viewer.move_nodes(group, name_pose_dict)

        if self.display_visuals:
            move(self.visual_group, self.visual_model, self.visual_data)

        if self.display_collisions:
            move(self.collision_group, self.collision_model,
                 self.collision_data)

    def displayCollisions(self, visibility):
        """Set whether to display collision objects or not."""
        self.viewer.show_group(self.collision_group, visibility)
        self.display_collisions = visibility

    def displayVisuals(self, visibility):
        """Set whether to display visual objects or not."""
        self.viewer.show_group(self.visual_group, visibility)
        self.display_visuals = visibility
        
__all__ = ['Panda3dVisualizer']
