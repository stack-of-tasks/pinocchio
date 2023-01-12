from .. import pinocchio_pywrap_default as pin
from ..shortcuts import buildModelsFromUrdf, createDatas

import time
import numpy as np

class BaseVisualizer(object):
    """Pinocchio visualizers are employed to easily display a model at a given configuration.
    BaseVisualizer is not meant to be directly employed, but only to provide a uniform interface and a few common methods.
    New visualizers should extend this class and override its methods as neeeded.
    """

    def __init__(self, model = pin.Model(), collision_model = None, visual_model = None, copy_models = False, data = None, collision_data = None, visual_data = None):
        """Construct a display from the given model, collision model, and visual model.
        If copy_models is True, the models are copied. Otherwise, they are simply kept as a reference."""

        if copy_models:
            self.model = model.copy()
            self.collision_model = collision_model.copy()
            self.visual_model = visual_model.copy()
        else:
            self.model = model
            self.collision_model = collision_model
            self.visual_model = visual_model

        if data is None:
            self.data = self.model.createData()
        else:
            self.data = data

        if collision_data is None and self.collision_model is not None:
            self.collision_data = self.collision_model.createData()
        else:
            self.collision_data = collision_data

        if visual_data is None and self.visual_model is not None:
            self.visual_data = self.visual_model.createData()
        else:
            self.visual_data = visual_data

    def rebuildData(self):
        """Re-build the data objects. Needed if the models were modified.
        Warning: this will delete any information stored in all data objects."""
        self.data, self.collision_data, self.visual_data = createDatas(self.model, self.collision_model, self.visual_model)

    def getViewerNodeName(self, geometry_object, geometry_type):
        """Return the name of the geometry object inside the viewer."""
        pass 

    def initViewer(self, *args, **kwargs):
        """Init the viewer by loading the gui and creating a window."""
        pass

    def loadViewerModel(self, *args, **kwargs):
        """Create the scene displaying the robot meshes in the viewer"""
        pass

    def reload(self, new_geometry_object, geometry_type = None):
        """ Reload a geometry_object given by its type"""
        pass

    def clean(self):
        """ Delete all the objects from the whole scene """
        pass

    def display(self, q=None):
        """Display the robot at configuration q or refresh the rendering
        from the current placements contained in data by placing all the bodies in the viewer."""
        pass

    def displayCollisions(self, visibility):
        """Set whether to display collision objects or not."""
        raise NotImplementedError()
 
    def displayVisuals(self, visibility):
        """Set whether to display visual objects or not."""
        raise NotImplementedError()
    
    def setBackgroundColor(self):
        """Set the visualizer background color."""
        raise NotImplementedError()

    def setCameraTarget(self, target):
        """Set the camera target."""
        raise NotImplementedError()

    def setCameraPosition(self, position: np.ndarray):
        """Set the camera's 3D position."""
        raise NotImplementedError()

    def setCameraZoom(self, zoom: float):
        """Set camera zoom value."""
        raise NotImplementedError()

    def setCameraPose(self, pose: np.ndarray = np.eye(4)):
        """Set camera 6D pose using a 4x4 matrix."""
        raise NotImplementedError()

    def captureImage(self, w=None, h=None):
        """Captures an image from the viewer and returns an RGB array."""
        raise NotImplementedError()

    def disableCameraControl(self):
        raise NotImplementedError()

    def enableCameraControl(self):
        raise NotImplementedError()

    def drawFrameVelocities(self, *args):
        """Draw current frame velocities."""
        raise NotImplementedError()

    def sleep(self, dt):
        time.sleep(dt)

    def play(self, q_trajectory, dt, capture=False):
        """Play a trajectory with given time step. Optionally capture RGB images and returns them."""
        imgs = []
        for k in range(q_trajectory.shape[1]):
            t0 = time.time()
            self.display(q_trajectory[:, k])
            if capture:
                img_arr = self.captureImage()
                imgs.append(img_arr)
            t1 = time.time()
            elapsed_time = t1 - t0
            if elapsed_time < dt:
                self.sleep(dt - elapsed_time)
        if capture:
            return imgs


class VideoContext:
    def __init__(self, viz: BaseVisualizer, fps: int, filename: str, **kwargs):
        self.viz = viz
        self.vid_writer

__all__ = ['BaseVisualizer']
