from .. import pinocchio_pywrap as pin
from ..shortcuts import buildModelsFromUrdf, createDatas

import time
import numpy as np
import os.path as osp


try:
    import imageio
    IMAGEIO_SUPPORT = True
except ImportError:
    IMAGEIO_SUPPORT = False


class BaseVisualizer(object):
    """Pinocchio visualizers are employed to easily display a model at a given configuration.
    BaseVisualizer is not meant to be directly employed, but only to provide a uniform interface and a few common methods.
    New visualizers should extend this class and override its methods as neeeded.
    """

    _video_writer = None

    def __init__(
        self,
        model=pin.Model(),
        collision_model=None,
        visual_model=None,
        copy_models=False,
        data=None,
        collision_data=None,
        visual_data=None,
    ):
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
        self.data, self.collision_data, self.visual_data = createDatas(
            self.model, self.collision_model, self.visual_model
        )

    def getViewerNodeName(self, geometry_object, geometry_type):
        """Return the name of the geometry object inside the viewer."""
        pass

    def initViewer(self, *args, **kwargs):
        """Init the viewer by loading the gui and creating a window."""
        pass

    def loadViewerModel(self, *args, **kwargs):
        """Create the scene displaying the robot meshes in the viewer"""
        pass

    def reload(self, new_geometry_object, geometry_type=None):
        """Reload a geometry_object given by its type"""
        pass

    def clean(self):
        """Delete all the objects from the whole scene"""
        pass

    def display(self, q=None):
        """Display the robot at configuration q or refresh the rendering
        from the current placements contained in data by placing all the bodies in the viewer."""
        pass

    def displayCollisions(self, visibility):
        """Set whether to display collision objects or not."""
        pass

    def displayVisuals(self, visibility):
        """Set whether to display visual objects or not."""
        raise NotImplementedError()

    def setBackgroundColor(self, *args, **kwargs):
        """Set the visualizer background color."""
        raise NotImplementedError()

    def setCameraTarget(self, target):
        """Set the camera target."""
        raise NotImplementedError()

    def setCameraPosition(self, position):
        """Set the camera's 3D position."""
        raise NotImplementedError()

    def setCameraZoom(self, zoom):
        """Set camera zoom value."""
        raise NotImplementedError()

    def setCameraPose(self, pose=np.eye(4)):
        """Set camera 6D pose using a 4x4 matrix."""
        raise NotImplementedError()

    def captureImage(self, w=None, h=None):
        """Captures an image from the viewer and returns an RGB array."""
        pass

    def disableCameraControl(self):
        raise NotImplementedError()

    def enableCameraControl(self):
        raise NotImplementedError()

    def drawFrameVelocities(self, *args, **kwargs):
        """Draw current frame velocities."""
        raise NotImplementedError()

    def sleep(self, dt):
        time.sleep(dt)

    def has_video_writer(self):
        return self._video_writer is not None

    def play(self, q_trajectory, dt=None, callback=None, capture=False, **kwargs):
        """Play a trajectory with given time step. Optionally capture RGB images and returns them."""
        nsteps = len(q_trajectory)
        if not capture:
            capture = self.has_video_writer()

        imgs = []
        for i in range(nsteps):
            t0 = time.time()
            self.display(q_trajectory[i])
            if callback is not None:
                callback(i, **kwargs)
            if capture:
                img_arr = self.captureImage()
                if not self.has_video_writer():
                    imgs.append(img_arr)
                else:
                    self._video_writer.append_data(img_arr)
            t1 = time.time()
            elapsed_time = t1 - t0
            if dt is not None and elapsed_time < dt:
                self.sleep(dt - elapsed_time)
        if capture and not self.has_video_writer():
            return imgs

    def create_video_ctx(self, filename=None, fps=30, directory=None, **kwargs):
        """Create a video recording context, generating the output filename if necessary.

        Code inspired from https://github.com/petrikvladimir/RoboMeshCat.
        """
        if not IMAGEIO_SUPPORT:
            import warnings, contextlib
            warnings.warn("Video context cannot be created because imageio is not available.", UserWarning)
            return contextlib.nullcontext()
        if filename is None:
            if directory is None:
                from tempfile import gettempdir

                directory = gettempdir()
            f_fmt = "%Y%m%d_%H%M%S"
            ext = "mp4"
            filename = time.strftime("{}.{}".format(f_fmt, ext))
            filename = osp.join(directory, filename)
        return VideoContext(self, fps, filename)


class VideoContext:
    def __init__(self, viz, fps, filename, **kwargs):
        self.viz = viz
        self.vid_writer = imageio.get_writer(filename, fps=fps, **kwargs)

    def __enter__(self):
        print("[Entering video recording context]")
        self.viz._video_writer = self.vid_writer

    def __exit__(self, *args):
        self.vid_writer.close()
        self.viz._video_writer = None


__all__ = ["BaseVisualizer"]
