from .. import pinocchio_pywrap as pin
from ..shortcuts import buildModelsFromUrdf, createDatas

import time

class BaseVisualizer(object):
    """Pinocchio visualizers are employed to easily display a model at a given configuration.
    BaseVisualizer is not meant to be directly employed, but only to provide a uniform interface and a few common methods.
    New visualizers should extend this class and override its methods as neeeded.
    """

    def __init__(self, model = pin.Model(), collision_model = None, visual_model = None, copy_models=False):
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

        self.data, self.collision_data, self.visual_data = createDatas(model,collision_model,visual_model)

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
        """Create the scene displaying the robot meshes in gepetto-viewer"""
        pass

    def display(self, q):
        """Display the robot at configuration q in the viewer by placing all the bodies."""
        pass

    def displayCollisions(self,visibility):
        """Set whether to display collision objects or not."""
        pass
 
    def displayVisuals(self,visibility):
        """Set whether to display visual objects or not."""
        pass

    def play(self, q_trajectory, dt):
        """Play a trajectory with given time step."""
        for k in range(q_trajectory.shape[1]):
            t0 = time.time()
            self.display(q_trajectory[:, k])
            t1 = time.time()
            elapsed_time = t1 - t0
            if elapsed_time < dt:
                time.sleep(dt - elapsed_time)

__all__ = ['BaseVisualizer']
