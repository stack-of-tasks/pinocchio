from .. import libpinocchio_pywrap as pin
from ..shortcuts import buildModelsFromUrdf, createDatas

import time

class AbstractDisplay(object):
    """Pinocchio displays are employed to easily display a model at a given configuration.
    AbstractDisplay is not meant to be directly employed, but only to provide a uniform interface and a few common methods.
    New displays should extend this class and override its methods as neeeded.
    """

    @classmethod
    def BuildFromURDF(cls,filename, package_dirs=None, root_joint=None, verbose=False, meshLoader=None):
        """Construct a display from the given URDF file"""

        display = cls()
        display.initFromURDF(filename, package_dirs, root_joint, verbose, meshLoader)
        return display

    def initFromURDF(self,filename, package_dirs=None, root_joint=None, verbose=False, meshLoader=None):
        """Initialize a display from the given URDF file"""

        model, collision_model, visual_model = buildModelsFromUrdf(filename, package_dirs, root_joint, verbose, meshLoader)
        cls = type(self)
        cls.__init__(self,model=model,collision_model=collision_model,visual_model=visual_model)

    def __init__(self, model = pin.Model(), collision_model = None, visual_model = None):
        """Construct a display from the given model, collision model, and visual model"""

        self.model = model
        self.collision_model = collision_model
        self.visual_model = visual_model

        self.data, self.collision_data, self.visual_data = createDatas(model,collision_model,visual_model)

    def getViewerNodeName(self, geometry_object, geometry_type):
        """Return the name of the geometry object inside the viewer"""
        pass 

    def initDisplay(self, *args, **kwargs):
        """Init display by loading the gui and creating a window."""
        pass

    def loadDisplayModel(self, *args, **kwargs):
        """Create the scene displaying the robot meshes in gepetto-viewer"""
        pass

    def display(self, q):
        """Display the robot at configuration q in the viewer by placing all the bodies."""
        pass

    def displayCollisions(self,visibility):
        """Set whether to diplay collision objects or not"""
        pass
 
    def displayVisuals(self,visibility):
        """Set whether to diplay visual objects or not"""
        pass

    def play(self, q_trajectory, dt):
        """Play a trajectory with given time step"""
        for k in range(q_trajectory.shape[1]):
            t0 = time.time()
            self.display(q_trajectory[:, k])
            t1 = time.time()
            elapsed_time = t1 - t0
            if elapsed_time < dt:
                time.sleep(dt - elapsed_time)

__all__ = ['AbstractDisplay']
