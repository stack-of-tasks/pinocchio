# Typical header of a Python script using Pinocchio
from pinocchio.utils import *
from pinocchio.explog import exp,log
from numpy.linalg import pinv,norm
import pinocchio as pin
import gepetto.corbaserver

# Example of a class Display that connect to Gepetto-viewer and implement a
# 'place' method to set the position/rotation of a 3D visual object in a scene.
class Display():
    '''
    Class Display: Example of a class implementing a client for the Gepetto-viewer server. The main
    method of the class is 'place', that sets the position/rotation of a 3D visual object in a scene.
    '''
    def __init__(self,windowName = "pinocchio" ):
        '''
        This function connect with the Gepetto-viewer server and open a window with the given name.
        If the window already exists, it is kept in the current state. Otherwise, the newly-created
        window is set up with a scene named 'world'.
        '''

        # Create the client and connect it with the display server.
        try:
            self.viewer=gepetto.corbaserver.Client()
        except:
            print "Error while starting the viewer client. "
            print "Check whether Gepetto-viewer is properly started"

        # Open a window for displaying your model.
        try:
            # If the window already exists, do not do anything.
            windowID = self.viewer.gui.getWindowID (windowName)
            print "Warning: window '"+windowName+"' already created."
            print "The previously created objects will not be destroyed and do not have to be created again."
        except:
            # Otherwise, create the empty window.
            windowID = self.viewer.gui.createWindow (windowName)
            # Start a new "scene" in this window, named "world", with just a floor.
            self.viewer.gui.createScene("world")
            self.viewer.gui.addSceneToWindow("world",windowID)

        # Finally, refresh the layout to obtain your first rendering.
        self.viewer.gui.refresh()

    def place(self,objName,M,refresh=True):
        '''
        This function places (ie changes both translation and rotation) of the object
        names "objName" in place given by the SE3 object "M". By default, immediately refresh
        the layout. If multiple objects have to be placed at the same time, do the refresh
        only at the end of the list.
        '''
        self.viewer.gui.applyConfiguration(objName,
                                           pin.se3ToXYZQUATtuple(M))
        if refresh: self.viewer.gui.refresh()

