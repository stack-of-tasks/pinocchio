# This examples shows how to load and move a robot in panda3d_viewer.
# Note: this feature requires panda3d_viewer to be installed, this can be done using
# pip install --user panda3d_viewer

import pinocchio as pin
import numpy as np
import sys

from os.path import dirname, join, abspath

# add path to the example-robot-data package
path = join(dirname(dirname(abspath(__file__))), 'models', 'others', 'python')
sys.path.append(path)
from example_robot_data import loadTalos

# import visualizer
from panda3d_viewer import ViewerClosedError
from pinocchio.visualize.panda3d_visualizer import Panda3dVisualizer

talos = loadTalos()
talos.setVisualizer(Panda3dVisualizer())
talos.initViewer()
talos.loadViewerModel(group_name='talos', color=(1, 1, 1, 1))


# Play a sample trajectory in a loop
def play_sample_trajectory():
    update_rate = 60
    cycle_time = 3
    traj = np.repeat(talos.q0.reshape((-1, 1)), cycle_time * update_rate, axis=1)
    beta = np.linspace(0, 1, traj.shape[1])
    traj[[2, 9, 10, 11, 22, 15, 16, 17, 30]] = (
        0.39 + 0.685 * np.cos(beta),
        -beta,
        2.0 * beta,
        -beta,
        0.1 + beta * 1.56,
        -beta,
        2.0 * beta,
        -beta,
        -0.1 - beta * 1.56,
    )

    while True:
        talos.play(traj, 1. / update_rate)
        traj = np.flip(traj, 1)


try:
    play_sample_trajectory()
except ViewerClosedError:
    # an exception will be thrown when the window is closed
    pass
