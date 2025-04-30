# This examples shows how to load and move a robot in panda3d_viewer.
# Note: this feature requires panda3d_viewer to be installed, this can be done using
# pip install panda3d_viewer
# ruff: noqa: E402


import sys
from pathlib import Path

import numpy as np

# Add path to the example-robot-data package from git submodule.
# If you have a proper install version, there is no need for this sys.path thing
path = Path(__file__).parent.parent / "models" / "example-robot-data" / "python"
sys.path.append(str(path))
from example_robot_data.robots_loader import TalosLoader
from panda3d_viewer import ViewerClosedError
from pinocchio.visualize.panda3d_visualizer import Panda3dVisualizer

# talos is a RobotWrapper object
talos = TalosLoader().robot
# Attach talos to the viewer scene
talos.setVisualizer(Panda3dVisualizer())
talos.initViewer()
talos.loadViewerModel(group_name="talos", color=(1, 1, 1, 1))
record_video = False


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

    if record_video:
        dt = 1 / 60
        fps = 1.0 / dt
        filename = "talos.mp4"
        ctx = talos.viz.create_video_ctx(filename, fps=fps)
        print(f"[video will be recorded @ {filename}]")
    else:
        from contextlib import nullcontext

        ctx = nullcontext()
        print("[no recording]")

    while True:
        with ctx:
            talos.play(traj.T, 1.0 / update_rate)
            traj = np.flip(traj, 1)
            if record_video:
                break


try:
    play_sample_trajectory()
except ViewerClosedError:
    # an exception will be thrown when the window is closed
    pass
