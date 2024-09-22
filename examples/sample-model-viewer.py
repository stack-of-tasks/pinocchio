from sys import argv

import pinocchio as pin
from numpy import pi
from pinocchio.visualize import GepettoVisualizer, MeshcatVisualizer, RVizVisualizer

# GepettoVisualizer: -g
# MeshcatVisualizer: -m
VISUALIZER = None
if len(argv) > 1:
    opt = argv[1]
    if opt == "-g":
        VISUALIZER = GepettoVisualizer
    elif opt == "-m":
        VISUALIZER = MeshcatVisualizer
    elif opt == "-r":
        VISUALIZER = RVizVisualizer
    else:
        raise ValueError("Unrecognized option: " + opt)

model = pin.buildSampleModelHumanoid()
visual_model = pin.buildSampleGeometryModelHumanoid(model)
collision_model = visual_model.copy()

q0 = pin.neutral(model)

if VISUALIZER:
    viz = VISUALIZER(model, collision_model, visual_model)
    viz.initViewer()
    viz.loadViewerModel()
    viz.display(q0)

    input("Enter to check a new configuration")

    q = q0.copy()
    q[8] = pi / 2
    q[14] = pi / 2
    q[23] = -pi / 2
    q[29] = pi / 2

    viz.display(q)

    input("Press enter to exit...")
