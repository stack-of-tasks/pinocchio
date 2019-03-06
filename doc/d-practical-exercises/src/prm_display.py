import time
from pinocchio.utils import se3ToXYZQUATtuple


def display_prm(robot, graph):
    '''Take a graph object containing a list of configurations q and
    a dictionnary of graph relations edge. Display the configurations by the correspond
    placement of the robot end effector. Display the graph relation by vertices connecting
    the robot end effector positions.
    '''

    gui = robot.viewer.gui

    try:
        gui.deleteNode('world/prm', True)
    except:
        pass
    gui.createRoadmap('world/prm', [1., .2, .2, .8], 1e-2, 1e-2, [1., .2, .2, .8])

    for q in graph.q:
        gui.addNodeToRoadmap('world/prm', se3ToXYZQUATtuple(robot.position(q, 6)))

        for parent, children in graph.children.items():
            for child in children:
                if child > parent:
                    q1, q2 = graph.q[parent], graph.q[child]
                    p1 = robot.position(q1, 6).translation.ravel().tolist()[0]
                    p2 = robot.position(q2, 6).translation.ravel().tolist()[0]
                    gui.addEdgeToRoadmap('world/prm', p1, p2)

                    gui.refresh()


def display_path(robot, path, sleeptime=1e-2):
    '''
    Display a path, i.e. a sequence of robot configuration, by moving the robot
    to each list element.
    '''
    for q in path:
        robot.display(q)
        time.sleep(sleeptime)
