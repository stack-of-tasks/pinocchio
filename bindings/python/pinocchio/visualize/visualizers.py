from enum import Enum
from importlib.util import find_spec
from os import environ

from .base_visualizer import BaseVisualizer
from .gepetto_visualizer import GepettoVisualizer
from .meshcat_visualizer import MeshcatVisualizer
from .panda3d_visualizer import Panda3dVisualizer
from .rviz_visualizer import RVizVisualizer


class Visualizer(Enum):
    BASE = BaseVisualizer
    GEPETTO = GepettoVisualizer
    MESHCAT = MeshcatVisualizer
    PANDA3D = Panda3dVisualizer
    RVIZ = RVizVisualizer

    @classmethod
    def default(cls):
        """
        Allow user to choose their prefered viewer with eg.
        export PINOCCHIO_VIEWER=RVIZ.

        Otherwise, try to find one which is installed.
        """
        # Allow user to define which viewer they want
        if "PINOCCHIO_VIEWER" in environ:
            selected = environ["PINOCCHIO_VIEWER"].upper()
            if hasattr(cls, selected):
                return getattr(cls, selected).value
            err = (
                f"The visualizer '{selected}' is not available.\n"
                "Please set PINOCCHIO_VIEWER to something installed among:\n"
                "- meshcat\n"
                "- gepetto-viewer\n"
                "- panda3d\n"
                "- rviz\n"
            )
            raise ImportError(err)

        # Otherwise, use the first available
        for v in ["meshcat", "gepetto", "panda3d_viewer", "rviz"]:
            if find_spec(v) is not None:
                return getattr(cls, v.replace("_viewer", "").upper()).value

        err = (
            "No visualizer could be found.\n"
            "Please install one of those:\n"
            "- meshcat\n"
            "- gepetto-viewer\n"
            "- panda3d\n"
            "- rviz\n"
        )
        raise ImportError(err)
