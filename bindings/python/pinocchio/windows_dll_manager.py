import contextlib
import os
from pathlib import Path


def get_dll_paths():
    pinocchio_paths = os.getenv("PINOCCHIO_WINDOWS_DLL_PATH")
    if pinocchio_paths is None:
        # Standard site-packages to bin path
        RELATIVE_DLL_PATH = "..\\..\\..\\bin"
        return [Path(__file__).parent / RELATIVE_DLL_PATH]
    else:
        return pinocchio_paths.split(os.pathsep)


class PathManager(contextlib.AbstractContextManager):
    """Restore PATH state after importing Python module"""

    def add_dll_directory(self, dll_dir: str):
        os.environ["PATH"] += os.pathsep + dll_dir

    def __enter__(self):
        self.old_path = os.environ["PATH"]
        return self

    def __exit__(self, *exc_details):
        os.environ["PATH"] = self.old_path


class DllDirectoryManager(contextlib.AbstractContextManager):
    """Restore DllDirectory state after importing Python module"""

    def add_dll_directory(self, dll_dir: str):
        # add_dll_directory can fail on relative path and non
        # existing path.
        # Since we don't know all the fail criterion we just ignore
        # thrown exception
        try:
            self.dll_dirs.append(os.add_dll_directory(dll_dir))
        except OSError:
            pass

    def __enter__(self):
        self.dll_dirs = []
        return self

    def __exit__(self, *exc_details):
        for d in self.dll_dirs:
            d.close()


def build_directory_manager():
    return DllDirectoryManager()
