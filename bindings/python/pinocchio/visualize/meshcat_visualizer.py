from .. import pinocchio_pywrap as pin
from ..utils import npToTuple

from . import BaseVisualizer

import os
import warnings
import numpy as np
from distutils.version import LooseVersion

try:
    import hppfcl
    WITH_HPP_FCL_BINDINGS = True
except:
    WITH_HPP_FCL_BINDINGS = False

def isMesh(geometry_object):
    """ Check whether the geometry object contains a Mesh supported by MeshCat """
    if geometry_object.meshPath == "":
        return False

    _, file_extension = os.path.splitext(geometry_object.meshPath)
    if file_extension.lower() in [".dae", ".obj", ".stl"]:
        return True

    return False

def loadMesh(mesh):
    import meshcat.geometry as mg

    if isinstance(mesh,hppfcl.HeightFieldOBBRSS):
        heights = mesh.getHeights()
        x_grid = mesh.getXGrid()
        y_grid = mesh.getYGrid()
        min_height = mesh.getMinHeight()

        X, Y = np.meshgrid(x_grid,y_grid)

        nx = len(x_grid)-1
        ny = len(y_grid)-1

        num_cells = (nx) * (ny) * 2 + (nx+ny)*4 + 2

        num_vertices = X.size
        num_tris = num_cells

        faces = np.empty((num_tris,3),dtype=int)
        vertices = np.vstack((np.stack((X.reshape(num_vertices),Y.reshape(num_vertices),heights.reshape(num_vertices)),axis=1),
                              np.stack((X.reshape(num_vertices),Y.reshape(num_vertices),np.full(num_vertices,min_height)),axis=1)))

        face_id = 0
        for y_id in range(ny):
            for x_id in range(nx):
                p0 = x_id + y_id * (nx+1)
                p1 = p0 + 1
                p2 = p1 + nx + 1
                p3 = p2 - 1

                faces[face_id] = np.array([p0,p3,p1])
                face_id += 1
                faces[face_id] = np.array([p3,p2,p1])
                face_id += 1

                if y_id == 0:
                    p0_low = p0 + num_vertices
                    p1_low = p1 + num_vertices

                    faces[face_id] = np.array([p0,p1_low,p0_low])
                    face_id += 1
                    faces[face_id] = np.array([p0,p1,p1_low])
                    face_id += 1

                if y_id == ny-1:
                    p2_low = p2 + num_vertices
                    p3_low = p3 + num_vertices

                    faces[face_id] = np.array([p3,p3_low,p2_low])
                    face_id += 1
                    faces[face_id] = np.array([p3,p2_low,p2])
                    face_id += 1

                if x_id == 0:
                    p0_low = p0 + num_vertices
                    p3_low = p3 + num_vertices

                    faces[face_id] = np.array([p0,p3_low,p3])
                    face_id += 1
                    faces[face_id] = np.array([p0,p0_low,p3_low])
                    face_id += 1

                if x_id == nx-1:
                    p1_low = p1 + num_vertices
                    p2_low = p2 + num_vertices

                    faces[face_id] = np.array([p1,p2_low,p2])
                    face_id += 1
                    faces[face_id] = np.array([p1,p1_low,p2_low])
                    face_id += 1

        # Last face
        p0 = num_vertices
        p1 = p0 + nx
        p2 = 2*num_vertices-1
        p3 = p2 - nx

        faces[face_id] = np.array([p0,p1,p2])
        face_id += 1
        faces[face_id] = np.array([p0,p2,p3])
        face_id += 1

    elif isinstance(mesh,(hppfcl.Convex,hppfcl.BVHModelBase)):
        if isinstance(mesh,hppfcl.BVHModelBase):
            num_vertices = mesh.num_vertices
            num_tris = mesh.num_tris

            call_triangles = mesh.tri_indices
            call_vertices = mesh.vertices

        elif isinstance(mesh,hppfcl.Convex):
            num_vertices = mesh.num_points
            num_tris = mesh.num_polygons

            call_triangles = mesh.polygons
            call_vertices = mesh.points

        faces = np.empty((num_tris,3),dtype=int)
        for k in range(num_tris):
            tri = call_triangles(k)
            faces[k] = [tri[i] for i in range(3)]

        if LooseVersion(hppfcl.__version__) >= LooseVersion("1.7.7"):
            vertices = call_vertices()
        else:
            vertices = np.empty((num_vertices,3))
            for k in range(num_vertices):
                vertices[k] = call_vertices(k)

        vertices = vertices.astype(np.float32)

    if num_tris > 0:
        mesh = mg.TriangularMeshGeometry(vertices, faces)
    else:
        mesh = mg.Points(
                    mg.PointsGeometry(vertices.T, color=np.repeat(np.ones((3,1)),num_vertices,axis=1)),
                    mg.PointsMaterial(size=0.002))

    return mesh

def createCapsule(length, radius, radial_resolution = 30, cap_resolution = 10):
    nbv = np.array([max(radial_resolution, 4), max(cap_resolution, 4)])
    h = length
    r = radius
    position = 0
    vertices = np.zeros((nbv[0] * (2 * nbv[1]) + 2, 3))
    for j in range(nbv[0]):
        phi = (( 2 * np.pi * j) / nbv[0])
        for i in range(nbv[1]):
            theta = ((np.pi / 2 * i) / nbv[1])
            vertices[position + i, :] = np.array([np.cos(theta) * np.cos(phi) * r,
                                               np.cos(theta) * np.sin(phi) * r,
                                               -h / 2 - np.sin(theta) * r])
            vertices[position + i + nbv[1], :] = np.array([np.cos(theta) * np.cos(phi) * r,
                                                        np.cos(theta) * np.sin(phi) * r,
                                                        h / 2 + np.sin(theta) * r])
        position += nbv[1] * 2
    vertices[-2, :] = np.array([0, 0, -h / 2 - r])
    vertices[-1, :] = np.array([0, 0, h / 2 + r])
    indexes = np.zeros((nbv[0] * (4 * (nbv[1] - 1) + 4), 3))
    index = 0
    stride = nbv[1] * 2
    last = nbv[0] * (2 * nbv[1]) + 1
    for j in range(nbv[0]):
        j_next = (j + 1) % nbv[0]
        indexes[index + 0] = np.array([j_next * stride + nbv[1], j_next * stride, j * stride])
        indexes[index + 1] = np.array([j * stride + nbv[1], j_next * stride + nbv[1], j * stride])
        indexes[index + 2] = np.array([j * stride + nbv[1] - 1, j_next * stride + nbv[1] - 1, last - 1])
        indexes[index + 3] = np.array([j_next * stride + 2 * nbv[1] - 1, j * stride + 2 * nbv[1] - 1, last])
        for i in range(nbv[1]-1):
            indexes[index + 4 + i * 4 + 0] = np.array([j_next * stride + i, j_next * stride + i + 1, j * stride + i])
            indexes[index + 4 + i * 4 + 1] = np.array([j_next * stride + i + 1, j * stride + i + 1, j * stride + i])
            indexes[index + 4 + i * 4 + 2] = np.array([j_next * stride + nbv[1] + i + 1, j_next * stride + nbv[1] + i, j * stride + nbv[1] + i])
            indexes[index + 4 + i * 4 + 3] = np.array([j_next * stride + nbv[1] + i + 1, j * stride + nbv[1] + i, j * stride + nbv[1] + i + 1])
        index += 4 * (nbv[1] - 1) + 4
    import meshcat.geometry
    return meshcat.geometry.TriangularMeshGeometry(vertices, indexes)

class MeshcatVisualizer(BaseVisualizer):
    """A Pinocchio display using Meshcat"""

    def getViewerNodeName(self, geometry_object, geometry_type):
        """Return the name of the geometry object inside the viewer."""
        if geometry_type is pin.GeometryType.VISUAL:
            return self.viewerVisualGroupName + '/' + geometry_object.name
        elif geometry_type is pin.GeometryType.COLLISION:
            return self.viewerCollisionGroupName + '/' + geometry_object.name

    def initViewer(self, viewer=None, open=False, loadModel=False):
        """Start a new MeshCat server and client.
        Note: the server can also be started separately using the "meshcat-server" command in a terminal:
        this enables the server to remain active after the current script ends.
        """

        import meshcat

        self.viewer = meshcat.Visualizer() if viewer is None else viewer

        if open:
            self.viewer.open()

        if loadModel:
            self.loadViewerModel()

    def loadPrimitive(self, geometry_object):

        import meshcat.geometry

        # Cylinders need to be rotated
        R = np.array([[1.,  0.,  0.,  0.],
                      [0.,  0., -1.,  0.],
                      [0.,  1.,  0.,  0.],
                      [0.,  0.,  0.,  1.]])
        RotatedCylinder = type("RotatedCylinder", (meshcat.geometry.Cylinder,), {"intrinsic_transform": lambda self: R })

        geom = geometry_object.geometry
        if isinstance(geom, hppfcl.Capsule):
            if hasattr(meshcat.geometry, 'TriangularMeshGeometry'):
                obj = createCapsule(2. * geom.halfLength, geom.radius)
            else:
                obj = RotatedCylinder(2. * geom.halfLength, geom.radius)
        elif isinstance(geom, hppfcl.Cylinder):
            obj = RotatedCylinder(2. * geom.halfLength, geom.radius)
        elif isinstance(geom, hppfcl.Cone):
            obj = RotatedCylinder(2. * geom.halfLength, 0, geom.radius, 0)
        elif isinstance(geom, hppfcl.Box):
            obj = meshcat.geometry.Box(npToTuple(2. * geom.halfSide))
        elif isinstance(geom, hppfcl.Sphere):
            obj = meshcat.geometry.Sphere(geom.radius)
        elif isinstance(geom, hppfcl.ConvexBase):
            obj = loadMesh(geom)
        else:
            msg = "Unsupported geometry type for %s (%s)" % (geometry_object.name, type(geom) )
            warnings.warn(msg, category=UserWarning, stacklevel=2)
            obj = None

        return obj

    def loadMesh(self, geometry_object):

        import meshcat.geometry

        # Mesh path is empty if Pinocchio is built without HPP-FCL bindings
        if geometry_object.meshPath == "":
            msg = "Display of geometric primitives is supported only if pinocchio is build with HPP-FCL bindings."
            warnings.warn(msg, category=UserWarning, stacklevel=2)
            return None

        # Get file type from filename extension.
        _, file_extension = os.path.splitext(geometry_object.meshPath)
        if file_extension.lower() == ".dae":
            obj = meshcat.geometry.DaeMeshGeometry.from_file(geometry_object.meshPath)
        elif file_extension.lower() == ".obj":
            obj = meshcat.geometry.ObjMeshGeometry.from_file(geometry_object.meshPath)
        elif file_extension.lower() == ".stl":
            obj = meshcat.geometry.StlMeshGeometry.from_file(geometry_object.meshPath)
        else:
            msg = "Unknown mesh file format: {}.".format(geometry_object.meshPath)
            warnings.warn(msg, category=UserWarning, stacklevel=2)
            obj = None

        return obj

    def loadViewerGeometryObject(self, geometry_object, geometry_type, color=None):
        """Load a single geometry object"""
        import meshcat.geometry

        viewer_name = self.getViewerNodeName(geometry_object, geometry_type)

        is_mesh = False
        try:
            if WITH_HPP_FCL_BINDINGS and isinstance(geometry_object.geometry, hppfcl.ShapeBase):
                obj = self.loadPrimitive(geometry_object)
            elif isMesh(geometry_object):
                obj = self.loadMesh(geometry_object)
                is_mesh = True
            elif WITH_HPP_FCL_BINDINGS and isinstance(geometry_object.geometry, (hppfcl.BVHModelBase,hppfcl.HeightFieldOBBRSS)):
                obj = loadMesh(geometry_object.geometry)
            else:
                msg = "The geometry object named " + geometry_object.name + " is not supported by Pinocchio/MeshCat for vizualization."
                warnings.warn(msg, category=UserWarning, stacklevel=2)
                return
            if obj is None:
                return
        except Exception as e:
            msg = "Error while loading geometry object: %s\nError message:\n%s" % (geometry_object.name, e)
            warnings.warn(msg, category=UserWarning, stacklevel=2)
            return

        if isinstance(obj, meshcat.geometry.Object):
            self.viewer[viewer_name].set_object(obj)
        elif isinstance(obj, meshcat.geometry.Geometry):
            material = meshcat.geometry.MeshPhongMaterial()
            # Set material color from URDF, converting for triplet of doubles to a single int.
            if color is None:
                meshColor = geometry_object.meshColor
            else:
                meshColor = color
            material.color = int(meshColor[0] * 255) * 256**2 + int(meshColor[1] * 255) * 256 + int(meshColor[2] * 255)
            # Add transparency, if needed.
            if float(meshColor[3]) != 1.0:
                material.transparent = True
                material.opacity = float(meshColor[3])
            self.viewer[viewer_name].set_object(obj, material)

        if is_mesh: # Apply the scaling
            scale = list(np.asarray(geometry_object.meshScale).flatten())
            self.viewer[viewer_name].set_property("scale",scale)

    def loadViewerModel(self, rootNodeName="pinocchio", color = None):
        """Load the robot in a MeshCat viewer.
        Parameters:
            rootNodeName: name to give to the robot in the viewer
            color: optional, color to give to the robot. This overwrites the color present in the urdf.
                   Format is a list of four RGBA floats (between 0 and 1)
        """

        # Set viewer to use to gepetto-gui.
        self.viewerRootNodeName = rootNodeName

        # Collisions
        self.viewerCollisionGroupName = self.viewerRootNodeName + "/" + "collisions"

        for collision in self.collision_model.geometryObjects:
            self.loadViewerGeometryObject(collision,pin.GeometryType.COLLISION,color)
        self.displayCollisions(False)

        # Visuals
        self.viewerVisualGroupName = self.viewerRootNodeName + "/" + "visuals"

        for visual in self.visual_model.geometryObjects:
            self.loadViewerGeometryObject(visual,pin.GeometryType.VISUAL,color)
        self.displayVisuals(True)

    def reload(self, new_geometry_object, geometry_type = None):
        """ Reload a geometry_object given by its name and its type"""
        if geometry_type == pin.GeometryType.VISUAL:
            geom_model = self.visual_model
        else:
            geom_model = self.collision_model
            geometry_type = pin.GeometryType.COLLISION

        geom_id = geom_model.getGeometryId(new_geometry_object.name)
        geom_model.geometryObjects[geom_id] = new_geometry_object

        self.delete(new_geometry_object, geometry_type)
        visual = geom_model.geometryObjects[geom_id]
        self.loadViewerGeometryObject(visual,geometry_type)

    def clean(self):
        self.viewer.delete()

    def delete(self, geometry_object, geometry_type):
        viewer_name = self.getViewerNodeName(geometry_object, geometry_type)
        self.viewer[viewer_name].delete()

    def display(self, q = None):
        """Display the robot at configuration q in the viewer by placing all the bodies."""
        if q is not None:
            pin.forwardKinematics(self.model,self.data,q)

        if self.display_collisions:
            self.updatePlacements(pin.GeometryType.COLLISION)

        if self.display_visuals:
            self.updatePlacements(pin.GeometryType.VISUAL)

    def updatePlacements(self, geometry_type):
        if geometry_type == pin.GeometryType.VISUAL:
            geom_model = self.visual_model
            geom_data = self.visual_data
        else:
            geom_model = self.collision_model
            geom_data = self.collision_data

        pin.updateGeometryPlacements(self.model, self.data, geom_model, geom_data)
        for visual in geom_model.geometryObjects:
            visual_name = self.getViewerNodeName(visual,geometry_type)
            # Get mesh pose.
            M = geom_data.oMg[geom_model.getGeometryId(visual.name)]
            # Manage scaling: force scaling even if this should be normally handled by MeshCat (but there is a bug here)
            if isMesh(visual):
                scale = np.asarray(visual.meshScale).flatten()
                S = np.diag(np.concatenate((scale,[1.0])))
                T = np.array(M.homogeneous).dot(S)
            else:
                T = M.homogeneous

            # Update viewer configuration.
            self.viewer[visual_name].set_transform(T)

    def captureImage(self):
        """Capture an image from the Meshcat viewer and return an RGB array."""
        try:
            img = self.viewer.get_image()
            img_arr = np.asarray(img)
            return img_arr
        except AttributeError:
            warnings.warn("meshcat.Visualizer does not have the get_image() method."
                          " You need meshcat >= 0.2.0 to get this feature.")

    def displayCollisions(self,visibility):
        """Set whether to display collision objects or not."""
        if self.collision_model is None:
            self.display_collisions = False
        else:
            self.display_collisions = visibility
        self.viewer[self.viewerCollisionGroupName].set_property("visible", visibility)

        if visibility:
            self.updatePlacements(pin.GeometryType.COLLISION)

    def displayVisuals(self,visibility):
        """Set whether to display visual objects or not."""
        if self.visual_model is None:
            self.display_visuals = False
        else:
            self.display_visuals = visibility
        self.viewer[self.viewerVisualGroupName].set_property("visible", visibility)

        if visibility:
            self.updatePlacements(pin.GeometryType.VISUAL)

__all__ = ['MeshcatVisualizer']
