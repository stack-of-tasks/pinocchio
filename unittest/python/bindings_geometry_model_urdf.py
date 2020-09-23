import unittest
import pinocchio as pin
import os

def checkGeom(geom1,geom2):
  return geom1.ngeoms == geom2.ngeoms

@unittest.skipUnless(pin.WITH_URDFDOM,"Needs URDFDOM")
class TestGeometryObjectUrdfBindings(unittest.TestCase):

    def setUp(self):
        self.current_file = os.path.dirname(str(os.path.abspath(__file__)))
        self.model_dir = os.path.abspath(os.path.join(self.current_file, "../../models/others/robots"))
        self.model_path = os.path.abspath(os.path.join(self.model_dir, "romeo_description/urdf/romeo.urdf"))

    def test_load(self):
        hint_list = [self.model_dir, "wrong/hint"]
        expected_mesh_path = os.path.join(self.model_dir,'romeo_description/meshes/V1/collision/LHipPitch.dae')

        model = pin.buildModelFromUrdf(self.model_path, pin.JointModelFreeFlyer())
        collision_model = pin.buildGeomFromUrdf(model, self.model_path, pin.GeometryType.COLLISION, hint_list)

        col = collision_model.geometryObjects[1]
        self.assertEqual(os.path.normpath(col.meshPath), os.path.normpath(expected_mesh_path))

    def test_self_load(self):
        hint_list = [self.model_dir]

        model = pin.buildModelFromUrdf(self.model_path, pin.JointModelFreeFlyer())
        collision_model_ref = pin.buildGeomFromUrdf(model, self.model_path, pin.GeometryType.COLLISION, hint_list)

        collision_model_self = pin.GeometryModel()
        pin.buildGeomFromUrdf(model, self.model_path, pin.GeometryType.COLLISION, collision_model_self, hint_list)
        self.assertTrue(checkGeom(collision_model_ref, collision_model_self))

        collision_model_self = pin.GeometryModel()
        pin.buildGeomFromUrdf(model, self.model_path, pin.GeometryType.COLLISION, collision_model_self, self.model_dir)
        self.assertTrue(checkGeom(collision_model_ref, collision_model_self))

        hint_vec = pin.StdVec_StdString()
        hint_vec.append(self.model_dir)

        collision_model_self = pin.GeometryModel()
        pin.buildGeomFromUrdf(model, self.model_path, pin.GeometryType.COLLISION, collision_model_self, hint_vec)
        self.assertTrue(checkGeom(collision_model_ref, collision_model_self))
 

    def test_multi_load(self):
        hint_list = [self.model_dir, "wrong/hint"]
        expected_collision_path = os.path.join(self.model_dir,'romeo_description/meshes/V1/collision/LHipPitch.dae')
        expected_visual_path = os.path.join(self.model_dir,'romeo_description/meshes/V1/visual/LHipPitch.dae')

        model = pin.buildModelFromUrdf(self.model_path, pin.JointModelFreeFlyer())

        collision_model = pin.buildGeomFromUrdf(model, self.model_path, pin.GeometryType.COLLISION, hint_list)
        col = collision_model.geometryObjects[1]
        self.assertEqual(os.path.normpath(col.meshPath), os.path.normpath(expected_collision_path))

        visual_model = pin.buildGeomFromUrdf(model, self.model_path, pin.GeometryType.VISUAL, hint_list)
        vis = visual_model.geometryObjects[1]
        self.assertEqual(os.path.normpath(vis.meshPath), os.path.normpath(expected_visual_path))

        model_2, collision_model_2, visual_model_2 = pin.buildModelsFromUrdf(self.model_path, hint_list, pin.JointModelFreeFlyer())

        self.assertEqual(model,model_2)

        col_2 = collision_model_2.geometryObjects[1]
        self.assertEqual(os.path.normpath(col_2.meshPath), os.path.normpath(expected_collision_path))

        vis_2 = visual_model_2.geometryObjects[1]
        self.assertEqual(os.path.normpath(vis_2.meshPath), os.path.normpath(expected_visual_path))

        model_c, collision_model_c = pin.buildModelsFromUrdf(self.model_path, hint_list, pin.JointModelFreeFlyer(), geometry_types=pin.GeometryType.COLLISION)

        self.assertEqual(model,model_c)

        col_c = collision_model_c.geometryObjects[1]
        self.assertEqual(os.path.normpath(col_c.meshPath), os.path.normpath(expected_collision_path))

        model_v, visual_model_v = pin.buildModelsFromUrdf(self.model_path, hint_list, pin.JointModelFreeFlyer(), geometry_types=pin.GeometryType.VISUAL)

        self.assertEqual(model,model_v)

        vis_v = visual_model_v.geometryObjects[1]
        self.assertEqual(os.path.normpath(vis_v.meshPath), os.path.normpath(expected_visual_path))
    
    def test_deprecated_signatures(self):
        model = pin.buildModelFromUrdf(self.model_path, pin.JointModelFreeFlyer())
  
        hint_list = [self.model_dir, "wrong/hint"]
        collision_model = pin.buildGeomFromUrdf(model, self.model_path, hint_list, pin.GeometryType.COLLISION)

        hint_vec = pin.StdVec_StdString()
        hint_vec.append(self.model_dir)
        collision_model = pin.buildGeomFromUrdf(model, self.model_path, hint_vec, pin.GeometryType.COLLISION)

        collision_model = pin.buildGeomFromUrdf(model, self.model_path, self.model_dir, pin.GeometryType.COLLISION)
      
        if pin.WITH_HPP_FCL_BINDINGS:
            collision_model = pin.buildGeomFromUrdf(model, self.model_path, hint_list, pin.GeometryType.COLLISION, pin.hppfcl.MeshLoader())
            collision_model = pin.buildGeomFromUrdf(model, self.model_path, hint_vec, pin.GeometryType.COLLISION, pin.hppfcl.MeshLoader())
            collision_model = pin.buildGeomFromUrdf(model, self.model_path, self.model_dir, pin.GeometryType.COLLISION, pin.hppfcl.MeshLoader())

if __name__ == '__main__':
    unittest.main()
