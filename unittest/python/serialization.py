import unittest

import pinocchio as pin

from test_case import PinocchioTestCase as TestCase

main_path = "./serialization-data"

class TestSerialization(TestCase):

  def testTXT(self):
    model = pin.buildSampleModelHumanoidRandom()
    filename = main_path + "/model.txt"
    model.saveToText(filename)

    model2 = pin.Model()
    model2.loadFromText(filename)

    self.assertTrue(model == model2)

  def testXML(self):
    model = pin.buildSampleModelHumanoidRandom()
    filename = main_path + "/model.xml"
    tag_name = "Model"
    model.saveToXML(filename,tag_name)
    
    model2 = pin.Model()
    model2.loadFromXML(filename,tag_name)
    
    self.assertTrue(model == model2)
  
  def testBIN(self):
    model = pin.buildSampleModelHumanoidRandom()
    filename = main_path + "/model.bin"
    model.saveToBinary(filename)
    
    model2 = pin.Model()
    model2.loadFromBinary(filename)
    
    self.assertTrue(model == model2)

if __name__ == '__main__':
  unittest.main()


