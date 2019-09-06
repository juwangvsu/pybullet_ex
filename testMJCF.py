import os
import inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0, parentdir)

import pybullet as p
import pybullet_data
import time

def adddomino(p):
  y2z = p.getQuaternionFromEuler([0, 0, 1.57])
  meshScale = [1, 1, 1]
  visualShapeId = p.createVisualShape(shapeType=p.GEOM_MESH,
                                    fileName="domino/domino.obj",
                                    rgbaColor=[1, 1, 1, 1],
                                    specularColor=[0.4, .4, 0],
                                    visualFrameOrientation=y2z,
                                    meshScale=meshScale)

  boxDimensions = [0.5 * 0.00635, 0.5 * 0.0254, 0.5 * 0.0508]
  collisionShapeId = p.createCollisionShape(p.GEOM_BOX, halfExtents=boxDimensions)
  objid=p.createMultiBody(baseMass=0.025,
                      baseCollisionShapeIndex=collisionShapeId,
                      baseVisualShapeIndex=visualShapeId,
                      basePosition=[0.04,  0.05, 1.06],
                      useMaximalCoordinates=True)
  #p.resetBaseVelocity(objid,linearVelocity=[2,2,1])


def test(args):
  p.connect(p.GUI)
  p.setGravity(0, 0, -10)
  p.setAdditionalSearchPath(pybullet_data.getDataPath())
  fileName = os.path.join("mjcf", args.mjcf)
  print("fileName")
  print(fileName)
  objs=p.loadMJCF(fileName)
  print('... loaded objs ...',objs)
  p.resetBasePositionAndOrientation(objs[1], [0.789351, 0.962124, 0],
                                  [0,0,0,1])
  p.stepSimulation()
  time.sleep(0.01)
  obj2s=p.loadMJCF(fileName)
  print('... loaded objs ...',obj2s)
  p.resetBasePositionAndOrientation(obj2s[1], [1.789351, 1.962124, 1],
                                  [0,0,0,1])
  adddomino(p)
  while (1):
    p.stepSimulation()
    p.getCameraImage(320, 240)
    time.sleep(0.01)


if __name__ == '__main__':
  import argparse
  parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
  parser.add_argument('--mjcf', help='MJCF filename', default="humanoid.xml")
  args = parser.parse_args()
  test(args)
