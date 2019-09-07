import pybullet as p
import time

p.connect(p.GUI)

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
                      basePosition=[-.5,  -2, 0.14],
                      useMaximalCoordinates=True)
  p.resetBaseVelocity(objid,linearVelocity=[0,10,1])

p.loadURDF("table_s/table.urdf", -.5000000, -2.00000, -.820000, 0.000000, 0.000000, 0.0, 1.0)
#p.setGravity(0, 0, -10)
arm = p.loadURDF("widowx/gun.urdf", useFixedBase=1)
#arm = p.loadURDF("widowx/widowx.urdf", basePosition=[-.5, -2, 0.01], baseOrientation=[0,0,0,1])
ball = p.loadURDF("sphere2.urdf", useFixedBase=1)
p.resetBasePositionAndOrientation(ball, [-.5, 2, 0.1], [0,0,0,1])
p.setGravity(0, 0, -10)

p.resetBasePositionAndOrientation(arm, [-0.098612, -2, 0.14018],
                                  [0.000000, 0.000000, 0.000000, 1.000000])

while (1):
  qKey = ord('a')
  keys = p.getKeyboardEvents()
  if qKey in keys and keys[qKey]&p.KEY_WAS_TRIGGERED:
    adddomino(p)
  p.stepSimulation()
  time.sleep(0.01)
  #p.saveWorld("test.py")
  viewMat = p.getDebugVisualizerCamera()[2]
  projMatrix = [0.7499999403953552, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, -1.0000200271606445, -1.0, 2.0, 2.0, -0.02000020071864128, 2.0]
  #projMatrix = p.getDebugVisualizerCamera()[3]
  width = 640
  height = 480
  img_arr = p.getCameraImage(width=width,
                             height=height,
                             viewMatrix=viewMat,
                             projectionMatrix=projMatrix)
