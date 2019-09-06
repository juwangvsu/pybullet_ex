#add parent dir to find package. Only needed for source code build, pip install doesn't need it.
import os, inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0, parentdir)

from pybullet_envs.bullet.racecarGymEnv import RacecarGymEnv
isDiscrete = False


def main():

  environment = RacecarGymEnv(renders=True, isDiscrete=isDiscrete)
  environment.reset()

  targetVelocitySlider = environment._p.addUserDebugParameter("wheelVelocity", -1, 1, 0)
  steeringSlider = environment._p.addUserDebugParameter("steering", -1, 1, 0)
  y2z = environment._p.getQuaternionFromEuler([0, 0, 1.57])
  meshScale = [1, 1, 1]
  visualShapeId = environment._p.createVisualShape(shapeType=environment._p.GEOM_MESH,
                                    fileName="domino/domino.obj",
                                    rgbaColor=[1, 1, 1, 1],
                                    specularColor=[0.4, .4, 0],
                                    visualFrameOrientation=y2z,
                                    meshScale=meshScale)

  boxDimensions = [0.5 * 0.00635, 0.5 * 0.0254, 0.5 * 0.0508]
  collisionShapeId = environment._p.createCollisionShape(environment._p.GEOM_BOX, halfExtents=boxDimensions)
  while (True):
    targetVelocity = environment._p.readUserDebugParameter(targetVelocitySlider)
    steeringAngle = environment._p.readUserDebugParameter(steeringSlider)
    if (isDiscrete):
      discreteAction = 0
      if (targetVelocity < -0.33):
        discreteAction = 0
      else:
        if (targetVelocity > 0.33):
          discreteAction = 6
        else:
          discreteAction = 3
      if (steeringAngle > -0.17):
        if (steeringAngle > 0.17):
          discreteAction = discreteAction + 2
        else:
          discreteAction = discreteAction + 1
      action = discreteAction
    else:
      action = [targetVelocity, steeringAngle]
    qKey = ord('a')
    keys = environment._p.getKeyboardEvents()
    if qKey in keys and keys[qKey]&environment._p.KEY_WAS_TRIGGERED:
      objid=environment._p.createMultiBody(baseMass=0.025,
                      baseCollisionShapeIndex=collisionShapeId,
                      baseVisualShapeIndex=visualShapeId,
                      basePosition=[0.04,  0.05, 0.06],
                      useMaximalCoordinates=True)
      environment._p.resetBaseVelocity(objid,linearVelocity=[2,2,1])
    state, reward, done, info = environment.step(action)
    obs = environment.getExtendedObservation()
    print("obs")
    print(obs)


if __name__ == "__main__":
  main()
