import pybullet as p
import pybullet_data
import time

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)

planeId = p.loadURDF("plane.urdf")

cubeStartPos = [0,0,0]
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])

robotID = p.loadURDF("leg_right.urdf",cubeStartPos, cubeStartOrientation, 
                   # useMaximalCoordinates=1, ## New feature in Pybullet
                   flags=p.URDF_USE_INERTIA_FROM_FILE)

for i in range (10000):
    p.stepSimulation()
    time.sleep(1./240.)

cubePos, cubeOrn = p.getBasePositionAndOrientation(robotID)
print(cubePos,cubeOrn)
p.disconnect()
