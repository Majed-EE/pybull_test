import pybullet as p
import time
import pybullet_data
import math
import numpy as np
import matplotlib.pyplot as plt

DURATION = 10000
ALPHA = 300


clid = p.connect(p.SHARED_MEMORY)
if (clid < 0):
	print("<0")
	physicsClient = p.connect(p.GUI)


p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
print("data path: %s " % pybullet_data.getDataPath())
planeId = p.loadURDF("plane.urdf")
StartPos_cuboid=[0.3,0.8, 0.5]

cube_big = p.loadURDF("models/simple_object/box.urdf",StartPos_cuboid,useFixedBase=1)
# p.loadURDF("./haptics_examples/objects/cube_big.urdf",[0.3,0.8, 0.5],useFixedBase=1)
cube = p.loadURDF("models/simple_object/box.urdf",[0.25,0.4, 1.02])
ur5_allegro = p.loadURDF("models/end_effector/left.urdf",[0,0,0], useFixedBase=1)
#0.8379473429355728, 0.7057802158888344, 0.34998928933959805)
p.setGravity(0,0,-10)
#GYM 

noJoints = p.getNumJoints(ur5_allegro)



p.changeDynamics(cube,-1, lateralFriction=0.5)
p.changeDynamics(cube,-1, rollingFriction=0.5)
#p.changeDynamics(ur5_allegro,-1, lateralFriction=0.5)
#p.changeDynamics(ur5_allegro,-1, rollingFriction=0.5)


#p.setJointMotorControlArray(sawyerallegro, range(7), pb.POSITION_CONTROL, targetPositions= [0, 0.2, 0.3, 1.5, 2.5, 0.2, 0.1])
cubePos, cubeOrn= p.getBasePositionAndOrientation(cube)

error= [1,0.2,2.2]
jointPoses = np.array(p.calculateInverseKinematics(ur5_allegro, 6, np.add(cubePos,error), [0,0.7,0.7,0]))



"""

p.setRealTimeSimulation(1)
p.setJointMotorControlArray(ur5_allegro, range(22), p.POSITION_CONTROL, targetPositions= jointPoses)

"""




jointPoses[10] = 0.4
jointPoses[11] = 1
"""
"""
while True:

 p.setJointMotorControlArray(ur5_allegro, range(22), p.POSITION_CONTROL, targetPositions= jointPoses)
 print(p.getJointState(ur5_allegro,7))
 #print(jointPoses)
 p.stepSimulation()

 #10,14,22
#sawyer_wsg50

"""




"""