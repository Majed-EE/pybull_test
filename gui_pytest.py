import pybullet as p
import time
import pybullet_data
# import math
import numpy as np
import keyboard

physicsClient=p.connect(p.GUI)
# p.setAdditionalSearchVisualizer()
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-9.8) # x,y,z direction


# camera position
p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
p.resetDebugVisualizerCamera(cameraDistance=0.4,cameraYaw=50,cameraPitch=-50,cameraTargetPosition=[0,0,0])


# define start pose
StartPos_cuboid=[1,0,0]
StartPos_sphere=[-1,0,0]
StartPos_arh=[0,0,0]
StartOrientation=p.getQuaternionFromEuler([0,0,0])


planeId=p.loadURDF("plane.urdf")
cuboidId=p.loadURDF(r"models/simple_object/box.urdf",StartPos_cuboid)
sphereId=p.loadURDF(r"models/simple_object/sphere.urdf",StartPos_sphere)
arhId=p.loadURDF(r"models/end_effector/left.urdf",StartPos_arh)
# armId=p.loadURDF(r"models/end_effector/arm.urdf",StartPos_sphere)
# ur5Id=p.loadURDF(r"models/end_effector/left.urdf")



# set constraints
p.createConstraint(parentBodyUniqueId=arhId,
                   parentLinkIndex=-1,
                   childBodyUniqueId=-1,
                   childLinkIndex=0,
                   jointType=p.JOINT_FIXED,
                   jointAxis=[0, 0, 0],
                   parentFramePosition=[0, 0, 0],
                   childFramePosition=[0, 0, 0],
                   parentFrameOrientation=[0, 0, 0, 1],
                   childFrameOrientation=[0, 0, 0, 1])




# ur5Id=p.loadURDF(r"left.urdf")
jointsnum=p.getNumJoints(sphereId)
# print(jointsnum)
# AABB returns a tuple of two points (min and max corners of the AABB)

aabb_cuboid = p.getAABB(cuboidId)
aabb_sphere = p.getAABB(sphereId)
# aabb_min = aabb[0]
# aabb_max = aabb[1]
threshold = 1e-5
# Calculate the dimensions
dimension_cuboid = [aabb_cuboid[1][i] - aabb_cuboid[0][i] for i in range(3)]
print(f"initial dimension cuboid {dimension_cuboid}")
for _ in range(1000):
    p.stepSimulation()
    time.sleep(1./24.)
    
    position, orientation = p.getBasePositionAndOrientation(sphereId)
    # print(position,orientation)
    dimension_cuboid = [aabb_cuboid[1][i] - aabb_cuboid[0][i] for i in range(3)]
    # print(f"initial dimension cuboid {dimension_cuboid}")
    filtered_position = list(dim if dim > threshold else 0 for dim in position)
    print(f"Filtered position (x, y, z): {filtered_position}")
    filtered_orientation = list(dim if dim > threshold else 0 for dim in orientation)
    print(f"Filtered orientation (x, y, z): {filtered_orientation}")

# # Get the position and orientation of the robot
position, orientation = p.getBasePositionAndOrientation(sphereId)