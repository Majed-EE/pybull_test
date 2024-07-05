

import pybullet as p
import time
import pybullet_data
import math
import numpy as np

# import keyboard
def is_key_pressed(key):
    print("this ran")
    return keyboard.is_pressed(key)

import copy
import threading
import tkinter as tk


iteration_label = None
iteration_text_var = None
flagg=0
flagg1=0
a=0
slip_check=None

  
physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version

#Removing Debug visualation tools in simulation
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
p.resetSimulation(p.RESET_USE_DEFORMABLE_WORLD)

p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
p.setGravity(0, 0, -10)



soft_body_position=[0, -0.5, 0.03]
soft_body_Id= p.loadSoftBody("ball.obj", simFileName = "ball.vtk", basePosition =soft_body_position, 
                            scale=0.06,
                            mass=0.15, 
                            useNeoHookean=1, 
                            NeoHookeanMu=100, 
                            NeoHookeanLambda=600,  
                            NeoHookeanDamping=0.2, 
                            collisionMargin=0)
p.setPhysicsEngineParameter(sparseSdfVoxelSize=0.25)


planeId = p.loadURDF("plane.urdf")
cube_id = p.loadURDF(r"models/simple_object/box.urdf",[0.5, -0.5, 0.3],p.getQuaternionFromEuler([0, 0, 0]))
sphere_id = p.loadURDF(r"models/simple_object/sphere.urdf",[0, 0.4, 0],p.getQuaternionFromEuler([0, 0, 0]))


# soft_body_Id= p.loadSoftBody("ball.obj", simFileName = "ball.vtk", basePosition =soft_body_position, 
#                             scale=0.07,
#                             mass=0.015, 
#                             useNeoHookean=1, 
#                             NeoHookeanMu=100, 
#                             NeoHookeanLambda=600,  
#                             NeoHookeanDamping=0.2, 
#                             collisionMargin=0)
# p.setPhysicsEngineParameter(sparseSdfVoxelSize=0.25)


aabb_min, aabb_max = p.getAABB(cube_id)
print(f"min:{aabb_min}")
print(f"max:{aabb_max}")

StartPos = [1, 0, 0]
StartOrientation = p.getQuaternionFromEuler([0, 0, 0])
pandaId = p.loadURDF(r"models/arm/lite6.urdf", [0, 0, 0], p.getQuaternionFromEuler([0, 0, 0]))


def invertQuaternion(quat): 
    # Invert a quaternion 
    return np.array([quat[0], -quat[1], -quat[2], -quat[3]])

robot_end_effector_link_index = 7  
allegro_hand_base_link_index = 0  

# Get the current position and orientation of the end-effector of the robotic arm
robot_end_effector_pos, robot_end_effector_orn = p.getLinkState(pandaId, robot_end_effector_link_index)[:2]

boxId = p.loadURDF(r"models/end_effector/left.urdf",robot_end_effector_pos,robot_end_effector_orn)
p.resetDebugVisualizerCamera(cameraDistance=1.2, cameraYaw=25, cameraPitch=-30, cameraTargetPosition=[0, 0, 0])
#fixing base of arm                 
p.createConstraint(parentBodyUniqueId=pandaId,
                   parentLinkIndex=-1,
                   childBodyUniqueId=-1,
                   childLinkIndex=0,
                   jointType=p.JOINT_FIXED,
                   jointAxis=[0, 0, 0],
                   parentFramePosition=[0, 0, 0],
                   childFramePosition=[0, 0, 0],
                   parentFrameOrientation=[0, 0, 0, 1],
                   childFrameOrientation=[0, 0, 0, 1])
                   

# Get the current position and orientation of the base link of the Allegro hand
allegro_hand_base_pos, allegro_hand_base_orn = p.getBasePositionAndOrientation(boxId)[:2]

# Calculate the relative position and orientation between the two objects--- this should come zero
relative_pos = [robot_end_effector_pos[i] - allegro_hand_base_pos[i] for i in range(3)] 
relative_orn = p.getDifferenceQuaternion(robot_end_effector_orn, invertQuaternion(allegro_hand_base_orn))

#sub=[0.066,0.005,0.059]
sub=[-0.01,0.00,-0.09] # offset
result = [a - b for a, b in zip(relative_pos,sub)]

# Create a fixed constraint to attach the Allegro hand to the robotic arm
constraint_id = p.createConstraint(
    parentBodyUniqueId=pandaId,
    parentLinkIndex=robot_end_effector_link_index,
    childBodyUniqueId=boxId,
    childLinkIndex=allegro_hand_base_link_index,
    jointType=p.JOINT_FIXED,
    jointAxis=[0, 0, 0],
    parentFramePosition=result, #search
    childFramePosition=[0, 0, 0],
    parentFrameOrientation=robot_end_effector_orn,
    childFrameOrientation=allegro_hand_base_orn
)


#all joints with 0 velocity
# v = [0,1,2,3,4,5,6,7] 
# p.setJointMotorControlArray(pandaId, v, p.VELOCITY_CONTROL, targetVelocities=[0]* len(v), forces=[100]* len(v))

y=[1,2,3,4,5,6,7]
p.setJointMotorControlArray( # erect position see after loading deafult position
        bodyUniqueId=pandaId,
        jointIndices=y,
        controlMode=p.POSITION_CONTROL,
        targetPositions=[math.radians(0),math.radians(0),math.radians(180),math.radians(0),math.radians(0),math.radians(0),math.radians(0)],
        # forces=[100,50,100,100,20,1000,1000],  # Adjust the force as needed
    )


#all joints of palm at 0 angle---- open hand positon
x=[1,6,11,16,0,2,3,4,5,7,8,9,10,12,13,14,15,17,18,19,20]
p.setJointMotorControlArray(
        bodyUniqueId=boxId,
        jointIndices=x,
        controlMode=p.POSITION_CONTROL,
        targetPositions=[math.radians(0)]*len(x),
        forces=[100]*len(x),  # Adjust the force as needed
    )
    
# joints info
jointsnum = p.getNumJoints(boxId)
jointsnum1 = p.getNumJoints(pandaId)
# print(f'number of joints in Hand {jointsnum}')
# print(f'number of joints in Arm {jointsnum1}\n')
for i in range(jointsnum):
    joinfo = p.getJointInfo(boxId, i)
    # print(f'info of joint {i} {joinfo}')    
print('\n\n')    
for i in range(jointsnum1):
    joinfo1 = p.getJointInfo(pandaId, i)
    # print(f'..Arm joint {i} {joinfo1}')
# print('\n\n')  

#friction setup
p.changeDynamics(planeId, -1, lateralFriction=0.9) 
p.changeDynamics(boxId, -1, lateralFriction=0.9) 
p.changeDynamics(cube_id, -1, lateralFriction=0.5) 
p.changeDynamics(soft_body_Id, -1, lateralFriction=0.9)

#enabling haptic data
p.enableJointForceTorqueSensor(boxId,5,1)
p.enableJointForceTorqueSensor(boxId,10,1)
p.enableJointForceTorqueSensor(boxId,15,1)
p.enableJointForceTorqueSensor(boxId,20,1)

# Create a thread for running the Tkinter window
# tkinter_thread = threading.Thread(target=msg_show)
# tkinter_thread.start()

for _ in range(100):
    p.stepSimulation()
    time.sleep(1./27.)

# Set the target position for the palm
objectposition, objectorientation = p.getBasePositionAndOrientation(soft_body_Id)

add=[0,0.2,0.65]
target_position = [a + b for a, b in zip(objectposition,add)]

num_joints = p.getNumJoints(pandaId)
end_effector_link_name = b'link_eef'  

for i in range(num_joints):
    joint_info = p.getJointInfo(pandaId, i)
    if joint_info[12] == end_effector_link_name:
        end_effector_link_index = joint_info[0]
        break

# Use PyBullet's inverse kinematics solver
joint_angles = p.calculateInverseKinematics(pandaId, end_effector_link_index, target_position,maxNumIterations=10000, residualThreshold=1e-6 )
#print(joint_angles)


#manual obstacle to start simulation

# while True:
#     if is_key_pressed('h'):
#         break

j1=[2,7,12]
a1=[50,50,50]
j2=[16,17,18,19]
a2=[90,20,50,10]
j3=[3,8,13,4,9,14]
a3=[35]*len(j3)
a1=[math.radians(a1[x]) for x in range(len(a1))]
a2=[math.radians(a2[x]) for x in range(len(a2))]
a3=[math.radians(a3[x]) for x in range(len(a3))]
j_all=j1+j2+j3
a_all=a1+a2+a3

for x in range(2):
    print(f"simulation starts in {1-x}")
    time.sleep(1)

for i in range(150):
   p.setJointMotorControlArray(
     bodyUniqueId=pandaId,
     jointIndices=[1,2,3,4,5,6],
     controlMode=p.POSITION_CONTROL,
     targetPositions=joint_angles,
     forces=[50]*6,  # Adjust the force as needed
    )
   
   k= p.getJointState(pandaId, 3)
#    p.setJointMotorControlArray(
#         bodyUniqueId=pandaId,
#         jointIndices=[6],
#         controlMode=p.POSITION_CONTROL,
# targetPositions=[math.radians(180)],
#         forces=[100],  # Adjust the force as needed
#     )
   p.stepSimulation()
   time.sleep(1./27.)

# for i in range(40):
#      p.stepSimulation() 
#      p.setJointMotorControlArray(
#         bodyUniqueId=boxId,
#         jointIndices=j_all,
#         controlMode=p.POSITION_CONTROL,
#         targetPositions=a_all,
#         forces=[1000]*len(a_all),  
#      )
  
    #  p.setJointMotorControlArray(
    #     bodyUniqueId=boxId,
    #     jointIndices=[2,7,12],
    #     controlMode=p.POSITION_CONTROL,
    #     targetPositions=[math.radians(50),math.radians(50),math.radians(50)],
    #     forces=[1000]*3,  
    #  )
  
    #  p.setJointMotorControlArray(
    #     bodyUniqueId=boxId,
    #     jointIndices=[16,17, 18,19],
    #     controlMode=p.POSITION_CONTROL,
    #     targetPositions=[math.radians(90),math.radians(20),math.radians(50),math.radians(10)],
    #     forces=[1000]*4,  
    #  )
     
    #  p.setJointMotorControlArray(
    #     bodyUniqueId=boxId,
    #     jointIndices=[3,8,13, 4,9,14],
    #     controlMode=p.POSITION_CONTROL,
    #     targetPositions=[math.radians(35)]*6,
    #     forces=[1000]*6,  
    #  ) 
  
    
    #  time.sleep(1./27.)


 # After pickup

# for i in range(10000):
#      p.stepSimulation() 
#      p.resetDebugVisualizerCamera(cameraDistance=1.2, cameraYaw=25+i, cameraPitch=-30, cameraTargetPosition=[0, 0, 0])

#      p.setJointMotorControlArray(
#         bodyUniqueId=pandaId,
#         jointIndices=[3],
#         controlMode=p.POSITION_CONTROL,
#         targetPositions=[math.radians(180)],
#         forces=[100],  
#      )
#      time.sleep(1./27.)
# cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
# print(cubePos, cubeOrn)
# # Close the Tkinter window when the simulation loop ends
# if iteration_label:
#     iteration_label.master.destroy()
# p.disconnect()    

