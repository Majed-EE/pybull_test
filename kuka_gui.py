import numpy as np
import pybullet as p
import time 
import pybullet_data
import inspect
import threading
import datetime
import csv

FILENAME = 'TRACE_NOSLIP.csv'
TRACE_HEADER = ['Event','Timestamp','J0_position', 'J0_velocity', 'J0_Fx', 'J0_Fy', 
                'J0_Fz', 'J0_Mx', 'J0_My', 'J0_Mz', 'J0_torque', 'J1_position', 'J1_velocity', 
                'J1_Fx', 'J1_Fy', 'J1_Fz', 'J1_Mx', 'J1_My', 'J1_Mz', 'J1_torque', 'J2_position', 
                'J2_velocity', 'J2_Fx', 'J2_Fy', 'J2_Fz', 'J2_Mx', 'J2_My', 'J2_Mz', 'J2_torque', 
                'J3_position', 'J3_velocity', 'J3_Fx', 'J3_Fy', 'J3_Fz', 'J3_Mx', 'J3_My', 'J3_Mz','J3_torque', 
                'J4_position', 'J4_velocity', 'J4_Fx', 'J4_Fy', 'J4_Fz', 'J4_Mx','J4_My', 'J4_Mz', 'J4_torque', 
                'J5_position', 'J5_velocity', 'J5_Fx', 'J5_Fy','J5_Fz', 'J5_Mx', 'J5_My', 'J5_Mz', 'J5_torque', 
                'J6_position', 'J6_velocity', 'J6_Fx', 'J6_Fy', 'J6_Fz', 'J6_Mx', 'J6_My', 'J6_Mz', 'J6_torque', 
                'J7_position''J7_velocity', 'J7_Fx', 'J7_Fy', 'J7_Fz', 'J7_Mx', 'J7_My', 'J7_Mz', 'J7_torque', 
                'J8_position', 'J8_velocity', 'J8_Fx', 'J8_Fy', 'J8_Fz', 'J8_Mx', 'J8_My', 'J8_Mz','J8_torque', 
                'J9_position', 'J9_velocity', 'J9_Fx', 'J9_Fy', 'J9_Fz', 'J9_Mx', 'J9_My''J9_Mz', 'J9_torque', 
                'J10_position', 'J10_velocity', 'J10_Fx', 'J10_Fy', 'J10_Fz','J10_Mx', 'J10_My', 'J10_Mz', 'J10_torque', 
                'J11_position', 'J11_velocity', 'J11_Fx','J11_Fy', 'J11_Fz', 'J11_Mx', 'J11_My', 'J11_Mz', 'J11_torque', 
                'J12_position', 'J12_velocity', 'J12_Fx', 'J12_Fy', 'J12_Fz', 'J12_Mx', 'J12_My', 'J12_Mz', 'J12_torque', 
                'J13_position','J13_velocity', 'J13_Fx', 'J13_Fy', 'J13_Fz', 'J13_Mx', 'J13_My', 'J13_Mz', 'J13_torque', 
                'J14_position', 'J14_velocity', 'J14_Fx', 'J14_Fy', 'J14_Fz', 'J14_Mx', 'J14_My', 'J14_Mz', 'J14_torque', 
                'J15_position', 'J15_velocity', 'J15_Fx', 'J15_Fy', 'J15_Fz', 'J15_Mx', 'J15_My', 'J15_Mz', 'J15_torque', 
                'J16_position', 'J16_velocity', 'J16_Fx', 'J16_Fy', 'J16_Fz', 'J16_Mx', 'J16_My', 'J16_Mz', 'J16_torque', 
                'J17_position', 'J17_velocity', 'J17_Fx', 'J17_Fy', 'J17_Fz', 'J17_Mx', 'J17_My', 'J17_Mz', 'J17_torque', 
                'J18_position', 'J18_velocity', 'J18_Fx', 'J18_Fy', 'J18_Fz', 'J18_Mx', 'J18_My', 'J18_Mz', 'J18_torque', 
                'J19_position', 'J19_velocity', 'J19_Fx', 'J19_Fy', 'J19_Fz', 'J19_Mx', 'J19_My', 'J19_Mz', 'J19_torque' ]
def save_to_csv(FILENAME, TRACE_CSV, type_open='a'):    
    with open(FILENAME,type_open,newline="") as trace_file:
        writer = csv.writer(trace_file, )
        writer.writerow(TRACE_CSV)
save_to_csv(FILENAME, TRACE_HEADER, 'w')


clid = p.connect(p.SHARED_MEMORY)
if (clid < 0):
    physicsClient = p.connect(p.GUI)

p.resetSimulation(p.RESET_USE_DEFORMABLE_WORLD)

p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally

def get_joint_states(robot, numJoints):
    DATUM = []
    joint_states = p.getJointStates(robot,range(numJoints),physicsClientId=physicsClient)   #0 to 20::: 4 are fixed
    for j in joint_states:
        for quadruple, k in enumerate(j): # index and value in j-->>ex [a,b,c] quadrupe=0, k=a so on and so forth
            if quadruple == 2: # likely jointReactionForces
                for l in k:
                    DATUM.append(l)
            else:
                DATUM.append(k)
    return DATUM

def get_joint_angles(robot, numJoints):
    DATUM=[]
    joint_states = p.getJointStates(robot,range(numJoints),physicsClientId=physicsClient)   #0 to 20::: 4 are fixed
    for j in joint_states:
        for quadruple, k in enumerate(j):
            if quadruple ==1 : #Just the joint angle (or velocity?)
                DATUM.append(k)
    return DATUM

def get_joint_states_hand_only(robot, numJoints):
    DATUM = []
    joint_states = p.getJointStates(robot,range(numJoints),physicsClientId=physicsClient)   #0 to 20::: 4 are fixed
    for jno, j in enumerate(joint_states):
        if jno in [16,17,18,19,20, 25,26,27,28,29, 34,35,36,37,38, 43,44,45,46,47]:  # what is this about (may be for kuka bot)? 
            #print(jno)
            for quadruple, k in enumerate(j):
                if quadruple == 2:
                    for l in k:
                        DATUM.append(l)
                else:
                    DATUM.append(k)
    return DATUM




planeId = p.loadURDF("plane.urdf")
kuka_allegro_hand_biotac = p.loadURDF("ll4ma_robots_description/robots/kuka-allegro-biotac.urdf")
numJoints_kuka=p.getNumJoints(kuka_allegro_hand_biotac)
arh_joints_kuka=[16,17,18,19,20, 25,26,27,28,29, 34,35,36,37,38, 43,44,45,46,47]
cube_big = p.loadSoftBody("ball.obj", simFileName = "ball.vtk", basePosition =  [-1.03,-0.03, 0.5], scale = 0.5, mass = 4, useNeoHookean = 1 , NeoHookeanMu = 700, NeoHookeanLambda = 800, NeoHookeanDamping = 0.01, useSelfCollision = 1, frictionCoeff = .5, collisionMargin = 0.01)
# p.setTimeStep(1/240.)
p.changeDynamics(cube_big,-1, lateralFriction=0.5)
for j in range(numJoints_kuka):
    p.enableJointForceTorqueSensor(kuka_allegro_hand_biotac, j, enableSensor=1)


p.setGravity(0,0,-9.8)


#reset
# p.changeDynamics(kuka_allegro_hand_biotac,-1, lateralFriction=0.5)
# p.changeDynamics(kuka_allegro_hand_biotac,-1, rollingFriction=0.5)
joint_cmd = [0 for _ in range(numJoints_kuka)]
# p.setJointMotorControlArray(kuka_allegro_hand_biotac, range(numJoints_kuka), p.POSITION_CONTROL,  targetPositions=joint_cmd)
# RECORD 1ST FROM HERE
# event is reset position

default_joint_angle=get_joint_angles(kuka_allegro_hand_biotac,numJoints_kuka)
default_joint_state=get_joint_states(kuka_allegro_hand_biotac,numJoints_kuka)
default_joint_state_arh=get_joint_states_hand_only(kuka_allegro_hand_biotac,numJoints_kuka)

string=f"default_joint_state: {default_joint_state} \n default_joint_angle: {default_joint_angle} \n default_joint_state_arh: {default_joint_state_arh}"

save_to_csv(FILENAME,['RESET_POSITION',datetime.datetime.now().time() ]+get_joint_states_hand_only(kuka_allegro_hand_biotac, numJoints_kuka),'a')



#THUMB
joint_cmd[43]= 90 * np.pi/180
joint_cmd[45]= 5 * np.pi/180
joint_cmd[46]= 10 * np.pi/180

thumb=[43,44,45]
joint_no=3
angle_degrees=90
joint_cmd[joint_no] = -angle_degrees *np.pi/180
p.setJointMotorControlArray(kuka_allegro_hand_biotac, thumb, p.POSITION_CONTROL,  targetPositions=[43,44,45])



# time.sleep(1/20.)
p.setRealTimeSimulation(1)
# print(string)
for c_step in range(100):
    p.stepSimulation()
    # time.sleep(1./24.)
   
#     joint_info=p.getJointState(
#         bodyUniqueId=kuka_allegro_hand_biotac, 
#         jointIndex=16)
#     print(joint_info)
#     time.sleep(1./10.)
