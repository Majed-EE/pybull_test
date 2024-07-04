import pybullet as p
import time
import pybullet_data
import math
import numpy as np
import matplotlib.pyplot as plt



############# hand control ################
finger_bend_angle=60



GUI=1
if GUI==1:
    physicsClient=p.connect(p.GUI)
else:
    physicsClient=p.connect(p.DIRECT)
# p.setAdditionalSearchVisualizer()
p.setAdditionalSearchPath(pybullet_data.getDataPath())
# p.resetSimulation(p.RESET_USE_DEFORMABLE_WORLD)

# p.setGravity(0,0,-9.8) # x,y,z direction


# camera position
p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
p.resetDebugVisualizerCamera(cameraDistance=0.4,cameraYaw=50,cameraPitch=-50,cameraTargetPosition=[0,0,0])


# define start pose
StartPos_cuboid=[0.5, 0, 0.04]     #[0.1,0,0.03998089919150022] 



StartPos_sphere=[-1,0,0.049987949685306586]  
StartPos_arh,StartOrn_arh=[0,0,0],p.getQuaternionFromEuler([0,0,0])
StartOrientation=p.getQuaternionFromEuler([0,0,0])
soft_body_position= [0.09,0,0.04]

planeId=p.loadURDF("plane.urdf")
StartPos_lite6,StartOrn_lite_6=[0,0,0],p.getQuaternionFromEuler([0,0,0])



########################## Rigid Bodies ###################
cuboidId=p.loadURDF(r"models/simple_object/box.urdf",StartPos_cuboid)
sphereId=p.loadURDF(r"models/simple_object/sphere.urdf",StartPos_sphere)

lite_arm_Id=p.loadURDF(r"models/arm/lite6.urdf",StartPos_lite6,StartOrn_lite_6)
initial_position=math.radians(180)
p.resetJointState(
            bodyUniqueId=lite_arm_Id, 
            jointIndex=3, 
            targetValue=initial_position
        )




# armId=p.loadURDF(r"models/end_effector/arm.urdf",StartPos_sphere)
# arhId=p.loadURDF(r"models/end_effector/left.urdf")
lite6_end_effector_link=7
arh_base_link=0

pos_lite6_end_effector_link, orn_lite6_end_effector_link=p.getLinkState(lite_arm_Id,lite6_end_effector_link)[:2]

arhId=p.loadURDF(r"models/end_effector/left.urdf",pos_lite6_end_effector_link,orn_lite6_end_effector_link)
print("line xxxxxxxxxxxxxx")
print(pos_lite6_end_effector_link, orn_lite6_end_effector_link)
print("line xxxxxxxxxxxxxx")


pos_arh_base_link, orn_arh_base_link=p.getBasePositionAndOrientation(arhId)[:2]
print(pos_arh_base_link,orn_arh_base_link)
print("XXXXXXXXXXXXXxxxxxx")


# cube_big = p.loadSoftBody("ball.obj", simFileName = "ball.vtk", basePosition =  [-1.03,-1.03, 0.5], scale = 0.05, mass = 4, useNeoHookean = 1 , NeoHookeanMu = 700, NeoHookeanLambda = 800, NeoHookeanDamping = 0.01, useSelfCollision = 1, frictionCoeff = .5, collisionMargin = 0.01)

# p.changeDynamics(cube_big,-1, lateralFriction=0.5)

sub=[-0.01,0.0,-0.09]
constraint_id = p.createConstraint(
    parentBodyUniqueId=lite_arm_Id,
    parentLinkIndex=lite6_end_effector_link,
    childBodyUniqueId=arhId,
    childLinkIndex=arh_base_link,
    jointType=p.JOINT_FIXED,
    jointAxis=[0, 0, 0],
    parentFramePosition=sub, #search
    childFramePosition=[0, 0, 0],
    #parentFrameOrientation=robot_end_effector_orn,
    #childFrameOrientation=allegro_hand_base_orn
)













def define_bot(bot_id):
    bot_movable_joints={}
    num_joint=p.getNumJoints(bot_id)
    print(f"total number of joint {num_joint}")
    for current_joint in range(num_joint):
        botJointInfo=p.getJointInfo(
            bodyUniqueId=bot_id,
            jointIndex=current_joint)
            
        if botJointInfo[2]!=4: # non fixed joints 
            
            defaultState=p.getJointState(bot_id,current_joint)
            bot_movable_joints[current_joint]={"Name":botJointInfo[1],
                                               "index":current_joint,
                                               "type":botJointInfo[2],
                                               "jointCurrentState":defaultState[0],
                                            "jointLowerLimit":botJointInfo[8], # 8 lower limit
                                            "jointUpperLimit":botJointInfo[9]} # 9 upper limit
            print(f"movable joint {bot_movable_joints[current_joint]}")
    return bot_movable_joints



def simulation_fun(bot_id,link):
    steps=50
    print("running simulation block")
    for c_step in range(steps):
         p.stepSimulation()
         time.sleep(1./24.)
         current_positino=p.getJointState(
             bodyUniqueId=bot_id,
             jointIndex=link)   
         print(f"current position for index {link}: {math.degrees(current_positino[0])}")


def bot_mover_demo(bot_id):
    bot_dict=define_bot(bot_id)
    for key in bot_dict.keys():
        print(key) 
        factor=0.5 # percent of extreme posiiton 
        target_state=factor*bot_dict[key]["jointLowerLimit"]
        p.setJointMotorControlArray(
            bodyUniqueId=bot_id, 
            jointIndices=[key], 
            controlMode=p.POSITION_CONTROL,  
            targetPositions=[target_state]
        )
        print(f"moving joint {key} to lower limit {math.degrees(bot_dict[key]['jointLowerLimit'])}")
        time.sleep(1)
        simulation_fun(bot_id,key)
        print(f"simulation done to extreme for joint{key}")
        time.sleep(4)
        print("going back to original position")
        target_state=bot_dict[key]["jointCurrentState"] # default starting state
        p.setJointMotorControlArray(
            bodyUniqueId=bot_id, 
            jointIndices=[key], 
            controlMode=p.POSITION_CONTROL,  
            targetPositions=[target_state]
        )
        print(f"moving joint {key} to default state {math.degrees(target_state)}")
        time.sleep(1)
        simulation_fun(bot_id,key)
        print(f"simulation done complete for joint {key}")
        time.sleep(4)
    print("simulation done")




# bot_mover_demo(lite_arm_Id)
for _ in range(100):
    p.stepSimulation()
    time.sleep(1./24.)