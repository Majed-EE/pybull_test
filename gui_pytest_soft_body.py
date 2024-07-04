import pybullet as p
import time
import pybullet_data
import math
import numpy as np
import matplotlib.pyplot as plt



############# hand control ################
finger_bend_angle=0
soft_body=True
# body_type=["soft","rigid"]
# press_body=body_type[1]





physicsClient=p.connect(p.GUI)
# p.setAdditionalSearchVisualizer()
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.resetSimulation(p.RESET_USE_DEFORMABLE_WORLD)

p.setGravity(0,0,-9.8) # x,y,z direction


# camera position
p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
p.resetDebugVisualizerCamera(cameraDistance=0.4,cameraYaw=50,cameraPitch=-50,cameraTargetPosition=[0,0,0])


# define start pose
if soft_body:
    StartPos_cuboid=[0.5, 0, 0.04]     #[0.1,0,0.03998089919150022] 
    soft_body_position= [0.09,0,0.04]
else:
    StartPos_cuboid=[0.15, 0, 0.04]     #[0.1,0,0.03998089919150022] 
    soft_body_position= [0.5,0,0.04]

StartPos_sphere=[-1,0,0.049987949685306586]  
StartPos_arh=[0,0,0]
StartOrientation=p.getQuaternionFromEuler([0,0,0])


planeId=p.loadURDF("plane.urdf")


########################## Rigid Bodies ###################
cuboidId=p.loadURDF(r"models/simple_object/box.urdf",StartPos_cuboid)
sphereId=p.loadURDF(r"models/simple_object/sphere.urdf",StartPos_sphere)
arhId=p.loadURDF(r"models/end_effector/left.urdf",StartPos_arh)
# armId=p.loadURDF(r"models/end_effector/arm.urdf",StartPos_sphere)
# ur5Id=p.loadURDF(r"models/end_effector/left.urdf")


##################################### Soft bodies #####################################



soft_body_Id= p.loadSoftBody("ball.obj", simFileName = "ball.vtk", basePosition =soft_body_position, 
                            scale=0.07,
                            mass=0.015, 
                            useNeoHookean=1, 
                            NeoHookeanMu=100, 
                            NeoHookeanLambda=600,  
                            NeoHookeanDamping=0.2, 
                            collisionMargin=0)
p.setPhysicsEngineParameter(sparseSdfVoxelSize=0.25)


##### draw axis lines ###########

# Function to draw axes at a given position and orientation
def draw_axes(body_id, link_id=-1, length=0.1, duration=0):
    # Get the position and orientation of the body or link
    if link_id == -1:
        position, orientation = p.getBasePositionAndOrientation(body_id)
    else:
        position, orientation = p.getLinkState(body_id, link_id)[:2]

    # Convert quaternion to rotation matrix
    rotation_matrix = p.getMatrixFromQuaternion(orientation)

    # Define the axes directions
    x_axis = [length * rotation_matrix[0], length * rotation_matrix[3], length * rotation_matrix[6]]
    y_axis = [length * rotation_matrix[1], length * rotation_matrix[4], length * rotation_matrix[7]]
    z_axis = [length * rotation_matrix[2], length * rotation_matrix[5], length * rotation_matrix[8]]

    # Draw the axes
    p.addUserDebugLine(position, [position[0] + x_axis[0], position[1] + x_axis[1], position[2] + x_axis[2]], [1, 0, 0], 2, duration)  # X-axis in red
    p.addUserDebugLine(position, [position[0] + y_axis[0], position[1] + y_axis[1], position[2] + y_axis[2]], [0, 1, 0], 2, duration)  # Y-axis in green
    p.addUserDebugLine(position, [position[0] + z_axis[0], position[1] + z_axis[1], position[2] + z_axis[2]], [0, 0, 1], 2, duration)  # Z-axis in blue

# Draw the axes for the box
# draw_axes(arhId)



# Function to draw the origin axes
def draw_origin_axes(length=0.6, duration=0):
    origin = [0, 0, 0]
    
    # X-axis in red
    p.addUserDebugLine(origin, [length, 0, 0], [1, 0, 0], 2, duration)
    # Y-axis in green
    p.addUserDebugLine(origin, [0, length, 0], [0, 1, 0], 2, duration)
    # Z-axis in blue
    p.addUserDebugLine(origin, [0, 0, length], [0, 0, 1], 2, duration)

# Draw the origin axes
draw_origin_axes()








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



#### arh control
tips_finger=[5,10,15,20]
for tip in tips_finger:
    p.enableJointForceTorqueSensor(
    bodyUniqueId=arhId,
    jointIndex=tip,
    enableSensor=1
    )




finger_1=[2,3,4]
finger_2=[7,8,9]
finger_3=[12,13,14]
finger_4=[16,18,19]
all_movable=[2,3,4,7,8,9,12,13,14,16,18,19]

target_position=math.radians(finger_bend_angle)

Force_applied = 100
finger= finger_2#all_movable#finger_2   #  [11]#finger_3
tip=finger[-1]+1
print(tip)
p.setJointMotorControlArray(
    bodyUniqueId=arhId,
    jointIndices=finger,
    controlMode=p.POSITION_CONTROL,
    targetPositions=[target_position]*len(finger),
    forces=[Force_applied]*len(finger) )



############# simulation ##########################
fx=[]
fy=[]
fz=[]
flag_touch=False
ch=0
ch_step=0

if soft_body:
    press_body_id=soft_body_Id
else:
    press_body_id=cuboidId #
# Get vertex positions before interaction
print("before interatction mesh information")
vertices_before = np.array(p.getMeshData(press_body_id)[1])
print(f"shape of mesh {np.shape(vertices_before)}")

# print(np.shape(vertices_before))

position, orientation = p.getBasePositionAndOrientation(sphereId)
filtered_position_sphere = list(dim if abs(dim) > threshold else 0 for dim in position)
print(f"Filtered position of sphere (x, y, z): {filtered_position_sphere}")
    # for tip in tips_finger:
for c_step in range(60):
    p.stepSimulation()
    time.sleep(1./24.)
    
    #### Sphere spatial information

    #     joint_state=p.getJointState(
    #         bodyUniqueId=arhId, 
    #         jointIndex=tip)
    #     print(f"joint info of joint {tip}")
    #     print(joint_state)
            # for tip in tips_finger:
  
    joint_state=p.getJointState(
        bodyUniqueId=arhId, 
        jointIndex=tip)
    print(f"joint info of joint {tip}")
    print(joint_state[2][:3])
    fx.append(joint_state[2][0])
    fy.append(joint_state[2][1])
    fz.append(joint_state[2][2])
    print("contact point info")
    contact_info=p.getContactPoints(arhId,soft_body_Id,tip)
    
    print(len(contact_info))
    if len(contact_info)>0:
        position, orientation = p.getBasePositionAndOrientation(sphereId)
        filtered_position_sphere = list(dim if abs(dim) > threshold else 0 for dim in position)
        print(f"Filtered position of sphere (x, y, z): {filtered_position_sphere}")
    # for tip in tips_finger:
        
        
        if len(contact_info)-ch>0:
            ch=len(contact_info)
            print("change step")
            ch_step=c_step
            print(c_step)


    #### cuboid spatial information
    position, orientation = p.getBasePositionAndOrientation(cuboidId)
    filtered_position_cuboid = list(dim if dim > threshold else 0 for dim in position)
    # print(f"Filtered position of cuboid (x, y, z): {filtered_position_cuboid}")
    # print(position,orientation)
    dimension_cuboid = [aabb_cuboid[1][i] - aabb_cuboid[0][i] for i in range(3)]
    # print(f"initial dimension cuboid {dimension_cuboid}")
# # Get the position and orientation of the robot
# position, orientation = p.getBasePositionAndOrientation(sphereId)

######################################## Simulation Ends ##################################################


# Get vertex positions after interaction
vertices_after = np.array(p.getMeshData(press_body_id)[1])
print("after interatction mesh information")
print(f"shape of mesh {np.shape(vertices_after)}")

print(vertices_after)
# Measure of deformation
deformation = vertices_after - vertices_before
deformation_magnitude = np.linalg.norm(deformation, axis=1)

print("Deformation magnitudes of vertices:")

print(deformation_magnitude)

"""Returns num mesh vertices and vertex positions."""
# kwargs = {}
# if hasattr(p, "MESH_DATA_SIMULATION_MESH"):
#     kwargs["flags"] = p.MESH_DATA_SIMULATION_MESH
# num_verts, mesh_vert_positions = p.getMeshData(soft_body_Id, **kwargs)
# print(num_verts,mesh_vert_positions)
# plot force
x = [x for x in range(len(fz))]#np.linspace(0, 5, 100)
# vertical=[fz[x] if x==ch_step else 0 for x in range(len(fz)) ]
# Create the plot
plt.plot(x, fx, label='fx')  # Plot the first line with a label
plt.plot(x, fy, label='fy')  # Plot the second line with a label
plt.plot(x, fz, label='fz')  # Plot the third li"ne with a label
plt.scatter(ch_step,fz[ch_step], label="contact_point_instance")
# print(vertical)
# Add labels and title
plt.xlabel('X-axis')
plt.ylabel('Y-axis')

plt.title('Three Graphs Overlaid')

# Add legend
plt.legend()

plt.show()
