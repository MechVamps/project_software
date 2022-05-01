from ctypes.wintypes import tagMSG
from tkinter import Y
from ikpy.chain import Chain
from ikpy.link import OriginLink,URDFLink
import numpy as np
import ikpy.utils.plot as plot_utils
import matplotlib.pyplot
from mpl_toolkits.mplot3d import Axes3D
import math
import serial
import time
from RealsenseUtil import get_target_point_camera_pose

## Input
tf_matrix_to_target = get_target_point_camera_pose()
origin = [128,20,-304]
#target = [178,30,-304] 
target = np.asarray([[1,0,0,origin[0]], [0, 1, 0, origin[1]], [0, 0, 1, origin[2]], [0, 0, 0, 1]]) @ tf_matrix_to_target # translation from needle point to camera center 
target = [target[0, 3], target[1, 3], target[2, 3]]
print("target point for ik: ", target)
# where do you want the object to be
angle = 0 # yaw angle
init_loc = [158,20,-304] # At start, this is where it is
#gantry_origin = [7.5,5.6,-304]
serial_port = 'COM7'
#serial_port =  '/dev/ttyACM0'

## Variables
stepsperrev = 200 #200 steps/rev
distance = 1.27 #mm distance travelled for 1/4-20
xlength = 161
ylength = 117
steps = np.zeros((3,1)) #x,y,z
# Coordinates to go under skin
under_skin = [10*math.sin(math.radians(angle)),10*math.cos(math.radians(angle)),0]
under_skin = np.add(target,under_skin)
print(under_skin)

# Establish Serial
ser = serial.Serial(serial_port, 115200, timeout=.1)
time.sleep(1) #give the connection a second to settle

def chain_and_gantry_ik(initial,final):
    ## Calculate end effector coordinates from givens
    hypotenuse = 120.5
    tran_x = hypotenuse*math.cos(math.radians(angle))
    tran_y = hypotenuse*math.sin(math.radians(angle))
    tran_z = -68


    ## Define Chain
    my_chain = Chain(name='gantry', links=[
        OriginLink(),
        URDFLink(
            name="x_gantry",
            origin_translation=[0, 0, 0],
            origin_orientation=[0, 0, 0],
            translation=[xlength,0,0],
            joint_type='prismatic'
        ),
        URDFLink(
            name="y_gantry",
            origin_translation=[0, 0, 0],
            origin_orientation=[0, 0, 0],
            translation=[0,ylength,0],
            joint_type='prismatic'
        ),
        URDFLink(
            name="gantry_to_preneedle",
            origin_translation=[7.5,5.6 + 14.4,-236],
            origin_orientation=[0, 0, 0],
            translation=[0,0,0],
            joint_type='prismatic'
        ),
        URDFLink(
            name="servo",
            origin_translation=[tran_x, tran_y, tran_z],
            origin_orientation=[0, 0, 0],
            rotation=[0,0,0]
        )
    ])

    ## IK does its thing
    location = my_chain.inverse_kinematics(final)
    location_in = my_chain.inverse_kinematics(initial)
    # Convert to number of motor rotations. This is what's inputted to arduino
    steps[0] = (location[1]*xlength)/distance - (location_in[1]*xlength)/distance
    steps[1] = (location[2]*ylength)/distance - (location_in[2]*ylength)/distance
    print(steps)
    return steps

def serial_to_arduino(steps,lin_act_dir):
    # Convert to str
    varx = str(int(steps[0]))
    vary = str(int(steps[1]))
    ard_angle = angle + 90 +65

    # Combine serial
    ser_input = varx + ',' + vary + ';' + str(ard_angle) + '?' + lin_act_dir
    print(ser_input)

    ser_input = bytes(ser_input, encoding="ascii")

    if True:
       ser.write(ser_input)
       time.sleep(1)

# Step 1
chain_and_gantry_ik(init_loc,target)
serial_to_arduino(steps,'0') #0 is retract
time.sleep(20)

# # Step 2
# chain_and_gantry_ik(target,under_skin)
# serial_to_arduino(steps,'0')
# time.sleep(10)

# # Step 3
# chain_and_gantry_ik(under_skin,under_skin)
# serial_to_arduino(steps,'1') #1 is extend
# time.sleep(10)

# # Step 4
# chain_and_gantry_ik(under_skin,target)
# serial_to_arduino(steps,'1')
# time.sleep(10)

# # Step 5
# chain_and_gantry_ik(target,init_loc)
# serial_to_arduino(steps,'1')
# time.sleep(20)