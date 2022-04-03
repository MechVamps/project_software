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

## Input
target = [85, 60, 0] # where do you want the object to be
angle = 30 # yaw angle, NEED TO FIX IK FOR THIS
init_loc = [0 ,0 ,0] # where is the object at the start of this
serial_port = 'COM7'

## Variables
stepsperrev = 200 #200 steps/rev
distance = 1.27 #mm distance travelled for 1/4-20
xlength = 187
ylength = 254
steps = np.zeros((3,1)) #x,y,z

def chain_and_gantry_ik(initial,final):
    ## Calculate end effector coordinates from givens
    hypotenuse = 30 ## CHANGE
    temp = hypotenuse*math.cos(math.radians(15)) # predetermined constant angle ## CHANGE 15
    tran_z = hypotenuse*math.sin(math.radians(15))
    tran_x = temp*math.sin(math.radians(angle+90))
    tran_y = temp*math.cos(math.radians(angle+90))

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
            name="gantry_to_servo",
            origin_translation=[-98.2, -0.7, -231.8],
            origin_orientation=[0, 0, 0],
            translation=[0,0,0],
            joint_type='prismatic'
        ),
        URDFLink(
            name="servo",
            origin_translation=[tran_x, tran_y, -tran_z],
            origin_orientation=[0, 0, 0],
            rotation=[0,0,0]
        )
    ])

    ## IK does its thing
    location = my_chain.inverse_kinematics(final)
    
    # Convert to number of motor rotations. This is what's inputted to arduino
    # (- initial/distance) is to make it not relative
    steps[0] = (location[1]*xlength)/distance - initial[0]/distance
    steps[1] = (location[2]*ylength)/distance - initial[1]/distance
    print(steps)
    return steps

def serial_to_arduino(port,steps):
    # Establish Serial
    ser = serial.Serial(port, 115200, timeout=.1)
    time.sleep(1) #give the connection a second to settle

    # Convert to str
    varx = str(int(steps[0]))
    vary = str(int(steps[1]))
    ser_input = varx + ',' + vary + ';' + angle
    print(ser_input)

    ser_input = bytes(ser_input, encoding="ascii")

    if True:
        ser.write(ser_input)
        time.sleep(1)

chain_and_gantry_ik(init_loc,target)
serial_to_arduino(serial_port,steps)