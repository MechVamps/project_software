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
#from RealsenseUtil import get_target_point_camera_pose

#################IF THERE IS ANGLE ERROR, IT'S BECAUSE OF THE +90 IN ALL OF THEM
## Input
#target = get_target_point_camera_pose()
target = [85, 60, -283] # where do you want the object to be
angle = 0 # yaw angle, NEED TO FIX IK FOR THIS
init_loc = [30.9,16.98,-283] # where is the object at the start of this
serial_port = 'COM7'

## Variables
stepsperrev = 200 #200 steps/rev
distance = 1.27 #mm distance travelled for 1/4-20
xlength = 187
ylength = 254
steps = np.zeros((3,1)) #x,y,z
# Coordinates to go under skin
under_skin = [25*math.cos(math.radians(angle+90)),25*math.sin(math.radians(angle+90)),0]
under_skin = np.add(target,under_skin)
print(under_skin)
def chain_and_gantry_ik(initial,final):
    ## Calculate end effector coordinates from givens
    hypotenuse = 30 ## CHANGE
    temp = hypotenuse*math.cos(math.radians(15)) # predetermined constant angle ## CHANGE 15
    tran_x = temp*math.sin(math.radians(angle+90))
    tran_y = temp*math.cos(math.radians(angle+90))

    tran_z = -23
    

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
            origin_translation=[30.9,16.98,-283],
            origin_orientation=[0, 0, 0],
            translation=[0,0,0],
            joint_type='prismatic'
        ),
        # URDFLink(
        #     name="servo",
        #     origin_translation=[tran_x, tran_y, -tran_z],
        #     origin_orientation=[0, 0, 0],
        #     rotation=[0,0,0]
        # )
    ])

    ## IK does its thing
    location = my_chain.inverse_kinematics(final)
    location_in = my_chain.inverse_kinematics(initial)
    # Convert to number of motor rotations. This is what's inputted to arduino
    steps[0] = (location[1]*xlength)/distance - (location_in[1]*xlength)/distance
    steps[1] = (location[2]*ylength)/distance - (location_in[2]*ylength)/distance
    print(steps)

    # # If want to plot
    # ax = matplotlib.pyplot.figure().add_subplot(111, projection='3d')
    # ax.set_xlabel('x')
    # ax.set_ylabel('y')
    # ax.set_zlabel('z')
    # my_chain.plot(location, ax)
    # matplotlib.pyplot.show()

    return steps

def serial_to_arduino(port,steps,lin_act_dir):
    # Establish Serial
    #ser = serial.Serial(port, 115200, timeout=.1)
    #time.sleep(1) #give the connection a second to settle

    # Convert to str
    varx = str(int(steps[0]))
    vary = str(int(steps[1]))

    # lin_act_dir is the direction of the linear actuator and can only be "push" or "pull"
    # Combine serial
    ser_input = varx + ',' + vary + ';' + str(angle) + '?' + lin_act_dir
    #print(ser_input)

    ser_input = bytes(ser_input, encoding="ascii")

    #if True:
    #    ser.write(ser_input)
    #    time.sleep(1)

# Step 1
chain_and_gantry_ik(init_loc,target)
serial_to_arduino(serial_port,steps,'pull')

# Step 2
chain_and_gantry_ik(target,under_skin)
serial_to_arduino(serial_port,steps,'pull')

# # Step 3
# chain_and_gantry_ik(under_skin,under_skin)
# serial_to_arduino(serial_port,steps,'push')

# # Step 4
# chain_and_gantry_ik(under_skin,target)
# serial_to_arduino(serial_port,steps,'push')

# # Step 5
# chain_and_gantry_ik(target,init_loc)
# serial_to_arduino(serial_port,steps,'push')