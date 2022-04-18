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

## Serial Setup to Arduino

target = [128,20,-304] #final
angleo = 0 # yaw angle
angle = angleo #+ 90 #-65
init_loc = [128,20,-304]
origin = [7.5,5.6,-304] # this is our origin

## Variables
stepsperrev = 200 #200 steps/rev
distance = 1.27 #mm distance travelled for 1/4-20
xlength = 161
ylength = 199
steps = np.zeros((3,1)) #x,y,z
# Coordinates to go under skin

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
            origin_translation=[7.5,5.6 + 14.4,-236],#[30.9,16.98,-283],
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
    location = my_chain.inverse_kinematics(final)
    location_in = my_chain.inverse_kinematics(initial)

    steps[0] = (location[1]*xlength)/distance - (location_in[1]*xlength)/distance
    steps[1] = (location[2]*ylength)/distance - (location_in[2]*ylength)/distance
    print(steps)

    ax = matplotlib.pyplot.figure().add_subplot(111, projection='3d')
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    my_chain.plot(location, ax)
    matplotlib.pyplot.show()
    
chain_and_gantry_ik(init_loc,target)


# number of steps
# this gives the number of revs. Advait does it so n*stepsperrev
# steps here is n in his code

