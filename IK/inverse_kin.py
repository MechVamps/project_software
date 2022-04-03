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

## Input
coords = [205, 150, 0]
angle = -20


## Variables
stepsperrev = 200 #200 steps/rev
distance = 1.27 #mm distance travelled for 1/4-20
xlength = 150
ylength = 245
zlength = 100
steps = np.zeros((3,1)) #x,y,z,yaw

## Trig angle for link coords
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
        origin_translation=[90.8, 3.2, -187.3],
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

location = my_chain.inverse_kinematics(coords)
#print(location) # Basically outputs percentage of gantry it's gone through
ax = matplotlib.pyplot.figure().add_subplot(111, projection='3d')
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')
my_chain.plot(location, ax)
matplotlib.pyplot.show()

# number of steps
# this gives the number of revs. Advait does it so n*stepsperrev
# steps here is n in his code
steps[0] = (location[1]*xlength)/distance#*stepsperrev
steps[1] = (location[2]*ylength)/distance#*stepsperrev
steps[2] = (location[3]*zlength)/distance#*stepsperrev
#steps[3] = angle/1.8
print(steps)