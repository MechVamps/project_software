from tkinter import Y
from ikpy.chain import Chain
from ikpy.link import OriginLink,URDFLink
import numpy as np
import ikpy.utils.plot as plot_utils
import matplotlib.pyplot
from mpl_toolkits.mplot3d import Axes3D
import math
import serial, time

## Input
x_distance = 20 #mm CHANGE
port = 'COM4'

## Variables
stepsperrev = 400 #400 steps/rev, we're using half steps
distance = 1.27 #mm distance travelled for 1/4-20
xlength = 132 # LENGTH OF GANTRY AXIS

## Define Chain
my_chain = Chain(name='gantry', links=[
    OriginLink(),
    URDFLink(
        name="x_gantry",
        origin_translation=[0, 0, 0],
        origin_orientation=[0, 0, 0],
        translation=[xlength,0,0],
        joint_type='prismatic'
    )
])

location = my_chain.inverse_kinematics(x_distance)
#print(location) # Basically outputs percentage of gantry it's gone through
ax = matplotlib.pyplot.figure().add_subplot(111, projection='3d')
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')
my_chain.plot(location, ax)
matplotlib.pyplot.show()

# number of steps
steps = (location[1]*xlength)/distance# this gives the number of revs. Advait does it so n*stepsperrev
# steps here is n in his code
print(steps)

## Establish Serial
ser = serial.Serial(port, 115200, timeout=.1)
time.sleep(1) #give the connection a second to settle

if True:
    #ser.write(input_value.encode())
    #time.sleep(2)
    ser.write(steps.encode())
    time.sleep(2)
    ser.write(b'0')
    # ser.close()
    # exit()