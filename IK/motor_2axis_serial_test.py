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
coords = [55, 60, 0]
port = 'COM7'

## Variables
stepsperrev = 200 #200 steps/rev
distance = 1.27 #mm distance travelled for 1/4-20
xlength = 150
ylength = 245
steps = np.zeros((4,1)) #x,y,z,yaw

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
steps[0] = (location[1]*xlength)/distance
steps[1] = (location[2]*ylength)/distance

## Establish Serial
ser = serial.Serial(port, 115200, timeout=.1)
time.sleep(1) #give the connection a second to settle

# Convert to str
varx = str(int(steps[0]))
vary = str(int(steps[1]))
ser_input = varx + ',' + vary
print(ser_input)

ser_input = bytes(ser_input, encoding="ascii")

if True:
    #ser.write(input_value.encode())
    #time.sleep(2)
    ser.write(ser_input)
    time.sleep(1)
    #ser.write(b'0')
    # ser.close()
    # exit()