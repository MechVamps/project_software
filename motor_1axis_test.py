from tkinter import Y
from ikpy.chain import Chain
from ikpy.link import OriginLink,URDFLink
import numpy as np
import ikpy.utils.plot as plot_utils
import matplotlib.pyplot
from mpl_toolkits.mplot3d import Axes3D
import math

## Input
x_distance = 5 #mm CHANGE

## Variables
stepsperrev = 200 #200 steps/rev
distance = 1.27 #mm distance travelled for 1/4-20
xlength = 150 # LENGTH OF GANTRY AXIS

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
steps = (location[1]*xlength)/distance*stepsperrev
print(steps)