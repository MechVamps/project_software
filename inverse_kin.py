from ikpy.chain import Chain
from ikpy.link import OriginLink,URDFLink
import numpy as np
import ikpy.utils.plot as plot_utils
import matplotlib.pyplot
from mpl_toolkits.mplot3d import Axes3D

my_chain = Chain(name='gantry', links=[
    OriginLink(),
    URDFLink(
        name="x_gantry",
        origin_translation=[0, 0, 0],
        origin_orientation=[0, 0, 0],
        translation=[150,0,0],
        joint_type='prismatic'
    ),
    URDFLink(
        name="y_gantry",
        origin_translation=[0, 0, 0],
        origin_orientation=[0, 0, 0],
        translation=[0,245,0],
        joint_type='prismatic'
    ),
    URDFLink(
        name="z_gantry",
        origin_translation=[0, 0, 0],
        origin_orientation=[0, 0, 0],
        translation=[0,0,-100],
        joint_type='prismatic'
    )
])

location = my_chain.inverse_kinematics([55, 100, -50])
print(location) # Basically outputs percentage of gantry it's gone through
ax = matplotlib.pyplot.figure().add_subplot(111, projection='3d')
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')
my_chain.plot(location, ax)
matplotlib.pyplot.show()