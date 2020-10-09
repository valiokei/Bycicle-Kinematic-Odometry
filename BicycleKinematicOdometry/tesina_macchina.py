from unreal_api.environment import Environment
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
from matplotlib.path import Path
import matplotlib.patches as patches
from matplotlib import style
from BicycleKinematicOdometry import BicycleKinematicOdometry


sensor_settings = {
    "RGBCamera": {
        "width": 84,
        "height": 84,
        "channels": "RGB",
        "FOV"	: 90,
        "show": True
    },
    "GPS": {},
    "CarEncoder": {}
}

action_manager_settings = {
    "CarActionManager": {
        "command_dict": {
            "TURN": 0,
            "STRAIGHT": 1
        },
        "settings": {
        }
    }
}

reset_manager_settings = {
    "ContinuousResetManager": {}
}

observations_step = ["RGBCamera", "GPS", "CarEncoder"]


env = Environment(port=9734, address='localhost', sensor_settings=sensor_settings,
                  action_manager_settings=action_manager_settings, reset_manager_settings=reset_manager_settings, render=True)


style.use('fivethirtyeight')

# ********* PER IL NOSTRO *************
figx = plt.figure()
ax1 = figx.add_subplot(2,2,1)
ax1.axis('equal')
ax = figx.add_subplot(2,2,2)
ax.axis('equal')
xs = []
ys = []
#  ********* PER IL NOSTRO *************

gxs = []
gys = []

def animateg(i):
    observations = env.get_obs(observations_step)
    gxs.append(observations[1][6])
    gys.append(observations[1][7])


    ax.clear()
    ax.plot(gxs, gys,'b')


#  ********* PER IL NOSTRO *************

classObject = BicycleKinematicOdometry()
def animatex(i):
    
    observations = env.get_obs(observations_step)
    steerings = [observations[2][0], observations[2][2],
                 observations[2][4], observations[2][6]]
    rotations = [observations[2][1], observations[2][3],
                 observations[2][5], observations[2][7]],

    wheelRadius = observations[2][8]
    delta_T = 0.1
    # that mean that the center of mass of the car is in the middle between front and rear wheel
    Lr = observations[2][10]/2 
    Lf = observations[2][10]/2

    classObject.SetStaticData(wheelRadius,delta_T,Lr,Lf)
    trajectory = classObject.carTrajector(steerings,rotations)

    xs.append(trajectory[0])
    ys.append(trajectory[1])

    ax1.clear()
    ax1.plot(xs, ys,'r')
#  ********* PER IL NOSTRO *************


ani = animation.FuncAnimation(figx, animateg, interval=100)
# plt.show()


#  ********* PER IL NOSTRO *************
ani2 = animation.FuncAnimation(figx, animatex, interval=100)
plt.show()
