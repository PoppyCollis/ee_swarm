import sys
# path to folder which contains situsim_v1_2
sys.path.insert(1, '..')
from situsim_v1_2 import *
import numpy as np
import matplotlib.pyplot as plt

# plot a list of robots' sensor outputs - for a robot with only 2 sensors
def plot_all_EBA_robots_second_sensors(all_ts, robots):
    fig, ax = plt.subplots(2, 1)
    for i, robot in enumerate(robots):
        ax[0].plot(all_ts[i], robot.left2_sensor.activations, label='robot' + str(i))
        ax[1].plot(all_ts[i], robot.right2_sensor.activations, label='robot' + str(i))
        ax[0].legend()
        ax[1].legend()
        ax[0].set_xlabel('Time')
        ax[1].set_xlabel('Time')
        ax[0].set_ylabel('Activation')
        ax[1].set_ylabel('Activation')
        ax[0].set_title('Second left sensor')
        ax[1].set_title('Second right sensor')
    fig.tight_layout()

# plot a list of robots' sensor noise in vertically arranged subplots
def plot_all_EBA_robots_second_sensor_noise(all_ts, robots):
    fig, ax = plt.subplots(2, 1)
    for i, robot in enumerate(robots):
        if robot.left2_sensor.noisemaker is not None:
            ax[0].plot(all_ts[i], robot.left2_sensor.noisemaker.noises, label='robot' + str(i), linewidth='2')
        if robot.right2_sensor.noisemaker is not None:
            ax[1].plot(all_ts[i], robot.right2_sensor.noisemaker.noises, label='robot' + str(i), linewidth='2')
        ax[0].legend()
        ax[1].legend()
        ax[0].set_xlabel('Time')
        ax[1].set_xlabel('Time')
        ax[0].set_ylabel('Noise')
        ax[1].set_ylabel('Noise')
        ax[0].set_title('Left sensor noise')
        ax[1].set_title('Right sensor noise')
    fig.tight_layout()
